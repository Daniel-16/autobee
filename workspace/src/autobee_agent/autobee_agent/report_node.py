"""Mission report synthesizer (LangChain + Gemini).

Closes the autonomy loop. Subscribes to:
    /mission_command       -- mission start (intent + params)
    /perception/event      -- per-frame detections from yolov8_perception_node
    /perception/anomaly    -- threshold-breach alerts

Aggregates per-mission statistics, then on mission completion (silence
window after last detection OR explicit /mission_done), calls Gemini to
write a short natural-language report keyed to the original user intent.

Publishes:
    /mission_report        -- std_msgs/String, plain-text human-readable summary
"""

import json
import os
import statistics
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from dotenv import load_dotenv
    for _env_candidate in (
        "/workspaces/AUTOBEE/.env",
        "/root/autobee_ws/.env",
        os.path.expanduser("~/autobee.env"),
    ):
        if os.path.isfile(_env_candidate):
            load_dotenv(_env_candidate, override=False)
            break
except ImportError:
    pass


REPORT_PROMPT = """You are AutoBee's report writer. AutoBee just finished an autonomous surveillance flight. Write a SHORT (2-4 sentence) natural-language report to the non-pilot operator who sent the original request.

The mission's "skill" tells you what to LEAD with:
  - hover_around_me / quick_survey / sweep_area / scan_in_place:
      Lead with crowd density: peak person count + density level. Mention any anomalies after.
  - count_people:
      Lead with the peak person count and (if available) trend. Be precise with numbers.
  - detect_anomaly_buildings:
      Lead with anomaly alerts ordered by severity (use the highest-severity alert's description verbatim).
      If anomalies is empty: say "no anomalies detected at <landmark>" or equivalent. Crowd numbers are secondary here.

General rules:
- Plain conversational English. No markdown. No preamble. No quotes.
- Translate landmark identifiers to plain English: "convocation_ground" -> "the convocation ground", "main_gate" -> "the main gate", "library" -> "the library entrance", "admin" -> "the administration block".
- If "landmark" is null, refer to the area as "your current location" -- do NOT invent a place.
- If "outcome" is "aborted" or "failed": lead with that fact and the reason. Analytics are secondary.
- If "frames_observed" == 0 or peak_person_count == 0: say so explicitly. Do not invent observations.
- Tone: factual, calm, security-operator briefing.

Input JSON describes the mission outcome:
{{
  "skill": "<skill name>",
  "intent": "<original user request>",
  "landmark": "<landmark id>" | null,
  "outcome": "completed" | "aborted" | "failed",
  "terminal_phase": "<last phase>",
  "failure_reason": "<reason if aborted/failed>",
  "frames_observed": <int>,
  "peak_person_count": <int>,
  "mean_person_count": <float>,
  "density_breakdown": {{"low": <int>, "medium": <int>, "high": <int>}},
  "anomalies": [{{"type": "...", "severity": "...", "description": "..."}}, ...],
  "final_analytics": {{ "trend": "...", "density_label": "...", "rate_per_sec": <float>, ... }} | null,
  "duration_s": <float>
}}

Reply with ONLY the report text.
"""

# Skill-specific deterministic templates. These skip Gemini entirely because
# their content doesn't benefit from generative phrasing.
DETERMINISTIC_REPORTS = {
    "return_home": "Returned to base. Mission ended cleanly.",
    "abort":       "Mission aborted at operator request.",
}


class ReportNode(Node):
    def __init__(self):
        super().__init__("report_node")

        self.declare_parameter("model", "gemini-2.5-flash")
        self.declare_parameter("silence_timeout_s", 12.0)

        self.model_name = str(self.get_parameter("model").value)
        self.silence_timeout = float(self.get_parameter("silence_timeout_s").value)

        self._chain = self._init_llm()

        self.report_pub = self.create_publisher(String, "/mission_report", 10)
        self.create_subscription(String, "/mission_command", self._on_command, 10)
        self.create_subscription(String, "/mission_status", self._on_status, 10)
        self.create_subscription(String, "/perception/event", self._on_event, 10)
        self.create_subscription(String, "/perception/anomaly", self._on_anomaly, 10)
        self.create_subscription(String, "/anomaly_alert", self._on_anomaly_alert, 10)
        self.create_subscription(String, "/crowd_analytics", self._on_analytics, 10)

        self._mission_active = False
        self._mission_intent = ""
        self._mission_start_wall = None
        self._counts = []
        self._density_counts = {"low": 0, "medium": 0, "high": 0}
        self._anomalies = []
        self._anomaly_alerts = []
        self._last_analytics = None
        self._last_event_time = None
        self._final_status = None

        self.create_timer(2.0, self._check_silence)

        if self._chain is None:
            self.get_logger().fatal(
                "Gemini client unavailable; report_node cannot synthesize summaries."
            )
        else:
            self.get_logger().info(
                f"report_node up ({self.model_name}); aggregates /perception/event, "
                f"publishes /mission_report after {self.silence_timeout}s of silence"
            )

    def _init_llm(self):
        api_key = os.environ.get("GOOGLE_API_KEY")
        if not api_key:
            return None
        try:
            from langchain_google_genai import ChatGoogleGenerativeAI
            from langchain_core.prompts import ChatPromptTemplate
            from langchain_core.output_parsers import StrOutputParser
        except ImportError:
            return None

        llm = ChatGoogleGenerativeAI(
            model=self.model_name,
            google_api_key=api_key,
            temperature=0.2,
        )
        prompt = ChatPromptTemplate.from_messages([
            ("system", REPORT_PROMPT),
            ("human", "{summary_json}"),
        ])
        return prompt | llm | StrOutputParser()

    def _on_command(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if cmd.get("action") != "survey_local_area":
            return
        self._mission_active = True
        self._mission_intent = str(cmd.get("intent", ""))
        self._mission_skill = str(cmd.get("skill", "")) or None
        landmark = cmd.get("landmark")
        self._mission_landmark = (
            str(landmark).strip().lower() if isinstance(landmark, str) and landmark.strip() else None
        )
        self._mission_start_wall = time.time()
        self._counts = []
        self._density_counts = {"low": 0, "medium": 0, "high": 0}
        self._anomalies = []
        self._anomaly_alerts = []
        self._last_analytics = None
        self._last_event_time = None
        self._final_status = None
        self.get_logger().info(
            f"new mission tracked (intent={self._mission_intent!r} "
            f"landmark={self._mission_landmark!r}); aggregating perception events"
        )

    def _on_event(self, msg: String):
        if not self._mission_active:
            return
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        count = int(event.get("person_count", 0))
        density = str(event.get("density", "low"))
        self._counts.append(count)
        if density in self._density_counts:
            self._density_counts[density] += 1
        self._last_event_time = time.time()

    def _on_anomaly(self, msg: String):
        if not self._mission_active:
            return
        self._anomalies.append(msg.data)
        self._last_event_time = time.time()

    def _on_status(self, msg: String):
        if not self._mission_active:
            return
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        event = status.get("event")
        if event in ("completed", "aborted", "failed"):
            self._final_status = status
            self._finalize_mission()

    def _on_anomaly_alert(self, msg: String):
        if not self._mission_active:
            return
        try:
            alert = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._anomaly_alerts.append(alert)
        self._last_event_time = time.time()

    def _on_analytics(self, msg: String):
        if not self._mission_active:
            return
        try:
            self._last_analytics = json.loads(msg.data)
        except json.JSONDecodeError:
            return

    def _check_silence(self):
        if not self._mission_active or self._last_event_time is None:
            return
        elapsed_silent = time.time() - self._last_event_time
        if elapsed_silent >= self.silence_timeout:
            self._finalize_mission()

    def _finalize_mission(self):
        if not self._mission_active:
            return
        # Always synthesize a report -- a zero-detection mission, an aborted
        # mission, and a failed mission are all valid outcomes the operator
        # deserves to hear about.
        anomaly_summary = [
            {"type": a.get("type"), "severity": a.get("severity"), "description": a.get("description")}
            for a in self._anomaly_alerts
        ]
        if self._final_status:
            outcome = self._final_status.get("event", "completed")
            failure_reason = self._final_status.get("reason", "")
            terminal_phase = self._final_status.get("phase", "")
        else:
            outcome = "completed"
            failure_reason = ""
            terminal_phase = ""

        summary = {
            "skill": getattr(self, "_mission_skill", None),
            "intent": self._mission_intent,
            "landmark": getattr(self, "_mission_landmark", None),
            "outcome": outcome,
            "terminal_phase": terminal_phase,
            "failure_reason": failure_reason,
            "frames_observed": len(self._counts),
            "peak_person_count": max(self._counts) if self._counts else 0,
            "mean_person_count": (
                round(statistics.fmean(self._counts), 2) if self._counts else 0.0
            ),
            "density_breakdown": dict(self._density_counts),
            "raw_anomaly_strings": self._anomalies,
            "anomalies": anomaly_summary,
            "final_analytics": self._last_analytics,
            "duration_s": round(time.time() - self._mission_start_wall, 1)
            if self._mission_start_wall else 0.0,
        }
        summary_json = json.dumps(summary)
        self.get_logger().info(f"mission summary: {summary_json}")

        # Deterministic short skills skip Gemini for both speed and reproducibility.
        skill = summary.get("skill")
        report_text = DETERMINISTIC_REPORTS.get(skill)
        if report_text is None:
            report_text = self._synthesize_report(summary_json)

        if report_text:
            out = String()
            out.data = report_text
            self.report_pub.publish(out)
            self.get_logger().info(f"report: {report_text}")

        self._mission_active = False

    def _synthesize_report(self, summary_json):
        if self._chain is None:
            return None
        try:
            return self._chain.invoke({"summary_json": summary_json}).strip()
        except Exception as e:
            self.get_logger().error(f"report synthesis failed: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = ReportNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
