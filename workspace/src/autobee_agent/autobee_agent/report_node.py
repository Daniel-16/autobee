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

Guidelines:
- Speak in plain conversational English, no markdown
- Anchor your report to the original intent (what the user actually asked about)
- Be specific with numbers: peak person count, dominant density level, anomaly count
- If no people were detected, say so clearly -- do not invent observations
- If anomalies fired, lead with that
- Tone: factual, calm, like a security operator briefing

Input JSON describes the mission outcome:
{{
  "intent": "<original user request>",
  "frames_observed": <int>,
  "peak_person_count": <int>,
  "mean_person_count": <float>,
  "density_breakdown": {{"low": <int>, "medium": <int>, "high": <int>}},
  "anomalies": [<list of anomaly strings>],
  "duration_s": <float>
}}

Reply with ONLY the report text. No JSON, no preamble, no quotes around the message.
"""


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
        self.create_subscription(String, "/perception/event", self._on_event, 10)
        self.create_subscription(String, "/perception/anomaly", self._on_anomaly, 10)

        self._mission_active = False
        self._mission_intent = ""
        self._mission_start_wall = None
        self._counts = []
        self._density_counts = {"low": 0, "medium": 0, "high": 0}
        self._anomalies = []
        self._last_event_time = None

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
        self._mission_start_wall = time.time()
        self._counts = []
        self._density_counts = {"low": 0, "medium": 0, "high": 0}
        self._anomalies = []
        self._last_event_time = None
        self.get_logger().info(
            f"new mission tracked (intent={self._mission_intent!r}); aggregating perception events"
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

    def _check_silence(self):
        if not self._mission_active or self._last_event_time is None:
            return
        elapsed_silent = time.time() - self._last_event_time
        if elapsed_silent >= self.silence_timeout:
            self._finalize_mission()

    def _finalize_mission(self):
        if not self._counts and not self._anomalies:
            self.get_logger().warn("mission ended with zero observations; skipping report")
            self._mission_active = False
            return

        summary = {
            "intent": self._mission_intent,
            "frames_observed": len(self._counts),
            "peak_person_count": max(self._counts) if self._counts else 0,
            "mean_person_count": (
                round(statistics.fmean(self._counts), 2) if self._counts else 0.0
            ),
            "density_breakdown": dict(self._density_counts),
            "anomalies": self._anomalies,
            "duration_s": round(time.time() - self._mission_start_wall, 1)
            if self._mission_start_wall else 0.0,
        }
        summary_json = json.dumps(summary)
        self.get_logger().info(f"mission summary: {summary_json}")

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
