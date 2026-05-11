"""LLM mission planner (LangChain + Gemini) — skill-dispatched.

Subscribes:  /user_intent (std_msgs/String) -- natural-language command
Publishes:
    /mission_command (std_msgs/String JSON) -- validated mission spec
    /mission_abort   (std_msgs/String)      -- direct abort for skill=abort

Output schema (superset of the legacy survey_local_area shape so the
executor and report node stay compatible during the rollout):
    {
      "action": "survey_local_area",
      "skill":  "<one of SKILL_DEFAULTS keys>",
      "radius_m": <float>,
      "altitude_m": <float>,
      "pattern": "orbit"|"spiral",
      "landmark": "<id>"|null,
      "intent": "<short phrase>"
    }

GOOGLE_API_KEY required (loaded from /workspaces/AUTOBEE/.env). No mock fallback.
"""

import json
import os

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


KNOWN_LANDMARKS = {"main_gate", "library", "convocation_ground", "admin"}

# Per-skill defaults for the legacy survey_local_area schema. Skills that
# need genuinely new executor behavior (true yaw-only, immediate land) are
# wired up in M2; for M1 they degrade to small-radius orbit with the
# existing executor, which is sim-flyable today.
SKILL_DEFAULTS = {
    "hover_around_me": {
        "pattern": "orbit",  "radius_m": 4.0,  "altitude_m": 6.0,
        "needs_landmark": False,
    },
    "scan_in_place": {
        # Real yaw-only manoeuvre (M2). radius_m kept to satisfy LLM-side
        # envelope clamps; executor ignores it for pattern=yaw_only.
        "pattern": "yaw_only", "radius_m": 3.0, "altitude_m": 6.0,
        "needs_landmark": False,
    },
    "sweep_area": {
        "pattern": "spiral", "radius_m": 10.0, "altitude_m": 8.0,
        "needs_landmark": False,
    },
    "quick_survey": {
        "pattern": "orbit",  "radius_m": 4.0,  "altitude_m": 6.0,
        "needs_landmark": False,
    },
    "count_people": {
        "pattern": "orbit",  "radius_m": 4.0,  "altitude_m": 6.0,
        "needs_landmark": False,
    },
    "detect_anomaly_buildings": {
        "pattern": "orbit",  "radius_m": 6.0,  "altitude_m": 8.0,
        "needs_landmark": True,
    },
    "return_home": {
        # Real "descend in place + land" — no orbit, no transit. radius/altitude
        # are ignored by the executor for pattern=land_now but must satisfy the
        # LLM envelope clamps.
        "pattern": "land_now", "radius_m": 3.0, "altitude_m": 4.0,
        "needs_landmark": False,
    },
    "abort": {
        # Special-cased: published to /mission_abort, never a mission_command.
        "is_abort": True,
    },
}

SKILL_NAMES = list(SKILL_DEFAULTS.keys())


SYSTEM_PROMPT = """You are AutoBee's mission planner. AutoBee is an autonomous surveillance drone (DJI Tello class). You convert a non-pilot user's natural-language request into ONE valid JSON skill invocation.

Available skills (use the EXACT name string as "skill"):
  - "hover_around_me"          : Orbit the takeoff point at small radius. Triggers: "hover around me", "orbit me", "watch me", "circle around me", "stay near me"
  - "scan_in_place"            : Hold position and look around. Triggers: "look around", "do a 360", "scan", "what's around", "survey from here"
  - "sweep_area"               : Spiral pattern outward, full coverage. Triggers: "sweep", "thorough check", "every corner", "full scan", "comprehensive"
  - "quick_survey"             : Short orbit at low altitude. Triggers: "quick check", "a glance", "fast look", "brief"
  - "count_people"             : Headcount-focused orbit. Triggers: "how many people", "headcount", "count", "how many are there"
  - "detect_anomaly_buildings" : Surveil a named building for anomalies. Triggers: "check the library", "is anything off at admin", "scan the convocation ground for anomalies", "anything weird at the main gate". REQUIRES a landmark in params.
  - "return_home"              : Return to base and land. Triggers: "come back", "return", "land", "go home", "RTH", "back to base"
  - "abort"                    : Immediate emergency stop. Triggers: "stop", "abort", "emergency", "danger", "land now"

Known campus landmarks (use the lowercase identifier in params.landmark):
  "main_gate", "library", "convocation_ground", "admin"

Output JSON schema (return ONLY valid JSON, no prose, no markdown fences):
{{
  "skill": "<one of the skill names above>",
  "intent": "<short phrase describing what to look for>",
  "params": {{
    "landmark": "<landmark id>" | null,
    "radius_m": <number> | null,
    "altitude_m": <number> | null
  }}
}}

Rules:
- "skill" MUST be one of the listed names — no invention, no aliases.
- "landmark" is REQUIRED for "detect_anomaly_buildings" and may be null for everything else (default).
- "params" may be {{}} — sensible defaults exist per skill.
- Set "radius_m" / "altitude_m" only when the user gives a clear cue ("low", "high", "tightly", "wide"); otherwise omit them.

Examples:

User: "hover around me"
{{"skill":"hover_around_me","intent":"orbit the operator","params":{{}}}}

User: "watch me from above"
{{"skill":"hover_around_me","intent":"orbit the operator from altitude","params":{{"altitude_m":10}}}}

User: "look around and tell me what you see"
{{"skill":"scan_in_place","intent":"360 sweep from current position","params":{{}}}}

User: "do a thorough sweep of every corner"
{{"skill":"sweep_area","intent":"full coverage sweep","params":{{}}}}

User: "quick check of the area"
{{"skill":"quick_survey","intent":"quick local survey","params":{{}}}}

User: "how many people are at the convocation ground?"
{{"skill":"count_people","intent":"headcount at the convocation ground","params":{{"landmark":"convocation_ground"}}}}

User: "check the library entrance for anything unusual"
{{"skill":"detect_anomaly_buildings","intent":"anomaly check at the library","params":{{"landmark":"library"}}}}

User: "is anything weird at the admin block?"
{{"skill":"detect_anomaly_buildings","intent":"anomaly check at the admin block","params":{{"landmark":"admin"}}}}

User: "come back to base"
{{"skill":"return_home","intent":"return to base","params":{{}}}}

User: "stop now"
{{"skill":"abort","intent":"emergency abort","params":{{}}}}
"""


class LLMAgentNode(Node):
    def __init__(self):
        super().__init__("llm_agent_node")

        self.declare_parameter("model", "gemini-2.5-flash")
        self.model_name = str(self.get_parameter("model").value)

        self._client = self._init_llm()

        self.cmd_pub = self.create_publisher(String, "/mission_command", 10)
        self.abort_pub = self.create_publisher(String, "/mission_abort", 10)
        self.create_subscription(String, "/user_intent", self._on_intent, 10)

        if self._client is None:
            self.get_logger().fatal(
                "Gemini client failed to initialise. Ensure GOOGLE_API_KEY is set in "
                "/workspaces/AUTOBEE/.env."
            )
        else:
            self.get_logger().info(
                f"llm_agent_node up ({self.model_name}); skills={SKILL_NAMES}"
            )

    def _init_llm(self):
        api_key = os.environ.get("GOOGLE_API_KEY")
        if not api_key:
            self.get_logger().warn("GOOGLE_API_KEY not set - running in MOCK mode")
            return None
        try:
            from langchain_google_genai import ChatGoogleGenerativeAI
            from langchain_core.prompts import ChatPromptTemplate
            from langchain_core.output_parsers import JsonOutputParser
        except ImportError as e:
            self.get_logger().warn(
                f"langchain not installed ({e}) - running in MOCK mode."
            )
            return None

        llm = ChatGoogleGenerativeAI(
            model=self.model_name,
            google_api_key=api_key,
            temperature=0.1,
        )
        prompt = ChatPromptTemplate.from_messages([
            ("system", SYSTEM_PROMPT),
            ("human", "{user_input}"),
        ])
        return prompt | llm | JsonOutputParser()

    def _on_intent(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        if self._client is None:
            self.get_logger().error("Gemini client not initialised; cannot plan mission.")
            return
        self.get_logger().info(f"intent: {text!r}")

        parsed = self._llm_parse(text)
        if parsed is None:
            self.get_logger().error("could not produce a valid skill invocation")
            return

        skill = parsed["skill"]
        intent = parsed["intent"]
        params = parsed["params"]
        defaults = SKILL_DEFAULTS[skill]

        if defaults.get("is_abort"):
            self.abort_pub.publish(String(data=intent or "user requested abort"))
            self.get_logger().warn(f"skill=abort → published /mission_abort: {intent!r}")
            return

        cmd = self._compile_command(skill, intent, params, defaults)
        if cmd is None:
            return

        out = String(data=json.dumps(cmd))
        self.cmd_pub.publish(out)
        self.get_logger().info(f"skill={skill} → mission: {out.data}")

    def _llm_parse(self, text):
        try:
            parsed = self._client.invoke({"user_input": text})
        except Exception as e:
            self.get_logger().error(f"LangChain/Gemini call failed: {e}")
            return None
        if not isinstance(parsed, dict):
            self.get_logger().error("LLM output is not a JSON object")
            return None

        skill = parsed.get("skill")
        if skill not in SKILL_DEFAULTS:
            self.get_logger().error(f"unknown skill {skill!r}; expected one of {SKILL_NAMES}")
            return None

        intent = str(parsed.get("intent") or text)
        params = parsed.get("params") or {}
        if not isinstance(params, dict):
            params = {}
        return {"skill": skill, "intent": intent, "params": params}

    def _compile_command(self, skill, intent, params, defaults):
        # Landmark: prefer params.landmark, else skill's force_landmark, else None.
        raw_landmark = params.get("landmark")
        if isinstance(raw_landmark, str):
            raw_landmark = raw_landmark.strip().lower() or None
        else:
            raw_landmark = None
        if raw_landmark is not None and raw_landmark not in KNOWN_LANDMARKS:
            self.get_logger().warn(
                f"unknown landmark {raw_landmark!r}; falling back to local-area survey"
            )
            raw_landmark = None
        landmark = raw_landmark or defaults.get("force_landmark")

        if defaults.get("needs_landmark") and landmark is None:
            self.get_logger().error(
                f"skill={skill} requires a landmark; got none. Refusing to plan."
            )
            return None

        # Radius/altitude: user-provided overrides skill defaults; both then
        # clamped to the Tello-class envelope. Indoor caps apply at executor.
        try:
            radius = float(params.get("radius_m") or defaults["radius_m"])
            altitude = float(params.get("altitude_m") or defaults["altitude_m"])
        except (TypeError, ValueError):
            self.get_logger().error("non-numeric radius_m/altitude_m in params")
            return None

        clamped_radius = max(3.0, min(15.0, radius))
        clamped_altitude = max(4.0, min(12.0, altitude))
        if clamped_radius != radius:
            self.get_logger().warn(f"radius {radius} clamped to {clamped_radius}")
        if clamped_altitude != altitude:
            self.get_logger().warn(f"altitude {altitude} clamped to {clamped_altitude}")

        return {
            "action": "survey_local_area",
            "skill": skill,
            "radius_m": clamped_radius,
            "altitude_m": clamped_altitude,
            "pattern": defaults["pattern"],
            "landmark": landmark,
            "intent": intent,
        }


def main(args=None):
    rclpy.init(args=args)
    node = LLMAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
