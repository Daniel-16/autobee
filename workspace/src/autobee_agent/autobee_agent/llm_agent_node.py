"""LLM mission planner (LangChain + Gemini).

Subscribes:  /user_intent (std_msgs/String) -- natural-language command
Publishes:   /mission_command (std_msgs/String) -- validated JSON mission spec

Uses LangChain's ChatGoogleGenerativeAI to call Gemini with a structured
system prompt + JsonOutputParser. Output is validated, clamped to the
Tello-class operational envelope (3..15m radius, 4..12m altitude), and
re-published as JSON.

GOOGLE_API_KEY is required (loaded from /workspaces/AUTOBEE/.env). The node
refuses to publish mission commands without a working Gemini client -- there
is no mock fallback by design.
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


KNOWN_LANDMARKS = {
    "main_gate",
    "library",
    "convocation_ground",
    "admin",
}

SYSTEM_PROMPT = """You are AutoBee's mission planner. AutoBee is an autonomous surveillance drone (DJI Tello class — small, indoor/outdoor, low altitude, ~10 minute flight time). You convert a non-pilot user's natural-language request into ONE valid JSON mission command.

Supported action: "survey_local_area"
The drone surveys an area with a spiral or orbit pattern, runs camera perception during flight to detect crowds and anomalies, then returns to its takeoff pose. The survey is centered on either (a) a named campus landmark, or (b) the current takeoff pose for "this place" / unnamed-area requests. Operational envelope is constrained for safe Tello-class flight.

Known campus landmarks (use the exact lowercase identifier in the JSON):
  - "main_gate"          : campus entry; also the implicit "base" for return-to-home
  - "library"            : library building entrance area
  - "convocation_ground" : large open assembly area; high-crowd venue
  - "admin"              : administration block

Output JSON schema (return ONLY valid JSON, no prose, no markdown fences):
{{
  "action": "survey_local_area",
  "radius_m": <number, 3..15>,
  "altitude_m": <number, 4..12>,
  "pattern": "spiral" | "orbit",
  "landmark": "<one of: main_gate, library, convocation_ground, admin>" | null,
  "intent": "<short phrase describing what to look for>"
}}

Use "landmark": null for local-area / "this place" / "around me" / "right here" / no-named-place commands. Set "landmark" only when the user explicitly names one of the known campus locations (or asks to "return to base", which maps to "main_gate").

Defaults when the user is vague: radius_m=6, altitude_m=8, pattern="orbit", landmark=null.
NOTE: "orbit" is preferred for short missions because it accumulates less drift in open-loop flight. Use "spiral" only when the user explicitly asks for full-coverage sweep / mapping / "every corner" / "thorough" wording.

"quick check" / small areas        -> radius_m=4, altitude_m=6, pattern="orbit"
"thorough sweep" / "every corner"  -> radius_m=10, altitude_m=10, pattern="spiral"
"circle around" / "fly around it"  -> pattern="orbit"
overcrowding / crowd density       -> intent: "detect crowd density"
weird / suspicious / anomaly       -> intent: "detect anomalies"
"return to base" / "RTH" / "go home" -> landmark="main_gate", radius_m=3, altitude_m=4, pattern="orbit", intent: "return to base"

Examples:
User: "check if the field is overcrowded"
{{"action":"survey_local_area","radius_m":6,"altitude_m":8,"pattern":"orbit","landmark":null,"intent":"detect crowd density"}}

User: "scan the convocation ground for overcrowding"
{{"action":"survey_local_area","radius_m":8,"altitude_m":8,"pattern":"orbit","landmark":"convocation_ground","intent":"detect crowd density at convocation ground"}}

User: "check the library entrance for unauthorized gatherings"
{{"action":"survey_local_area","radius_m":6,"altitude_m":8,"pattern":"orbit","landmark":"library","intent":"detect unauthorized gatherings at library"}}

User: "return to base immediately"
{{"action":"survey_local_area","radius_m":3,"altitude_m":4,"pattern":"orbit","landmark":"main_gate","intent":"return to base"}}

User: "do a quick orbit and tell me if anything looks weird"
{{"action":"survey_local_area","radius_m":4,"altitude_m":6,"pattern":"orbit","landmark":null,"intent":"detect anomalies"}}

User: "thorough sweep of this area, every corner"
{{"action":"survey_local_area","radius_m":10,"altitude_m":10,"pattern":"spiral","landmark":null,"intent":"thorough surveillance"}}
"""


class LLMAgentNode(Node):
    def __init__(self):
        super().__init__("llm_agent_node")

        self.declare_parameter("model", "gemini-2.5-flash")
        self.model_name = str(self.get_parameter("model").value)

        self._client = self._init_llm()

        self.cmd_pub = self.create_publisher(String, "/mission_command", 10)
        self.create_subscription(String, "/user_intent", self._on_intent, 10)

        if self._client is None:
            self.get_logger().fatal(
                "Gemini client failed to initialise. Ensure GOOGLE_API_KEY is set in "
                "/workspaces/AUTOBEE/.env and `pip install langchain-google-genai python-dotenv` "
                "have been run. Node will refuse to publish mission commands."
            )
        else:
            self.get_logger().info(
                f"llm_agent_node up ({self.model_name}); listening on /user_intent, "
                f"publishing to /mission_command"
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
                f"langchain not installed ({e}) - running in MOCK mode. "
                "Install with: pip install langchain-google-genai"
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
            self.get_logger().error(
                "Gemini client not initialised; cannot plan mission. "
                "Set GOOGLE_API_KEY in /workspaces/AUTOBEE/.env and restart."
            )
            return
        self.get_logger().info(f"intent: {text!r}")

        command = self._llm_command(text)
        if command is None:
            self.get_logger().error("could not produce a valid mission command")
            return

        out = String()
        out.data = json.dumps(command)
        self.cmd_pub.publish(out)
        self.get_logger().info(f"mission: {out.data}")

    def _llm_command(self, text):
        try:
            parsed = self._client.invoke({"user_input": text})
        except Exception as e:
            self.get_logger().error(f"LangChain/Gemini call failed: {e}")
            return None
        return self._validate(parsed, text)

    def _validate(self, command, original_text):
        if not isinstance(command, dict):
            self.get_logger().error("LLM output is not a JSON object")
            return None
        if command.get("action") != "survey_local_area":
            self.get_logger().error(f"unsupported action: {command.get('action')!r}")
            return None
        try:
            radius = float(command["radius_m"])
            altitude = float(command["altitude_m"])
        except (KeyError, TypeError, ValueError):
            self.get_logger().error("missing or non-numeric radius_m/altitude_m")
            return None

        # Tello-class operational envelope
        clamped_radius = max(3.0, min(15.0, radius))
        clamped_altitude = max(4.0, min(12.0, altitude))
        if clamped_radius != radius:
            self.get_logger().warn(f"radius {radius} clamped to {clamped_radius}")
        if clamped_altitude != altitude:
            self.get_logger().warn(f"altitude {altitude} clamped to {clamped_altitude}")

        pattern = command.get("pattern", "spiral")
        if pattern not in ("spiral", "orbit"):
            self.get_logger().warn(f"unknown pattern {pattern!r}, defaulting to spiral")
            pattern = "spiral"

        landmark = command.get("landmark")
        if landmark is not None:
            landmark = str(landmark).strip().lower() or None
        if landmark is not None and landmark not in KNOWN_LANDMARKS:
            # Drop unknown landmark identifiers — better to fall back to local
            # survey than to send the executor to a fabricated coordinate.
            self.get_logger().warn(
                f"unknown landmark {landmark!r}; falling back to local-area survey"
            )
            landmark = None

        intent = str(command.get("intent") or original_text)

        return {
            "action": "survey_local_area",
            "radius_m": clamped_radius,
            "altitude_m": clamped_altitude,
            "pattern": pattern,
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
