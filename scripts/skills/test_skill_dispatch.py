#!/usr/bin/env python3
"""M1 acceptance gate: assert llm_agent_node emits the right skill per phrase.

Prereq: run `ros2 launch autobee_core demo.launch.py` in another terminal first.
Then in a second terminal:

    python3 scripts/skills/test_skill_dispatch.py

The script publishes a representative phrase per skill to /user_intent and
asserts the first /mission_command (or /mission_abort, for skill=abort) carries
the expected skill name. Exits non-zero on the first failure.
"""

import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# (skill_name, phrase_to_send). One per skill — keep the list short; full
# phrase coverage can wait until M5 acceptance harness.
TEST_CASES = [
    ("hover_around_me",          "hover around me"),
    ("scan_in_place",            "look around and tell me what you see"),
    ("sweep_area",               "do a thorough sweep of every corner"),
    ("quick_survey",             "quick check of the area"),
    ("count_people",             "how many people are at the convocation ground?"),
    ("detect_anomaly_buildings", "check the library for anything unusual"),
    ("return_home",              "come back to base"),
    ("abort",                    "stop now"),
]

INTENT_TOPIC = "/user_intent"
COMMAND_TOPIC = "/mission_command"
ABORT_TOPIC = "/mission_abort"
RESPONSE_TIMEOUT_S = 25.0  # Gemini round-trip can be slow


class SkillTester(Node):
    def __init__(self):
        super().__init__("skill_dispatch_tester")
        self.intent_pub = self.create_publisher(String, INTENT_TOPIC, 10)
        self.create_subscription(String, COMMAND_TOPIC, self._on_command, 10)
        self.create_subscription(String, ABORT_TOPIC, self._on_abort, 10)
        self._latest_command = None
        self._latest_abort = None
        self._waiting_for_abort = False

    def _on_command(self, msg: String):
        try:
            self._latest_command = json.loads(msg.data)
        except json.JSONDecodeError:
            self._latest_command = None

    def _on_abort(self, msg: String):
        if self._waiting_for_abort:
            self._latest_abort = msg.data

    def reset(self):
        self._latest_command = None
        self._latest_abort = None

    def send(self, phrase: str):
        self.intent_pub.publish(String(data=phrase))

    def wait_for_command(self, deadline: float):
        while time.time() < deadline and self._latest_command is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._latest_command

    def wait_for_abort(self, deadline: float):
        self._waiting_for_abort = True
        while time.time() < deadline and self._latest_abort is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self._waiting_for_abort = False
        return self._latest_abort


def main():
    rclpy.init()
    node = SkillTester()
    # Allow pub/sub discovery before the first send.
    deadline = time.time() + 2.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    passed = 0
    failed = 0
    print(f"\nrunning {len(TEST_CASES)} skill-dispatch cases\n")
    for expected_skill, phrase in TEST_CASES:
        node.reset()
        print(f"  [{expected_skill:26s}] phrase={phrase!r}")
        node.send(phrase)
        deadline = time.time() + RESPONSE_TIMEOUT_S

        if expected_skill == "abort":
            got = node.wait_for_abort(deadline)
            if got is None:
                print("    ✗ no /mission_abort within timeout")
                failed += 1
                continue
            print(f"    ✓ /mission_abort received: {got!r}")
            passed += 1
            continue

        cmd = node.wait_for_command(deadline)
        if cmd is None:
            print("    ✗ no /mission_command within timeout")
            failed += 1
            continue
        got_skill = cmd.get("skill")
        if got_skill != expected_skill:
            print(f"    ✗ skill mismatch: got {got_skill!r} expected {expected_skill!r}")
            print(f"       full command: {cmd}")
            failed += 1
            continue
        print(
            f"    ✓ skill={got_skill}  pattern={cmd.get('pattern')}  "
            f"r={cmd.get('radius_m')} alt={cmd.get('altitude_m')} "
            f"landmark={cmd.get('landmark')}"
        )
        passed += 1

        # Give the executor time to complete the previous mission (or abort it)
        # so the next test isn't fighting an in-flight schedule. M2 will add
        # a /mission_status sync; M1 just sleeps.
        time.sleep(2.0)

    print(f"\n{passed}/{len(TEST_CASES)} passed, {failed} failed")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
