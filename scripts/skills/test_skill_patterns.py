#!/usr/bin/env python3
"""M2 acceptance gate: assert each skill produces the right schedule shape in sim.

Prereq: `ros2 launch autobee_core demo.launch.py` running in another terminal.

Per skill we:
  1. publish a phrase to /user_intent
  2. wait for /mission_command (validates LLM still works after refactor)
  3. wait for /mission_status started
  4. collect all phase transitions until /mission_status completed (or timeout)
  5. assert the expected phase appears in the trace

Each sim mission takes ~30-60s. Whole script runs ~5 min.
"""

import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# (skill, phrase, expected_phase_in_trace)
# Only the *new* M2 behaviors and one regression check on the existing path.
TEST_CASES = [
    ("scan_in_place",   "look around and tell me what you see",  "YAW_360"),
    ("return_home",     "come back to base",                     "DESCEND"),
    ("hover_around_me", "hover around me",                       "CLIMB"),  # regression
]

INTENT_TOPIC = "/user_intent"
COMMAND_TOPIC = "/mission_command"
STATUS_TOPIC = "/mission_status"
COMPLETED_TIMEOUT_S = 90.0


class PatternTester(Node):
    def __init__(self):
        super().__init__("skill_pattern_tester")
        self.intent_pub = self.create_publisher(String, INTENT_TOPIC, 10)
        self.create_subscription(String, COMMAND_TOPIC, self._on_command, 10)
        self.create_subscription(String, STATUS_TOPIC,  self._on_status,  10)
        self._latest_command = None
        self._phases_seen = set()
        self._terminal_event = None

    def _on_command(self, msg: String):
        try:
            self._latest_command = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _on_status(self, msg: String):
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        # Verbose tracing — print every status message as it arrives so a
        # silent failure (e.g., executor crashed mid-mission) is visible.
        print(f"     status: event={status.get('event')!r} phase={status.get('phase')!r}"
              f" reason={status.get('reason', '')!r}")
        phase = status.get("phase")
        if phase:
            self._phases_seen.add(phase)
        event = status.get("event")
        if event in ("completed", "aborted", "failed"):
            self._terminal_event = event

    def reset(self):
        self._latest_command = None
        self._phases_seen = set()
        self._terminal_event = None

    def send(self, phrase: str):
        self.intent_pub.publish(String(data=phrase))

    def spin_until(self, predicate, deadline: float):
        while time.time() < deadline and not predicate():
            rclpy.spin_once(self, timeout_sec=0.2)


def main():
    rclpy.init()
    node = PatternTester()
    # Discovery warm-up
    deadline = time.time() + 2.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    failed = 0
    for skill, phrase, expected_phase in TEST_CASES:
        print(f"\n[{skill}] phrase={phrase!r}  expect phase={expected_phase}")
        node.reset()
        node.send(phrase)

        d = time.time() + 25.0
        node.spin_until(lambda: node._latest_command is not None, d)
        cmd = node._latest_command
        if cmd is None:
            print("  ✗ no /mission_command")
            failed += 1
            continue
        if cmd.get("skill") != skill:
            print(f"  ✗ skill mismatch: got {cmd.get('skill')!r}")
            failed += 1
            continue
        print(f"  ✓ /mission_command  skill={cmd['skill']}  pattern={cmd.get('pattern')}")

        d = time.time() + COMPLETED_TIMEOUT_S
        node.spin_until(lambda: node._terminal_event is not None, d)
        if node._terminal_event is None:
            print(f"  ✗ no terminal /mission_status within {COMPLETED_TIMEOUT_S}s")
            print(f"     phases observed so far: {sorted(node._phases_seen)}")
            failed += 1
            continue
        if node._terminal_event != "completed":
            print(f"  ✗ mission ended with event={node._terminal_event!r}")
            failed += 1
            continue
        if expected_phase not in node._phases_seen:
            print(f"  ✗ expected phase {expected_phase!r} not in trace; saw {sorted(node._phases_seen)}")
            failed += 1
            continue
        print(f"  ✓ completed; phases included {expected_phase}")

        time.sleep(2.0)

    total = len(TEST_CASES)
    print(f"\n{total - failed}/{total} passed, {failed} failed")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
