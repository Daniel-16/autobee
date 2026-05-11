#!/usr/bin/env python3
"""M4 acceptance gate: per-skill /mission_report content checks.

Prereq: `ros2 launch autobee_core demo.launch.py` running.

Per skill we:
  1. publish the phrase to /user_intent
  2. wait for the mission to complete (terminal /mission_status)
  3. wait for /mission_report
  4. assert content rules:
       - return_home: exact deterministic template
       - landmark skills: report contains the landmark in plain English
       - other LLM-driven skills: non-empty

Each survey-style mission ~30-60s; return_home ~3s. Total ~3 min.
"""

import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# (skill, phrase, expected_content)
# expected_content forms:
#   {"exact": "..."}      – string match
#   {"contains": "..."}   – substring (case-insensitive)
#   {"non_empty": True}   – just confirm a report came back
TEST_CASES = [
    ("return_home", "come back to base",
     {"exact": "Returned to base. Mission ended cleanly."}),
    ("count_people", "how many people are at the convocation ground?",
     {"contains": "convocation ground"}),
    ("hover_around_me", "hover around me",
     {"non_empty": True}),
]

INTENT_TOPIC = "/user_intent"
COMMAND_TOPIC = "/mission_command"
STATUS_TOPIC = "/mission_status"
REPORT_TOPIC = "/mission_report"

COMMAND_TIMEOUT_S = 25.0
COMPLETED_TIMEOUT_S = 90.0
REPORT_TIMEOUT_S = 30.0  # Gemini response budget


class ReportTester(Node):
    def __init__(self):
        super().__init__("skill_report_tester")
        self.intent_pub = self.create_publisher(String, INTENT_TOPIC, 10)
        self.create_subscription(String, COMMAND_TOPIC, self._on_command, 10)
        self.create_subscription(String, STATUS_TOPIC,  self._on_status,  10)
        self.create_subscription(String, REPORT_TOPIC,  self._on_report,  10)
        self.cmd = None
        self.terminal = None
        self.report = None

    def reset(self):
        self.cmd = None
        self.terminal = None
        self.report = None

    def _on_command(self, msg: String):
        try:
            self.cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _on_status(self, msg: String):
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        ev = status.get("event")
        if ev in ("completed", "aborted", "failed"):
            self.terminal = ev

    def _on_report(self, msg: String):
        self.report = msg.data

    def send(self, phrase: str):
        self.intent_pub.publish(String(data=phrase))

    def spin_until(self, predicate, deadline: float):
        while time.time() < deadline and not predicate():
            rclpy.spin_once(self, timeout_sec=0.2)


def _check_content(report: str, expected: dict):
    if "exact" in expected:
        return report == expected["exact"], f"exact match against {expected['exact']!r}"
    if "contains" in expected:
        return expected["contains"].lower() in report.lower(), f"substring {expected['contains']!r}"
    if expected.get("non_empty"):
        return bool(report and report.strip()), "non-empty"
    return False, "unknown rule"


def main():
    rclpy.init()
    node = ReportTester()
    deadline = time.time() + 2.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    failed = 0
    for skill, phrase, expected in TEST_CASES:
        print(f"\n[{skill}] phrase={phrase!r}")
        node.reset()
        node.send(phrase)

        d = time.time() + COMMAND_TIMEOUT_S
        node.spin_until(lambda: node.cmd is not None, d)
        if node.cmd is None or node.cmd.get("skill") != skill:
            print(f"  ✗ wrong/missing /mission_command: {node.cmd}")
            failed += 1
            continue
        print(f"  ✓ /mission_command  skill={skill}")

        d = time.time() + COMPLETED_TIMEOUT_S
        node.spin_until(lambda: node.terminal is not None, d)
        if node.terminal != "completed":
            print(f"  ✗ mission ended event={node.terminal!r}")
            failed += 1
            continue
        print(f"  ✓ mission completed")

        d = time.time() + REPORT_TIMEOUT_S
        node.spin_until(lambda: node.report is not None, d)
        if node.report is None:
            print(f"  ✗ no /mission_report within {REPORT_TIMEOUT_S}s")
            failed += 1
            continue
        ok, rule = _check_content(node.report, expected)
        if not ok:
            print(f"  ✗ report did not satisfy rule: {rule}")
            print(f"     got: {node.report!r}")
            failed += 1
            continue
        print(f"  ✓ report satisfies rule: {rule}")
        print(f"     report: {node.report!r}")

        time.sleep(2.0)

    total = len(TEST_CASES)
    print(f"\n{total - failed}/{total} passed, {failed} failed")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
