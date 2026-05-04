# Acceptance harness

Programmatic checks for the PRD §11 test scenarios (T-01..T-09) plus a
topic-graph smoke test. Run inside `autobee_dev`.

## Files

- `smoke_test.sh` — fast (~8s/topic) liveness + rate check on every node and
  topic in the contract. Run this first; if it fails, fix the launch file
  before running anything else.
- `run_acceptance.py` — issues each PRD §11 scenario as a real `/user_intent`
  command, captures the resulting topics, applies pass/fail criteria, prints
  a summary. Writes `acceptance_results.jsonl` (one line per scenario).

## Workflow

In one terminal:

```bash
ros2 launch autobee_core demo.launch.py
```

In another:

```bash
bash scripts/acceptance/smoke_test.sh
python3 scripts/acceptance/run_acceptance.py            # all non-SLAM scenarios
python3 scripts/acceptance/run_acceptance.py --only T-09   # demo-day only
python3 scripts/acceptance/run_acceptance.py --include-slam-tests
```

## Pass criteria summary

| ID    | Hard pass criteria |
|-------|--------------------|
| T-01  | LLM emits `landmark="convocation_ground"`; mission completes; `TRANSIT_OUT` phase observed; report fires |
| T-02  | LLM emits `landmark="library"`; mission completes; report fires |
| T-03  | LLM emits `landmark="main_gate"`; mission completes (RTH semantics) |
| T-04  | LLM emits any known-landmark; mission completes |
| T-05  | No crash on gibberish (absence-of-stack-trace) |
| T-06  | Mission ends with `aborted`/`failed` after simulated perception kill |
| T-07  | Anomaly alert fires *if* density crosses threshold (inconclusive otherwise) |
| T-08  | SKIPPED unless `--include-slam-tests`; only checks completion, not ATE |
| T-09  | LLM emits `landmark=null`; mission completes; report fires (**demo-day**) |

## What this harness does NOT do

- Measure SLAM ATE (needs `evo_ape` against ground truth bag — Phase 4)
- Measure mAP@50 (needs labeled ground-truth detections)
- Verify command latency at p95 (needs many samples)
- Spawn 20+ actors for T-07 (the world has 8 stand-ins; T-07 is verified
  separately by spawning more `model://person` actors in `campus.sdf`)

These are deliverables for Phase 4 (SLAM) and the formal evaluation chapter,
not the in-loop acceptance harness.
