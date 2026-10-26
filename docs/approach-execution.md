# Checked-approach execution inspection

PR #58 increases confirmed proposal counts without reaching any of the four
original mission goals. This read-only diagnostic examines both halves of its
retained paired experiment, without changing thresholds or rerunning simulation.

The input report must match the committed canary report byte for byte. Each XZ
recording and decompressed log must match its retained SHA-256. Complete footers,
tick counts, independent planning checks and the existing independent approach
authorization oracle bind every event to the original evidence.

For each authorization, the report retains every control tick up to (excluding)
the next authorization, or through the final recorded tick. The latter interval
is explicitly censored. Fields include route progress, estimated speed, issued
acceleration and steering, health/emergency status, trajectory mode, selected
disposition, endpoint, initial target speed, first-segment feedforward and peak
planned speed. First endpoint changes, stationary profiles and nonpositive
acceleration after authorization are recorded with the full matching snapshot.

An endpoint change is expected during normal replanning and is not itself a
failure. First-segment feedforward is only one controller term. Estimated speed
is not ground truth, and these observations alone do not identify a causal
controller or planner defect. No threshold is tuned from this inspection.

```sh
python3 scripts/test_approach_execution.py
python3 scripts/inspect_approach_execution.py \
  --evidence retained-small-approaches \
  --output artifacts/approach-execution/report.json
bash scripts/check.sh
```

Use artifact 12650721879 from run 40319988642 for the input directory. Results
are pending. The dedicated hosted workflow runs the boundary tests, inspection,
then required workspace checks before publishing a repository report. Local
execution is unavailable in the current managed environment.
