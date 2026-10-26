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

Use artifact 12650721879 from run 40319988642 for the input directory.

## Recorded result

[Run 40326155954](https://github.com/rsasaki0109/rust_driving/actions/runs/40326155954)
passed the four boundary tests, all eight retained-recording inspections and
`bash scripts/check.sh` on source
`f0d117cd39e37a94d779376c7d40926f103ece9a`, completing at
2026-10-26 22:15 JST. These were hosted checks; local checks did not run.
This does not claim that every repository workflow passed.

[Full trace](../assets/approach-execution/f0d117cd39e37a94d779376c7d40926f103ece9a.json)
and [compact event summaries](../assets/approach-execution/summary.json).
The compact file transcribes the same inspector's structured workflow-log output,
excluding per-tick traces; it is not an independent second computation.
Artifact `12652850448` (`approach-execution`, 563,107 bytes) has ZIP SHA-256
`0c3c73cb4c8e97028f22ac2ca23c882d1ab3557da83075bbeda522af14fa0e00`.

All ten candidate authorizations after the initial approach encounter their first
subsequent nonpositive acceleration command within 0.10–0.30 seconds. Their
maximum estimated speed over each bounded event interval is at most
0.314920 m/s. This explains why the earlier 0.5 m/s departure counter did not
count those short advances; it does not establish physical speed accuracy.

For example, reference seed 42 authorizes its second approach at 14.30 s.
At 14.35 s the emitted path still has a 1.443486 m/s peak and the issued
acceleration is +2 m/s². At 14.45 s the estimated, initial-target and peak planned
speeds all equal 0.310051 m/s, first-segment feedforward and issued acceleration
are both zero, and the selected disposition remains `CheckedStop`.
There is no health fault or emergency at that snapshot. At 19.70 s the peak
profile speed falls below 0.2 m/s and the command is −2.5 m/s².
Thus the emitted plan itself loses the higher speed target at the observed
transition; this is not evidence of a higher target being ignored by control.

The native seed 42 baseline also shows a post-stop restart with a maximum
estimated speed of 0.315057 m/s. The behavior is therefore not unique to the
new 0.5 m progress setting. Endpoint changes occur one control tick after the
initial baseline authorizations too, so endpoint inequality alone does not
diagnose cancellation.

## Candidate mechanism and next acceptance test

In `crates/planning/src/stopping.rs::profile`, fallback stopping search caps cruise
at the initial speed when it exceeds 0.2 m/s. In `LatticePlanner::plan_inner`,
an active braking-obstruction commitment also caps cruise at ego speed. Either
can produce a profile with no positive initial acceleration; the present
summaries do not distinguish which branch caused each transition.

Before changing this policy, capture the actual stop-search branch, requested
versus effective cruise limit and active stopping commitment at the identified
ticks, preserving exact old emitted outputs. Any continuation experiment must
recheck the full changed trajectory and eight-second stationary tail on fresh
sensor evidence, preserve failure braking, and compare the original deadlines,
progress, emergency counts and independent audits on both plants.
Do not simply raise the 0.2 m/s boundary or bypass complete stopping checks.

PR #58 remains unadopted and default behavior remains unchanged. These traces
identify a planner-speed limitation to investigate, not a completed mission fix.
