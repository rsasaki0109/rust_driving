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

## Stop-request transition follow-up (validated)

The inspector now retains the existing post-planning stop-request and active-hold
flags, plus the selected candidate's searched-stop flag, on every snapshot.
Each event also exposes the adjacent before/after snapshots of its first new
stop request. An already-present request, or one first seen at the next
authorization, is not counted as a new request in the preceding event.

These are recorded **exit-state** flags. They are not entry-state instrumentation
or a direct log of the requested/effective cruise limits. A searched profile
alone cannot distinguish later search from earlier fallback. The paired snapshots
allow the source transition rules to be checked without treating a zero-speed
command alone as proof of the selected branch.

Removing only these added fields must reproduce the entire original archived
inspection report exactly before the new report can be saved. New tests cover
request-versus-active-hold distinction, snapshot preservation, existing requests
and the next-authorization boundary. The original driving code is unchanged.

### Result and branch attribution

[Follow-up run 40333044701](https://github.com/rsasaki0109/rust_driving/actions/runs/40333044701)
completed at 2026-10-26 23:14 JST on source
`ed3a3afbdff3c550f450cc4e9ed9dfaa0d38113a`. All six boundary tests,
eight recorded-log inspections, the full original-report projection comparison,
and hosted `bash scripts/check.sh` passed. No local checks ran.

[Full state trace](../assets/approach-execution/ed3a3afbdff3c550f450cc4e9ed9dfaa0d38113a.json)
and [state-transition summary](../assets/approach-execution/state-summary.json).
The summary is the inspector's structured log output, not an independent rerun.
Artifact 12656154169 is 584,596 bytes; ZIP SHA-256:
`f9e8caa38c4c449d4d402f0ea4041e9a85059412f2cb9d5ffd231c893988d977`.

Nine of the ten candidate restarts have a new stop request at exactly their
first nonpositive acceleration tick. In all nine, the preceding tick has no
request and no active hold, speed crosses from at most 0.2 to above 0.2 m/s,
the selected profile is a searched checked stop, and the resulting peak speed
equals current estimated speed. Neither health faults, emergency mode nor a new
approach adoption occurs at the transition. Active hold is still false.

This distinguishes the trigger using the pinned planner's state-update rules:
for a non-emergency checked stop, a false-to-true request requires
`Selection.searched`. The later-stop path explicitly sets that value false;
the earlier fallback path sets it to `used && ego.speed > 0.2`. Before this
transition there is no retained braking request. Thus these nine newly armed
requests identify the earlier-stop fallback, whose cruise cap becomes current
speed above 0.2 m/s. On subsequent calls the retained braking request can maintain
that cap. This is source-based attribution for the recorded fixed-route cases,
not a new direct branch telemetry field or proof that removing the cap is safe.

The exception is reference seed 7's 24.20 s restart: at 24.35 s speed is only
0.040893 m/s, acceleration is −0.001996 m/s² and both request and active-hold
flags remain false. Its first new request is much later, at an emergency tick
at 45.20 s. It must not be counted as the same threshold-crossing mechanism.

The next bounded intervention should distinguish a previously authorized,
fully checked approach from an ordinary collision-braking fallback. Any
continuation must still pass the complete current-scene trajectory and
stationary-tail checks, and must retain immediate braking when those checks
fail. Test the state transition around 0.2 m/s, cached observations, changed
blockers and rejected continuation before a closed-loop paired comparison.
Keep the default policy unchanged until the original cohort qualifies.
