# Checked stopping-candidate evidence

This read-only inspection distinguishes collisions on a braking approach from
collisions during its complete eight-second stationary tail. It also separates
collision-free proposals below the existing two-metre progress threshold from
larger proposals. None of those classifications authorizes motion: the original
fresh-acquisition confirmation gates and stopping commitments remain unchanged.

At the exact pipeline planner-call boundary, an optional inspection callback
receives immutable acquired inputs and the live planner. The probe clones that
planner and clears only the same temporary stopping commitments as the ordinary
checked-approach search, before applying its two-metre/confirmation admission.
Every tested stop still uses the ordinary reachable profile and complete sweep.
The original maneuver anchor, budgets, scene clocks and direction context remain.

The example exactly replays every original output, including the inspected tick.
Only the final healthy stationary tick is probed. The Python checker binds the
original four mission logs to the PR #54 canonical hashes, audits the probe's
selected profile, and independently certifies selected-path directional
clearance through its entire stationary tail. Rejected-stop contact times are
conservative production-sweep diagnostics, not an independent physical collision
proof. Repeated tests of the same endpoint count as separate sweeps.

```sh
bash scripts/check.sh
cargo build --release --locked -p rustdriving-pipeline --example inspect_stop_candidates
ANALYSIS_SOURCE="$(git rev-parse HEAD)" \
python3 scripts/check_stop_candidates.py retained-chassis artifacts/stop-candidates \
  target/release/examples/inspect_stop_candidates
```

The evidence input is artifact 12638847273 from run 40295079674. Use a new output
directory. No simulation rerun, policy change, shorter hold, or uncertainty-budget
reduction is involved.

## Result: clear stopping proposals miss the progress gate

The final healthy stationary tick at 65 s has a collision-free checked stopping
proposal in all four cases, but every clear proposal advances less than the
existing two-metre admission threshold.

| Plant / seed | Largest clear progress (m) | Moving conflicts | Tail conflicts | Clear sweeps below 2 m |
| --- | ---: | ---: | ---: | ---: |
| Reference / 7 | 1.612227 | 130 | 12 | 2 |
| Reference / 42 | 1.624916 | 130 | 12 | 2 |
| Native RNE / 7 | 1.514871 | 130 | 13 | 2 |
| Native RNE / 42 | 0.671060 | 133 | 13 | 2 |

Each probe selects the +3.9 m-side checked stop. Its travel takes about
1.62–1.84 s followed by the full eight-second stationary tail. The independent
directional-clearance checker accepts each selected proposal under the recorded
forecast semantics. Those semantics hold the final prediction sample beyond
eight seconds; this does not establish physical uncertainty coverage after the
forecast horizon. Rejected alternatives can have much longer arrival times
(for example, the native seed-42 center stop); they are not selected.

This establishes one concrete admission barrier at the inspected snapshots:
clear positive-progress proposals cannot enter confirmation because their gains
are below 2 m. It does not prove this is the only cause over the whole episode,
authorize a proposal, or establish that reducing the threshold yields completion.

The next experiment should compare a bounded smaller-progress admission policy
in closed loop while preserving the fresh-acquisition confirmation rules,
braking limits, complete hold sweep, original goals/deadlines and regression
cohort. It must measure repeated creep or oscillation as well as progress.
The current policy remains unchanged.

## Validation and retained evidence

[Hosted run 40314797813](https://github.com/rsasaki0109/rust_driving/actions/runs/40314797813)
passed `bash scripts/check.sh` on formatted source
`2380eb7fc25f4bb86924af4952e3292d342829d5`, including the planner
non-interference regression and four evidence-classification tests. All 5,204
original control outputs reproduce exactly, including the inspected callbacks.
All four selected-proposal audits and independent directional certificates pass.
The local execution environment was unavailable; these are hosted results.

The [complete retained report](../assets/stop-candidates/2380eb7fc25f4bb86924af4952e3292d342829d5.json)
contains every attempted stopping sweep, selected profile and certificate.
[Artifact 12647218276](https://github.com/rsasaki0109/rust_driving/actions/runs/40314797813/artifacts/12647218276)
contains the per-case probe output and report; ZIP SHA-256:
`63e647e395becddb3d9ff0f9a418e1b1ecf8be276ee570df6a6ebbaa94167a73`.

This diagnostic establishes neither calibrated uncertainty nor a completed
original mission. Comparative engineering maturity remains approximately 29%.
