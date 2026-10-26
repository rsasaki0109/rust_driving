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
python3 scripts/check_stop_candidates.py retained-chassis artifacts/stop-candidates \
  target/release/examples/inspect_stop_candidates
```

The evidence input is artifact 12638847273 from run 40295079674. Use a new output
directory. No simulation rerun, policy change, shorter hold, or uncertainty-budget
reduction is involved. Results are pending; global path infeasibility and mission
completion remain unproven.
