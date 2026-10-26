# Experimental anchored side recovery

Original mission stalls occur at recorded nominal-path conflicts about three
seconds ahead (PR #55). The nonlocal planner currently reapplies the full
lateral correction interval at each tick even after the original maneuver's
completion arc. This can defer correction when actual tracking lags.

The optional `PipelineConfig.anchored_side_recovery` extends the existing
anchored correction to the supplied route. It preserves the original completion
arc and uses a wheelbase/lateral-acceleration floor for the residual correction.
The experimental branch does not cap that floor to the old transition length.
Existing corridor, curvature-speed, full collision sweeps, stationary tails and
passing-side rejection remain in force. The option defaults to false and is
omitted from old sensor-header serialization. Scenarios do not enable it.

The paired example first reproduces every original output exactly, then feeds
the same acquired sensor frames into a separate pipeline with only the new flag
enabled. Commands from that pipeline never move the simulator. Any extra clear
proposals therefore establish only a fixed-input planning result, not mission
completion or closed-loop safety. Historical estimator observations are retained.

```sh
bash scripts/check.sh
cargo build --release --locked -p rustdriving-pipeline --example inspect_side_recovery
python3 scripts/check_side_recovery.py retained-chassis artifacts/side-recovery \
  target/release/examples/inspect_side_recovery
```

`retained-chassis` is artifact 12638847273 from run 40295079674. The exact
canonical report and compressed/raw sensor hashes bind the four original
missions. The report compares healthy stationary complete-path counts, verifies
both planner traces and preserves the full paired output. Startup snapshots are
included and must not be interpreted as sustained-stall recovery.

## Recorded result: no increase in complete paths

On analysis source `2f2396f6b593f6fd5ab38a9047ea520b0c959b8e`,
[hosted validation](https://github.com/rsasaki0109/rust_driving/actions/runs/40310159376)
passed `bash scripts/check.sh`, including the two new Rust controls, and exactly
replayed all 5,204 original control outputs before comparing the new option.

| Plant / seed | Healthy stationary complete selections, old → new | Final selection |
| --- | ---: | --- |
| Reference / 7 | 3 → 3 | CheckedStop |
| Reference / 42 | 3 → 3 | CheckedStop |
| Native RNE / 7 | 3 → 3 | CheckedStop |
| Native RNE / 42 | 3 → 3 | CheckedStop |

Counts include startup and are not recovery successes. At the final tick, the
center and +3.9 m candidates remain CheckedStop and the opposite side remains
excluded. The +3.9 m first-conflict time changes by less than 0.001 s in each
case. This fixed-input experiment provides no evidence that anchored correction
alone solves the original stalls; no closed-loop improvement is established.

The [retained summary](../assets/side-recovery/2f2396f6b593f6fd5ab38a9047ea520b0c959b8e.json)
records every case. Full paired outputs are in
[artifact 12644948448](https://github.com/rsasaki0109/rust_driving/actions/runs/40310159376/artifacts/12644948448),
ZIP SHA-256 `4a0b921df38013680220212463f9e85ee60a03e4ece7fc7e0d82150d9d8669b6`.
The local executor was unavailable; these are hosted results.

Keep this proposal unadopted. The next investigation should distinguish
constraints imposed by the full nominal path and its stopping tail from actual
committed-prefix requirements, retaining independent clearance and braking
checks. Neither this result nor the earlier radius diagnostic justifies reducing
forecast budgets. Comparative engineering maturity remains approximately 29%.
