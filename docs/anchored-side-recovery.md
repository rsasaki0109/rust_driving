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

Validation is pending. The option must remain experimental until original
closed-loop mission and regression-cohort evidence establishes its effect.
No forecast budget or confidence claim changes.
