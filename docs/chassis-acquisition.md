# Acquired chassis motion and endpoint sampling

Dynamic RNE reports a chassis/COM pose and instantaneous wheel/gyro rates after ten physics substeps. The original acquired-motion experiment used a rear-point predictor and held the final gyro value across the preceding sensor interval. During changing turns, this could make a stationary LiDAR object appear to move.

The correction records the known chassis reference and the measurement sampling convention independently of road fitting. It does not change physics, sensor noise, the tracker fitter, forecast error budgets, roads, goals or deadlines.

## Operational contract

`rear_axle_offset_m=1.5` uses the pinned native vehicle calibration. `instantaneous_odometry=true` integrates adjacent synchronous measured endpoints: mean nonnegative wheel speed and mean gyro rate determine midpoint body displacement and integrated yaw. Current course speed remains `hypot(current_wheel_speed, offset * current_gyro)`; recorded yaw remains body heading. The existing chassis planning/control course and steering conversion use that same declared reference.

The native adapter selects this sampling mode only for acquired-motion fitting without local route geometry, delayed measurement-history replay, map localization or relocalization. Default native, reference, kinematic and existing local/history modes retain their preceding behavior. This adds no scenario flag.

Both interval endpoints must be acquired at the adjacent control times. Missing, cached or discontinuous measurements cannot cover the interval: the existing motion-discontinuity latch brakes and prevents later scalar readings from silently bridging the gap. Configuration rejects this sampling mode with delayed-history or map/relocalization policies. Invalid numeric support leaves the estimator unchanged.

Absent sampling metadata selects the preceding held-rate calculation. Old chassis-reference logs therefore retain their original semantics, including the unsuccessful first attempt below. This is a low-speed no-slip approximation, not a measured tire-slip estimate.

## Same-input result

The [protocol](../integrations/planning/chassis-acquisition-v2.json) retains the original first attempt and its acceptance thresholds. Body-reference calibration alone reduced the seed-7 outlier by about 22%, below the predeclared requirement to halve it. That failed result is [retained](../assets/chassis-acquisition-v1/canaries/70afaff1b89e511c38fdaed625c56c95656f3851.json); the threshold was not relaxed.

The corrected estimator consumes the same original raw sensor bytes twice. The original pipeline must reproduce every recorded output exactly before the counterfactual is scored. Counterfactual commands are never applied. Each seed has the same 535 eligible eight-second origins; physical actor data is used only by the evaluator.

| Native mission | Original maximum | Body reference alone | Reference and endpoint sampling | Final p95 |
|---|---:|---:|---:|---:|
| seed 7 | 8.542 m | 6.683 m | 0.685 m | 0.316 m |
| seed 42 | 2.122 m | 1.753 m | 0.498 m | 0.274 m |

Values are the additional radius required beyond the nominal object disc at eight seconds. Maximum required radius falls about 92% and 77% respectively. These are retrospective errors on viewed simulated inputs, not calibrated confidence or an unseen holdout. Published error budgets are not reduced.

A fresh closed-loop seed-7 canary needs at most 0.573 m additional eight-second radius and retains all preceding gates, with zero recorded collisions or road violations. It still fails its original 65 s goal: progress is 44.600 m versus 45.543 m previously, with five emergency ticks in both. Forecast accuracy alone does not establish mission availability.

## Qualification

[Full source-keyed results](../assets/chassis-acquisition-v2/qualifications/92c72b1c473f0b6209f09c4ce1ff9508026b2d40.json) and the [compact comparison](../assets/chassis-acquisition-v2/summary.json) retain all 48 original inputs and all original failed gates. Every case keeps its preceding gates and every preceding arrival is retained; strict passes remain 36/48 and arrivals remain 20. All 24 reference runs retain exact run/sensor/summary bytes. All 48 historical sensor logs replay, and default dynamic, kinematic acquired-motion and existing local-mode controls retain complete run/replay bytes.

The [hosted full qualification](https://github.com/rsasaki0109/rust_driving/actions/runs/40295079674) passes with runtime source `92c72b1`. Its artifact `12638847273` retains raw paired inputs, recordings, replay receipts and controls. The original all-case strict adoption gate is still false.

Closed-loop costs vary by case. The native seed-42 enabled detour takes 57.5 s versus 56.2 s and has 27 emergency ticks versus 15, while retaining its original goal and physical gates. The summary includes every progress, timing and emergency-count comparison. This repair establishes improved viewed forecast accuracy, not uniformly smoother driving.

Analytic Rust controls use independently integrated constant and changing body motion with rotated left/right turns and a stationary observed object. They check false-motion removal, current-speed ownership, finite atomic failure, missing-endpoint braking and replay rejection after sampling/reference metadata is changed. A separate Python filter reconstructs the posterior from measured endpoints and compares full covariance without using the Rust predictor. Success publication waits for historical replay, byte controls and the final binary-identity check; injected failures at each of those stages must retain an unpassed report.

`bash scripts/check.sh` passes on the exact formatted source [92c72b1](https://github.com/rsasaki0109/rust_driving/commit/92c72b1c473f0b6209f09c4ce1ff9508026b2d40), on [GitHub Actions](https://github.com/rsasaki0109/rust_driving/actions/runs/40294008604/job/121475928949). Native release build, strict all-target Clippy and adapter calibration tests also pass. The managed local executor is unavailable; these are hosted results. The first full-comparison setup stopped before its 48 executions because its baseline archive requested an absent `.cargo` directory; the canary result and setup failure remain recorded.

The final result-publication repair also passes `bash scripts/check.sh` on source `73bf792` in [its hosted verification](https://github.com/rsasaki0109/rust_driving/actions/runs/40297566338). Five failure/success publication controls pass; the two premature-success controls reject the actual preceding checker. Runtime Rust sources are unchanged from the fully qualified source. [Verification receipts](../assets/chassis-acquisition-v2/verification.json) and archived workflow bytes retain both checks.

The [full protocol](../integrations/planning/chassis-acquisition-v2.json) preserves all 48 frozen inputs and original gates, requires all historical sensor logs to replay, and compares default dynamic, kinematic acquired-motion and existing local-mode run/replay bytes against the actual preceding native binary.

```sh
bash scripts/check.sh
bash scripts/setup-rne.sh
cargo +1.95.0 build --release --locked --manifest-path integrations/rne/Cargo.toml
cargo +1.90.0 build --release --locked -p rustdriving-pipeline --example inspect_chassis_inputs
python scripts/check_chassis_acquisition.py --stage full \
  --evidence retained-directional-evidence --output artifacts/chassis-qualification \
  --binary target/release/rustdriving \
  --native-binary integrations/rne/target/release/rustdriving-rne \
  --example-binary target/release/examples/inspect_chassis_inputs \
  --baseline-native path/to/native-binary-from-6fd7e8a
```

The evidence directory is the original directional-reserve artifact from run 40258418216; its report and compressed/raw recording hashes are checked before evaluation. The manual qualification workflow builds both native versions with the pinned engine and toolchains. Use a fresh output destination.

The original eight incomplete missions and independent uncertainty calibration remain open. The [grouped calibration result](forecast-calibration.md) still cannot provide a finite 99% threshold; this change adds no independent seed groups. The comparative engineering maturity estimate remains about 29%.
