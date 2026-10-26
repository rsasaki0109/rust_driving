# Smaller checked approaches: closed-loop canary

The [stopping inspection](stop-candidate-evidence.md) found clear proposals of
0.671–1.625 m, below the existing two-metre admission floor. This experiment
preselects a 0.5 m floor for the original reference/native missions with seeds
7 and 42. These are previously viewed cases, not an independent holdout.

`checked_approach_min_progress_m` is optional in scenario and pipeline metadata.
Absent preserves the original 2 m behavior and omits the new serialized field.
Explicit values must be finite, within 0.5–2 m, and require checked approaches.
A threshold change clears accumulated confirmation support; cached observations
cannot seed replacement credit. Three fresh acquisitions spanning at least
0.2 seconds, the existing stability limits, reachable braking, complete
eight-second stationary tail, forecast budgets and original goals remain.

The canary first regenerates all 48 frozen baseline inputs and checks their
original hashes, then executes only the four selected missions. Each runs once
without the option and once with 0.5 m, on the same new binary. Default run,
sensor and summary bytes must equal the original qualified recordings.
The candidate's operational config may differ only by the declared option.
Existing physical, forecast-coverage, confirmation, stopping, directional and
replay auditors remain active. The independent confirmation oracle reads the
declared floor and still rejects malformed values and unsupported credit.

Reports retain progress, arrivals, emergency steps, failed gates and exact
measured departure/settling times. Departures use a 0.5 m/s threshold and settling
uses 0.2 m/s hysteresis; the initial departure is included. Nonzero lateral-side
changes are counted separately. These observations do not by themselves prove
comfort or absence of all oscillation.

```sh
bash scripts/check.sh
# Build the native adapter with its pinned engine and Rust 1.95.0 first.
ANALYSIS_SOURCE="$(git rev-parse HEAD)" \
python3 scripts/check_small_approaches.py --evidence retained-chassis \
  --output artifacts/small-approaches --binary target/release/rustdriving \
  --native-binary integrations/rne/target/release/rustdriving-rne
```

Input evidence is artifact 12638847273 from run 40295079674. Use a new output
directory. A successful workflow means the four-case experiment and its
contracts completed; inspect new failed gates and outcomes separately.
No full-cohort adoption or maturity increase is claimed.

## Recorded outcome: keep experimental

[Hosted run 40319988642](https://github.com/rsasaki0109/rust_driving/actions/runs/40319988642)
completed on 2026-10-26 at 21:46 JST. It passed `bash scripts/check.sh`,
built the pinned native adapter and completed all eight paired runs on formatted
source `75d6c3fe11b5194ba522708f23a3a5c3eb43de46`.
The managed local executor was unavailable; these are hosted checks, not local
results or a claim that all repository workflows passed.

[Retained report](../assets/small-approaches/75d6c3fe11b5194ba522708f23a3a5c3eb43de46.json).
Raw recordings: artifact `12650721879` (`small-approach-canary`, 51,341,513 bytes),
ZIP SHA-256 `9a1b9fac4ad82b7fb0fd730ac0ae876f547b28ab1587f00ede03918dddd46c68`.

| Plant / seed | Progress, 2 m → 0.5 m (m) | Gain (m) | Emergency ticks | Confirmed adoptions |
| --- | ---: | ---: | ---: | ---: |
| reference / 7 | 44.506 → 45.992 | +1.486 | 6 → 7 | 1 → 4 |
| reference / 42 | 44.372 → 45.910 | +1.538 | 7 → 7 | 1 → 2 |
| native / 7 | 44.600 → 45.858 | +1.258 | 5 → 5 | 1 → 4 |
| native / 42 | 45.570 → 45.751 | +0.181 | 5 → 5 | 2 → 4 |

Every default control reproduced the original run/sensor/summary bytes exactly.
All eight runs replayed 1,301 ticks and retained zero collisions and road
violations. The candidate introduced no new failed gates, and all four pairs
retained full measured forecast coverage at the scored horizons. This establishes
the recorded simulation checks, not calibrated physical safety.

Arrival remains **0/4 → 0/4** at the unchanged 65-second deadline. Every candidate
ends at zero speed in `StationaryForecastBlocked`. Reference seed 7 also increases
emergency entries from 3 to 4; its extra emergency tick must not be hidden behind
the unchanged gate result. Minimum physical clearance decreases in every case,
although the recorded checks still pass.

Each candidate retains exactly the same two measured departures and two settling
timestamps as its own control, and zero nonzero lateral-side changes. The speed
hysteresis does not count motion below 0.5 m/s as a departure, so these counts do
not establish absence of creeping. More confirmed adoptions do not establish
successful vehicle execution or obstacle passage.

**Decision:** retain the opt-in experiment in draft; do not change the default
2 m policy or claim the original stall is resolved. A full 48-case qualification
has not run, and a successful contract is not policy adoption. The next useful
investigation is to trace post-confirmation trajectory replacement and commanded
versus measured speed on these retained candidate recordings. It should explain
why additional certified proposals yield only limited motion before changing
another threshold. The present evidence does not yet attribute that limitation
to the controller, replanning or scene constraints.
