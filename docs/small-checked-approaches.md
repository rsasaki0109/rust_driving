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
No full-cohort adoption or maturity increase is claimed. Results are pending.
