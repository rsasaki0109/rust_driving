# Forecast uncertainty calibration

The offline calibration audit measures how much of the published additional
forecast radius is required to contain recorded physical actor discs. Its score
is the largest required/published radius ratio across every eligible acquisition
and the fixed 0.2, 1, 2, 4 and 8 second horizons. A score above one is an observed
joint miss. The driver never reads physical actors or this analysis.

Forecast origins use each acquired track identity once. Actor association is
fixed at acquisition time, using the preceding 2 m distance and 0.5 m separation
rules; future positions cannot choose a more favorable actor. Unmatched or
ambiguous associations and absent future actors produce infinite scores.
Origins without a complete eight-second observation window are explicitly
right-censored. Short horizons use this same predeclared observation domain.

## Dependence and finite-sample rank

Thousands of overlapping forecasts do not provide thousands of independent
episodes. The frozen retrospective split uses seed 7 for calibration and seed
42 for evaluation. A seed group contains both plants, all worlds and all policy
variants. Each group's score is its worst case, rather than an average.
Previously viewed recordings are not a fresh holdout.

For target coverage p and n calibration groups, the split-conformal order
statistic has rank ceil((n + 1) p). When that rank exceeds n, its threshold is
infinite. At 99% coverage, at least 99 independent calibration groups are needed
even to obtain a finite rank. This calculation alone does not establish
exchangeability, representative coverage or physical safety. The current
single calibration seed cannot support a finite 99% threshold.

## Reproduction

The audit verifies both compressed and raw recording hashes against the first
canonical directional-reserve comparison, checks frozen input bytes and original
outcomes, and reproduces the original per-horizon coverage before producing
additional scores. It reads all 48 new-source cases; it does not rerun simulation.

```sh
python scripts/test_forecast_calibration.py
python scripts/check_forecast_calibration.py \
  --evidence retained-directional-evidence \
  --output artifacts/forecast-calibration/report.json
```

The evidence directory is the extracted `directional-reserve-comparison`
artifact from run 40258418216. The
[protocol](../integrations/planning/forecast-calibration-v1.json) pins its source,
artifact and split before this audit. The workflow checks that the active
driver sources still equal the recorded driver sources.

Per-horizon descriptive summaries retain required and published radius, unused
radius, the declared constant-velocity error radius and world-pose padding.
These are not all additive terms: the driver encloses alternative discs before
adding world-pose padding. No fitted scale changes operational forecasts.

The original mission outcomes, failed gates and 99% coverage requirements are
retained separately. Original incomplete missions and measured uncertainty
calibration remain open; the maturity estimate stays about 29%.
