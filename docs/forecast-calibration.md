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

The finite-sample rank follows Angelopoulos and Bates,
[A Gentle Introduction, sections 1.1 and 2.3](https://arxiv.org/abs/2107.07511).
The distinction between individual times and complete time-series observations
is discussed by Stankevičiūtė, Alaa and van der Schaar,
[Conformal Time-Series Forecasting, section 3 and figure 1](https://papers.neurips.cc/paper/2021/file/312f1ba2a72318edaaa995a67835fad5-Paper.pdf).
This audit uses a maximum over the fixed horizon set; it does not implement
that paper's RNN or its per-horizon Bonferroni construction.

## Viewed result

[The retained analysis](../assets/forecast-calibration-v1/canaries/47a949707d480f3d62cff8d4ad47a52fe1842866.json) verifies
all 48 case records, including 19,658 eligible acquisition origins and 2,240
right-censored origins across those records. Every original per-horizon
coverage result and outcome reproduces; strict passes remain 36/48 and arrivals
remain 20. These repeated case observations are not independent calibration
groups.

The largest case score is 0.8091, but the calibration set still has only one
seed group. Its 99% order-statistic rank is two, which exceeds its sample count;
the reported scale is therefore infinite. Evaluation coverage under a finite
threshold is not reported. All 48 observed case scores are at most one;
that retrospective observation does not qualify a reduced forecast envelope.

| Original mission | Seed | 8 s required radius p95 | 8 s required maximum | 8 s published median |
|---|---:|---:|---:|---:|
| reference-2d | 7 | 0.230 m | 0.759 m | 6.329 m |
| reference-2d | 42 | 0.166 m | 0.586 m | 6.332 m |
| rne-dynamic | 7 | 1.722 m | 8.542 m | 6.300 m |
| rne-dynamic | 42 | 1.271 m | 2.122 m | 6.313 m |

These are **additional** radii beyond the nominal object radius. For native
mission seed 7, the forecast at 5.0 s needs 8.542 m additional radius eight
seconds later, against 11.971 m published. Its p95 is much smaller; relying
only on typical errors would hide that outlier. None of these four original
missions reaches its goal. The [compact summary](../assets/forecast-calibration-v1/summary.json)
retains every case rather than selecting the favorable plant or seed.

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
adding world-pose padding. The reported median and p95 use nearest-rank descriptive percentiles. These
samples cover planar simulator actor discs at the five specified horizons;
they do not certify coverage between horizons, unseen physical shape or ego
occupancy. No fitted scale changes operational forecasts.

The original mission outcomes, failed gates and 99% coverage requirements are
retained separately. Original incomplete missions and measured uncertainty
calibration remain open; the maturity estimate stays about 29%.
