# Directional clearance with declared motion error

The checked-approach planner can receive an acquired world-frame velocity and
its existing `forecast_envelope` budgets. It maximizes the original empirical
transverse-motion reserve over the resulting velocity disc, then keeps the
larger of that result and the original nominal-forecast reserve. A nearly zero
velocity with an unresolved direction therefore retains the full transverse
reserve even when its fitted direction changes.

This corrects directional clearance inside the existing
`replan_checked_approaches` configuration. It adds no scenario option or
recording field. Configurations outside that scope retain the original policy.
The declared budgets are supplied assumptions; the correction does not estimate
motion covariance or establish calibrated confidence.

## Model and ownership

For acquired velocity `v`, acquisition age `age = now - last_seen`, and
trajectory-relative time `t`, the velocity disc radius is

`e(t) = velocity_error_m_s + acceleration_error_m_s2 * (age + t)`.

The maximum angular term follows the two tangent directions from the origin to
this disc. When the disc contains velocities perpendicular to the path, that
term keeps its original 0.7 m maximum. The transverse-speed term is maximized
separately. Their maximum is the exact supremum of the original empirical
motion reserve over the disc.

The final margin also retains the original 0.30 m base, time reserve, explicit
refinement floor, occupied forecast radii and acceleration-to-chord reserve.
The disc radius and angular support grow monotonically over a forecast
interval, so checking their end-time support covers that entire interval.
This remains an empirical reserve; the original finite position-forecast
horizon and endpoint hold gain no physical guarantee from this calculation.

Only acquired world-frame tracks provide velocity and time ownership. Cached
acquisitions consume their existing age. Missing identities, duplicate
identities, future acquisitions and invalid active budgets cannot fall back to
an apparently clear forecast. They cause collision checks to fail closed.
The disabled context retains the original arithmetic and behavior.

## Verification

An independent Python oracle uses angular intervals and critical points rather
than the Rust tangent-circle formula. Its path checker integrates acceleration
and uses midpoint/Lipschitz bounds. It reconstructs world-track ownership,
native course heading and the existing margins from sensor-only recordings;
simulator actors do not enter either the planner or this oracle.

The comparison freezes all 48 preceding later-stop inputs before execution.
The unchanged baseline source and the new source each run every reference and
native case with their own sensor replay. The ordinary mission and an envelope
configuration outside checked approaches are additional byte-identity controls.

Acceptance requires independent path certificates, no newly failed original
physical/forecast gates, no lost baseline goal arrivals, and no increase in
emergency control steps or healthy emergency transitions on the original
reference seed-7 mission. The original all-case strict and mission-arrival gate
remains separate. Reports retain every failed case and distinguish bounded
regression qualification from full adoption.

Standard reference and native CI also invokes the later-stop suite with\n\`--verify-directional-reserves\`, retaining the independent path certificates\nfor subsequent driving changes. This is a validation-tool option, not a\nscenario or driver setting.\n\nThe source-only policy and unchanged budgets are recorded in
[`directional-reserve-v1.json`](../integrations/planning/directional-reserve-v1.json).
The paired qualification workflow at the recorded tested-source commit runs the pinned workspace check,
analytic controls and paired source comparison. Evidence is retained by tested
source commit, including incomplete or failed comparisons.

## Viewed comparison

The source-only paired comparison retains all 48 cases in each version. Ordinary
mission and non-checked envelope controls reproduce every original output byte.
The independent checker certifies 60,332 selected paths using 2,580,937 interval
bounds under the new directional margins.

| Metric across the viewed cohort | Baseline | Corrected |
|---|---:|---:|
| Strict passes | 36/48 | 36/48 |
| Goal arrivals | 20 | 20 |
| Healthy emergency-state transitions | 418 | 402 |
| Emergency control steps | 449 | 455 |
| Reference mission, seed 7: emergency steps | 7 | 6 |
| Reference mission, seed 42: emergency steps | 6 | 7 |

This is a mixed result, rather than uniform closed-loop stability improvement.
For example, native known-closure seed 42 retains its arrival but increases
emergency control steps from 14 to 27. Reference mission progress also decreases
by about 1 m in both seeds; neither completes the original mission.

The correction qualifies against the predeclared bounded regression conditions:
unchanged physical/forecast gates, unchanged arrivals, byte-identical controls
and valid stronger trajectory certificates. Existing research options remain
off by default. The eight original incomplete missions and measured uncertainty
calibration remain open; the engineering estimate stays about 29%, and the 30%
waypoint is unmet.

The [compact comparison](../assets/directional-reserve-v1/summary.json) retains
each case's gains and losses. The first complete report and paired workflow are
preserved at its recorded tested-source commit; the temporary workflow publisher
is excluded from the maintained tree. Standard CI retains the independent
checker on both plants.
