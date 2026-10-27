# Bounded continuation profile primitive

The recorded restart investigation identified a current-speed cap in earlier
stopping fallback at nine of ten candidate restarts. This first implementation
adds an experimental profile constructor in `planning::approach_continuation`.
It is **not connected to the live planner** and does not resolve mission stalls.

The caller supplies a bounded path geometry, initial speed, maximum stopping
arc distance and positive speed/acceleration/braking bounds. Up to sixteen
descending candidate stop distances use the existing reachable speed envelope,
without capping desired cruise at current speed above 0.2 m/s. Every candidate
must terminate at the sampled distance, stop within the bound, retain the full
eight-second stationary tail and pass the caller's complete-path certificate.
A rejected candidate cannot be returned. Invalid bounds and physically
unreachable stopping distances reject before certification.

The supplied geometry and numerical distance bound are not authorization.
The unconstrained wrapper does not check sensor freshness, blocker identities,
road containment, lateral acceleration or route speed zones. The new constrained
entry point supplies the existing speed-zone and curvature limits to the same
reachable envelope; authorization and road containment remain caller duties. It must not be connected
to vehicle control until those existing constraints and a valid approach
authorization are provided by its caller. The public API is experimental.

Tests span initial speeds 0.19, 0.20, 0.21 and 0.31 m/s, independently reconstruct
constant-acceleration segment distances, check braking limits and the complete
hold, reject every candidate when the certificate refuses it, and use a moving
obstacle that conflicts during the stationary tail. Malformed and unreachable
bounds must never reach the certificate callback.

The initial four tests and hosted `bash scripts/check.sh` passed in
[run 40411729415](https://github.com/rsasaki0109/rust_driving/actions/runs/40411729415)
on formatted source `bfd34f62e1731f91864996adb59aef0b1bf8cece`.
The constraints extension passed hosted `bash scripts/check.sh` in
[run 40420127615](https://github.com/rsasaki0109/rust_driving/actions/runs/40420127615)
on formatted source `724012d6ecf030220f184e2c6b6531f450edec1d` (eight module tests). No local executor is
available. No closed-loop improvement, adoption or maturity increase is claimed.
Next integrate the constructor with an acquisition-owned continuation permit and
the existing complete path certificates, then compare the original paired cases.

## Route and curvature constraints (validated)

`stopped_profile_with_constraints` accepts optional existing route speed zones,
padding and a positive lateral acceleration bound. It preserves the supplied
initial speed when already above an active limit, retaining bounded braking
instead of instantly clamping the state. The existing interval-overlap logic
covers speed zones narrower than the geometry sample spacing. A route's cached
arc lengths must exactly match reconstruction before any projection is used.

Invalid route/zone/padding/curvature inputs reject before the certificate callback.
Generated samples must have finite nonnegative speeds and times, finite positions
and strictly increasing times before full-path certification. New tests cover
active-zone acceleration, initial overspeed recovery, a narrow between-knot zone,
a radius-two bend and invalid constraints. Neither entry point is wired into the
live planner yet; fresh continuation ownership and full scene/road certificates
remain prerequisites for that integration.

## Continuation lifetime (validated)

`ContinuationPermit` records the endpoint, route progress, lateral target,
blocker identities, localization/route generations and immutable expiry of an
already verified adoption. Construction requires an adopted three-acquisition
record spanning at least 0.2 seconds, matching acquisition and blocker context.
This is eligibility state, not independent proof of the supplied adoption.

Each observation must be healthy, monotonic and no more than 0.35 seconds old.
Cached acquisitions can preserve existing eligibility briefly but cannot extend
its deadline; neither can fresh observations. Caller-supplied duration must be
positive and at most eight seconds. This is an experimental maximum, not a
calibrated safety or completion bound.

Missing/future/regressed/stale observations, a changed frame/route/blocker/target,
retreat beyond 0.2 m of greatest observed progress, reaching the original endpoint
progress or expiry revoke eligibility permanently. The caller may also explicitly
revoke it when a path certificate fails. Restoring inputs never reactivates it;
a newly verified adoption is required. Endpoint bounds are never advanced.

Four new tests cover immutable expiry, changed contexts and clocks, stale
acquisitions, endpoint completion, unsupported adoption and invalid duration.
This state machine and the profile constructor still require live-planner
integration with complete current-scene and road certificates. No new driving
behavior or closed-loop improvement is claimed.

## Combined eligibility and certified generation (validated before continuous-range extension)

`ContinuationPermit::certified_profile` connects the permit and constrained
constructor. Eligibility is checked before candidate generation. A route is
mandatory (an empty zone slice represents no speed zones); its validated
projection must agree with the original endpoint progress and current start.
Candidate knots outside the authorized progress range do not reach the caller's
certificate. The caller still verifies continuous between-knot progress bounds,
road containment, target geometry and every collision interval through the tail.

A revoked permit cannot reach certification. If no candidate passes all checks,
including when inputs are malformed or the route is absent, the permit is
permanently revoked. The function returns no command and remains unconnected
to the live planner.

Three new tests exercise valid generation followed by stale observations,
complete certificate rejection followed by restored inputs, oversized proposed
distance and an unrelated candidate start. This tests composition rather than
only the components in isolation. The permit and combined constructor passed all fifteen module tests and hosted
`bash scripts/check.sh` in [run 40421778930](https://github.com/rsasaki0109/rust_driving/actions/runs/40421778930),
on formatted source `102383a9c61cf96f0baa41c1f26c14bbbaead415`.
No local checks ran.

## Continuous authorized-range check (validated)

The combined constructor now also checks progress containment between trajectory
knots before invoking the caller certificate. A folding route can have two
authorized endpoint projections while the segment interior projects onto an
unauthorized return leg; checking knots alone cannot exclude that shortcut.

Split the supplied route at the authorized progress boundaries into permitted
and forbidden subroutes. At each candidate segment midpoint, distance to either
fixed subroute is 1-Lipschitz. Midpoint distance plus/minus half the segment length
therefore bounds the whole interval. Accept an interval only when its upper
distance bound to permitted geometry is strictly below its lower bound to
forbidden geometry, with a numerical margin. Otherwise subdivide, at most ten
levels; unresolved comparisons reject. Stationary tails are checked too.
This is conservative: touching a progress boundary can reject an otherwise
usable path. It does not certify road width or obstacle clearance.

Two tests cover a U-shaped shortcut whose endpoint-only checks pass and an
offset straight segment with both accepted interior and rejected boundary tie.
The existing combined tests continue to check expiry and certificate failure.
The live planner remains unchanged pending integration and closed-loop comparison.

The continuous authorized-range extension passed seventeen module tests and
hosted `bash scripts/check.sh` in [run 40427966547](https://github.com/rsasaki0109/rust_driving/actions/runs/40427966547),
on formatted source `b67403afcc418b1dbea42309c8b969963723ccee`.
Local checks did not run.

## Planner-owned candidate checks (validated)

`LatticePlanner::checked_continuation_candidate` now supplies the constrained
generator with the planner's speed limits, vehicle radius and current scene
clock. The recorded observation must match that clock, every authorized blocker
must still be present, and malformed/duplicate prediction inputs reject.
A changed context revokes eligibility before a candidate can be returned.

The candidate certificate uses the existing directional collision sweep for
every prediction through the full stationary tail, with the conservative
one-metre refined-clearance floor. A separate bounded 1-Lipschitz distance check
certifies continuous road-corridor containment, including the vehicle radius.
Unresolved corner intervals reject. This closes the generic callback gap for
candidate generation; the API still trusts the caller's verified adoption and
frame generations.

Three tests cover a clear candidate versus a moving obstacle crossing during
the hold, mismatched scene clocks/missing blockers, and a shortcut across a road
corner. The method only returns a checked Yield candidate; it does not mutate
live maneuver/stop commitments, adopt that candidate or send commands. Pipeline
selection, diagnostic recording, replay audits and closed-loop comparison remain
before this becomes a driving-policy change.

The planner adapter passed twenty module tests and hosted `bash scripts/check.sh`
in [run 40429454279](https://github.com/rsasaki0109/rust_driving/actions/runs/40429454279)
on formatted source `aaba94b103b75039c4dcc12a5a2bdfc731f0af36`.

## Shared admission and exact start (validated)

The candidate entry now shares ordinary planning's configuration validation,
including checked-approach option dependencies and numerical bounds. Previously,
setting only `replan_checked_approaches` could generate a candidate despite the
ordinary planner rejecting that configuration. Candidate fixtures now enable the
required stop-retention/search and lateral-margin options.

The trajectory must also start at the supplied ego position within 1e-7 metres,
instead of allowing a 0.2 m gap that the complete-path sweep did not cover.
Two regression tests compare rejection of ten invalid configurations across both
entry points, verify sticky revocation after repair, and distinguish an exact
start from 0.1 m longitudinal/lateral shifts. The projection-progress tolerance
remains separate. These changes do not enable live candidate selection.

The admission follow-up passed twenty-two module tests and hosted
`bash scripts/check.sh` in
[run 40431380588](https://github.com/rsasaki0109/rust_driving/actions/runs/40431380588),
on formatted source `f6c3187be3df2b41bf630260f3476161a57a147d`.

## Retained-maneuver geometry (validated)

`checked_maneuver_continuation` generates its own geometry using the same
anchored lateral-transition, heading correction and local-route recovery helper
as ordinary planning. It requires an existing finite maneuver anchor, agreement
with the planner's retained target, and a target in the configured candidate set.
Missing/changed anchors, changed candidate widths, malformed routes or failed
scene/corridor certificates revoke the permit.

The horizon is limited by remaining authorized progress and the planning route's
goal margin; the complete supplied-route progress certificate remains authoritative
when fitted and supplied route arc lengths differ. This can conservatively reject
usable geometry. No authorization deadline or endpoint is extended.

Three tests cover repeated candidate generation around the 0.2 m/s threshold with
both ordinary and local geometry, invalid/changed anchors and a moving obstacle
during the full hold. Inputs in the multi-tick test are prescribed states, not a
closed-loop vehicle simulation. The method leaves maneuver and stop commitments
unchanged and still requires caller-owned adoption/frame verification.
Live selection, recorded diagnostics and paired closed-loop comparison remain
unimplemented; no mission recovery is claimed.

The retained-maneuver extension passed twenty-five module tests and hosted
`bash scripts/check.sh` in
[run 40433466438](https://github.com/rsasaki0109/rust_driving/actions/runs/40433466438)
on formatted source `d0e1c0206792b232419bffbebd8e97a03fe98fad`.

## Actual adoption witness (validated)

The planner now records a private witness only when its actual `plan` call
adopts and returns a newly confirmed stopping approach.
`take_continuation_permit` consumes this witness once and checks the original
control/acquisition clocks and exact supplied-route ego progress before creating
a permit from the planner-owned confirmation record. A failed request consumes
the witness too. Every new plan and route reset clears an unconsumed witness.
Taking a permit leaves the published adoption diagnostics unchanged.

Three tests use the actual emergency/hold/three-acquisition adoption sequence
rather than fabricated permit records. They cover single issuance, invalidation
by replan/reset/changed clock/bad progress/unhealthy request, and refusal to issue
from an adopted-looking diagnostic alone. The cloneable planner's witness is
local to each planner instance; cloning is for independent simulation branches,
not a global uniqueness mechanism.

The caller still supplies health and route/localization generations and must
request the permit in the adoption control tick. Low-level `from_adoption`
remains available for offline research with caller-verified records. Live
continuation selection and diagnostic/replay integration are not enabled.

The adoption-witness extension passed twenty-eight module tests and hosted
`bash scripts/check.sh` in
[run 40436521064](https://github.com/rsasaki0109/rust_driving/actions/runs/40436521064)
on formatted source `7642177b1f22dcd9909f1e0d41537b153c178e62`.

## Adoption-to-candidate integration (validated)

Centerline continuation no longer requires a prior lateral-shift anchor: the
ordinary planner can remain at target zero without ever setting one. Side
continuations still require their retained anchor; any present malformed anchor
rejects. The shared geometry helper already defines the unanchored center case.

A new test connects an actual three-acquisition adoption to one-time permit
issuance and next-tick continuation generation at 0.21 m/s. Independent dense
samples reconstruct constant-acceleration motion and the fixture's forecast
radius interpolation, checking braking/acceleration limits, road width, original
progress bounds and clearance through the complete eight-second hold. A later
conflicting scene must revoke that same permit permanently. A separate regression
covers an unanchored centerline explicitly.

This is a composed planner test with prescribed ego state, not closed-loop
vehicle control or evidence of mission recovery. Live selection, replay output
and paired mission qualification remain outstanding.

The adoption-to-candidate integration passed thirty module tests and hosted
`bash scripts/check.sh` in
[run 40439048247](https://github.com/rsasaki0109/rust_driving/actions/runs/40439048247)
on formatted source `973679a9cf51e7982fc41b00df0ec23b0b3f0d80`.

## Pipeline result inspection (validated)

`DrivingPipeline::step_with_planner_result_inspection` adds a read-only hook
immediately after the planner returns, while the effective speed limits, supplied
planning route and acquisition-owned predictions still match that call. A probe
can clone the planner and inspect/consume the clone's adoption witness without
changing the live instance. The existing pre-plan inspection API is preserved.

The callback sees the raw planner trajectory, before later regulatory mode
overrides and health/command guards. It is not the final executable output and
cannot replace it. Early-return localization transitions may omit planning and
therefore the callback; invalid clocks reject before invoking it.

Two integration tests compare every serialized output against an uninspected
pipeline across eighteen sensor steps including LiDAR failure, deliberately
mutate/replan a shadow clone, verify full sensor replay, and confirm an invalid
clock invokes no callback or state advance. This enables recorded-log shadow
comparison; it does not select continuation candidates in live control.

The pipeline result inspection and its output/replay invariance tests passed
hosted `bash scripts/check.sh` in
[run 40440374494](https://github.com/rsasaki0109/rust_driving/actions/runs/40440374494)
on formatted source `23bf7eb47b66b0f36ac6f44caeefabeb3078233d`.

## Frozen-log shadow comparison (execution pending)

The `inspect_continuations` pipeline example verifies every original sensor
output byte-for-byte before retaining any shadow evidence. A post-plan clone
consumes actual adoption witnesses and attempts bounded continuations with the
current scene clock and predictions. Permit observations use the recorded
localization epoch and a generation incremented whenever the supplied planning
route changes. Unhealthy/emergency or overridden output cannot authorize a probe.

The original planner still runs normally on every tick. This deliberately measures
candidate availability after the original policy's decision; it does not simulate
what subsequent sensing or commitments would have been after executing a candidate.
Every probe retains ego, route, predictions, original/candidate paths and permit
status. Candidate collision certificates are production checks, not a new
independent physical qualification.

`scripts/inspect_continuations.py` requires the frozen PR #58 report byte-for-byte,
verifies compressed and raw sensor hashes with the existing retained reader, and
runs both variants for reference/native seeds 7/42. It retains compressed JSONL
evidence with hashes plus compact counts. Missing/count-mismatched sensor footers
and any original-output mismatch fail the run. Partial outputs without a final
complete report are not evidence of a successful comparison.

The dedicated workflow runs required checks first, then downloads the retained
paired artifact and executes this inspection. No result or mission improvement
is claimed until that execution completes.

## Comparative probe audit (validation pending)

The Python report now independently recounts adoptions, grants, attempts and
candidate records from retained JSONL rather than trusting only the Rust footer.
It rejects missing/misordered records, invalid numeric/boolean state, a candidate
without a live healthy permit, renewed endpoint bounds, resumed probes without
a new adoption, and candidates at or beyond the immutable eight-second deadline.
It also checks initial speed, monotonic profile time and the full stopped tail.

Descriptive counters separate candidates on the adoption tick from later ticks,
and count peak speeds more than 0.1 m/s above the original plan. A separate count
restricts that comparison to later ticks with measured speed at most 0.32 m/s.
These counters do not establish acceleration command, physical progress or
closed-loop safety. Three unit tests cover accounting, rejection/revocation and
ten corrupted evidence cases. They join the required workspace check script.

## Explicit planner selection (validation pending)

`LatticePlanner::plan_with_continuation` is an explicit experimental entry
point. It regenerates and certifies the whole retained-maneuver candidate before
committing any planning state. Acceptance replaces the earlier immediate-stop
request: obstruction requested/active flags and the searched-stop anchor clear,
while the newly certified yield endpoint is recorded in planning-route coordinates.
The maneuver anchor, permit endpoint and deadline are preserved.

Acceptance returns `ContinuationSelection` with prior stop flags, acquired
clock, expiry, original/selected progress, target and blocker identities.
Ordinary lattice diagnostics and old adoption diagnostics are cleared so they
cannot falsely describe the selected continuation. The caller must retain this
new selection evidence. Rejection or expiry revokes the permit and calls the
ordinary planner, preserving its emergency/braking behavior.

Two planner tests check successful state replacement and unchanged ordinary
fallback for expiry/conflict. A pipeline-crate integration test runs the actual
adoption sequence, explicit selection and PurePursuit controller, requiring
positive bounded acceleration and retained emergency rejection. This is a
prescribed-state planner/controller test, not a closed-loop plant simulation.

The default DrivingPipeline still uses ordinary `plan`; no configuration flag
enables this entry yet. Pipeline selection diagnostics, route/health ownership,
regulatory guards and paired reference/native closed-loop qualification remain
required before enabling it in driving scenarios.
