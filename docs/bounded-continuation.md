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

## Continuation lifetime (validation pending)

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

## Combined eligibility and certified generation (validation pending)

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
only the components in isolation. Both the previous permit addition and this
composition still require their hosted workflow results to be reviewed.
