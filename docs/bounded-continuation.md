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
The following constraints extension is pending a new workflow run. No local executor is
available. No closed-loop improvement, adoption or maturity increase is claimed.
Next integrate the constructor with an acquisition-owned continuation permit and
the existing complete path certificates, then compare the original paired cases.

## Route and curvature constraints (validation pending)

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
