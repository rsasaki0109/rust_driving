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
The helper itself does not check sensor freshness, blocker identities, road
containment, lateral acceleration or route speed zones. It must not be connected
to vehicle control until those existing constraints and a valid approach
authorization are provided by its caller. The public API is experimental.

Tests span initial speeds 0.19, 0.20, 0.21 and 0.31 m/s, independently reconstruct
constant-acceleration segment distances, check braking limits and the complete
hold, reject every candidate when the certificate refuses it, and use a moving
obstacle that conflicts during the stationary tail. Malformed and unreachable
bounds must never reach the certificate callback.

Validation is pending in the dedicated hosted workflow, which runs
`bash scripts/check.sh` including the new Rust tests. No local executor is
available. No closed-loop improvement, adoption or maturity increase is claimed.
Next integrate the constructor with an acquisition-owned continuation permit and
the existing complete path certificates, then compare the original paired cases.
