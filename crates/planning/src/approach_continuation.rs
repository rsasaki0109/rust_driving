//! Experimental stopping-profile construction; not connected to the driving policy.
//! A caller must separately own fresh approach authorization and supply the full
//! current-scene collision/road certificate. A returned profile grants no authority.
use crate::speed;
use rustdriving_core::{Route, RouteSpeedZone, TrajectoryPoint, Vec2};

/// Explicit bounds for one already-authorized approach corridor.
pub struct ContinuationBounds {
    pub maximum_distance_m: f64,
    pub cruise_m_s: f64,
    pub acceleration_m_s2: f64,
    pub deceleration_m_s2: f64,
}

/// Existing route and curvature restrictions for a proposed continuation.
/// These inputs do not establish corridor containment or fresh authorization.
#[derive(Default)]
pub struct ProfileConstraints<'a> {
    pub route_speed_zones: Option<(&'a Route, &'a [RouteSpeedZone], f64)>,
    pub lateral_acceleration_m_s2: Option<f64>,
}

/// Search at most sixteen stops without the ordinary fallback's current-speed
/// cruise cap. Every offered profile includes its reachable braking and complete
/// eight-second stationary tail. The callback must certify the entire profile.
/// This unconstrained wrapper does not qualify geometry, route speed zones,
/// sensor freshness or authorization; it is not invoked by the live planner.
/// Use `stopped_profile_with_constraints` when route/curvature limits apply.
pub fn stopped_profile(
    geometry: &[Vec2],
    initial_speed: f64,
    bounds: &ContinuationBounds,
    certify: impl FnMut(&[TrajectoryPoint]) -> bool,
) -> Option<Vec<TrajectoryPoint>> {
    stopped_profile_with_constraints(
        geometry,
        initial_speed,
        bounds,
        &ProfileConstraints::default(),
        certify,
    )
}

/// Generate a fully stopped candidate while preserving the supplied existing
/// speed-zone and curvature limits. Overspeed recovery retains the measured
/// initial speed and uses bounded braking rather than an instantaneous clamp.
pub fn stopped_profile_with_constraints(
    geometry: &[Vec2],
    initial_speed: f64,
    bounds: &ContinuationBounds,
    constraints: &ProfileConstraints<'_>,
    mut certify: impl FnMut(&[TrajectoryPoint]) -> bool,
) -> Option<Vec<TrajectoryPoint>> {
    if constraints
        .lateral_acceleration_m_s2
        .is_some_and(|v| !v.is_finite() || v <= 0.)
        || constraints
            .route_speed_zones
            .is_some_and(|(route, zones, padding)| {
                !padding.is_finite()
                    || padding < 0.
                    || route.points.len() < 2
                    || !route.half_width.is_finite()
                    || route.half_width <= 0.
                    || route.points.iter().any(|p| !p.finite())
                    || route.points.windows(2).any(|p| {
                        let distance = p[0].distance(p[1]);
                        !distance.is_finite() || distance <= 0.
                    })
                    || !Route::new(route.points.clone(), route.half_width).is_ok_and(|rebuilt| {
                        rebuilt.lengths == route.lengths && rebuilt.length().is_finite()
                    })
                    || zones.len() > 128
                    || zones.iter().any(|z| !z.valid())
            })
    {
        return None;
    }
    if !(2..=81).contains(&geometry.len())
        || geometry.iter().any(|p| !p.finite())
        || !initial_speed.is_finite()
        || initial_speed < 0.
        || [
            bounds.maximum_distance_m,
            bounds.cruise_m_s,
            bounds.acceleration_m_s2,
            bounds.deceleration_m_s2,
        ]
        .iter()
        .any(|v| !v.is_finite() || *v <= 0.)
    {
        return None;
    }
    let available = geometry
        .windows(2)
        .map(|p| p[0].distance(p[1]))
        .sum::<f64>();
    if !available.is_finite() || bounds.maximum_distance_m > available {
        return None;
    }
    let minimum = if initial_speed == 0. {
        0.
    } else {
        (initial_speed.powi(2) / (2. * bounds.deceleration_m_s2) * (1. + 1e-12)).max(1e-7)
    };
    if !minimum.is_finite() || minimum > bounds.maximum_distance_m {
        return None;
    }
    let limits = speed::Limits {
        zones: constraints.route_speed_zones,
        acceleration: bounds.acceleration_m_s2,
        deceleration: bounds.deceleration_m_s2,
        lateral: constraints.lateral_acceleration_m_s2,
        cruise: bounds.cruise_m_s,
    };
    for i in 0..if minimum == bounds.maximum_distance_m {
        1
    } else {
        16
    } {
        let distance =
            bounds.maximum_distance_m + (minimum - bounds.maximum_distance_m) * (i as f64 / 15.);
        let Some(path) = speed::profile(geometry, initial_speed, &limits, Some(distance)) else {
            continue;
        };
        if path.iter().any(|p| {
            !p.position.finite()
                || !p.speed.is_finite()
                || p.speed < 0.
                || !p.time.is_finite()
                || p.time < 0.
        }) || path.windows(2).any(|p| p[1].time <= p[0].time)
        {
            continue;
        }
        let Some(tail) = path.windows(2).next_back() else {
            continue;
        };
        let travelled = path
            .windows(2)
            .map(|p| p[0].position.distance(p[1].position))
            .sum::<f64>();
        if tail[0].position != tail[1].position
            || tail[0].speed != 0.
            || tail[1].speed != 0.
            || tail[1].time - tail[0].time < 8. - 1e-9
            || travelled > bounds.maximum_distance_m + 1e-8
            || (travelled - distance).abs() > 1e-7 * distance.max(1.)
        {
            continue;
        }
        if certify(&path) {
            return Some(path);
        }
    }
    None
}

/// Inputs for validating the lifetime of an already verified approach adoption.
/// Route and localization generations must change whenever their frames change.
pub struct ContinuationObservation<'a> {
    pub now_s: f64,
    pub acquisition_s: Option<f64>,
    pub localization_generation: u64,
    pub route_generation: u64,
    pub blocking_ids: &'a [u64],
    pub lateral_target_m: f64,
    pub progress_m: f64,
    pub healthy: bool,
}

// Written only when plan() actually returns a newly confirmed stopping approach.
#[derive(Clone)]
pub(super) struct AdoptionWitness {
    pub now: f64,
    pub acquisition: Option<f64>,
    pub progress: f64,
    pub record: rustdriving_core::CheckedApproachDiagnostics,
}

/// Bounded lifetime of an existing checked adoption, not a collision certificate.
/// Construction requires the caller's independently verified adoption record.
/// Every proposed path still needs current-scene, road and full-stop checks.
pub struct ContinuationPermit {
    expires_s: f64,
    last_control_s: f64,
    last_acquisition_s: f64,
    localization_generation: u64,
    route_generation: u64,
    blocking_ids: Vec<u64>,
    lateral_target_m: f64,
    greatest_progress_m: f64,
    end_progress_m: f64,
    endpoint: Vec2,
    valid: bool,
}

impl ContinuationPermit {
    /// The duration is an explicit experimental policy bound in (0, 8] seconds.
    /// No observation, including a fresh one, extends the original deadline.
    pub fn from_adoption(
        adoption: &rustdriving_core::CheckedApproachDiagnostics,
        observation: &ContinuationObservation<'_>,
        maximum_duration_s: f64,
    ) -> Option<Self> {
        let first = adoption.first_acquisition_s?;
        let last = adoption.last_acquisition_s?;
        let end = adoption.proposal_end_progress_m?;
        let endpoint = adoption.proposal_endpoint?;
        let target = adoption.proposal_lateral_target_m?;
        if !adoption.adopted
            || adoption.supporting_acquisitions != 3
            || !first.is_finite()
            || first < 0.
            || !last.is_finite()
            || last - first < 0.2 - 1e-9
            || observation.acquisition_s != Some(last)
            || !endpoint.finite()
            || !end.is_finite()
            || !target.is_finite()
            || !observation.progress_m.is_finite()
            || end - observation.progress_m < 0.5
            || !maximum_duration_s.is_finite()
            || maximum_duration_s <= 0.
            || maximum_duration_s > 8.
            || adoption.blocking_prediction_ids.is_empty()
            || adoption.blocking_prediction_ids.len() > 9
            || adoption
                .blocking_prediction_ids
                .windows(2)
                .any(|p| p[0] >= p[1])
        {
            return None;
        }
        let expires = observation.now_s + maximum_duration_s;
        if !expires.is_finite() || expires <= observation.now_s {
            return None;
        }
        let mut permit = Self {
            expires_s: expires,
            last_control_s: observation.now_s,
            last_acquisition_s: last,
            localization_generation: observation.localization_generation,
            route_generation: observation.route_generation,
            blocking_ids: adoption.blocking_prediction_ids.clone(),
            lateral_target_m: target,
            greatest_progress_m: observation.progress_m,
            end_progress_m: end,
            endpoint,
            valid: true,
        };
        permit.observe(observation).then_some(permit)
    }

    /// Revocation is sticky. Restored inputs cannot resurrect a revoked permit.
    pub fn revoke(&mut self) {
        self.valid = false;
    }

    /// Check eligibility only; callers must separately certify each complete path.
    pub fn observe(&mut self, observation: &ContinuationObservation<'_>) -> bool {
        let acquisition_valid = observation.acquisition_s.is_some_and(|stamp| {
            stamp.is_finite()
                && stamp >= self.last_acquisition_s
                && stamp <= observation.now_s
                && observation.now_s - stamp <= 0.35 + 1e-9
        });
        if !self.valid
            || !observation.healthy
            || !observation.now_s.is_finite()
            || observation.now_s < self.last_control_s
            || observation.now_s >= self.expires_s
            || !acquisition_valid
            || observation.localization_generation != self.localization_generation
            || observation.route_generation != self.route_generation
            || observation.blocking_ids != self.blocking_ids.as_slice()
            || !observation.lateral_target_m.is_finite()
            || (observation.lateral_target_m - self.lateral_target_m).abs() > 0.01
            || !observation.progress_m.is_finite()
            || observation.progress_m < self.greatest_progress_m - 0.2
            || observation.progress_m >= self.end_progress_m
        {
            self.revoke();
            return false;
        }
        self.last_control_s = observation.now_s;
        self.last_acquisition_s = observation.acquisition_s.expect("validated acquisition");
        self.greatest_progress_m = self.greatest_progress_m.max(observation.progress_m);
        true
    }

    /// Combine eligibility with profile generation and a mandatory full-path
    /// certificate. A route (with an empty zone slice if unrestricted) is required
    /// so the entire piecewise-linear candidate stays within the authorized progress range.
    /// The caller still certifies road containment, target geometry and all
    /// current-scene collision intervals, including the stationary tail, and
    /// the road corridor (separate from the progress-bound certificate).
    pub fn certified_profile(
        &mut self,
        observation: &ContinuationObservation<'_>,
        geometry: &[Vec2],
        initial_speed: f64,
        bounds: &ContinuationBounds,
        constraints: &ProfileConstraints<'_>,
        mut certify: impl FnMut(&[TrajectoryPoint]) -> bool,
    ) -> Option<Vec<TrajectoryPoint>> {
        if !self.observe(observation) {
            return None;
        }
        let Some((route, _, _)) = constraints.route_speed_zones else {
            self.revoke();
            return None;
        };
        // The constructor validates this route before invoking the callback.
        let end_progress = self.end_progress_m;
        let endpoint = self.endpoint;
        let result = stopped_profile_with_constraints(
            geometry,
            initial_speed,
            bounds,
            constraints,
            |path| {
                let recorded_end = route.project(endpoint).0;
                if !recorded_end.is_finite() || (recorded_end - end_progress).abs() > 1e-7 {
                    return false;
                }
                let start = route.project(path[0].position).0;
                if !start.is_finite() || (start - observation.progress_m).abs() > 0.2 {
                    return false;
                }
                if path.iter().any(|p| {
                    let progress = route.project(p.position).0;
                    !progress.is_finite()
                        || progress < observation.progress_m - 0.2
                        || progress > end_progress + 1e-8
                }) {
                    return false;
                }
                continuous_progress_contains(
                    route,
                    path,
                    observation.progress_m - 0.2,
                    end_progress,
                ) && certify(path)
            },
        );
        if result.is_none() {
            self.revoke();
        }
        result
    }

    /// Original endpoint and route progress, never advanced by renewed sensing.
    pub fn endpoint_bound(&self) -> Option<(Vec2, f64)> {
        self.valid.then_some((self.endpoint, self.end_progress_m))
    }
}

// Route projection can jump to another branch between trajectory knots. Prove
// continuous containment by comparing distance to permitted/forbidden subroutes.
// Distance to any fixed set is 1-Lipschitz: midpoint +/- half-chord bounds the
// whole interval. Unresolved intervals reject after ten subdivisions.
fn continuous_progress_contains(
    route: &Route,
    path: &[TrajectoryPoint],
    lower: f64,
    upper: f64,
) -> bool {
    if !lower.is_finite() || !upper.is_finite() || lower >= upper || path.len() < 2 {
        return false;
    }
    let mut permitted = Vec::new();
    let mut forbidden = Vec::new();
    for (i, pair) in route.points.windows(2).enumerate() {
        let start = route.lengths[i];
        let end = route.lengths[i + 1];
        let mut cuts = vec![start, end];
        for bound in [lower, upper] {
            if bound > start && bound < end {
                cuts.push(bound);
            }
        }
        cuts.sort_by(f64::total_cmp);
        let at = |s: f64| pair[0].plus(pair[1].minus(pair[0]).scaled((s - start) / (end - start)));
        for interval in cuts.windows(2) {
            let segment = (at(interval[0]), at(interval[1]));
            let middle = interval[0] + (interval[1] - interval[0]) / 2.;
            if middle >= lower && middle <= upper {
                permitted.push(segment);
            } else {
                forbidden.push(segment);
            }
        }
    }
    if permitted.is_empty() {
        return false;
    }
    if forbidden.is_empty() {
        return true;
    }
    fn distance(point: Vec2, segments: &[(Vec2, Vec2)]) -> f64 {
        segments
            .iter()
            .map(|(a, b)| {
                let delta = b.minus(*a);
                let length = a.distance(*b);
                if length == 0. {
                    return point.distance(*a);
                }
                let unit = delta.scaled(1. / length);
                let offset = point.minus(*a);
                let along = (offset.x * unit.x + offset.y * unit.y).clamp(0., length);
                point.distance(a.plus(unit.scaled(along)))
            })
            .fold(f64::INFINITY, f64::min)
    }
    fn certify(
        a: Vec2,
        b: Vec2,
        permitted: &[(Vec2, Vec2)],
        forbidden: &[(Vec2, Vec2)],
        depth: u8,
    ) -> bool {
        let middle = a.plus(b.minus(a).scaled(0.5));
        let radius = a.distance(b) / 2.;
        let inside = distance(middle, permitted);
        let outside = distance(middle, forbidden);
        if !radius.is_finite() || !inside.is_finite() || !outside.is_finite() {
            return false;
        }
        // A scale-aware numerical margin rejects ambiguous boundary ties.
        let margin = 1e-10 * (1. + inside + outside + radius);
        if inside + radius + margin < outside - radius {
            return true;
        }
        if inside + margin >= outside || depth == 0 {
            return false;
        }
        certify(a, middle, permitted, forbidden, depth - 1)
            && certify(middle, b, permitted, forbidden, depth - 1)
    }
    path.windows(2)
        .all(|p| certify(p[0].position, p[1].position, &permitted, &forbidden, 10))
}

/// A planner-owned candidate entry point; does not adopt or execute its result.
impl crate::LatticePlanner {
    /// Consume the latest actual plan adoption once, during its control tick.
    /// A failed request also consumes the witness; observations cannot renew it.
    /// The caller owns health and route/localization generations. This does not
    /// select a continuation or replace its current-scene collision certificate.
    pub fn take_continuation_permit(
        &mut self,
        observation: &ContinuationObservation<'_>,
        maximum_duration_s: f64,
    ) -> Option<ContinuationPermit> {
        let witness = self.continuation_adoption.take()?;
        if !self.replan_checked_approaches
            || self.invalid_configuration()
            || self.approach_clock != Some((witness.now, witness.acquisition))
            || observation.now_s != witness.now
            || observation.acquisition_s != witness.acquisition
            || observation.progress_m != witness.progress
        {
            return None;
        }
        ContinuationPermit::from_adoption(&witness.record, observation, maximum_duration_s)
    }

    /// Generate continuation geometry from the planner's retained lateral maneuver.
    /// This is still a read-only candidate probe, not a policy selection or command.
    /// The permit's adoption and frame ownership must be verified by the caller.
    pub fn checked_maneuver_continuation(
        &self,
        permit: &mut ContinuationPermit,
        observation: &ContinuationObservation<'_>,
        ego: rustdriving_core::EgoState,
        route: &Route,
        objects: &[rustdriving_core::Prediction],
    ) -> Option<rustdriving_core::Trajectory> {
        let candidate = (|| {
            if self.invalid_configuration()
                || !self.replan_checked_approaches
                || !ego.pose.position.finite()
                || !ego.pose.yaw.is_finite()
                || !ego.speed.is_finite()
                || ego.speed < 0.
                || !self.vehicle.radius.is_finite()
                || self.vehicle.radius <= 0.
                || !self.vehicle.wheelbase.is_finite()
                || self.vehicle.wheelbase <= 0.
                || !route.half_width.is_finite()
                || route.half_width <= self.vehicle.radius
                || route.points.iter().any(|p| !p.finite())
                || route.points.windows(2).any(|p| {
                    let length = p[0].distance(p[1]);
                    !length.is_finite() || length <= 0.
                })
                || !Route::new(route.points.clone(), route.half_width)
                    .is_ok_and(|r| r.lengths == route.lengths && r.length().is_finite())
                || !permit.observe(observation)
            {
                return None;
            }
            let target = observation.lateral_target_m;
            if !self.previous_lateral.is_finite()
                || (self.previous_lateral - target).abs() >= 0.01
                || self.maneuver_start.is_some_and(|(start, offset, transition)| {
                    !start.is_finite() || !offset.is_finite()
                        || !transition.is_finite() || transition <= 0.
                })
                // A centerline plan need not ever have initiated a lateral shift.
                // Side continuations still require their retained maneuver anchor.
                || (self.maneuver_start.is_none() && target.abs() >= 0.01)
            {
                return None;
            }
            let fitted;
            let planning_route = if self.local_route_geometry {
                fitted = crate::corners::fit(route, self.vehicle).ok()?;
                &fitted
            } else {
                route
            };
            let side = (planning_route.half_width
                - self.vehicle.radius
                - self.lateral_candidate_margin_m?)
                .clamp(0., 4.);
            let index = [0., 1., -1., 0.875, -0.875, 0.75, -0.75, 0.5, -0.5]
                .iter()
                .position(|fraction| (target - side * fraction).abs() < 1e-7)?;
            if index >= 3 && !self.refine_lateral_candidates {
                return None;
            }
            let (progress, lateral) = planning_route.project(ego.pose.position);
            if target * lateral < 0. && lateral.abs() > 1. {
                return None;
            }
            let (_, end_progress) = permit.endpoint_bound()?;
            // The permit uses the supplied route's arc; the planning horizon uses
            // its own route. The subsequent complete certificate enforces the
            // original bound continuously, even when those arcs differ.
            let available = end_progress - route.project(ego.pose.position).0;
            let horizon = available
                .min(40.)
                .min((planning_route.length() - progress - 1.).max(0.));
            if !horizon.is_finite() || horizon <= 0. {
                return None;
            }
            let (geometry, _) =
                self.candidate_geometry(ego, planning_route, route, horizon, (target, index >= 3))?;
            self.checked_continuation_candidate(permit, observation, &geometry, ego, route, objects)
        })();
        if candidate.is_none() {
            permit.revoke();
        }
        candidate
    }

    /// Use current planner scene context and the ordinary directional collision
    /// sweep, including the complete stationary tail. The one-metre refined
    /// clearance floor is conservatively applied to every continuation.
    /// The caller remains responsible for supplying a verified adoption and
    /// matching frame generations, and for committing/recording an accepted plan.
    pub fn checked_continuation_candidate(
        &self,
        permit: &mut ContinuationPermit,
        observation: &ContinuationObservation<'_>,
        geometry: &[Vec2],
        ego: rustdriving_core::EgoState,
        route: &Route,
        objects: &[rustdriving_core::Prediction],
    ) -> Option<rustdriving_core::Trajectory> {
        if !self.replan_checked_approaches
            || self.invalid_configuration()
            || self.approach_clock != Some((observation.now_s, observation.acquisition_s))
            || !ego.pose.position.finite()
            || !ego.pose.yaw.is_finite()
            || !ego.speed.is_finite()
            || ego.speed < 0.
            || !self.vehicle.radius.is_finite()
            || self.vehicle.radius <= 0.
            || !route.half_width.is_finite()
            || route.half_width <= self.vehicle.radius
            || geometry
                .first()
                .is_none_or(|p| p.distance(ego.pose.position) > 1e-7)
            || observation
                .blocking_ids
                .iter()
                .any(|id| !objects.iter().any(|o| o.id == *id))
            || objects.iter().enumerate().any(|(i, o)| {
                objects[..i].iter().any(|previous| previous.id == o.id)
                    || o.positions.is_empty()
                    || !o.dt.is_finite()
                    || o.dt <= 0.
                    || !(o.dt * (o.positions.len() - 1) as f64).is_finite()
                    || !o.valid_radii()
                    || o.positions.iter().any(|p| !p.finite())
            })
        {
            permit.revoke();
            return None;
        }
        let bounds = ContinuationBounds {
            maximum_distance_m: geometry.windows(2).map(|p| p[0].distance(p[1])).sum(),
            cruise_m_s: self.cruise_speed,
            acceleration_m_s2: self.max_acceleration_m_s2,
            deceleration_m_s2: self.max_deceleration_m_s2,
        };
        let constraints = ProfileConstraints {
            route_speed_zones: Some((route, &self.speed_zones, self.vehicle.radius)),
            lateral_acceleration_m_s2: self.max_lateral_acceleration_m_s2,
        };
        let heading = Vec2::new(ego.pose.yaw.cos(), ego.pose.yaw.sin());
        let points = permit.certified_profile(
            observation,
            geometry,
            ego.speed,
            &bounds,
            &constraints,
            |path| {
                corridor_contains(route, path, route.half_width - self.vehicle.radius)
                    && objects.iter().all(|object| {
                        crate::collision::with_motion_direction(
                            path,
                            object,
                            self.vehicle.radius,
                            heading,
                            crate::REFINED_CANDIDATE_CLEARANCE_M,
                            self.motion_directions
                                .as_ref()
                                .map(|context| context.for_prediction(object.id)),
                        )
                        .is_none()
                    })
            },
        )?;
        Some(rustdriving_core::Trajectory {
            points,
            mode: rustdriving_core::DrivingMode::Yield,
            lateral_target: observation.lateral_target_m,
        })
    }
}

// Distance to the route polyline is 1-Lipschitz. A whole interval must fit;
// uncertain intervals reject at the finite subdivision budget.
fn corridor_contains(route: &Route, path: &[TrajectoryPoint], radius: f64) -> bool {
    fn distance(point: Vec2, route: &Route) -> f64 {
        route
            .points
            .windows(2)
            .map(|p| {
                let delta = p[1].minus(p[0]);
                let length = p[0].distance(p[1]);
                let unit = delta.scaled(1. / length);
                let offset = point.minus(p[0]);
                let along = (offset.x * unit.x + offset.y * unit.y).clamp(0., length);
                point.distance(p[0].plus(unit.scaled(along)))
            })
            .fold(f64::INFINITY, f64::min)
    }
    fn fits(a: Vec2, b: Vec2, route: &Route, radius: f64, depth: u8) -> bool {
        let middle = a.plus(b.minus(a).scaled(0.5));
        let distance = distance(middle, route);
        let half = a.distance(b) / 2.;
        if !distance.is_finite() || !half.is_finite() {
            return false;
        }
        let margin = 1e-10 * (1. + distance + half + radius);
        if distance + half + margin < radius {
            return true;
        }
        if distance + margin >= radius || depth == 0 {
            return false;
        }
        fits(a, middle, route, radius, depth - 1) && fits(middle, b, route, radius, depth - 1)
    }
    radius.is_finite()
        && radius > 0.
        && path.len() >= 2
        && path
            .windows(2)
            .all(|p| fits(p[0].position, p[1].position, route, radius, 10))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn geometry() -> Vec<Vec2> {
        (0..=40).map(|i| Vec2::new(i as f64 * 0.1, 0.)).collect()
    }
    fn bounds() -> ContinuationBounds {
        ContinuationBounds {
            maximum_distance_m: 1.5,
            cruise_m_s: 2.,
            acceleration_m_s2: 2.,
            deceleration_m_s2: 2.5,
        }
    }
    fn verify(path: &[TrajectoryPoint], initial: f64, b: &ContinuationBounds) {
        assert_eq!(path[0].speed, initial);
        assert_eq!(path[0].time, 0.);
        for pair in path.windows(2) {
            let dt = pair[1].time - pair[0].time;
            assert!(dt > 0.);
            let a = (pair[1].speed - pair[0].speed) / dt;
            assert!(a >= -b.deceleration_m_s2 - 1e-8 && a <= b.acceleration_m_s2 + 1e-8);
            let ds = pair[0].position.distance(pair[1].position);
            assert!((ds - (pair[0].speed + pair[1].speed) * dt / 2.).abs() < 1e-8);
        }
        let tail = &path[path.len() - 2..];
        assert_eq!(tail[0].position, tail[1].position);
        assert_eq!(tail[0].speed, 0.);
        assert_eq!(tail[1].speed, 0.);
        assert!(tail[1].time - tail[0].time >= 8. - 1e-9);
        assert!(tail[1].position.x <= b.maximum_distance_m + 1e-8);
    }

    #[test]
    fn crossing_point_two_does_not_silently_cancel_reacceleration() {
        for initial in [0.19, 0.2, 0.21, 0.31] {
            let b = bounds();
            let path = stopped_profile(&geometry(), initial, &b, |p| {
                verify(p, initial, &b);
                true
            })
            .unwrap();
            assert!(path.iter().any(|p| p.speed > initial + 0.5));
        }
    }

    #[test]
    fn rejection_never_returns_an_uncertified_path_or_shortens_the_hold() {
        let b = bounds();
        let mut count = 0;
        assert!(
            stopped_profile(&geometry(), 0.31, &b, |p| {
                count += 1;
                verify(p, 0.31, &b);
                false
            })
            .is_none()
        );
        assert!((1..=16).contains(&count));
    }

    #[test]
    fn full_tail_collision_rejects_all_stopping_candidates() {
        use rustdriving_core::Prediction;
        let moving = Prediction {
            id: 1,
            positions: (0..=40)
                .map(|i| Vec2::new(0.75, 6. - i as f64 * 0.2))
                .collect(),
            radius: 2.,
            additional_radius_m: vec![],
            dt: 0.2,
        };
        // The crossing obstacle reaches every allowed stopped endpoint during
        // the hold; an endpoint-at-arrival-only test would miss that interval.
        let mut checked = 0;
        assert!(
            stopped_profile(&geometry(), 0.21, &bounds(), |p| {
                checked += 1;
                crate::collision::first_contact_time(p, &moving, 1.25, Vec2::new(1., 0.)).is_none()
            })
            .is_none()
        );
        assert!(checked > 0);
    }

    #[test]
    fn unreachable_or_malformed_input_never_reaches_certificate() {
        let mut calls = 0;
        for value in [f64::NAN, f64::INFINITY, -1., 0., 5.] {
            let mut b = bounds();
            b.maximum_distance_m = value;
            assert!(
                stopped_profile(&geometry(), 0.31, &b, |_| {
                    calls += 1;
                    true
                })
                .is_none()
            );
        }
        let mut b = bounds();
        b.maximum_distance_m = 0.001;
        assert!(
            stopped_profile(&geometry(), 2., &b, |_| {
                calls += 1;
                true
            })
            .is_none()
        );
        assert_eq!(calls, 0);
    }
    #[test]
    fn speed_zone_caps_acceleration_without_clamping_initial_overspeed() {
        let route = Route::new(geometry(), 5.).unwrap();
        let zones = vec![RouteSpeedZone {
            start_s_m: 0.,
            end_s_m: 4.,
            speed_m_s: 0.4,
        }];
        let constraints = ProfileConstraints {
            route_speed_zones: Some((&route, &zones, 0.)),
            lateral_acceleration_m_s2: None,
        };
        for initial in [0.21, 0.8] {
            let b = bounds();
            let path =
                stopped_profile_with_constraints(&geometry(), initial, &b, &constraints, |_| true)
                    .unwrap();
            verify(&path, initial, &b);
            for p in &path {
                let recovery = (initial * initial - 2. * b.deceleration_m_s2 * p.position.x)
                    .max(0.)
                    .sqrt();
                assert!(p.speed <= 0.4_f64.max(recovery) + 1e-8);
            }
        }
    }

    #[test]
    fn narrow_zone_between_knots_is_not_skipped() {
        let line = vec![Vec2::new(0., 0.), Vec2::new(1., 0.), Vec2::new(2., 0.)];
        let route = Route::new(line.clone(), 5.).unwrap();
        let zones = vec![RouteSpeedZone {
            start_s_m: 0.45,
            end_s_m: 0.55,
            speed_m_s: 0.3,
        }];
        let constraints = ProfileConstraints {
            route_speed_zones: Some((&route, &zones, 0.)),
            lateral_acceleration_m_s2: None,
        };
        let path = stopped_profile_with_constraints(&line, 0.21, &bounds(), &constraints, |_| true)
            .unwrap();
        assert!(
            path.iter()
                .filter(|p| p.position.x <= 1.)
                .all(|p| p.speed <= 0.3 + 1e-8)
        );
    }

    #[test]
    fn curved_continuation_keeps_lateral_acceleration_limit() {
        let arc: Vec<_> = (0..=40)
            .map(|i| {
                let angle = i as f64 * 0.025;
                Vec2::new(2. * angle.sin(), 2. * (1. - angle.cos()))
            })
            .collect();
        let constraints = ProfileConstraints {
            route_speed_zones: None,
            lateral_acceleration_m_s2: Some(0.1),
        };
        let path = stopped_profile_with_constraints(&arc, 0.21, &bounds(), &constraints, |_| true)
            .unwrap();
        verify(&path, 0.21, &bounds());
        // Radius two metres implies curvature 0.5 and v <= sqrt(0.1/0.5).
        assert!(path.iter().all(|p| p.speed <= (0.2_f64).sqrt() + 1e-8));
    }

    #[test]
    fn invalid_constraints_do_not_reach_certificate() {
        let route = Route::new(geometry(), 5.).unwrap();
        let zones = vec![RouteSpeedZone {
            start_s_m: 1.,
            end_s_m: 0.,
            speed_m_s: 0.4,
        }];
        let invalid = [
            ProfileConstraints {
                route_speed_zones: Some((&route, &zones, 0.)),
                lateral_acceleration_m_s2: None,
            },
            ProfileConstraints {
                route_speed_zones: None,
                lateral_acceleration_m_s2: Some(f64::NAN),
            },
            ProfileConstraints {
                route_speed_zones: None,
                lateral_acceleration_m_s2: Some(0.),
            },
            ProfileConstraints {
                route_speed_zones: Some((&route, &[], -1.)),
                lateral_acceleration_m_s2: None,
            },
        ];
        for constraints in invalid {
            assert!(
                stopped_profile_with_constraints(
                    &geometry(),
                    0.21,
                    &bounds(),
                    &constraints,
                    |_| panic!("invalid input reached certificate")
                )
                .is_none()
            );
        }
    }
    fn observation(now: f64) -> ContinuationObservation<'static> {
        ContinuationObservation {
            now_s: now,
            acquisition_s: Some(now),
            localization_generation: 1,
            route_generation: 2,
            blocking_ids: &[7],
            lateral_target_m: 3.9,
            progress_m: 10.,
            healthy: true,
        }
    }
    fn adoption() -> rustdriving_core::CheckedApproachDiagnostics {
        rustdriving_core::CheckedApproachDiagnostics {
            supporting_acquisitions: 3,
            first_acquisition_s: Some(0.),
            last_acquisition_s: Some(0.2),
            proposal_end_progress_m: Some(12.),
            proposal_endpoint: Some(Vec2::new(12., 3.9)),
            proposal_lateral_target_m: Some(3.9),
            blocking_prediction_ids: vec![7],
            adopted: true,
        }
    }

    #[test]
    fn permit_deadline_is_not_renewed_by_cached_or_fresh_observations() {
        let mut permit =
            ContinuationPermit::from_adoption(&adoption(), &observation(0.2), 1.).unwrap();
        let mut cached = observation(0.5);
        cached.acquisition_s = Some(0.2);
        assert!(permit.observe(&cached));
        assert!(permit.observe(&observation(1.1)));
        assert!(!permit.observe(&observation(1.2)));
        assert!(permit.endpoint_bound().is_none());
        assert!(!permit.observe(&observation(1.15)));
    }

    #[test]
    fn changed_context_or_bad_clock_stickily_revokes_permit() {
        for case in 0..9 {
            let mut permit =
                ContinuationPermit::from_adoption(&adoption(), &observation(0.2), 2.).unwrap();
            let mut changed = observation(0.3);
            match case {
                0 => changed.healthy = false,
                1 => changed.route_generation += 1,
                2 => changed.localization_generation += 1,
                3 => changed.blocking_ids = &[8],
                4 => changed.lateral_target_m = -3.9,
                5 => changed.acquisition_s = None,
                6 => changed.acquisition_s = Some(0.4),
                7 => changed.now_s = 0.1,
                _ => changed.acquisition_s = Some(0.1),
            }
            assert!(!permit.observe(&changed));
            assert!(!permit.observe(&observation(0.4)));
        }
    }

    #[test]
    fn stale_observations_and_endpoint_completion_expire_permit() {
        let mut permit =
            ContinuationPermit::from_adoption(&adoption(), &observation(0.2), 2.).unwrap();
        let mut stale = observation(0.56);
        stale.acquisition_s = Some(0.2);
        assert!(!permit.observe(&stale));
        let mut permit =
            ContinuationPermit::from_adoption(&adoption(), &observation(0.2), 2.).unwrap();
        let mut completed = observation(0.3);
        completed.progress_m = 12.;
        assert!(!permit.observe(&completed));
        assert!(!permit.observe(&observation(0.4)));
    }

    #[test]
    fn unsupported_adoption_and_unbounded_duration_cannot_grant_permit() {
        let mut unconfirmed = adoption();
        unconfirmed.adopted = false;
        assert!(ContinuationPermit::from_adoption(&unconfirmed, &observation(0.2), 1.).is_none());
        for duration in [0., -1., f64::NAN, f64::INFINITY, 8.1] {
            assert!(
                ContinuationPermit::from_adoption(&adoption(), &observation(0.2), duration)
                    .is_none()
            );
        }
        let mut mismatch = observation(0.2);
        mismatch.blocking_ids = &[9];
        assert!(ContinuationPermit::from_adoption(&adoption(), &mismatch, 1.).is_none());
        let mut retreat =
            ContinuationPermit::from_adoption(&adoption(), &observation(0.2), 1.).unwrap();
        let mut backwards = observation(0.3);
        backwards.progress_m = 9.;
        assert!(!retreat.observe(&backwards));
    }
    #[test]
    fn combined_generation_requires_live_permit_and_current_certificate() {
        let line: Vec<_> = (0..=40)
            .map(|i| Vec2::new(10. + i as f64 * 0.1, 3.9))
            .collect();
        let route = Route::new(vec![Vec2::new(0., 0.), Vec2::new(20., 0.)], 6.).unwrap();
        let constraints = ProfileConstraints {
            route_speed_zones: Some((&route, &[], 0.)),
            lateral_acceleration_m_s2: None,
        };
        let mut permit =
            ContinuationPermit::from_adoption(&adoption(), &observation(0.2), 1.).unwrap();
        let mut seen = 0;
        let path = permit
            .certified_profile(
                &observation(0.3),
                &line,
                0.21,
                &bounds(),
                &constraints,
                |p| {
                    seen += 1;
                    assert!(p.last().unwrap().position.x <= 12. + 1e-8);
                    true
                },
            )
            .unwrap();
        assert!(seen > 0);
        assert!(path.iter().any(|p| p.speed > 0.7));
        let mut stale = observation(0.6);
        stale.acquisition_s = Some(0.2);
        assert!(
            permit
                .certified_profile(&stale, &line, 0.21, &bounds(), &constraints, |_| panic!(
                    "revoked permit reached certificate"
                ))
                .is_none()
        );
        assert!(
            permit
                .certified_profile(
                    &observation(0.7),
                    &line,
                    0.21,
                    &bounds(),
                    &constraints,
                    |_| panic!("permit revived")
                )
                .is_none()
        );
    }

    #[test]
    fn rejected_complete_paths_revoke_combined_generation() {
        let line: Vec<_> = (0..=40)
            .map(|i| Vec2::new(10. + i as f64 * 0.1, 3.9))
            .collect();
        let route = Route::new(vec![Vec2::new(0., 0.), Vec2::new(20., 0.)], 6.).unwrap();
        let constraints = ProfileConstraints {
            route_speed_zones: Some((&route, &[], 0.)),
            lateral_acceleration_m_s2: None,
        };
        let mut permit =
            ContinuationPermit::from_adoption(&adoption(), &observation(0.2), 1.).unwrap();
        assert!(
            permit
                .certified_profile(
                    &observation(0.3),
                    &line,
                    0.21,
                    &bounds(),
                    &constraints,
                    |_| false
                )
                .is_none()
        );
        assert!(permit.endpoint_bound().is_none());
        assert!(
            permit
                .certified_profile(
                    &observation(0.4),
                    &line,
                    0.21,
                    &bounds(),
                    &constraints,
                    |_| panic!("rejection did not revoke")
                )
                .is_none()
        );
    }

    #[test]
    fn candidate_cannot_extend_authorized_progress_or_change_start() {
        let route = Route::new(vec![Vec2::new(0., 0.), Vec2::new(20., 0.)], 6.).unwrap();
        let constraints = ProfileConstraints {
            route_speed_zones: Some((&route, &[], 0.)),
            lateral_acceleration_m_s2: None,
        };
        let line: Vec<_> = (0..=40)
            .map(|i| Vec2::new(10. + i as f64 * 0.1, 3.9))
            .collect();
        let mut b = bounds();
        b.maximum_distance_m = 4.;
        let mut permit =
            ContinuationPermit::from_adoption(&adoption(), &observation(0.2), 1.).unwrap();
        let path = permit
            .certified_profile(&observation(0.3), &line, 0.21, &b, &constraints, |p| {
                assert!(p.iter().all(|p| route.project(p.position).0 <= 12. + 1e-8));
                true
            })
            .unwrap();
        assert!(path.last().unwrap().position.x <= 12. + 1e-8);
        let unrelated: Vec<_> = line.iter().map(|p| Vec2::new(p.x + 3., p.y)).collect();
        assert!(
            permit
                .certified_profile(
                    &observation(0.4),
                    &unrelated,
                    0.21,
                    &bounds(),
                    &constraints,
                    |_| panic!("wrong start reached certificate")
                )
                .is_none()
        );
    }
    #[test]
    fn continuous_progress_rejects_shortcut_between_valid_knots() {
        let route = Route::new(
            vec![
                Vec2::new(0., 0.),
                Vec2::new(0., 10.),
                Vec2::new(10., 10.),
                Vec2::new(10., 0.),
                Vec2::new(1., 0.),
            ],
            6.,
        )
        .unwrap();
        let path = vec![
            TrajectoryPoint {
                position: Vec2::new(0., 1.),
                speed: 1.,
                time: 0.,
            },
            TrajectoryPoint {
                position: Vec2::new(10., 1.),
                speed: 1.,
                time: 10.,
            },
        ];
        assert!(path.iter().all(|p| route.project(p.position).0 < 30.));
        // The interior lies nearest the return leg, beyond authorized progress.
        assert!(route.project(Vec2::new(5., 1.)).0 > 30.);
        assert!(!continuous_progress_contains(&route, &path, 0., 30.));
    }

    #[test]
    fn continuous_progress_accepts_offset_interior_and_rejects_ambiguous_boundary() {
        let route = Route::new(vec![Vec2::new(0., 0.), Vec2::new(20., 0.)], 6.).unwrap();
        let mut path = vec![
            TrajectoryPoint {
                position: Vec2::new(10., 3.9),
                speed: 1.,
                time: 0.,
            },
            TrajectoryPoint {
                position: Vec2::new(11.5, 3.9),
                speed: 0.,
                time: 3.,
            },
            TrajectoryPoint {
                position: Vec2::new(11.5, 3.9),
                speed: 0.,
                time: 11.,
            },
        ];
        assert!(continuous_progress_contains(&route, &path, 9.8, 12.));
        path[1].position.x = 12.;
        path[2].position.x = 12.;
        assert!(!continuous_progress_contains(&route, &path, 9.8, 12.));
    }
    #[test]
    fn planner_candidate_uses_current_scene_and_complete_collision_certificate() {
        let route = Route::new(vec![Vec2::new(0., 0.), Vec2::new(20., 0.)], 6.).unwrap();
        let line: Vec<_> = (0..=15)
            .map(|i| Vec2::new(10. + i as f64 * 0.1, 3.9))
            .collect();
        let mut planner = crate::LatticePlanner {
            replan_checked_approaches: true,
            search_obstruction_stops: true,
            settle_obstruction_stops: true,
            lateral_candidate_margin_m: Some(0.1),
            ..crate::LatticePlanner::default()
        };
        planner.observe_checked_approach_scene(0.3, Some(0.3));
        let mut ego = rustdriving_core::EgoState::default();
        ego.pose.position = line[0];
        ego.speed = 0.21;
        let safe = rustdriving_core::Prediction {
            id: 7,
            positions: vec![Vec2::new(100., 100.); 41],
            radius: 0.5,
            dt: 0.2,
            additional_radius_m: vec![],
        };
        let mut permit =
            ContinuationPermit::from_adoption(&adoption(), &observation(0.2), 1.).unwrap();
        let path = planner
            .checked_continuation_candidate(
                &mut permit,
                &observation(0.3),
                &line,
                ego,
                &route,
                std::slice::from_ref(&safe),
            )
            .unwrap();
        assert!(path.points.iter().any(|p| p.speed > 0.7));
        let moving = rustdriving_core::Prediction {
            positions: (0..=40)
                .map(|i| Vec2::new(11., 12. - i as f64 * 0.4))
                .collect(),
            ..safe
        };
        let mut permit =
            ContinuationPermit::from_adoption(&adoption(), &observation(0.2), 1.).unwrap();
        assert!(
            planner
                .checked_continuation_candidate(
                    &mut permit,
                    &observation(0.3),
                    &line,
                    ego,
                    &route,
                    &[moving]
                )
                .is_none()
        );
        assert!(permit.endpoint_bound().is_none());
    }

    #[test]
    fn planner_candidate_rejects_mismatched_scene_and_missing_blocker() {
        let route = Route::new(vec![Vec2::new(0., 0.), Vec2::new(20., 0.)], 6.).unwrap();
        let line = vec![Vec2::new(10., 3.9), Vec2::new(11.5, 3.9)];
        let mut planner = crate::LatticePlanner {
            replan_checked_approaches: true,
            search_obstruction_stops: true,
            settle_obstruction_stops: true,
            lateral_candidate_margin_m: Some(0.1),
            ..crate::LatticePlanner::default()
        };
        let mut ego = rustdriving_core::EgoState::default();
        ego.pose.position = line[0];
        ego.speed = 0.21;
        let safe = rustdriving_core::Prediction {
            id: 7,
            positions: vec![Vec2::new(100., 100.); 41],
            radius: 0.5,
            dt: 0.2,
            additional_radius_m: vec![],
        };
        for clock in [None, Some(0.2), Some(0.3)] {
            planner.observe_checked_approach_scene(0.3, clock);
            let mut permit =
                ContinuationPermit::from_adoption(&adoption(), &observation(0.2), 1.).unwrap();
            let objects = if clock == Some(0.3) {
                &[][..]
            } else {
                std::slice::from_ref(&safe)
            };
            assert!(
                planner
                    .checked_continuation_candidate(
                        &mut permit,
                        &observation(0.3),
                        &line,
                        ego,
                        &route,
                        objects
                    )
                    .is_none()
            );
            assert!(permit.endpoint_bound().is_none());
        }
    }

    #[test]
    fn continuous_corridor_rejects_cut_across_inside_corner() {
        let route = Route::new(
            vec![Vec2::new(0., 0.), Vec2::new(0., 5.), Vec2::new(5., 5.)],
            1.,
        )
        .unwrap();
        let point = |position, time| TrajectoryPoint {
            position,
            speed: 1.,
            time,
        };
        let shortcut = vec![point(Vec2::new(0., 0.), 0.), point(Vec2::new(5., 5.), 10.)];
        assert!(!corridor_contains(&route, &shortcut, 0.8));
        let following = vec![
            point(Vec2::new(0., 0.), 0.),
            point(Vec2::new(0., 5.), 5.),
            point(Vec2::new(5., 5.), 10.),
        ];
        assert!(corridor_contains(&route, &following, 0.8));
    }

    #[test]
    fn candidate_cannot_bypass_ordinary_configuration_rejection() {
        use rustdriving_core::Planner;
        let route = Route::new(vec![Vec2::new(0., 0.), Vec2::new(20., 0.)], 6.).unwrap();
        let line = vec![Vec2::new(10., 3.9), Vec2::new(11.5, 3.9)];
        let mut ego = rustdriving_core::EgoState::default();
        ego.pose.position = line[0];
        ego.speed = 0.21;
        let objects = [rustdriving_core::Prediction {
            id: 7,
            positions: vec![Vec2::new(100., 100.); 41],
            radius: 0.5,
            dt: 0.2,
            additional_radius_m: vec![],
        }];
        for case in 0..10 {
            let mut planner = crate::LatticePlanner {
                replan_checked_approaches: true,
                search_obstruction_stops: true,
                settle_obstruction_stops: true,
                lateral_candidate_margin_m: Some(0.1),
                ..crate::LatticePlanner::default()
            };
            planner.observe_checked_approach_scene(0.3, Some(0.3));
            let valid = planner.clone();
            match case {
                0 => planner.search_obstruction_stops = false,
                1 => planner.settle_obstruction_stops = false,
                2 => planner.lateral_candidate_margin_m = None,
                3 => planner.lateral_candidate_margin_m = Some(f64::NAN),
                4 => planner.lateral_candidate_margin_m = Some(1.1),
                5 => planner.checked_approach_min_progress_m = Some(0.1),
                6 => planner.cruise_speed = f64::NAN,
                7 => planner.max_acceleration_m_s2 = 0.,
                8 => planner.max_deceleration_m_s2 = -1.,
                _ => planner.max_lateral_acceleration_m_s2 = Some(0.),
            }
            let ordinary = planner.clone().plan(ego, &route, &objects);
            assert_eq!(ordinary.mode, rustdriving_core::DrivingMode::Emergency);
            assert!(ordinary.points.is_empty());
            let mut permit =
                ContinuationPermit::from_adoption(&adoption(), &observation(0.2), 1.).unwrap();
            assert!(
                planner
                    .checked_continuation_candidate(
                        &mut permit,
                        &observation(0.3),
                        &line,
                        ego,
                        &route,
                        &objects
                    )
                    .is_none()
            );
            assert!(permit.endpoint_bound().is_none());
            // Repairing settings cannot revive the old authorization.
            assert!(
                valid
                    .checked_continuation_candidate(
                        &mut permit,
                        &observation(0.3),
                        &line,
                        ego,
                        &route,
                        &objects
                    )
                    .is_none()
            );
        }
    }

    #[test]
    fn candidate_certificate_must_start_at_measured_ego_position() {
        let route = Route::new(vec![Vec2::new(0., 0.), Vec2::new(20., 0.)], 6.).unwrap();
        let mut planner = crate::LatticePlanner {
            replan_checked_approaches: true,
            search_obstruction_stops: true,
            settle_obstruction_stops: true,
            lateral_candidate_margin_m: Some(0.1),
            ..crate::LatticePlanner::default()
        };
        planner.observe_checked_approach_scene(0.3, Some(0.3));
        let mut ego = rustdriving_core::EgoState::default();
        ego.pose.position = Vec2::new(10., 3.9);
        ego.speed = 0.21;
        let objects = [rustdriving_core::Prediction {
            id: 7,
            positions: vec![Vec2::new(100., 100.); 41],
            radius: 0.5,
            dt: 0.2,
            additional_radius_m: vec![],
        }];
        for offset in [Vec2::default(), Vec2::new(0.1, 0.), Vec2::new(0., -0.1)] {
            let line = vec![ego.pose.position.plus(offset), Vec2::new(11.5, 3.9)];
            let mut permit =
                ContinuationPermit::from_adoption(&adoption(), &observation(0.2), 1.).unwrap();
            let result = planner.checked_continuation_candidate(
                &mut permit,
                &observation(0.3),
                &line,
                ego,
                &route,
                &objects,
            );
            if offset == Vec2::default() {
                let path = result.unwrap();
                assert_eq!(path.points[0].position, ego.pose.position);
            } else {
                assert!(result.is_none());
                assert!(permit.endpoint_bound().is_none());
            }
        }
    }

    fn maneuver_fixture() -> (
        crate::LatticePlanner,
        Route,
        rustdriving_core::EgoState,
        Vec<rustdriving_core::Prediction>,
    ) {
        let planner = crate::LatticePlanner {
            replan_checked_approaches: true,
            search_obstruction_stops: true,
            settle_obstruction_stops: true,
            lateral_candidate_margin_m: Some(0.2),
            previous_lateral: 3.9,
            maneuver_start: Some((0., 3.9, 10.)),
            ..crate::LatticePlanner::default()
        };
        let route = Route::new(
            vec![Vec2::new(0., 0.), Vec2::new(20., 0.)],
            planner.vehicle.radius + 3.9 + 0.2,
        )
        .unwrap();
        let mut ego = rustdriving_core::EgoState::default();
        ego.pose.position = Vec2::new(10., 3.9);
        ego.speed = 0.21;
        let objects = vec![rustdriving_core::Prediction {
            id: 7,
            positions: vec![Vec2::new(100., 100.); 41],
            radius: 0.5,
            dt: 0.2,
            additional_radius_m: vec![],
        }];
        (planner, route, ego, objects)
    }

    #[test]
    fn retained_maneuver_generates_reaccelerating_candidates_across_ticks() {
        for local in [false, true] {
            let (mut planner, route, mut ego, objects) = maneuver_fixture();
            planner.local_route_geometry = local;
            let mut permit =
                ContinuationPermit::from_adoption(&adoption(), &observation(0.2), 1.).unwrap();
            for (tick, speed) in [0.19, 0.21, 0.31].into_iter().enumerate() {
                let now = 0.3 + tick as f64 * 0.1;
                ego.pose.position.x = 10. + tick as f64 * 0.02;
                ego.speed = speed;
                let mut observed = observation(now);
                observed.progress_m = ego.pose.position.x;
                planner.observe_checked_approach_scene(now, Some(now));
                let path = planner
                    .checked_maneuver_continuation(&mut permit, &observed, ego, &route, &objects)
                    .unwrap();
                assert_eq!(path.points[0].position, ego.pose.position);
                assert_eq!(path.points[0].speed, speed);
                assert_eq!(path.lateral_target, 3.9);
                assert!(path.points.iter().any(|p| p.speed > 0.7));
                assert!(
                    path.points
                        .iter()
                        .all(|p| (p.position.y - 3.9).abs() < 1e-8 && p.position.x < 12.)
                );
                assert_eq!(permit.endpoint_bound(), Some((Vec2::new(12., 3.9), 12.)));
                assert_eq!(planner.maneuver_start, Some((0., 3.9, 10.)));
                assert!(!planner.obstruction_stop_requested);
            }
        }
    }

    #[test]
    fn retained_maneuver_rejects_missing_changed_or_invalid_anchor() {
        for case in 0..5 {
            let (mut planner, mut route, ego, objects) = maneuver_fixture();
            planner.observe_checked_approach_scene(0.3, Some(0.3));
            match case {
                0 => planner.maneuver_start = None,
                1 => planner.previous_lateral = -3.9,
                2 => planner.maneuver_start = Some((0., 3.9, 0.)),
                3 => planner.lateral_candidate_margin_m = Some(0.4),
                _ => route.lengths[1] += 1.,
            }
            let mut permit =
                ContinuationPermit::from_adoption(&adoption(), &observation(0.2), 1.).unwrap();
            assert!(
                planner
                    .checked_maneuver_continuation(
                        &mut permit,
                        &observation(0.3),
                        ego,
                        &route,
                        &objects,
                    )
                    .is_none()
            );
            assert!(permit.endpoint_bound().is_none());
        }
    }

    #[test]
    fn retained_maneuver_still_checks_obstacles_through_full_hold() {
        let (mut planner, route, ego, mut objects) = maneuver_fixture();
        planner.observe_checked_approach_scene(0.3, Some(0.3));
        objects[0].positions = (0..=40)
            .map(|i| Vec2::new(11., 12. - i as f64 * 0.4))
            .collect();
        let mut permit =
            ContinuationPermit::from_adoption(&adoption(), &observation(0.2), 1.).unwrap();
        assert!(
            planner
                .checked_maneuver_continuation(
                    &mut permit,
                    &observation(0.3),
                    ego,
                    &route,
                    &objects,
                )
                .is_none()
        );
        assert!(permit.endpoint_bound().is_none());
        assert_eq!(planner.maneuver_start, Some((0., 3.9, 10.)));
    }

    fn actual_adoption_fixture() -> (
        crate::LatticePlanner,
        Route,
        rustdriving_core::EgoState,
        Vec<rustdriving_core::Prediction>,
    ) {
        use rustdriving_core::{DrivingMode, EgoState, Planner};
        let mut planner = crate::LatticePlanner {
            settle_obstruction_stops: true,
            search_obstruction_stops: true,
            replan_checked_approaches: true,
            lateral_candidate_margin_m: Some(0.35),
            ..crate::LatticePlanner::default()
        };
        let narrow = Route::new(vec![Vec2::default(), Vec2::new(100., 0.)], 2.1).unwrap();
        let mut obstacle = rustdriving_core::Prediction {
            id: 7,
            positions: vec![Vec2::new(18., 0.); 41],
            radius: 1.,
            dt: 0.2,
            additional_radius_m: vec![],
        };
        let mut ego = EgoState {
            speed: 8.,
            ..EgoState::default()
        };
        assert_eq!(
            planner
                .plan(ego, &narrow, std::slice::from_ref(&obstacle))
                .mode,
            DrivingMode::Emergency
        );
        ego.pose.position.x = 10.;
        ego.speed = 0.;
        planner.plan(ego, &narrow, std::slice::from_ref(&obstacle));
        assert!(planner.holding_obstruction_stop());
        let wide = Route::new(narrow.points.clone(), 5.5).unwrap();
        obstacle.positions = vec![Vec2::new(50., 0.); 41];
        obstacle.additional_radius_m = (0..=40)
            .map(|i| {
                let t = i as f64 * 0.2;
                0.2 + 0.3 * t + 0.05 * t * t
            })
            .collect();
        let objects = vec![obstacle];
        for now in [0., 0.1, 0.2] {
            planner.observe_checked_approach_scene(now, Some(now));
            let path = planner.plan(ego, &wide, &objects);
            if now < 0.2 {
                assert!(!planner.checked_approach_diagnostics().adopted);
                assert!(
                    planner
                        .take_continuation_permit(&observation(now), 1.)
                        .is_none()
                );
            } else {
                assert!(planner.checked_approach_diagnostics().adopted);
                assert_eq!(
                    planner.checked_approach_diagnostics().proposal_endpoint,
                    Some(path.points.last().unwrap().position)
                );
            }
        }
        (planner, wide, ego, objects)
    }

    #[test]
    fn actual_planner_adoption_issues_only_one_permit() {
        let (mut planner, _, _, _) = actual_adoption_fixture();
        let status = planner.checked_approach_diagnostics().clone();
        let mut observed = observation(0.2);
        observed.lateral_target_m = status.proposal_lateral_target_m.unwrap();
        observed.blocking_ids = &status.blocking_prediction_ids;
        let permit = planner.take_continuation_permit(&observed, 1.).unwrap();
        assert_eq!(
            permit.endpoint_bound(),
            Some((
                status.proposal_endpoint.unwrap(),
                status.proposal_end_progress_m.unwrap(),
            ))
        );
        assert!(planner.take_continuation_permit(&observed, 8.).is_none());
        // Consuming a continuation witness does not rewrite recorded adoption.
        assert_eq!(planner.checked_approach_diagnostics(), &status);
    }

    #[test]
    fn actual_adoption_witness_cannot_survive_replan_reset_or_bad_request() {
        use rustdriving_core::Planner;
        for case in 0..6 {
            let (mut planner, route, ego, objects) = actual_adoption_fixture();
            let status = planner.checked_approach_diagnostics().clone();
            let mut observed = observation(0.2);
            observed.lateral_target_m = status.proposal_lateral_target_m.unwrap();
            observed.blocking_ids = &status.blocking_prediction_ids;
            match case {
                0 => planner.reset_route_state(),
                1 => {
                    planner.plan(ego, &route, &objects);
                }
                2 => planner.observe_checked_approach_scene(0.25, Some(0.2)),
                3 => observed.progress_m += 0.1,
                4 => observed.healthy = false,
                _ => observed.now_s = 0.25,
            }
            assert!(planner.take_continuation_permit(&observed, 1.).is_none());
            planner.observe_checked_approach_scene(0.2, Some(0.2));
            observed = observation(0.2);
            observed.lateral_target_m = status.proposal_lateral_target_m.unwrap();
            observed.blocking_ids = &status.blocking_prediction_ids;
            assert!(planner.take_continuation_permit(&observed, 1.).is_none());
        }
    }

    #[test]
    fn diagnostic_record_alone_cannot_issue_planner_permit() {
        let (mut planner, _, _, _) = maneuver_fixture();
        planner.observe_checked_approach_scene(0.2, Some(0.2));
        // Even an adopted-looking diagnostic is not the internal plan witness.
        planner.approach_confirmation.diagnostics = adoption();
        assert!(
            planner
                .take_continuation_permit(&observation(0.2), 1.)
                .is_none()
        );
    }

    #[test]
    fn centerline_continuation_does_not_require_a_previous_lateral_shift() {
        let (mut planner, route, mut ego, objects) = maneuver_fixture();
        planner.previous_lateral = 0.;
        planner.maneuver_start = None;
        ego.pose.position.y = 0.;
        let mut record = adoption();
        record.proposal_lateral_target_m = Some(0.);
        record.proposal_endpoint = Some(Vec2::new(12., 0.));
        let mut observed = observation(0.2);
        observed.lateral_target_m = 0.;
        let mut permit = ContinuationPermit::from_adoption(&record, &observed, 1.).unwrap();
        observed.now_s = 0.3;
        observed.acquisition_s = Some(0.3);
        planner.observe_checked_approach_scene(0.3, Some(0.3));
        let path = planner.checked_maneuver_continuation(
            &mut permit, &observed, ego, &route, &objects,
        ).unwrap();
        assert!(path.points.iter().all(|p| p.position.y.abs() < 1e-10));
        assert!(path.points.iter().any(|p| p.speed > 0.7));
        assert!(planner.maneuver_start.is_none());
    }

    #[test]
    fn actual_adoption_flows_into_independently_sampled_continuation() {
        let (mut planner, route, mut ego, objects) = actual_adoption_fixture();
        let status = planner.checked_approach_diagnostics().clone();
        let mut observed = observation(0.2);
        observed.lateral_target_m = status.proposal_lateral_target_m.unwrap();
        observed.blocking_ids = &status.blocking_prediction_ids;
        let mut permit = planner.take_continuation_permit(&observed, 1.).unwrap();
        let bound = permit.endpoint_bound().unwrap();
        let anchor = planner.maneuver_start;
        ego.speed = 0.21;
        observed.now_s = 0.3;
        observed.acquisition_s = Some(0.3);
        planner.observe_checked_approach_scene(0.3, Some(0.3));
        let path = planner.checked_maneuver_continuation(
            &mut permit, &observed, ego, &route, &objects,
        ).expect("actual confirmed approach should produce a checked continuation");
        assert_eq!(path.points[0].position, ego.pose.position);
        assert_eq!(path.points[0].speed, ego.speed);
        assert!(path.points.iter().any(|p| p.speed > 0.7));
        let tail = path.points.windows(2).next_back().unwrap();
        assert_eq!(tail[0].position, tail[1].position);
        assert_eq!(tail[0].speed, 0.);
        assert_eq!(tail[1].speed, 0.);
        assert!(tail[1].time - tail[0].time >= 8. - 1e-9);
        // Independent dense samples reconstruct accelerated motion and forecast
        // radius interpolation; no production collision/projection helper is used.
        for pair in path.points.windows(2) {
            let a = pair[0];
            let b = pair[1];
            let duration = b.time - a.time;
            let length = a.position.distance(b.position);
            assert!(duration > 0.);
            let acceleration = (b.speed - a.speed) / duration;
            assert!(acceleration <= planner.max_acceleration_m_s2 + 1e-8);
            assert!(acceleration >= -planner.max_deceleration_m_s2 - 1e-8);
            assert!((0.5 * (a.speed + b.speed) * duration - length).abs() < 1e-7);
            for i in 0..=100 {
                let dt = duration * i as f64 / 100.;
                let arc = a.speed * dt + 0.5 * acceleration * dt * dt;
                let position = if length > 0. {
                    a.position.plus(b.position.minus(a.position).scaled(arc / length))
                } else { a.position };
                let knot = ((a.time + dt) / 0.2).clamp(0., 40.);
                let k = knot.floor() as usize;
                let next = (k + 1).min(40);
                let radius = 1. + objects[0].additional_radius_m[k]
                    + (objects[0].additional_radius_m[next]
                        - objects[0].additional_radius_m[k]) * (knot - k as f64);
                assert!(position.distance(Vec2::new(50., 0.))
                    >= planner.vehicle.radius + radius + crate::REFINED_CANDIDATE_CLEARANCE_M - 1e-8);
                assert!(position.y.abs() + planner.vehicle.radius <= route.half_width + 1e-8);
                assert!(position.x >= ego.pose.position.x - 0.2 && position.x <= bound.1 + 1e-8);
            }
        }
        assert_eq!(permit.endpoint_bound(), Some(bound));
        assert_eq!(planner.maneuver_start, anchor);
        assert_eq!(planner.checked_approach_diagnostics(), &status);

        // A new conflicting scene must revoke this same real adoption's permit.
        let mut blocked = objects.clone();
        blocked[0].positions = vec![ego.pose.position; 41];
        observed.now_s = 0.4;
        observed.acquisition_s = Some(0.4);
        planner.observe_checked_approach_scene(0.4, Some(0.4));
        assert!(planner.checked_maneuver_continuation(
            &mut permit, &observed, ego, &route, &blocked,
        ).is_none());
        assert!(permit.endpoint_bound().is_none());
        assert!(planner.checked_maneuver_continuation(
            &mut permit, &observed, ego, &route, &objects,
        ).is_none());
    }
}
