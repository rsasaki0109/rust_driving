//! Experimental stopping-profile construction; not connected to the driving policy.
//! A caller must separately own fresh approach authorization and supply the full
//! current-scene collision/road certificate. A returned profile grants no authority.
use crate::speed;
use rustdriving_core::{TrajectoryPoint, Vec2};

/// Explicit bounds for one already-authorized approach corridor.
pub struct ContinuationBounds {
    pub maximum_distance_m: f64,
    pub cruise_m_s: f64,
    pub acceleration_m_s2: f64,
    pub deceleration_m_s2: f64,
}

/// Search at most sixteen stops without the ordinary fallback's current-speed
/// cruise cap. Every offered profile includes its reachable braking and complete
/// eight-second stationary tail. The callback must certify the entire profile.
/// This helper does not qualify geometry, route speed zones, sensor freshness or
/// authorization; it is deliberately not invoked by the live planner.
pub fn stopped_profile(
    geometry: &[Vec2],
    initial_speed: f64,
    bounds: &ContinuationBounds,
    mut certify: impl FnMut(&[TrajectoryPoint]) -> bool,
) -> Option<Vec<TrajectoryPoint>> {
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
    let available = geometry.windows(2).map(|p| p[0].distance(p[1])).sum::<f64>();
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
        zones: None,
        acceleration: bounds.acceleration_m_s2,
        deceleration: bounds.deceleration_m_s2,
        lateral: None,
        cruise: bounds.cruise_m_s,
    };
    for i in 0..if minimum == bounds.maximum_distance_m { 1 } else { 16 } {
        let distance = bounds.maximum_distance_m
            + (minimum - bounds.maximum_distance_m) * (i as f64 / 15.);
        let Some(path) = speed::profile(geometry, initial_speed, &limits, Some(distance)) else {
            continue;
        };
        let Some(tail) = path.windows(2).next_back() else {
            continue;
        };
        let travelled = path.windows(2).map(|p| p[0].position.distance(p[1].position)).sum::<f64>();
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
        let tail = &path[path.len()-2..];
        assert_eq!(tail[0].position, tail[1].position);
        assert_eq!(tail[0].speed, 0.);
        assert_eq!(tail[1].speed, 0.);
        assert!(tail[1].time-tail[0].time >= 8.-1e-9);
        assert!(tail[1].position.x <= b.maximum_distance_m+1e-8);
    }

    #[test]
    fn crossing_point_two_does_not_silently_cancel_reacceleration() {
        for initial in [0.19, 0.2, 0.21, 0.31] {
            let b = bounds();
            let path = stopped_profile(&geometry(), initial, &b, |p| {
                verify(p, initial, &b);
                true
            }).unwrap();
            assert!(path.iter().any(|p| p.speed > initial+0.5));
        }
    }

    #[test]
    fn rejection_never_returns_an_uncertified_path_or_shortens_the_hold() {
        let b = bounds();
        let mut count = 0;
        assert!(stopped_profile(&geometry(), 0.31, &b, |p| {
            count += 1;
            verify(p, 0.31, &b);
            false
        }).is_none());
        assert!((1..=16).contains(&count));
    }

    #[test]
    fn full_tail_collision_rejects_all_stopping_candidates() {
        use rustdriving_core::Prediction;
        let moving = Prediction {
            id: 1,
            positions: (0..=40).map(|i| Vec2::new(0.75, 6.-i as f64*0.2)).collect(),
            radius: 2.,
            additional_radius_m: vec![],
            dt: 0.2,
        };
        // The crossing obstacle reaches every allowed stopped endpoint during
        // the hold; an endpoint-at-arrival-only test would miss that interval.
        let mut checked = 0;
        assert!(stopped_profile(&geometry(), 0.21, &bounds(), |p| {
            checked += 1;
            crate::collision::first_contact_time(p, &moving, 1.25, Vec2::new(1.,0.)).is_none()
        }).is_none());
        assert!(checked > 0);
    }

    #[test]
    fn unreachable_or_malformed_input_never_reaches_certificate() {
        let mut calls = 0;
        for value in [f64::NAN, f64::INFINITY, -1., 0., 5.] {
            let mut b = bounds();
            b.maximum_distance_m = value;
            assert!(stopped_profile(&geometry(), 0.31, &b, |_| {calls+=1;true}).is_none());
        }
        let mut b = bounds();
        b.maximum_distance_m = 0.001;
        assert!(stopped_profile(&geometry(), 2., &b, |_| {calls+=1;true}).is_none());
        assert_eq!(calls, 0);
    }
}
