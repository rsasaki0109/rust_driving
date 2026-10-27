//! Piecewise-linear, synchronized circular sweeps for candidate trajectory validation.
use super::direction::Envelope;
use rustdriving_core::{Prediction, TrajectoryPoint, Vec2};

/// Earliest contact under the supplied time parameterization. Inputs are validated by
/// the planner. Prediction knots are linearly interpolated, then held at the last point.
/// Each subinterval uses its end-time uncertainty margin, conservatively covering the
/// increasing envelope. This does not validate the vehicle's longitudinal feasibility.
pub(super) fn first_contact_time(
    path: &[TrajectoryPoint],
    object: &Prediction,
    vehicle_radius: f64,
    held_heading: Vec2,
) -> Option<f64> {
    with_minimum_clearance(path, object, vehicle_radius, held_heading, 0.0)
}

#[derive(Clone, Copy)]
struct Safety {
    vehicle_radius: f64,
    minimum_clearance: f64,
    direction: Option<Envelope>,
}

pub(super) fn with_minimum_clearance(
    path: &[TrajectoryPoint],
    object: &Prediction,
    vehicle_radius: f64,
    held_heading: Vec2,
    minimum_clearance: f64,
) -> Option<f64> {
    with_motion_direction(
        path,
        object,
        vehicle_radius,
        held_heading,
        minimum_clearance,
        None,
    )
}

pub(super) fn with_motion_direction(
    path: &[TrajectoryPoint],
    object: &Prediction,
    vehicle_radius: f64,
    held_heading: Vec2,
    minimum_clearance: f64,
    direction: Option<Envelope>,
) -> Option<f64> {
    let safety = Safety {
        vehicle_radius,
        minimum_clearance,
        direction,
    };
    let first = path.first()?;
    let initial_tangent = path
        .windows(2)
        .map(|pair| pair[1].position.minus(pair[0].position))
        .find(|tangent| tangent.x.hypot(tangent.y) > 1e-6)
        .unwrap_or(held_heading);
    if first.position.distance(position_at(object, first.time))
        <= vehicle_radius
            + object.radius_at(first.time)
            + minimum_clearance.max(effective_margin(
                first.time,
                velocity_at(object, first.time),
                initial_tangent,
                direction,
            ))
    {
        return Some(first.time);
    }
    for pair in path.windows(2) {
        let (a, b) = (&pair[0], &pair[1]);
        let mut start = a.time;
        let first_knot = ((start / object.dt).floor() as usize).saturating_add(1);
        // A forecast may turn inside a trajectory segment: sweeping only its endpoint
        // chord would miss that turn. Visit each intervening forecast knot as well.
        for index in first_knot..object.positions.len() {
            let end = index as f64 * object.dt;
            if end >= b.time {
                break;
            }
            if end > start {
                if let Some(time) = segment_contact(a, b, object, safety, start, end, held_heading)
                {
                    return Some(time);
                }
                start = end;
            }
        }
        if let Some(time) = segment_contact(a, b, object, safety, start, b.time, held_heading) {
            return Some(time);
        }
    }
    None
}

/// Revisit the actual sweep intervals only for an offline receipt. The operational
/// collision routine above remains the authority and is not modified by inspection.
pub(super) fn inspect_contact(
    path: &[TrajectoryPoint],
    object: &Prediction,
    vehicle_radius: f64,
    held_heading: Vec2,
    minimum_clearance: f64,
    direction: Option<Envelope>,
) -> Option<super::StopContact> {
    let time = with_motion_direction(
        path,
        object,
        vehicle_radius,
        held_heading,
        minimum_clearance,
        direction,
    )?;
    let first = path.first()?;
    let initial_tangent = path
        .windows(2)
        .map(|p| p[1].position.minus(p[0].position))
        .find(|t| t.x.hypot(t.y) > 1e-6)
        .unwrap_or(held_heading);
    let receipt = |start,
                   end,
                   ego_position,
                   forecast_radius_bound_m,
                   effective_margin_m: f64,
                   acceleration_chord_bound_m| super::StopContact {
        prediction_id: object.id,
        time_s: time,
        interval_start_s: start,
        interval_end_s: end,
        ego_position,
        prediction_position: position_at(object, time),
        vehicle_radius_m: vehicle_radius,
        forecast_radius_bound_m,
        minimum_clearance_m: minimum_clearance,
        effective_margin_m,
        acceleration_chord_bound_m,
        combined_radius_m: vehicle_radius
            + forecast_radius_bound_m
            + minimum_clearance.max(effective_margin_m)
            + acceleration_chord_bound_m,
    };
    let initial_margin = effective_margin(
        first.time,
        velocity_at(object, first.time),
        initial_tangent,
        direction,
    );
    if first.position.distance(position_at(object, first.time))
        <= vehicle_radius + object.radius_at(first.time) + minimum_clearance.max(initial_margin)
    {
        return Some(receipt(
            first.time,
            first.time,
            first.position,
            object.radius_at(first.time),
            initial_margin,
            0.,
        ));
    }
    let safety = Safety {
        vehicle_radius,
        minimum_clearance,
        direction,
    };
    for pair in path.windows(2) {
        let (a, b) = (&pair[0], &pair[1]);
        let mut start = a.time;
        let first_knot = ((start / object.dt).floor() as usize).saturating_add(1);
        let ends = (first_knot..object.positions.len())
            .map(|i| i as f64 * object.dt)
            .take_while(|end| *end < b.time)
            .chain(std::iter::once(b.time));
        for end in ends {
            if end <= start {
                continue;
            }
            if segment_contact(a, b, object, safety, start, end, held_heading).is_some() {
                let fraction = if b.time > a.time {
                    (time - a.time) / (b.time - a.time)
                } else {
                    0.
                };
                let displacement = b.position.minus(a.position);
                let tangent = if displacement.x.hypot(displacement.y) > 1e-6 {
                    displacement
                } else {
                    held_heading
                };
                return Some(receipt(
                    start,
                    end,
                    a.position.plus(displacement.scaled(fraction)),
                    object.radius_at(start).max(object.radius_at(end)),
                    effective_margin(
                        end,
                        velocity_at(object, 0.5 * start + 0.5 * end),
                        tangent,
                        direction,
                    ),
                    (b.speed - a.speed).abs() * (b.time - a.time) / 8.,
                ));
            }
            start = end;
        }
    }
    unreachable!("a reported operational contact must have a sweep interval")
}

fn effective_margin(time: f64, velocity: Vec2, tangent: Vec2, direction: Option<Envelope>) -> f64 {
    let legacy = margin(time, velocity, tangent);
    direction.map_or(legacy, |model| {
        legacy.max(0.30 + 0.06 * time.min(5.0) + model.reserve(time, tangent))
    })
}

fn margin(time: f64, velocity: Vec2, tangent: Vec2) -> f64 {
    // Moving across the candidate needs more empirical clearance than static or
    // parallel traffic. This reserve is not a certified prediction-error bound.
    // The transverse reserve follows direction, rather than vanishing with a
    // slow crossing's speed. Perpendicular motion retains 1 m at time zero.
    let fraction = transverse_fraction(velocity, tangent);
    let transverse_speed = velocity.x.hypot(velocity.y) * fraction;
    let motion_reserve = (0.40 * transverse_speed.min(1.)).max(0.70 * fraction);
    0.30 + 0.06 * time.min(5.0) + motion_reserve
}

fn transverse_fraction(velocity: Vec2, tangent: Vec2) -> f64 {
    let speed = velocity.x.hypot(velocity.y);
    if speed == 0. {
        return 0.;
    }
    let length = tangent.x.hypot(tangent.y);
    if length > 1e-6 {
        ((velocity.x / speed) * (tangent.y / length) - (velocity.y / speed) * (tangent.x / length))
            .abs()
            .min(1.)
    } else {
        // A stationary hold has no segment heading. Reserve for any observed
        // motion rather than choosing an arbitrary world-frame axis.
        1.
    }
}

fn velocity_at(object: &Prediction, time: f64) -> Vec2 {
    let index = (time / object.dt).floor() as usize;
    if index >= object.positions.len() - 1 {
        Vec2::default()
    } else {
        object.positions[index + 1]
            .minus(object.positions[index])
            .scaled(1.0 / object.dt)
    }
}

fn position_at(object: &Prediction, time: f64) -> Vec2 {
    let step = time / object.dt;
    let index = (step.floor() as usize).min(object.positions.len() - 1);
    let next = (index + 1).min(object.positions.len() - 1);
    object.positions[index].plus(
        object.positions[next]
            .minus(object.positions[index])
            .scaled((step - index as f64).clamp(0.0, 1.0)),
    )
}

fn segment_contact(
    a: &TrajectoryPoint,
    b: &TrajectoryPoint,
    object: &Prediction,
    safety: Safety,
    start: f64,
    end: f64,
    held_heading: Vec2,
) -> Option<f64> {
    let ego_at = |time| {
        let fraction = if b.time > a.time {
            (time - a.time) / (b.time - a.time)
        } else {
            0.0
        };
        a.position
            .plus(b.position.minus(a.position).scaled(fraction))
    };
    let relative_start = ego_at(start).minus(position_at(object, start));
    let relative_end = ego_at(end).minus(position_at(object, end));
    let displacement = b.position.minus(a.position);
    let tangent = if displacement.x.hypot(displacement.y) > 1e-6 {
        displacement
    } else {
        held_heading
    };
    contact_fraction(
        relative_start,
        relative_end,
        // Constant acceleration along a segment differs from its temporal chord
        // by at most |delta_v| * duration / 8. Inflate the circle by this bound.
        safety.vehicle_radius
            + object.radius_at(start).max(object.radius_at(end))
            + safety.minimum_clearance.max(effective_margin(
                end,
                velocity_at(object, 0.5 * start + 0.5 * end),
                tangent,
                safety.direction,
            ))
            + (b.speed - a.speed).abs() * (b.time - a.time) / 8.0,
    )
    .map(|fraction| start + (end - start) * fraction)
}

/// Solve |start + u * delta| <= radius for the earliest u in [0, 1].
fn contact_fraction(start: Vec2, end: Vec2, radius: f64) -> Option<f64> {
    let c = start.x * start.x + start.y * start.y - radius * radius;
    if c <= 0.0 {
        return Some(0.0);
    }
    let delta = end.minus(start);
    let a = delta.x * delta.x + delta.y * delta.y;
    let b = start.x * delta.x + start.y * delta.y;
    if a == 0.0 || b >= 0.0 {
        return None;
    }
    let discriminant = b * b - a * c;
    if discriminant < 0.0 {
        return None;
    }
    // Equivalent to (-b - sqrt(discriminant)) / a, avoiding cancellation
    // when the start is close to the envelope boundary.
    let fraction = c / (-b + discriminant.sqrt());
    (0.0..=1.0).contains(&fraction).then_some(fraction)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn offline_receipt_matches_analytic_entry_and_conservative_hold() {
        let mut ego = path(Vec2::default(), Vec2::new(10., 0.), [0., 1.]);
        ego[0].speed = 10.;
        ego[1].speed = 10.;
        let object = Prediction {
            id: 1,
            radius: 0.5,
            positions: vec![Vec2::new(5., 0.)],
            dt: 1.,
            additional_radius_m: vec![],
        };
        let c = inspect_contact(&ego, &object, 0.5, Vec2::new(1., 0.), 1., None).unwrap();
        assert!((c.time_s - 0.3).abs() < 1e-12);
        assert!((c.ego_position.x - 3.).abs() < 1e-12);
        assert_eq!(c.combined_radius_m, 2.);
        assert_eq!(c.acceleration_chord_bound_m, 0.);
        assert_eq!(
            c.time_s,
            with_minimum_clearance(&ego, &object, 0.5, Vec2::new(1., 0.), 1.).unwrap()
        );
        ego[1].speed = 8.;
        let curved_time = inspect_contact(&ego, &object, 0.5, Vec2::new(1., 0.), 1., None).unwrap();
        assert_eq!(curved_time.acceleration_chord_bound_m, 0.25);
        assert!((curved_time.time_s - 0.275).abs() < 1e-12);
        let overlap = Prediction {
            positions: vec![Vec2::default()],
            ..object.clone()
        };
        let initial = inspect_contact(&ego, &overlap, 0.5, Vec2::new(1., 0.), 1., None).unwrap();
        assert_eq!(initial.time_s, 0.);
        assert_eq!(initial.interval_end_s, 0.);
        let hold = path(Vec2::default(), Vec2::default(), [0., 8.]);
        let growing = Prediction {
            positions: vec![Vec2::new(5., 0.); 3],
            dt: 4.,
            additional_radius_m: vec![0., 2., 4.],
            ..object
        };
        let c = inspect_contact(&hold, &growing, 1., Vec2::new(1., 0.), 0., None).unwrap();
        assert_eq!(c.time_s, 4.);
        assert_eq!(c.interval_start_s, 4.);
        assert_eq!(c.interval_end_s, 8.);
        assert_eq!(c.forecast_radius_bound_m, 4.5);
        assert!((c.combined_radius_m - 6.1).abs() < 1e-12);
        assert_eq!(c.ego_position, Vec2::default());
        let far = Prediction {
            positions: vec![Vec2::new(100., 0.)],
            additional_radius_m: vec![],
            ..growing
        };
        assert!(inspect_contact(&hold, &far, 1., Vec2::new(1., 0.), 0., None).is_none());
    }

    #[test]
    fn directional_context_never_removes_a_nominal_crossing_reserve() {
        use super::super::direction::Context;
        use rustdriving_core::{ForecastEnvelopeConfig, Track};
        let context = Context::observe(
            ForecastEnvelopeConfig {
                position_error_m: 0.,
                velocity_error_m_s: 0.,
                acceleration_error_m_s2: 0.,
            },
            &[Track {
                id: 1,
                position: Vec2::default(),
                velocity: Vec2::default(),
                radius: 0.5,
                last_seen: 0.,
            }],
            0.,
        );
        let ego = path(Vec2::default(), Vec2::default(), [0., 1.]);
        let object = Prediction {
            id: 1,
            radius: 0.5,
            positions: vec![Vec2::new(1.8, 0.), Vec2::new(1.8, 0.01)],
            dt: 1.,
            additional_radius_m: vec![],
        };
        let heading = Vec2::new(1., 0.);
        assert_eq!(first_contact_time(&ego, &object, 1., heading), Some(0.));
        assert_eq!(
            with_motion_direction(
                &ego,
                &object,
                1.,
                heading,
                0.,
                Some(context.for_prediction(1))
            ),
            Some(0.)
        );
    }

    #[test]
    fn an_explicit_floor_rejects_a_legacy_near_miss_and_keeps_larger_motion_reserves() {
        let path = [
            TrajectoryPoint {
                position: Vec2::new(0., 0.),
                speed: 5.,
                time: 0.,
            },
            TrajectoryPoint {
                position: Vec2::new(20., 0.),
                speed: 5.,
                time: 4.,
            },
        ];
        let object = Prediction {
            id: 1,
            positions: vec![Vec2::new(10., 2.1); 41],
            radius: 0.5,
            additional_radius_m: vec![],
            dt: 0.2,
        };
        assert!(first_contact_time(&path, &object, 1., Vec2::new(1., 0.)).is_none());
        assert!(with_minimum_clearance(&path, &object, 1., Vec2::new(1., 0.), 1.).is_some());
        let hold = [
            TrajectoryPoint {
                position: Vec2::default(),
                speed: 0.,
                time: 0.,
            },
            TrajectoryPoint {
                position: Vec2::default(),
                speed: 0.,
                time: 8.,
            },
        ];
        let crossing = Prediction {
            positions: (0..=40)
                .map(|i| Vec2::new(2.6, 4. - i as f64 * 0.2))
                .collect(),
            ..object
        };
        assert!(with_minimum_clearance(&hold, &crossing, 1., Vec2::new(1., 0.), 1.).is_some());
    }

    #[test]
    fn the_clearance_floor_covers_an_interior_radius_peak_in_the_full_hold() {
        let hold = path(Vec2::default(), Vec2::default(), [0., 8.]);
        let object = Prediction {
            id: 1,
            positions: vec![Vec2::new(4., 0.); 3],
            radius: 0.5,
            additional_radius_m: vec![0., 2., 0.],
            dt: 4.,
        };
        assert!(with_minimum_clearance(&hold, &object, 1., Vec2::new(1., 0.), 1.).is_some());
    }

    fn path(start: Vec2, end: Vec2, times: [f64; 2]) -> [TrajectoryPoint; 2] {
        [
            TrajectoryPoint {
                position: start,
                speed: 0.0,
                time: times[0],
            },
            TrajectoryPoint {
                position: end,
                speed: 0.0,
                time: times[1],
            },
        ]
    }

    #[test]
    fn radius_growth_and_an_interior_peak_block_a_stationary_hold() {
        let hold = path(Vec2::default(), Vec2::default(), [0., 8.]);
        let mut object = Prediction {
            id: 9,
            positions: vec![Vec2::new(5., 0.); 3],
            radius: 0.5,
            additional_radius_m: vec![],
            dt: 4.,
        };
        assert_eq!(
            first_contact_time(&hold, &object, 1., Vec2::new(1., 0.)),
            None
        );
        for envelope in [vec![0., 2., 4.], vec![0., 4., 0.]] {
            object.additional_radius_m = envelope;
            assert!(first_contact_time(&hold, &object, 1., Vec2::new(1., 0.)).is_some());
        }
    }

    #[test]
    fn analytic_entry_tangency_overlap_and_miss() {
        assert_eq!(
            contact_fraction(Vec2::new(-5.0, 0.0), Vec2::new(5.0, 0.0), 1.0),
            Some(0.4)
        );
        assert_eq!(
            contact_fraction(Vec2::new(-5.0, 1.0), Vec2::new(5.0, 1.0), 1.0),
            Some(0.5)
        );
        assert_eq!(
            contact_fraction(Vec2::default(), Vec2::new(5.0, 0.0), 1.0),
            Some(0.0)
        );
        assert_eq!(
            contact_fraction(Vec2::new(-5.0, 2.0), Vec2::new(5.0, 2.0), 1.0),
            None
        );
        assert_eq!(
            contact_fraction(Vec2::new(2.0, 0.0), Vec2::new(3.0, 0.0), 1.0),
            None
        );
    }

    #[test]
    fn synchronized_crossing_detected_between_clear_endpoints() {
        let ego = path(Vec2::default(), Vec2::new(10.0, 0.0), [0.0, 1.0]);
        let object = Prediction {
            additional_radius_m: vec![],
            id: 1,
            positions: vec![Vec2::new(5.0, 5.0), Vec2::new(5.0, -5.0)],
            radius: 0.5,
            dt: 1.0,
        };
        assert!(ego[0].position.distance(object.positions[0]) > 2.06);
        assert!(ego[1].position.distance(object.positions[1]) > 2.06);
        let time = first_contact_time(&ego, &object, 0.5, Vec2::default()).unwrap();
        assert!((time - (0.5 - 2.06 / 200.0_f64.sqrt())).abs() < 1e-12);
    }

    #[test]
    fn parallel_and_static_traffic_keep_clearance_in_rotated_frames() {
        for rotated in [false, true] {
            let rotate = |p: Vec2| {
                if rotated { Vec2::new(-p.y, p.x) } else { p }
            };
            let ego = path(
                rotate(Vec2::default()),
                rotate(Vec2::new(10.0, 0.0)),
                [0.0, 1.0],
            );
            let parallel = Prediction {
                additional_radius_m: vec![],
                id: 1,
                positions: vec![rotate(Vec2::new(0.0, 1.5)), rotate(Vec2::new(10.0, 1.5))],
                radius: 0.5,
                dt: 1.0,
            };
            assert!(first_contact_time(&ego, &parallel, 0.5, Vec2::default()).is_none());
            let stationary = Prediction {
                additional_radius_m: vec![],
                id: 2,
                positions: vec![rotate(Vec2::new(5.0, 1.5))],
                radius: 0.5,
                dt: 1.0,
            };
            assert!(first_contact_time(&ego, &stationary, 0.5, Vec2::default()).is_none());
        }
    }

    #[test]
    fn slow_crossing_motion_retains_a_meter_of_directional_reserve() {
        for angle in [0., 0.7, 2.4] {
            let ego = path(Vec2::default(), Vec2::default(), [0., 1.]);
            let heading = Vec2::new(1., 0.).rotated(angle);
            for speed in [0.1, 0.2, 0.5, 2.5] {
                let crossing = Prediction {
                    additional_radius_m: vec![],
                    id: 1,
                    positions: vec![
                        Vec2::new(1.8, 0.).rotated(angle),
                        Vec2::new(1.8, speed).rotated(angle),
                    ],
                    radius: 0.5,
                    dt: 1.,
                };
                // The physical gap is 0.8 m at time zero. Measured transverse
                // motion needs the same geometric reserve at either speed.
                assert_eq!(first_contact_time(&ego, &crossing, 0.5, heading), Some(0.));
                let parallel = Prediction {
                    positions: vec![
                        Vec2::new(0., 1.8).rotated(angle),
                        Vec2::new(speed, 1.8).rotated(angle),
                    ],
                    ..crossing
                };
                assert!(first_contact_time(&ego, &parallel, 0.5, heading).is_none());
            }
        }
    }

    #[test]
    fn fast_almost_parallel_motion_keeps_the_previous_transverse_speed_reserve() {
        let ego = path(Vec2::default(), Vec2::default(), [0., 1.]);
        let object = Prediction {
            additional_radius_m: vec![],
            id: 1,
            positions: vec![Vec2::new(1.48, 0.), Vec2::new(9.48, 0.5)],
            radius: 0.5,
            dt: 1.,
        };
        // The earlier 0.5 m reserve already rejects this 0.48 m initial gap.
        // A small crossing angle cannot narrow that existing speed envelope.
        assert_eq!(
            first_contact_time(&ego, &object, 0.5, Vec2::new(1., 0.)),
            Some(0.)
        );
    }

    #[test]
    fn stationary_hold_reserves_for_motion_without_an_arbitrary_heading() {
        let ego = path(Vec2::default(), Vec2::default(), [0.0, 1.0]);
        let moving = Prediction {
            additional_radius_m: vec![],
            id: 1,
            positions: vec![Vec2::new(1.5, 5.0), Vec2::new(1.5, -5.0)],
            radius: 0.5,
            dt: 1.0,
        };
        // Actual circle separation is 0.5 m at the closest point: the baseline
        // static reserve permits it, while a moving-object hold reserve rejects it.
        assert!(first_contact_time(&ego, &moving, 0.5, Vec2::default()).is_some());
        let stationary = Prediction {
            positions: vec![Vec2::new(1.5, 0.0)],
            ..moving
        };
        assert!(first_contact_time(&ego, &stationary, 0.5, Vec2::default()).is_none());
    }

    #[test]
    fn known_held_heading_preserves_parallel_clearance_and_crossing_reserve() {
        for rotated in [false, true] {
            let rotate = |p: Vec2| if rotated { Vec2::new(-p.y, p.x) } else { p };
            let ego = path(Vec2::default(), Vec2::default(), [0.0, 1.0]);
            let heading = rotate(Vec2::new(1.0, 0.0));
            let parallel = Prediction {
                additional_radius_m: vec![],
                id: 1,
                positions: vec![rotate(Vec2::new(-5.0, 1.5)), rotate(Vec2::new(5.0, 1.5))],
                radius: 0.5,
                dt: 1.0,
            };
            assert!(first_contact_time(&ego, &parallel, 0.5, heading).is_none());
            assert!(first_contact_time(&ego, &parallel, 0.5, Vec2::default()).is_some());
            let crossing = Prediction {
                positions: vec![rotate(Vec2::new(1.5, -5.0)), rotate(Vec2::new(1.5, 5.0))],
                ..parallel
            };
            assert!(first_contact_time(&ego, &crossing, 0.5, heading).is_some());
        }
    }

    #[test]
    fn geometric_crossing_at_different_times_is_clear() {
        let ego = path(Vec2::default(), Vec2::new(10.0, 0.0), [0.0, 1.0]);
        let object = Prediction {
            additional_radius_m: vec![],
            id: 1,
            positions: vec![Vec2::new(5.0, 5.0), Vec2::new(5.0, 0.0)],
            radius: 0.5,
            dt: 1.0,
        };
        assert!(first_contact_time(&ego, &object, 0.5, Vec2::default()).is_none());
    }

    #[test]
    fn intermediate_prediction_turn_is_not_replaced_with_an_endpoint_chord() {
        let ego = path(Vec2::default(), Vec2::default(), [0.0, 1.0]);
        let object = Prediction {
            additional_radius_m: vec![],
            id: 1,
            positions: vec![Vec2::new(-5.0, 2.0), Vec2::default(), Vec2::new(-5.0, 2.0)],
            radius: 0.5,
            dt: 0.5,
        };
        assert!(first_contact_time(&ego, &object, 0.2, Vec2::default()).unwrap() < 0.5);
    }

    #[test]
    fn terminal_prediction_and_single_point_obstacles_remain_occupied() {
        let ego = path(Vec2::default(), Vec2::new(10.0, 0.0), [2.0, 3.0]);
        let mut object = Prediction {
            additional_radius_m: vec![],
            id: 1,
            positions: vec![Vec2::new(5.0, 5.0), Vec2::new(5.0, 0.0)],
            radius: 0.5,
            dt: 0.5,
        };
        assert!(first_contact_time(&ego, &object, 0.2, Vec2::default()).is_some());
        object.positions = vec![Vec2::new(5.0, 0.0)];
        assert!(first_contact_time(&ego, &object, 0.2, Vec2::default()).is_some());
    }
}
