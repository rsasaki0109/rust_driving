use rustdriving_control::PurePursuit;
use rustdriving_core::{Controller, DrivingMode, EgoState, Planner, Prediction, Route, Vec2};
use rustdriving_planning::LatticePlanner;
use rustdriving_planning::approach_continuation::ContinuationObservation;

#[test]
fn selected_continuation_reaches_controller_as_positive_acceleration() {
    let mut planner = LatticePlanner::default();
    planner.settle_obstruction_stops = true;
    planner.search_obstruction_stops = true;
    planner.replan_checked_approaches = true;
    planner.lateral_candidate_margin_m = Some(0.35);
    let narrow = Route::new(vec![Vec2::default(), Vec2::new(100., 0.)], 2.1).unwrap();
    let mut object = Prediction {
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
            .plan(ego, &narrow, std::slice::from_ref(&object))
            .mode,
        DrivingMode::Emergency
    );
    ego.pose.position.x = 10.;
    ego.speed = 0.;
    planner.plan(ego, &narrow, std::slice::from_ref(&object));
    assert!(planner.holding_obstruction_stop());
    let route = Route::new(narrow.points.clone(), 5.5).unwrap();
    object.positions = vec![Vec2::new(50., 0.); 41];
    object.additional_radius_m = (0..=40)
        .map(|i| {
            let t = i as f64 * 0.2;
            0.2 + 0.3 * t + 0.05 * t * t
        })
        .collect();
    for now in [0., 0.1, 0.2] {
        planner.observe_checked_approach_scene(now, Some(now));
        planner.plan(ego, &route, std::slice::from_ref(&object));
    }
    let status = planner.checked_approach_diagnostics().clone();
    assert!(status.adopted);
    let mut observed = ContinuationObservation {
        now_s: 0.2,
        acquisition_s: Some(0.2),
        localization_generation: 0,
        route_generation: 0,
        blocking_ids: &status.blocking_prediction_ids,
        lateral_target_m: status.proposal_lateral_target_m.unwrap(),
        progress_m: 10.,
        healthy: true,
    };
    let mut permit = planner.take_continuation_permit(&observed, 1.).unwrap();
    observed.now_s = 0.3;
    observed.acquisition_s = Some(0.3);
    planner.observe_checked_approach_scene(0.3, Some(0.3));
    ego.speed = 0.21;
    let result = planner.plan_with_continuation(
        &mut permit,
        &observed,
        ego,
        &route,
        std::slice::from_ref(&object),
    );
    assert!(result.selected.is_some());
    assert!(!planner.braking_obstruction_stop());
    assert!(!planner.holding_obstruction_stop());
    let mut controller = PurePursuit::with_vehicle(planner.vehicle);
    let command = controller.control(ego, &result.trajectory, 0.05);
    assert!(command.acceleration > 0.);
    assert!(command.acceleration <= planner.max_acceleration_m_s2 + 1e-8);
    assert!(command.steering.is_finite());
    // The real controller still rejects a subsequent emergency trajectory.
    let mut emergency = result.trajectory;
    emergency.mode = DrivingMode::Emergency;
    assert!(controller.control(ego, &emergency, 0.05).acceleration <= -5.99);
}
