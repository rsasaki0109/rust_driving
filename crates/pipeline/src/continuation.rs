//! Opt-in ownership of checked continuation state in the sensor pipeline.
use rustdriving_core::{EgoState, Planner, Prediction, Route, Trajectory, Vec2};
use rustdriving_planning::LatticePlanner;
use rustdriving_planning::approach_continuation::{
    ContinuationObservation, ContinuationPermit, ContinuationSelection,
};
use serde::{Deserialize, Serialize};

/// Planner selection evidence; subsequent pipeline guards may still brake.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ContinuationDiagnostic {
    pub now_s: f64,
    pub acquisition_s: f64,
    pub expires_s: f64,
    pub original_endpoint: Vec2,
    pub original_end_progress_m: f64,
    pub selected_end_progress_m: f64,
    pub lateral_target_m: f64,
    pub blocking_ids: Vec<u64>,
    pub previous_stop_requested: bool,
    pub previous_stop_active: bool,
}
impl From<ContinuationSelection> for ContinuationDiagnostic {
    fn from(s: ContinuationSelection) -> Self {
        Self {
            now_s: s.now_s,
            acquisition_s: s.acquisition_s,
            expires_s: s.expires_s,
            original_endpoint: s.original_endpoint,
            original_end_progress_m: s.original_end_progress_m,
            selected_end_progress_m: s.selected_end_progress_m,
            lateral_target_m: s.lateral_target_m,
            blocking_ids: s.blocking_ids,
            previous_stop_requested: s.previous_stop_requested,
            previous_stop_active: s.previous_stop_active,
        }
    }
}
struct Active {
    permit: ContinuationPermit,
    route: Route,
    epoch: u64,
    ids: Vec<u64>,
    target: f64,
}
#[derive(Default)]
pub(super) struct ContinuationDriver {
    active: Option<Active>,
}
pub(super) struct Context<'a> {
    pub enabled: bool,
    pub eligible: bool,
    pub now: f64,
    pub epoch: u64,
    pub ego: EgoState,
    pub route: &'a Route,
    pub objects: &'a [Prediction],
}
impl ContinuationDriver {
    pub fn clear(&mut self) {
        self.active = None;
    }

    pub fn plan(
        &mut self,
        planner: &mut LatticePlanner,
        context: Context<'_>,
    ) -> (Trajectory, Option<ContinuationDiagnostic>) {
        let Context {
            enabled,
            eligible,
            now,
            epoch,
            ego,
            route,
            objects,
        } = context;
        if !enabled || !eligible {
            self.clear();
            return (planner.plan(ego, route, objects), None);
        }
        if self.active.as_ref().is_some_and(|a| {
            a.epoch != epoch
                || a.route.points != route.points
                || a.route.half_width != route.half_width
                || a.route.lengths != route.lengths
        }) {
            self.clear();
        }
        let acquisition = planner
            .checked_approach_scene_clock()
            .filter(|(time, _)| *time == now)
            .and_then(|(_, stamp)| stamp);
        let progress = route.project(ego.pose.position).0;
        let (trajectory, selected) = if let Some(active) = self.active.as_mut() {
            let observed = ContinuationObservation {
                now_s: now,
                acquisition_s: acquisition,
                localization_generation: epoch,
                route_generation: 0,
                blocking_ids: &active.ids,
                lateral_target_m: active.target,
                progress_m: progress,
                healthy: eligible,
            };
            let result =
                planner.plan_with_continuation(&mut active.permit, &observed, ego, route, objects);
            (
                result.trajectory,
                result.selected.map(ContinuationDiagnostic::from),
            )
        } else {
            (planner.plan(ego, route, objects), None)
        };
        if self
            .active
            .as_ref()
            .is_some_and(|a| a.permit.endpoint_bound().is_none())
        {
            self.clear();
        }
        let record = planner.checked_approach_diagnostics().clone();
        if record.adopted {
            let target = record
                .proposal_lateral_target_m
                .expect("actual adopted target");
            let observed = ContinuationObservation {
                now_s: now,
                acquisition_s: acquisition,
                localization_generation: epoch,
                route_generation: 0,
                blocking_ids: &record.blocking_prediction_ids,
                lateral_target_m: target,
                progress_m: progress,
                healthy: eligible,
            };
            self.active = planner
                .take_continuation_permit(&observed, 8.)
                .map(|permit| Active {
                    permit,
                    route: route.clone(),
                    epoch,
                    ids: record.blocking_prediction_ids,
                    target,
                });
        }
        (trajectory, selected)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rustdriving_core::DrivingMode;

    fn adopted() -> (
        ContinuationDriver,
        LatticePlanner,
        Route,
        EgoState,
        Vec<Prediction>,
    ) {
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
        let route = Route::new(narrow.points.clone(), 5.5).unwrap();
        object.positions = vec![Vec2::new(50., 0.); 41];
        object.additional_radius_m = (0..=40)
            .map(|i| {
                let t = i as f64 * 0.2;
                0.2 + 0.3 * t + 0.05 * t * t
            })
            .collect();
        let objects = vec![object];
        let mut driver = ContinuationDriver::default();
        for now in [0., 0.1, 0.2] {
            planner.observe_checked_approach_scene(now, Some(now));
            let (_, selected) = driver.plan(
                &mut planner,
                Context {
                    enabled: true,
                    eligible: true,
                    now,
                    epoch: 0,
                    ego,
                    route: &route,
                    objects: &objects,
                },
            );
            assert!(selected.is_none());
        }
        assert!(driver.active.is_some());
        (driver, planner, route, ego, objects)
    }

    #[test]
    fn pipeline_owner_selects_on_later_ticks_and_preserves_deadline() {
        let (mut driver, mut planner, route, mut ego, objects) = adopted();
        ego.speed = 0.21;
        for now in [0.3, 0.4] {
            planner.observe_checked_approach_scene(now, Some(now));
            let (path, selected) = driver.plan(
                &mut planner,
                Context {
                    enabled: true,
                    eligible: true,
                    now,
                    epoch: 0,
                    ego,
                    route: &route,
                    objects: &objects,
                },
            );
            let selected = selected.unwrap();
            assert_eq!(selected.now_s, now);
            assert_eq!(selected.expires_s, 8.2);
            assert!(path.points.iter().any(|p| p.speed > 0.7));
            assert!(!planner.braking_obstruction_stop());
            assert!(!planner.holding_obstruction_stop());
            let encoded = serde_json::to_string(&selected).unwrap();
            let decoded: ContinuationDiagnostic = serde_json::from_str(&encoded).unwrap();
            assert_eq!(decoded.original_endpoint, selected.original_endpoint);
            assert_eq!(decoded.expires_s, selected.expires_s);
        }
    }

    #[test]
    fn pipeline_owner_withdraws_on_health_epoch_route_scene_or_policy_change() {
        for case in 0..5 {
            let (mut driver, mut planner, mut route, mut ego, objects) = adopted();
            ego.speed = 0.21;
            if case == 2 {
                route.half_width += 0.1;
            }
            planner.observe_checked_approach_scene(0.3, if case == 3 { None } else { Some(0.3) });
            let (_, selected) = driver.plan(
                &mut planner,
                Context {
                    enabled: case != 4,
                    eligible: case != 0,
                    now: 0.3,
                    epoch: u64::from(case == 1),
                    ego,
                    route: &route,
                    objects: &objects,
                },
            );
            assert!(selected.is_none());
            assert!(driver.active.is_none());
            planner.observe_checked_approach_scene(0.4, Some(0.4));
            let (_, selected) = driver.plan(
                &mut planner,
                Context {
                    enabled: true,
                    eligible: true,
                    now: 0.4,
                    epoch: 0,
                    ego,
                    route: &route,
                    objects: &objects,
                },
            );
            assert!(selected.is_none());
        }
    }
}
