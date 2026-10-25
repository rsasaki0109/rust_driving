//! Acquisition-bounded progress followed by a fully checked braking contingency.
use super::speed::{self, Limits};
use rustdriving_core::{MotionPrefixDiagnostics, TrajectoryPoint, Vec2};

pub(super) const PREFIX_S: f64 = 0.1;

#[derive(Clone, Default)]
pub(super) struct Support {
    consumed: Option<f64>,
    control: Option<f64>,
    first: Option<f64>,
    last: Option<f64>,
    count: u8,
    until: Option<f64>,
    target: Option<f64>,
    ids: Vec<u64>,
}
impl Support {
    pub fn withdraw(&mut self, now: Option<f64>) {
        if let Some(now) = now.filter(|t| t.is_finite() && *t >= 0.) {
            self.consumed = Some(self.consumed.map_or(now, |old| old.max(now)));
        }
        self.first = None;
        self.last = None;
        self.count = 0;
        self.until = None;
        self.target = None;
        self.ids.clear();
    }
    pub fn span(&self, clock: Option<(f64, Option<f64>)>) -> Option<f64> {
        let (now, Some(scene)) = clock? else { return None };
        if !now.is_finite() || !scene.is_finite() || scene < 0. || scene > now
            || self.control.is_some_and(|old| now < old)
            || self.consumed.is_some_and(|old| scene < old) {
            return None;
        }
        let remaining = (scene + PREFIX_S - now).min(PREFIX_S);
        (remaining > 1e-8).then_some(remaining)
    }
    pub fn observe(&mut self, clock: Option<(f64, Option<f64>)>,
        target: f64, ids: &[u64], feasible: bool) -> Option<f64> {
        let span = self.span(clock);
        let now = clock.map(|c| c.0);
        let Some(remaining) = span else { self.withdraw(now); return None };
        let (now, scene) = clock.expect("validated clock");
        let scene = scene.expect("validated acquisition");
        self.control = Some(now);
        let fresh = self.consumed.is_none_or(|old| scene > old);
        if fresh { self.consumed = Some(scene); }
        if !feasible || !target.is_finite() || ids.is_empty() || ids.len() > 128
            || ids.windows(2).any(|p| p[0] >= p[1]) {
            self.withdraw(Some(now));
            return None;
        }
        let same = self.target.is_some_and(|old| (old - target).abs() <= 0.01)
            && self.ids == ids
            && self.last.is_some_and(|old| scene - old <= 0.35 + 1e-9);
        if !same {
            self.first = None;
            self.last = None;
            self.count = 0;
            self.until = None;
            if !fresh { self.withdraw(Some(now)); return None; }
        }
        if fresh {
            self.first.get_or_insert(scene);
            self.last = Some(scene);
            self.count = self.count.saturating_add(1).min(3);
            self.target = Some(target);
            self.ids = ids.to_vec();
            if self.count >= 3 && scene - self.first.expect("first support") >= 0.2 - 1e-9 {
                self.until = Some(scene + PREFIX_S);
            }
        }
        self.until.filter(|until| *until - now > 1e-8).map(|_| remaining)
    }
    pub fn diagnostics(&self, span: f64, control_interval: f64) -> MotionPrefixDiagnostics {
        MotionPrefixDiagnostics {
            selected: true, acquisition_count: self.count,
            first_acquisition_s: self.first, last_acquisition_s: self.last,
            authorized_until_s: self.until, prefix_duration_s: span, control_interval_s: control_interval,
            lateral_target_m: self.target, blocking_prediction_ids: self.ids.clone(),
            shared_pose_cancellation: false,
        }
    }
}

/// Follow at most the owned acquisition prefix, then use the supplied hard
/// braking authority. Every returned segment has exact acceleration-distance
/// integration and an eight-second stationary tail. Collision checking belongs
/// to the caller and covers this entire result, including the prefix and hold.
pub(super) fn profile(geometry: &[Vec2], nominal: &[TrajectoryPoint],
    limits: &Limits<'_>, span: f64) -> Option<Vec<TrajectoryPoint>> {
    if geometry.len() < 2 || geometry.len() > 81 || nominal.len() < 2
        || !span.is_finite() || span < 0. || (span > 0. && span <= 1e-8) || span > PREFIX_S + 1e-9
        || !limits.deceleration.is_finite() || limits.deceleration <= 0. {
        return None;
    }
    let mut prefix = vec![*nominal.first()?];
    if prefix[0].time != 0. { return None; }
    let mut endpoint = (span == 0.).then_some(prefix[0]);
    for pair in nominal.windows(2) {
        if span == 0. { break; }
        let (a, b) = (pair[0], pair[1]);
        if b.time < span - 1e-12 {
            prefix.push(b);
            continue;
        }
        let duration = b.time - a.time;
        if !duration.is_finite() || duration <= 0. { return None; }
        let elapsed = span - a.time;
        if elapsed < 0. { return None; }
        let acceleration = (b.speed - a.speed) / duration;
        let length = a.position.distance(b.position);
        let advance = a.speed * elapsed + 0.5 * acceleration * elapsed * elapsed;
        if length <= 1e-12 || !advance.is_finite() { return None; }
        let speed = a.speed + acceleration * elapsed;
        if !speed.is_finite() || speed < 0. { return None; }
        endpoint = Some(TrajectoryPoint {
            position: a.position.plus(b.position.minus(a.position).scaled((advance / length).clamp(0., 1.))),
            speed, time: span,
        });
        break;
    }
    let endpoint = endpoint?;
    let traveled = speed::distance_at(nominal, span);
    if !traveled.is_finite() || (span > 0. && (traveled <= 1e-8 || endpoint.speed <= 1e-8)) { return None; }
    if span == 0. && endpoint.speed <= 1e-8 {
        return Some(vec![endpoint, TrajectoryPoint { time: 8., speed: 0., ..endpoint }]);
    }
    if span > 0. { prefix.push(endpoint); }
    let mut tail_geometry = vec![endpoint.position];
    let mut distance = 0.;
    for pair in geometry.windows(2) {
        distance += pair[0].distance(pair[1]);
        if distance > traveled + 1e-9 { tail_geometry.push(pair[1]); }
    }
    // A micrometer avoids demanding a numerically negative terminal velocity.
    let stop = endpoint.speed.powi(2) / (2. * limits.deceleration) + 1e-6;
    let available: f64 = tail_geometry.windows(2).map(|p| p[0].distance(p[1])).sum();
    if !stop.is_finite() || stop >= available { return None; }
    let tail_limits = Limits {
        zones: limits.zones, acceleration: limits.acceleration,
        deceleration: limits.deceleration, lateral: limits.lateral, cruise: 0.,
    };
    let tail = speed::profile(&tail_geometry, endpoint.speed, &tail_limits, Some(stop))?;
    let last = tail.last()?;
    let before = tail.get(tail.len().checked_sub(2)?)?;
    if last.speed != 0. || last.position != before.position
        || (last.time - before.time - 8.).abs() > 1e-8 { return None; }
    prefix.extend(tail.into_iter().skip(1).map(|mut p| { p.time += span; p }));
    (prefix.len() <= 84).then_some(prefix)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn clock(now: f64, scene: f64) -> Option<(f64, Option<f64>)> {
        Some((now, Some(scene)))
    }
    #[test]
    fn cached_motion_consumes_one_acquisition_budget_without_renewal() {
        let mut s = Support::default();
        assert!(s.observe(clock(0., 0.), 3.5, &[4], true).is_none());
        assert!(s.observe(clock(0.05, 0.), 3.5, &[4], true).is_none());
        assert_eq!(s.count, 1);
        assert!(s.observe(clock(0.1, 0.1), 3.5, &[4], true).is_none());
        assert!((s.observe(clock(0.2, 0.2), 3.5, &[4], true).unwrap() - 0.1).abs() < 1e-12);
        assert!((s.observe(clock(0.25, 0.2), 3.5, &[4], true).unwrap() - 0.05).abs() < 1e-12);
        assert!(s.observe(clock(0.3, 0.2), 3.5, &[4], true).is_none());
    }
    #[test]
    fn faults_changed_ids_and_changed_targets_cannot_inherit_a_prefix() {
        for which in 0..5 {
            let mut s = Support::default();
            s.observe(clock(0., 0.), 3.5, &[4], true);
            s.observe(clock(0.1, 0.1), 3.5, &[4], true);
            let accepted = match which {
                0 => s.observe(Some((0.2, None)), 3.5, &[4], true),
                1 => s.observe(clock(0.2, 0.2), -3.5, &[4], true),
                2 => s.observe(clock(0.2, 0.2), 3.5, &[5], true),
                3 => s.observe(clock(0.2, 0.2), 3.5, &[4], false),
                _ => s.observe(clock(0.05, 0.05), 3.5, &[4], true),
            };
            assert!(accepted.is_none());
            assert!(s.observe(clock(0.21, 0.2), 3.5, &[4], true).is_none());
        }
    }
    #[test]
    fn prefix_and_hard_backup_have_independent_closed_form_kinematics() {
        let geometry: Vec<_> = (0..=80).map(|i| Vec2::new(i as f64 * 0.5, 0.)).collect();
        let limits = Limits { zones: None, acceleration: 2., deceleration: 2.5, lateral: None, cruise: 8. };
        let nominal = speed::profile(&geometry, 1.5, &limits, None).unwrap();
        let path = profile(&geometry, &nominal, &limits, 0.1).unwrap();
        assert!((path[1].speed - 1.7).abs() < 1e-12);
        assert!((path[1].position.x - 0.16).abs() < 1e-12);
        assert!((path.last().unwrap().position.x - (0.16 + 1.7_f64.powi(2) / 5. + 1e-6)).abs() < 1e-9);
        for pair in path.windows(2) {
            let dt = pair[1].time - pair[0].time;
            assert!(dt > 0.);
            let a = (pair[1].speed - pair[0].speed) / dt;
            assert!((-2.5 - 1e-7..=2. + 1e-7).contains(&a));
            let integrated = (pair[0].speed + pair[1].speed) * dt / 2.;
            assert!((integrated - pair[0].position.distance(pair[1].position)).abs() < 1e-9);
        }
        assert_eq!(path.last().unwrap().speed, 0.);
        assert!((path.last().unwrap().time - path[path.len() - 2].time - 8.).abs() < 1e-9);
    }
    #[test]
    fn joint_rigid_registration_leaves_synchronized_collision_distance_unchanged() {
        use rustdriving_core::{Pose, Prediction};
        let path = vec![
            TrajectoryPoint { position: Vec2::new(0., 0.), speed: 4., time: 0. },
            TrajectoryPoint { position: Vec2::new(8., 0.), speed: 4., time: 2. },
        ];
        let object = Prediction { id: 7, radius: 0.7, dt: 0.2,
            positions: (0..=10).map(|i| Vec2::new(4., 3. - i as f64 * 0.6)).collect(),
            additional_radius_m: (0..=10).map(|i| i as f64 * 0.02).collect() };
        let expected = super::super::collision::first_contact_time(&path, &object, 1.25, Vec2::new(1.,0.)).unwrap();
        for yaw in [-2.7_f64, -0.3, 0.8, 2.1] {
            let pose = Pose { position: Vec2::new(18., -31.), yaw };
            let transform = |p: Vec2| Vec2::new(p.x * yaw.cos() - p.y * yaw.sin() + pose.position.x,
                p.x * yaw.sin() + p.y * yaw.cos() + pose.position.y);
            let shifted: Vec<_> = path.iter().map(|p| TrajectoryPoint { position: transform(p.position), ..*p }).collect();
            let moving = Prediction { positions: object.positions.iter().map(|p| transform(*p)).collect(), ..object.clone() };
            let actual = super::super::collision::first_contact_time(&shifted, &moving, 1.25, Vec2::new(yaw.cos(), yaw.sin())).unwrap();
            assert!((actual - expected).abs() < 1e-10);
        }
    }

}
