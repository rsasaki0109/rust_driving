use rustdriving_core::{Pose, Route, Vec2, VehicleConfig};
use rustdriving_pipeline::replay::{SensorLog, verify};
use rustdriving_pipeline::{DrivingPipeline, PipelineConfig, SensorFrame};

fn input(time: f64, rotation: f64, rate: f64) -> SensorFrame {
    let (speed, offset) = (2.0, 1.5);
    let angle = rate * time;
    // Independent constant-body-twist integral at the chassis reference.
    let position = Vec2::new(
        speed / rate * angle.sin() + offset * (angle.cos() - 1.0),
        speed / rate * (1.0 - angle.cos()) + offset * angle.sin(),
    )
    .rotated(rotation)
    .plus(Vec2::new(12.0, -7.0));
    let body = Pose {
        position,
        yaw: rotation + angle,
    };
    let center = Vec2::new(30.0, 4.0)
        .rotated(rotation)
        .plus(Vec2::new(12.0, -7.0));
    let points: Vec<_> = (0..16)
        .map(|i| {
            let a = std::f64::consts::PI - 0.7 + i as f64 * 1.4 / 15.0;
            center
                .plus(Vec2::new(a.cos(), a.sin()).rotated(rotation))
                .minus(body.position)
                .rotated(-body.yaw)
        })
        .collect();
    serde_json::from_value(serde_json::json!({
        "time":time,
        "odometry":{"stamp":time,"speed":speed,"yaw_rate":rate},
        "gnss":{"stamp":time,"position":position,"variance":0.02},
        "absolute_heading":{"stamp":time,"yaw_rad":body.yaw,"variance_rad2":0.0001},
        "lidar":{"stamp":time,"points":points}
    }))
    .unwrap()
}

fn configuration(rotation: f64, calibrated: bool) -> PipelineConfig {
    let origin = Vec2::new(12.0, -7.0);
    let mut config = PipelineConfig::new(
        Route::new(
            vec![origin, origin.plus(Vec2::new(100.0, 0.0).rotated(rotation))],
            8.0,
        )
        .unwrap(),
        Pose {
            position: origin,
            yaw: rotation,
        },
        VehicleConfig::default(),
    );
    config.absolute_heading_fusion = true;
    config.fit_recent_track_motion = true;
    config.enclose_track_measurements = true;
    config.rear_axle_offset_m = calibrated.then_some(1.5);
    config
}

#[test]
fn a_chassis_turn_does_not_create_static_object_motion_on_the_original_route() {
    for rotation in [-0.8, 0.0, 1.1] {
        for rate in [-0.6, 0.6] {
            let config = configuration(rotation, true);
            assert!(!config.local_route_geometry);
            assert!(config.motion_limits.is_none());
            let mut calibrated = DrivingPipeline::new(config).unwrap();
            let mut rear_point = DrivingPipeline::new(configuration(rotation, false)).unwrap();
            let mut old_peak = 0.0_f64;
            let mut new_peak = 0.0_f64;
            for i in 0..=20 {
                let frame = input(i as f64 * 0.05, rotation, rate);
                let current = calibrated.step(&frame).unwrap();
                let preceding = rear_point.step(&frame).unwrap();
                assert_eq!(current.tracks.len(), 1);
                assert_eq!(preceding.tracks.len(), 1);
                if i >= 4 {
                    new_peak = new_peak.max(
                        current.tracks[0]
                            .velocity
                            .x
                            .hypot(current.tracks[0].velocity.y),
                    );
                    old_peak = old_peak.max(
                        preceding.tracks[0]
                            .velocity
                            .x
                            .hypot(preceding.tracks[0].velocity.y),
                    );
                }
            }
            assert!(
                old_peak > 0.6,
                "uncalibrated fixture did not expose the reference mismatch: {old_peak}"
            );
            assert!(
                new_peak < 0.02,
                "calibrated static track moved at {new_peak} m/s"
            );
        }
    }
}

#[test]
fn sensor_replay_owns_the_body_reference_without_enabling_local_route_fitting() {
    let config = configuration(0.0, true);
    let mut pipeline = DrivingPipeline::new(config.clone()).unwrap();
    let mut log = SensorLog::new("analytic-chassis-acquisitions", config);
    for i in 0..=12 {
        let frame = input(i as f64 * 0.05, 0.0, 0.6);
        let output = pipeline.step(&frame).unwrap();
        log.record(frame, output);
    }
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    assert!(
        verify(std::io::Cursor::new(bytes), vec![])
            .unwrap()
            .verified
    );
    log.header.config.rear_axle_offset_m = None;
    let mut changed = vec![];
    log.write(&mut changed).unwrap();
    assert!(verify(std::io::Cursor::new(changed), vec![]).is_err());
}
