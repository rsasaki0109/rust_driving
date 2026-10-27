//! Offline stopping-proposal inspection; no probe commands are applied.
use rustdriving_pipeline::DrivingPipeline;
use rustdriving_pipeline::replay::{LOG_SCHEMA_VERSION, LogRecord};
use std::fs::{File, OpenOptions};
use std::io::{BufRead, BufReader, Write};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().collect();
    if args.len() != 4 {
        return Err("usage: inspect_stop_candidates SENSOR_LOG OUTPUT_JSONL TIME_S".into());
    }
    let target: f64 = args[3].parse()?;
    if !target.is_finite() || target < 0. {
        return Err("invalid inspection time".into());
    }
    let mut reader = BufReader::new(File::open(&args[1])?);
    let mut output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&args[2])?;
    let mut pipeline = None;
    let mut ticks = 0_usize;
    let mut inspected = 0_usize;
    let mut ended = false;
    loop {
        let mut line = String::new();
        let size = std::io::Read::take(&mut reader, 8 * 1024 * 1024 + 1).read_line(&mut line)?;
        if size == 0 {
            break;
        }
        if size > 8 * 1024 * 1024 || ended {
            return Err("oversized record or data after footer".into());
        }
        match serde_json::from_str::<LogRecord>(&line)? {
            LogRecord::Header { header } => {
                if pipeline.is_some() || ticks != 0 || header.schema_version != LOG_SCHEMA_VERSION {
                    return Err("invalid sensor header".into());
                }
                if !header.config.replan_checked_approaches
                    || !header.config.record_planning_diagnostics
                {
                    return Err("captured checked approaches required".into());
                }
                pipeline = Some(DrivingPipeline::new(header.config.clone())?);
                serde_json::to_writer(
                    &mut output,
                    &serde_json::json!({
                        "kind":"configuration", "config":header.config, "closed_loop":false
                    }),
                )?;
                output.write_all(b"\n")?;
            }
            LogRecord::Tick { tick } => {
                let driver = pipeline.as_mut().ok_or("tick before header")?;
                let mut evidence = None;
                let actual = if tick.input.time == target {
                    if !tick.expected.health.is_empty() || tick.expected.estimate.speed > 0.2 {
                        return Err("inspection requires a healthy stationary tick".into());
                    }
                    inspected += 1;
                    driver.step_with_planner_inspection(
                        &tick.input,
                        &mut |planner, ego, route, objects| {
                            let here = route.project(ego.pose.position).0;
                            let describe = |probe: rustdriving_planning::StopProbe| {
                                let attempts: Vec<_> = probe.attempts.iter().map(|a| {
                                    serde_json::json!({
                                        "candidate_index":a.candidate_index,
                                        "end_progress_m":a.end_progress_m,
                                        "progress_gain_m":a.end_progress_m-here,
                                        "hold_start_s":a.hold_start_s,
                                        "hold_end_s":a.hold_end_s,
                                        "first_contact_s":a.first_contact_s,
                                        "contact":a.contact.as_ref().map(|c| serde_json::json!({
                                            "prediction_id":c.prediction_id, "time_s":c.time_s,
                                            "interval_start_s":c.interval_start_s, "interval_end_s":c.interval_end_s,
                                            "ego_position":c.ego_position, "prediction_position":c.prediction_position,
                                            "vehicle_radius_m":c.vehicle_radius_m,
                                            "forecast_radius_bound_m":c.forecast_radius_bound_m,
                                            "minimum_clearance_m":c.minimum_clearance_m,
                                            "effective_margin_m":c.effective_margin_m,
                                            "acceleration_chord_bound_m":c.acceleration_chord_bound_m,
                                            "combined_radius_m":c.combined_radius_m
                                        }))
                                    })
                                }).collect();
                                serde_json::json!({
                                    "initial_progress_m":here, "trajectory":probe.trajectory,
                                    "planning_diagnostics":probe.diagnostics, "attempts":attempts
                                })
                            };
                            let mut original = describe(planner.inspect_stopping_proposals(ego, route, objects));
                            // Only a clone changes geometry policy. All current scene,
                            // speed, road, side-commitment and stop checks remain active.
                            let mut recovery = planner.clone();
                            recovery.point_reference_recovery = true;
                            original["point_reference_recovery_probe"] = describe(
                                recovery.inspect_stopping_proposals(ego, route, objects));
                            original["geometry_comparison"] = serde_json::json!({
                                "local_route_geometry":planner.local_route_geometry,
                                "original_point_reference_recovery":planner.point_reference_recovery,
                                "probe_point_reference_recovery":true,
                                "policy_changed":!planner.point_reference_recovery,
                                "ego":ego, "route":route, "predictions":objects,
                                "closed_loop":false
                            });
                            let scaled: Vec<_> = [0.5, 1., 2.].into_iter().map(|scale| {
                                let candidate = planner.inspect_stopping_proposals_with_join_scale(
                                    ego, route, objects, scale).expect("healthy stationary probe");
                                serde_json::json!({"scale":scale, "probe":describe(candidate)})
                            }).collect();
                            original["join_scale_probes"] = serde_json::json!(scaled);
                            evidence = Some(original);
                        },
                    )?
                } else {
                    driver.step(&tick.input)?
                };
                if serde_json::to_vec(&actual)? != serde_json::to_vec(&tick.expected)? {
                    return Err(format!("original sensor replay differs at tick {ticks}").into());
                }
                if let Some(probe) = evidence {
                    serde_json::to_writer(
                        &mut output,
                        &serde_json::json!({
                            "kind":"probe", "time":tick.input.time, "original":actual, "probe":probe
                        }),
                    )?;
                    output.write_all(b"\n")?;
                }
                ticks += 1;
            }
            LogRecord::End { ticks: expected } => {
                if pipeline.is_none() || ticks == 0 || ticks != expected || inspected != 1 {
                    return Err("invalid count or missing inspection".into());
                }
                ended = true;
            }
        }
    }
    if !ended {
        return Err("missing sensor footer".into());
    }
    serde_json::to_writer(
        &mut output,
        &serde_json::json!({
            "kind":"end", "ticks":ticks, "legacy_verified":true, "closed_loop":false
        }),
    )?;
    output.write_all(b"\n")?;
    Ok(())
}
