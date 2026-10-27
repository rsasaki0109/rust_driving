//! Sensor-only shadow continuation inspection. Never applies a candidate command.
use rustdriving_pipeline::DrivingPipeline;
use rustdriving_pipeline::replay::{LogRecord, LOG_SCHEMA_VERSION};
use rustdriving_planning::approach_continuation::{ContinuationObservation, ContinuationPermit};
use std::fs::{File, OpenOptions};
use std::io::{BufRead, BufReader, Write};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().collect();
    if args.len() != 3 {
        return Err("usage: inspect_continuations SENSOR_LOG OUTPUT_JSONL".into());
    }
    let mut reader = BufReader::new(File::open(&args[1])?);
    let mut output = OpenOptions::new().write(true).create_new(true).open(&args[2])?;
    let mut pipeline = None;
    let mut ticks = 0_usize;
    let mut ended = false;
    let mut permit: Option<ContinuationPermit> = None;
    let mut ids = Vec::new();
    let mut target = 0.;
    let mut previous_route = None;
    let mut route_generation = 0_u64;
    let mut adoptions = 0_usize;
    let mut grants = 0_usize;
    let mut attempted = 0_usize;
    let mut candidates = 0_usize;
    loop {
        let mut line = String::new();
        let size = std::io::Read::take(&mut reader, 8 * 1024 * 1024 + 1).read_line(&mut line)?;
        if size == 0 { break; }
        if size > 8 * 1024 * 1024 || ended {
            return Err("oversized record or data after footer".into());
        }
        match serde_json::from_str::<LogRecord>(&line)? {
            LogRecord::Header { header } => {
                if pipeline.is_some() || ticks != 0 || header.schema_version != LOG_SCHEMA_VERSION
                    || !header.config.replan_checked_approaches
                    || !header.config.record_planning_diagnostics {
                    return Err("captured checked-approach header required".into());
                }
                pipeline = Some(DrivingPipeline::new(header.config.clone())?);
                serde_json::to_writer(&mut output, &serde_json::json!({
                    "kind":"configuration", "config":header.config,
                    "maximum_permit_duration_s":8., "closed_loop":false,
                    "inspection_phase":"after_original_plan_before_output_guards"
                }))?;
                output.write_all(b"\n")?;
            }
            LogRecord::Tick { tick } => {
                let driver = pipeline.as_mut().ok_or("tick before header")?;
                let mut snapshot = None;
                let actual = driver.step_with_planner_result_inspection(
                    &tick.input, &mut |planner, ego, route, objects, path| {
                        snapshot = Some((planner.clone(), ego, route.clone(),
                            objects.to_vec(), path.clone()));
                    },
                )?;
                if serde_json::to_vec(&actual)? != serde_json::to_vec(&tick.expected)? {
                    return Err(format!("original sensor replay differs at tick {ticks}").into());
                }
                if let Some((mut planner, ego, route, objects, raw)) = snapshot {
                    let route_key = serde_json::to_vec(&route)?;
                    if previous_route.as_ref() != Some(&route_key) {
                        route_generation = route_generation.checked_add(1).ok_or("route generation overflow")?;
                        previous_route = Some(route_key);
                    }
                    let adopted = planner.checked_approach_diagnostics().adopted;
                    let acquisition = planner.checked_approach_scene_clock()
                        .filter(|(now, _)| *now == tick.input.time).and_then(|(_, stamp)| stamp);
                    let healthy = actual.health.is_empty() && !actual.emergency
                        && serde_json::to_vec(&raw)? == serde_json::to_vec(&actual.trajectory)?;
                    if adopted {
                        adoptions += 1;
                        let record = planner.checked_approach_diagnostics().clone();
                        ids = record.blocking_prediction_ids;
                        target = record.proposal_lateral_target_m.ok_or("adoption without target")?;
                        let observed = ContinuationObservation {
                            now_s: tick.input.time, acquisition_s: acquisition,
                            localization_generation: actual.localization_epoch,
                            route_generation, blocking_ids: &ids, lateral_target_m: target,
                            progress_m: route.project(ego.pose.position).0, healthy,
                        };
                        permit = planner.take_continuation_permit(&observed, 8.);
                        grants += usize::from(permit.is_some());
                    }
                    if permit.is_some() || adopted {
                        let observed = ContinuationObservation {
                            now_s: tick.input.time, acquisition_s: acquisition,
                            localization_generation: actual.localization_epoch,
                            route_generation, blocking_ids: &ids, lateral_target_m: target,
                            progress_m: route.project(ego.pose.position).0, healthy,
                        };
                        let bound = permit.as_ref().and_then(|p| p.endpoint_bound());
                        let mut candidate = None;
                        if let Some(active) = permit.as_mut() {
                            attempted += 1;
                            candidate = planner.checked_maneuver_continuation(
                                active, &observed, ego, &route, &objects);
                            candidates += usize::from(candidate.is_some());
                        }
                        let alive = permit.as_ref().and_then(|p| p.endpoint_bound()).is_some();
                        serde_json::to_writer(&mut output, &serde_json::json!({
                            "kind":"probe", "tick":ticks, "time":tick.input.time,
                            "new_adoption":adopted, "healthy_unrestricted_output":healthy,
                            "scene_acquisition_s":acquisition,
                            "localization_generation":actual.localization_epoch,
                            "route_generation":route_generation, "route":route,
                            "ego":ego, "predictions":objects, "endpoint_bound":bound,
                            "original":raw, "candidate":candidate, "permit_alive":alive,
                            "closed_loop":false
                        }))?;
                        output.write_all(b"\n")?;
                        if !alive { permit = None; }
                    }
                } else {
                    // A localization transition may skip ordinary planning.
                    permit = None;
                }
                ticks += 1;
            }
            LogRecord::End { ticks: expected } => {
                if pipeline.is_none() || ticks == 0 || ticks != expected {
                    return Err("invalid sensor footer".into());
                }
                ended = true;
            }
        }
    }
    if !ended { return Err("missing sensor footer".into()); }
    let summary = serde_json::json!({
        "kind":"end", "ticks":ticks, "adoptions":adoptions, "permits":grants,
        "attempted_ticks":attempted, "candidate_ticks":candidates,
        "legacy_verified":true, "closed_loop":false
    });
    serde_json::to_writer(&mut output, &summary)?;
    output.write_all(b"\n")?;
    println!("{summary}");
    Ok(())
}
