//! Offline paired sensor-input inspection. Counterfactual commands are never applied.
use rustdriving_pipeline::DrivingPipeline;
use rustdriving_pipeline::replay::{LOG_SCHEMA_VERSION, LogRecord};
use std::fs::{File, OpenOptions};
use std::io::{BufRead, BufReader, Write};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().collect();
    if args.len() != 4 {
        return Err("usage: inspect_chassis_inputs SENSOR_LOG OUTPUT_JSONL REAR_OFFSET_M".into());
    }
    let offset: f64 = args[3].parse()?;
    let mut reader = BufReader::new(File::open(&args[1])?);
    let mut output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&args[2])?;
    let (mut baseline, mut corrected) = (None, None);
    let mut ticks = 0_usize;
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
                if baseline.is_some()
                    || ticks != 0
                    || header.schema_version != LOG_SCHEMA_VERSION
                    || header.source != "rne-dynamic-rapier-lidar"
                    || header.config.rear_axle_offset_m.is_some()
                    || header.config.local_route_geometry
                {
                    return Err("inspection requires one original nonlocal native header".into());
                }
                header.config.validate()?;
                let mut calibrated = header.config.clone();
                calibrated.rear_axle_offset_m = Some(offset);
                calibrated.validate()?;
                baseline = Some(DrivingPipeline::new(header.config.clone())?);
                corrected = Some(DrivingPipeline::new(calibrated.clone())?);
                serde_json::to_writer(
                    &mut output,
                    &serde_json::json!({
                        "kind":"configuration", "original_config":header.config,
                        "counterfactual_config":calibrated, "closed_loop":false
                    }),
                )?;
                output.write_all(b"\n")?;
            }
            LogRecord::Tick { tick } => {
                let actual = baseline
                    .as_mut()
                    .ok_or("tick before header")?
                    .step(&tick.input)?;
                if serde_json::to_vec(&actual)? != serde_json::to_vec(&tick.expected)? {
                    return Err(format!("original sensor replay differs at tick {ticks}").into());
                }
                let counterfactual = corrected
                    .as_mut()
                    .ok_or("missing counterfactual")?
                    .step(&tick.input)?;
                serde_json::to_writer(
                    &mut output,
                    &serde_json::json!({
                        "kind":"tick", "counterfactual":counterfactual
                    }),
                )?;
                output.write_all(b"\n")?;
                ticks += 1;
            }
            LogRecord::End { ticks: expected } => {
                if baseline.is_none() || ticks == 0 || ticks != expected {
                    return Err("invalid input count footer".into());
                }
                ended = true;
            }
        }
    }
    if !ended {
        return Err("missing input count footer".into());
    }
    serde_json::to_writer(
        &mut output,
        &serde_json::json!({
            "kind":"end", "ticks":ticks, "legacy_verified":true, "closed_loop":false
        }),
    )?;
    output.write_all(b"\n")?;
    output.flush()?;
    Ok(())
}
