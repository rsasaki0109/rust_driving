use std::{fs, process::Command};

#[test]
fn scenario_preflight_rejects_invalid_geometry_without_creating_driving_outputs() {
    let root = std::env::temp_dir().join(format!("rustdriving-validation-{}", std::process::id()));
    fs::create_dir(&root).unwrap();
    let input = root.join("scenario.json");
    let mut value: serde_json::Value =
        serde_json::from_str(include_str!("../../../scenarios/blocked.json")).unwrap();
    let command = || {
        Command::new(env!("CARGO_BIN_EXE_rustdriving"))
            .current_dir(&root)
            .arg("validate-scenario")
            .arg("--scenario")
            .arg(&input)
            .output()
            .unwrap()
    };
    value["objects"][0]["radius"] = serde_json::json!(3.3);
    fs::write(&input, serde_json::to_vec(&value).unwrap()).unwrap();
    let invalid = command();
    assert_eq!(invalid.status.code(), Some(2));
    assert!(String::from_utf8_lossy(&invalid.stderr).contains("invalid object parameters"));
    value["objects"][0]["radius"] = serde_json::json!(2.0);
    value["lateral_candidate_margin_m"] = serde_json::json!(0.35);
    let bytes = serde_json::to_vec(&value).unwrap();
    fs::write(&input, &bytes).unwrap();
    let valid = command();
    assert!(valid.status.success());
    assert_eq!(
        serde_json::from_slice::<serde_json::Value>(&valid.stdout).unwrap()["valid"],
        true
    );
    assert_eq!(fs::read(&input).unwrap(), bytes);
    assert_eq!(fs::read_dir(&root).unwrap().count(), 1);
    let missing = Command::new(env!("CARGO_BIN_EXE_rustdriving"))
        .arg("validate-scenario")
        .output()
        .unwrap();
    assert_eq!(missing.status.code(), Some(2));
    fs::remove_dir_all(root).unwrap();
}

#[test]
fn continuation_scenario_policy_roundtrips_and_reaches_shared_pipeline_config() {
    use rustdriving_sim::{Scenario, pipeline_config};
    let mut value: serde_json::Value =
        serde_json::from_str(include_str!("../../../scenarios/blocked.json")).unwrap();
    let legacy: Scenario = serde_json::from_value(value.clone()).unwrap();
    assert!(!legacy.continue_checked_approaches);
    assert!(serde_json::to_value(&legacy).unwrap().get("continue_checked_approaches").is_none());
    value["continue_checked_approaches"] = serde_json::json!(true);
    let unsupported: Scenario = serde_json::from_value(value.clone()).unwrap();
    assert!(unsupported.validate().is_err());
    for key in ["fit_recent_track_motion", "enclose_track_measurements",
        "forecast_consistency_guard", "forecast_recent_velocity_hypotheses",
        "settle_obstruction_stops", "search_obstruction_stops",
        "replan_checked_approaches", "record_planning_diagnostics"] {
        value[key] = serde_json::json!(true);
    }
    value["forecast_cold_max_speed_m_s"] = serde_json::json!(15.);
    value["forecast_envelope"] = serde_json::json!({
        "position_error_m":0.2,"velocity_error_m_s":0.3,"acceleration_error_m_s2":0.1
    });
    value["lateral_candidate_margin_m"] = serde_json::json!(0.35);
    let enabled: Scenario = serde_json::from_value(value.clone()).unwrap();
    assert!(enabled.validate().is_ok());
    let config = pipeline_config(&enabled);
    assert!(config.continue_checked_approaches);
    assert!(config.validate().is_ok());
    let decoded: Scenario = serde_json::from_value(serde_json::to_value(enabled).unwrap()).unwrap();
    assert!(pipeline_config(&decoded).continue_checked_approaches);
    value["continue_checked_approaches"] = serde_json::json!(1);
    assert!(serde_json::from_value::<Scenario>(value).is_err());
}
