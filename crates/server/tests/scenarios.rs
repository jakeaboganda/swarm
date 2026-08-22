//! Every committed scenario still loads, validates, and names a real map.
//!
//! Nothing else checks this. The scenario files are the repo's entry points --
//! `scripts/run.sh` and the README both address them by name -- but they are
//! data, so a rename in the schema or a moved map file breaks them silently
//! and only shows up the next time someone tries to watch something.

use std::path::{Path, PathBuf};

/// Every `scenario_*.json` at the workspace root.
fn scenarios() -> Vec<PathBuf> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let mut found: Vec<PathBuf> = std::fs::read_dir(&root)
        .expect("read the workspace root")
        .filter_map(|entry| {
            let path = entry.expect("read a directory entry").path();
            let name = path.file_name()?.to_str()?;
            (name.starts_with("scenario") && name.ends_with(".json")).then_some(path)
        })
        .collect();
    found.sort();
    found
}

#[test]
fn every_committed_scenario_loads_and_validates() {
    let scenarios = scenarios();
    assert!(
        scenarios.len() >= 10,
        "only found {} scenarios; is the glob still right?",
        scenarios.len()
    );

    for path in &scenarios {
        let name = path.file_name().expect("a file name").to_string_lossy();
        let config = server::scenario::load_scenario(path.to_str().expect("utf-8 path"))
            .unwrap_or_else(|e| panic!("{name}: {e:#}"));
        assert!(!config.roster.is_empty(), "{name} has an empty roster");
    }
}

#[test]
fn every_scenario_map_loads_into_a_drivable_road() {
    // A scenario's `map` is a path or a built-in name, and neither is checked
    // until the server starts. `load_map` also validates the surface mesh, so
    // this covers "the file is there" and "it tessellates" in one go.
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    for path in scenarios() {
        let name = path.file_name().expect("a file name").to_string_lossy();
        let config =
            server::scenario::load_scenario(path.to_str().expect("utf-8 path")).expect("loads");
        let Some(map) = config.map.as_deref() else {
            continue; // the arena world names no map
        };
        // Scenario map paths are relative to the workspace root, which is
        // where the server is run from.
        let resolved = if map.ends_with(".xodr") {
            root.join(map).to_string_lossy().into_owned()
        } else {
            map.to_string()
        };
        let network = server::load_map(Some(&resolved))
            .unwrap_or_else(|e| panic!("{name} names map {map:?}: {e:#}"))
            .unwrap_or_else(|| panic!("{name} named a map that produced no road"));
        assert!(
            network.driving_lanes().count() > 0,
            "{name}: map {map:?} has no driving lanes to spawn on"
        );
    }
}
