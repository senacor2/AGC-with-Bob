//! Gemeinsame Test-Hilfsfunktionen für AGC-Simulator-Integrationstests.
//!
//! Stellt `press_keys()` und `load_scenario_state()` bereit, damit alle
//! Use-Case-Tests DRY bleiben und nicht jeder Test eine eigene Kopie mitführt.

use agc_sim::scenarios::{load_scenario_file, ScenarioData};
use agc_sim::sim_loop::{sim_tick, SimState};

// ---------------------------------------------------------------------------
// Pfad-Auflösung
// ---------------------------------------------------------------------------

/// Löst einen projektrelativen Pfad auf (relativ zum Workspace-Root).
///
/// Integrationstests laufen mit CWD = Package-Verzeichnis (`agc-sim/`).
/// `docs/scenarios/` liegt aber im Workspace-Root eine Ebene höher.
/// Diese Funktion baut den korrekten absoluten Pfad aus `CARGO_MANIFEST_DIR`.
pub fn workspace_path(relative: &str) -> String {
    // CARGO_MANIFEST_DIR = agc-sim/  (Package-Verzeichnis)
    // Workspace-Root     = agc-sim/../
    let manifest = env!("CARGO_MANIFEST_DIR");
    format!("{manifest}/../{relative}")
}

// ---------------------------------------------------------------------------
// Tasten-Eingabe
// ---------------------------------------------------------------------------

/// Schiebt eine Folge von AGC-Rohkeycodes in den SimDsky-FIFO und
/// ruft pro Taste einen `sim_tick()` auf.
///
/// Entspricht dem Muster aus `dispatcher_integration.rs`.
pub fn press_keys(state: &mut SimState, keys: &[u8]) {
    for &k in keys {
        state.dsky.push_key(k);
        sim_tick(state);
    }
}

// ---------------------------------------------------------------------------
// Szenario-Loader
// ---------------------------------------------------------------------------

/// Liest eine Szenario-Datei und gibt einen fertigen `SimState` zurück.
///
/// `path` ist relativ zum Workspace-Root (z.B. `docs/scenarios/leo_circ.scenario`).
///
/// # Panics
/// Panikt wenn die Datei nicht gelesen oder geparst werden kann (Testfehler).
pub fn load_scenario_state(path: &str) -> SimState {
    let abs_path = workspace_path(path);
    let data = load_scenario_file(&abs_path).unwrap_or_else(|e| {
        panic!(
            "Szenario-Datei '{}' konnte nicht geladen werden: {}",
            path, e
        )
    });
    state_from_scenario(&data)
}

/// Erstellt einen `SimState` direkt aus `ScenarioData`.
pub fn state_from_scenario(data: &ScenarioData) -> SimState {
    let mut state = SimState::new();
    state.load_scenario(data);
    state
}
