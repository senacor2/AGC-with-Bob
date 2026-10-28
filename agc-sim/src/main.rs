//! AGC Simulator — Terminal-Anwendung.
//!
//! Startet die DSKY-TUI und läuft bis der Nutzer `q` oder `Ctrl+C` drückt.
//!
//! # Verwendung
//! ```sh
//! cargo run -p agc-sim
//! cargo run -p agc-sim -- --scenario docs/scenarios/tei_pre.scenario
//! ```
//! Mind. Terminal-Größe: 94×28 (DSKY 62 + Info-Panel 32 Spalten, 28 Zeilen).

fn main() {
    // Einfaches manuelles Argument-Parsing: --scenario <path>
    let mut scenario_path: Option<String> = None;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        if arg == "--scenario" {
            scenario_path = args.next();
        }
    }

    if let Err(e) = agc_sim::sim_loop::run_tui(scenario_path.as_deref()) {
        eprintln!("AGC-Sim Fehler: {e}");
        std::process::exit(1);
    }
}
