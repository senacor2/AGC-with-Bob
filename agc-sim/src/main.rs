//! AGC Simulator — Terminal-Anwendung.
//!
//! Startet die DSKY-TUI und läuft bis der Nutzer `q` oder `Ctrl+C` drückt.
//!
//! # Verwendung
//! ```sh
//! cargo run -p agc-sim
//! ```
//! Mind. Terminal-Größe: 94×28 (DSKY 62 + Info-Panel 32 Spalten, 28 Zeilen).

fn main() {
    if let Err(e) = agc_sim::sim_loop::run_tui() {
        eprintln!("AGC-Sim Fehler: {e}");
        std::process::exit(1);
    }
}
