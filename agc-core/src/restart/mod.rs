//! Restart-Infrastruktur: Phasentabelle für GOJAM-sichere Berechnungen.
//!
//! # Verwendung
//!
//! Jede mehrstufige Berechnung, die einen GOJAM-Neustart überleben muss,
//! verwendet das Checkpoint-Muster mit [`PhaseTable`]:
//!
//! ```rust,ignore
//! use agc_core::restart::PhaseTable;
//!
//! fn guidance_step(pt: &mut PhaseTable) {
//!     pt.set_phase(3, 2);   // Checkpoint vor Schritt 2
//!     // … Schritt 2 …
//!     pt.set_phase(3, 3);   // Checkpoint vor Schritt 3
//!     // … Schritt 3 …
//!     pt.set_phase(3, 0);   // Gruppe deaktivieren nach Abschluss
//! }
//! ```
//!
//! AGC source: `Comanche055/PHASE_TABLE_MAINTENANCE.agc`, `PHASCHNG` routine.

pub mod phase_table;

pub use phase_table::PhaseTable;
