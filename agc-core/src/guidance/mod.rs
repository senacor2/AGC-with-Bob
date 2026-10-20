//! Guidance-Module — Umsetzung der AGC-Guidance-Programme.
//!
//! Die Guidance-Schicht enthält die P-Programme des AGC, die Bahnmanöver
//! planen und überwachen:
//!
//! - **P11** — Earth Orbit Insertion Monitor: überwacht die Aufstiegsphase
//!   nach dem Liftoff und propagiert den Zustandsvektor.
//!   AGC-Quelle: `Comanche055/P11.agc`
//!
//! - **P38** — Stable Orbit Insertion (SOI): berechnet das Lambert-Manöver
//!   zur Annäherung an das passive Fahrzeug.
//!   AGC-Quelle: `Comanche055/STABLE_ORBIT_-_P38-P39.agc`
//!
//! - **P39** — Midcourse Correction: verfeinert das SOI-Manöver anhand der
//!   aktuell gemessenen Passivfahrzeug-Position.
//!   AGC-Quelle: `Comanche055/STABLE_ORBIT_-_P38-P39.agc`
//!
//! # Hinweis für `no_std`
//! Kein Heap, kein `static mut`. Phase-Table-Checkpoints sichern den
//! Fortschritt restart-sicher ab.

use crate::navigation::state_vector::StateVector;
use crate::restart::phase_table::PhaseTable;

pub mod p11;
pub mod p38_39;

/// Gemeinsame Abstraktion für alle AGC-Guidance-Programme (P-Programme).
///
/// Jedes Programm implementiert diesen Trait und wird phasenweise durch den
/// Executive aufgerufen. Die Phase wird aus der [`PhaseTable`] gelesen und
/// nach jedem abgeschlossenen Schritt per Checkpoint gesichert.
///
/// Gibt `Some(())` zurück, wenn die Phase erfolgreich abgeschlossen wurde,
/// und `None`, wenn ein Alarm ausgelöst wurde oder der Zustand ungültig ist.
pub trait GuidanceProgram {
    /// Führt eine Programmphase aus.
    ///
    /// # Parameter
    /// - `phase`: aktuelle Programmphase (aus der Phase-Table, Gruppe des Programms)
    /// - `sv`: aktueller Bahn-Zustandsvektor (ECI J2000, m / m/s)
    /// - `pt`: Phase-Table — wird genutzt, um Checkpoints zu setzen
    ///
    /// # Rückgabe
    /// `Some(())` bei Erfolg, `None` bei Alarm oder ungültigem Zustand.
    fn run_phase(phase: u8, sv: &StateVector, pt: &mut PhaseTable) -> Option<()>;
}
