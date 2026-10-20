//! Guidance-Module — Umsetzung der AGC-Guidance-Programme.
//!
//! Die Guidance-Schicht enthält die P-Programme des AGC, die Bahnmanöver
//! planen und überwachen:
//!
//! - **P11** — Earth Orbit Insertion Monitor: überwacht die Aufstiegsphase
//!   nach dem Liftoff und propagiert den Zustandsvektor.
//!   AGC-Quelle: `Comanche055/P11.agc`
//!
//! - **P20** — Rendezvous Navigation: aktualisiert den CSM-Zustandsvektor
//!   via VHF-Ranging (skalarer Kalman-Update).
//!   AGC-Quelle: `Comanche055/P20-P25.agc`
//!
//! - **P22** — Landmark Tracking: berechnet MGA aus Positions-/Geschwindigkeitsvektor.
//!   AGC-Quelle: `Comanche055/P20-P25.agc`
//!
//! - **P30** — External ΔV Targeting: Apogee/Perigee + DELVSIN aus TIG + DELVSLV.
//!   AGC-Quelle: `Comanche055/P30,P37.agc`
//!
//! - **P37** — Return to Earth: Lambert-Manöver zur Rückkehrbahn.
//!   AGC-Quelle: `Comanche055/P30,P37.agc`
//!
//! - **P32** — CSI (Co-Elliptic Sequence Initiation): Δv in Horizontalebene.
//!   AGC-Quelle: `Comanche055/P32-P33,_P72-P73.agc`
//!
//! - **P33** — CDH (Constant Delta Height): Δv zur Höhennivellierung.
//!   AGC-Quelle: `Comanche055/P32-P33,_P72-P73.agc`
//!
//! - **P38** — Stable Orbit Insertion (SOI): berechnet das Lambert-Manöver
//!   zur Annäherung an das passive Fahrzeug.
//!   AGC-Quelle: `Comanche055/STABLE_ORBIT_-_P38-P39.agc`
//!
//! - **P39** — Midcourse Correction: verfeinert das SOI-Manöver anhand der
//!   aktuell gemessenen Passivfahrzeug-Position.
//!   AGC-Quelle: `Comanche055/STABLE_ORBIT_-_P38-P39.agc`
//!
//! - **P40** — SPS Burn: Haupttriebwerksbrand (Ignition → Burn → Cutoff).
//!   AGC-Quelle: `Comanche055/P40-P47.agc`
//!
//! - **P41** — Gimbal Motor Test: Pitch/Yaw-Gimbal-Sequenz vor dem Burn.
//!   AGC-Quelle: `Comanche055/P40-P47.agc`
//!
//! - **P47** — Trim Maneuver: Post-Burn Δv-Restfehler-Berechnung.
//!   AGC-Quelle: `Comanche055/P40-P47.agc`
//!
//! - **P51/P52/P53** — IMU-Alignment.
//!   AGC-Quelle: `Comanche055/P51-P53.agc`
//!
//! # Hinweis für `no_std`
//! Kein Heap, kein `static mut`. Phase-Table-Checkpoints sichern den
//! Fortschritt restart-sicher ab.

use crate::navigation::state_vector::StateVector;
use crate::restart::phase_table::PhaseTable;

pub mod p11;
pub mod p20_25;
pub mod p30_37;
pub mod p32_33;
pub mod p38_39;
pub mod p40_47;
pub mod p51_53;

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

// ---------------------------------------------------------------------------
// Integrationstests — P11 über Executive + PhaseTable
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use core::sync::atomic::{AtomicU8, Ordering};

    use crate::alarm::AlarmState;
    use crate::executive::Executive;
    use crate::guidance::p11::{p11_step, P11State};
    use crate::navigation::state_vector::StateVector;
    use crate::restart::phase_table::PhaseTable;
    use crate::types::{Met, Vec3};

    // -----------------------------------------------------------------------
    // Hilfsfunktionen
    // -----------------------------------------------------------------------

    /// Erzeugt einen ISS-ähnlichen LEO-Zustandsvektor (r = 6 778 000 m,
    /// Kreisbahngeschwindigkeit, Zeit = 100 s nach TEPHEM).
    ///
    /// μ_Erde = 3.986004418 × 10¹⁴ m³/s²
    fn leo_sv() -> StateVector {
        let r = 6_778_000.0_f64;
        let mu = 3.986_004_418e14_f64;
        let v_circ = libm::sqrt(mu / r);
        StateVector::new(
            Vec3::new(r, 0.0, 0.0),
            Vec3::new(0.0, v_circ, 0.0),
            Met::from_seconds(100.0),
        )
    }

    // -----------------------------------------------------------------------
    // Atomics für Job-Callback-Kommunikation
    //
    // Der Executive akzeptiert fn() ohne Parameter. Um den Aufrufzähler
    // aus der Job-Funktion zurückzumelden, verwenden wir je Test einen
    // eigenen AtomicU8, damit parallele Testläufe keine Interferenz haben.
    // -----------------------------------------------------------------------

    /// Zählt Aufrufe für `executive_schedules_and_runs_p11_job`.
    static P11_SCHED_COUNT: AtomicU8 = AtomicU8::new(0);

    fn p11_sched_callback() {
        P11_SCHED_COUNT.fetch_add(1, Ordering::Relaxed);
    }

    /// Zählt Aufrufe für `executive_with_p11_step_end_to_end`.
    static P11_E2E_COUNT: AtomicU8 = AtomicU8::new(0);

    fn p11_e2e_callback() {
        P11_E2E_COUNT.fetch_add(1, Ordering::Relaxed);
    }

    // -----------------------------------------------------------------------
    // Test 1: Executive plant P11-Job ein und führt ihn aus
    //
    // Zeigt, dass der Executive den Job korrekt aufnimmt und ausführt.
    // Im realen System würde p11_job_callback() p11_step() aufrufen;
    // der Callback hier protokolliert nur den Aufruf atomar.
    // -----------------------------------------------------------------------
    #[test]
    fn executive_schedules_and_runs_p11_job() {
        P11_SCHED_COUNT.store(0, Ordering::Relaxed);

        let mut exec = Executive::new();

        // P11-Job mit typischer Guidance-Priorität 10 einplanen
        let slot = exec.schedule(10, p11_sched_callback);
        assert!(slot.is_some(), "Executive sollte Slot vergeben");
        assert_eq!(exec.active_count(), 1);

        // Executive führt Job aus
        let ran = exec.run_highest();
        assert!(ran, "Executive sollte Job ausgeführt haben");
        assert_eq!(exec.active_count(), 0);

        // Callback wurde genau einmal aufgerufen
        assert_eq!(
            P11_SCHED_COUNT.load(Ordering::Relaxed),
            1,
            "P11-Job-Callback muss genau einmal aufgerufen worden sein"
        );

        // Nach Ausführung: Executive ist idle
        assert!(!exec.run_highest(), "Executive sollte nach Job idle sein");
    }

    // -----------------------------------------------------------------------
    // Test 2: P11 + PhaseTable — vollständiger Phase-2 → done-Zyklus
    //
    // P11 führt Phase 2 und 3 in einem Zug aus (AGC-Verhalten: Phase 2
    // wechselt intern auf Phase 3 und schließt sofort ab). Nach dem ersten
    // Aufruf ist state.phase = 0 und Phase-Table Gruppe 3 inaktiv.
    // -----------------------------------------------------------------------
    #[test]
    fn p11_phase_table_checkpoint_cycle() {
        let sv = leo_sv();
        let mut state = P11State::new(Met::from_seconds(0.0));
        assert_eq!(state.phase, 2);

        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();

        // Aufruf: Phase 2 führt intern Phase 3 durch → done
        // Phase 2 setzt Checkpoint 3, führt Phase-3-Logik aus, setzt state.phase = 0
        let result = p11_step(&mut state, &sv, &mut pt, &mut alarm);
        assert!(
            result.is_some(),
            "Phase 2 → done: p11_step sollte Some liefern"
        );
        assert_eq!(state.phase, 0, "state.phase soll 0 (done) sein");
        assert_eq!(
            pt.get_phase(3),
            Some(0),
            "Phase-Table Gruppe 3 soll nach Abschluss 0 sein"
        );
        assert!(alarm.last().is_none(), "Kein Alarm erwartet");

        // Weiterer Aufruf mit Phase 0 → None (bereits fertig)
        let result_done = p11_step(&mut state, &sv, &mut pt, &mut alarm);
        assert!(result_done.is_none(), "Phase 0: p11_step soll None liefern");
    }

    // -----------------------------------------------------------------------
    // Test 2b: Restart in Phase 3 — Phase-Table-Wiederaufnahme
    //
    // Simuliert GOJAM: state.phase = 3, Phase-Table Gruppe 3 = 3.
    // Nach dem Aufruf ist state.phase = 0, Phase-Table Gruppe 3 = 0.
    // -----------------------------------------------------------------------
    #[test]
    fn p11_phase_table_restart_in_phase3() {
        let sv = leo_sv();
        let mut state = P11State::new(Met::from_seconds(0.0));
        state.phase = 3;

        let mut pt = PhaseTable::new();
        pt.set_phase(3, 3).unwrap();

        let mut alarm = AlarmState::new();

        let result = p11_step(&mut state, &sv, &mut pt, &mut alarm);
        assert!(
            result.is_some(),
            "Restart Phase 3 → done: sollte Some liefern"
        );
        assert_eq!(state.phase, 0, "state.phase soll 0 sein");
        assert_eq!(pt.get_phase(3), Some(0), "Phase-Table Gruppe 3 soll 0 sein");
        assert!(alarm.last().is_none(), "Kein Alarm erwartet");
    }

    // -----------------------------------------------------------------------
    // Test 3: Executive + p11_step (kombiniert)
    //
    // Zeigt das vollständige Pattern: Executive plant Job ein, führt ihn aus.
    // Anschließend wird p11_step direkt aufgerufen, um das Zusammenspiel
    // von Executive → p11_step → PhaseTable zu demonstrieren.
    // -----------------------------------------------------------------------
    #[test]
    fn executive_with_p11_step_end_to_end() {
        P11_E2E_COUNT.store(0, Ordering::Relaxed);

        let sv = leo_sv();
        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();

        // Schritt 1: Executive plant P11-Job ein
        let mut exec = Executive::new();
        exec.schedule(10, p11_e2e_callback).unwrap();

        // Schritt 2: Executive führt Job aus (simuliert ISR-Triggerung)
        exec.run_highest();
        assert_eq!(
            P11_E2E_COUNT.load(Ordering::Relaxed),
            1,
            "Job muss nach run_highest ausgeführt worden sein"
        );

        // Schritt 3: P11 Phase 2 → done direkt aufrufen
        // (im echten System würde der Job-Body p11_step via Mutex<RefCell> aufrufen)
        let mut state = P11State::new(Met::from_seconds(0.0));
        let r1 = p11_step(&mut state, &sv, &mut pt, &mut alarm);
        assert!(r1.is_some(), "Phase 2: p11_step sollte Some liefern");
        assert_eq!(state.phase, 0, "state.phase soll 0 (done) sein");

        // Phase-Table Gruppe 3 = 0 (done)
        assert_eq!(pt.get_phase(3), Some(0), "Phase-Table Gruppe 3 soll 0 sein");
    }
}
