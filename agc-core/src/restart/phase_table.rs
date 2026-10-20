//! Phasentabellen-Mechanismus für Restart-Sicherheit.
//!
//! # Konzept
//!
//! Der AGC schreibt vor jedem kritischen Schritt einer mehrstufigen Berechnung
//! den Fortschritt in die Phase Table. Nach einem GOJAM/Neustart liest
//! [`FRESH_START_AND_RESTART.agc`] die Tabelle und setzt an der zuletzt
//! gesicherten Stelle fort.
//!
//! ## Checkpoint-Muster
//!
//! ```rust,ignore
//! use agc_core::restart::PhaseTable;
//!
//! fn guidance_step(phase_table: &mut PhaseTable) {
//!     // Vor Schritt 2 sichern — bei Neustart hier weitermachen
//!     phase_table.set_phase(3, 2);
//!     // … Schritt 2 ausführen …
//!
//!     // Vor Schritt 3 sichern
//!     phase_table.set_phase(3, 3);
//!     // … Schritt 3 ausführen …
//!
//!     // Gruppe deaktivieren nach Abschluss
//!     phase_table.set_phase(3, 0);
//! }
//! ```
//!
//! ## Gruppen und Phasen
//!
//! | Wert | Bedeutung                                              |
//! |------|--------------------------------------------------------|
//! | `0`  | Gruppe inaktiv — kein Restart für diese Gruppe        |
//! | `1`  | Letztes Display reaktivieren (G.1 im AGC)             |
//! | `2+` | Sprungziel des Restarts (geradzahlig = doppelt, ungerade = einfach) |
//!
//! # AGC-Quelle
//! `Comanche055/PHASE_TABLE_MAINTENANCE.agc`, `PHASCHNG`-Routine (pp. 1404–1413).

/// Anzahl der Restart-Gruppen.
///
/// Der AGC verwendete die Gruppen 1–7 (oktal). Index 0 bleibt ungenutzt
/// (Sentinel), um den 1-basierten AGC-Code direkt zu spiegeln.
pub const GROUP_COUNT: usize = 8; // Indizes 0..=7; 0 ist Sentinel

/// Phase-Table-Sentinel für eine inaktive Gruppe.
pub const PHASE_INACTIVE: u8 = 0;

/// Phasentabelle — statische, restart-sichere Fortschrittsverfolgung.
///
/// Einheit: je Gruppe ein `u8`-Phasenwert (`0` = inaktiv).
/// Skalierung: keine numerische Einheit; Werte sind Sprungziel-Indizes.
///
/// # AGC-Quelle
/// `Comanche055/PHASE_TABLE_MAINTENANCE.agc`, `PHASCHNG` routine.
#[derive(Debug)]
pub struct PhaseTable {
    phases: [u8; GROUP_COUNT],
}

impl PhaseTable {
    /// Erstellt eine leere (alle Gruppen inaktiv) Phasentabelle.
    pub const fn new() -> Self {
        Self {
            phases: [PHASE_INACTIVE; GROUP_COUNT],
        }
    }

    /// Setzt die Phase einer Gruppe.
    ///
    /// - `group`: 1–7 (wie im AGC, oktal 1–7). Wert 0 ist ungültig und wird ignoriert.
    /// - `phase`: 0 = inaktiv, 1–127 = Restart-Ziel.
    ///
    /// Gibt `None` zurück falls `group` außerhalb 1..=7 liegt.
    ///
    /// AGC source: `Comanche055/PHASE_TABLE_MAINTENANCE.agc`, `PHASCHNG` routine.
    pub fn set_phase(&mut self, group: u8, phase: u8) -> Option<()> {
        if group == 0 || group as usize >= GROUP_COUNT {
            return None;
        }
        self.phases[group as usize] = phase;
        Some(())
    }

    /// Liest die aktuelle Phase einer Gruppe.
    ///
    /// Gibt `None` zurück falls `group` außerhalb 1..=7 liegt.
    ///
    /// AGC source: `Comanche055/PHASE_TABLE_MAINTENANCE.agc`, `PHASCHNG` routine.
    pub fn get_phase(&self, group: u8) -> Option<u8> {
        if group == 0 || group as usize >= GROUP_COUNT {
            return None;
        }
        Some(self.phases[group as usize])
    }

    /// Gibt `true` zurück wenn die Gruppe aktiv ist (Phase ≠ 0).
    pub fn is_active(&self, group: u8) -> bool {
        self.get_phase(group).is_some_and(|p| p != PHASE_INACTIVE)
    }

    /// Setzt alle Gruppen auf inaktiv zurück (Fresh Start / GOJAM).
    ///
    /// AGC source: `Comanche055/FRESH_START_AND_RESTART.agc`, Fresh-Start-Sequenz.
    pub fn clear_all(&mut self) {
        self.phases = [PHASE_INACTIVE; GROUP_COUNT];
    }
}

impl Default for PhaseTable {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{PhaseTable, PHASE_INACTIVE};

    #[test]
    fn new_table_all_inactive() {
        let t = PhaseTable::new();
        for g in 1_u8..=7 {
            assert_eq!(t.get_phase(g), Some(PHASE_INACTIVE), "group {g}");
            assert!(!t.is_active(g));
        }
    }

    #[test]
    fn set_and_get_phase() {
        let mut t = PhaseTable::new();
        t.set_phase(3, 6).unwrap();
        assert_eq!(t.get_phase(3), Some(6));
        assert!(t.is_active(3));
    }

    #[test]
    fn set_phase_zero_deactivates_group() {
        let mut t = PhaseTable::new();
        t.set_phase(2, 4).unwrap();
        t.set_phase(2, 0).unwrap();
        assert_eq!(t.get_phase(2), Some(0));
        assert!(!t.is_active(2));
    }

    #[test]
    fn set_phase_out_of_range_returns_none() {
        let mut t = PhaseTable::new();
        assert!(t.set_phase(0, 1).is_none());
        assert!(t.set_phase(8, 1).is_none());
    }

    #[test]
    fn get_phase_out_of_range_returns_none() {
        let t = PhaseTable::new();
        assert!(t.get_phase(0).is_none());
        assert!(t.get_phase(8).is_none());
    }

    #[test]
    fn clear_all_resets_all_groups() {
        let mut t = PhaseTable::new();
        t.set_phase(1, 10).unwrap();
        t.set_phase(4, 7).unwrap();
        t.set_phase(7, 3).unwrap();
        t.clear_all();
        for g in 1_u8..=7 {
            assert_eq!(
                t.get_phase(g),
                Some(PHASE_INACTIVE),
                "group {g} after clear"
            );
        }
    }

    /// Simuliert einen GOJAM-Restart-Zyklus:
    /// Phase sichern → Phase lesen → Neustart simulieren → Phase nach Neustart prüfen.
    #[test]
    fn restart_roundtrip() {
        let mut t = PhaseTable::new();

        // Schritt 2 beginnt — Phase 2 in Gruppe 5 sichern
        t.set_phase(5, 2).unwrap();
        assert_eq!(t.get_phase(5), Some(2));

        // GOJAM tritt auf → Neustart liest Phase 2 für Gruppe 5
        let restart_phase = t.get_phase(5).unwrap();
        assert_eq!(restart_phase, 2);

        // Nach erfolgreicher Wiederaufnahme: Gruppe deaktivieren
        t.set_phase(5, 0).unwrap();
        assert!(!t.is_active(5));
    }
}
