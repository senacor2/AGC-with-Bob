/// VHF Ranging Hardware-Abstraction-Layer.
///
/// **Nur Command Module (CM) — kein Rendezvous-Radar.**
/// Das CM besitzt keinen Rendezvous-Radar (dieser ist LM-only, Luminary).
/// Stattdessen nutzt Comanche055 das VHF-Ranging-System (V87/V88) zur
/// CM-LM-Entfernungsmessung über das VHF-Sprachfunk-Subsystem, wenn das
/// CM im aktiven CSI/CDH-Rendezvous-Modus arbeitet.
///
/// Dieses HAL ist **optional** — Guidance-Code prüft [`VhfRangingHal::is_available`]
/// bevor er Messungen anfordert.
///
/// # AGC-Kanalreferenzen
/// - Ranging-Anforderung: Kanal **034** (octal) Bit 4 (VHFRQST).
/// - Ranging-Messung eingelesen über Eingangskanal **034** Bit 1–12
///   (VHFRANGE, 12 Bit, Einheit: Seemeilen × 2⁻¹¹ ≈ 0.488 m Auflösung).
///   Quelle: `Comanche055/R31.agc` (RANGERD-Routine, V87/V88).
///
/// # Einheiten
/// - Entfernung: `u16` — roher AGC-Ranging-Wert.
///   1 LSB ≈ 0.488 m (Vollskala ≈ 2000 Seemeilen).
pub trait VhfRangingHal {
    /// Gibt zurück ob das VHF-Ranging-System verfügbar und initialisiert ist.
    ///
    /// Guidance-Code muss diese Methode prüfen bevor er Messungen anfordert.
    fn is_available(&self) -> bool;

    /// Fordert eine Entfernungsmessung an (setzt VHFRQST, Kanal 34 Bit 4).
    ///
    /// Ergebnis steht nach ca. einem T4RUPT-Zyklus (≈ 100 ms) über
    /// [`read_range`][VhfRangingHal::read_range] bereit.
    ///
    /// AGC source: `Comanche055/R31.agc`, RANGERD-Routine.
    fn request_range(&mut self);

    /// Liest den zuletzt empfangenen Ranging-Wert aus Kanal 34.
    ///
    /// Gibt `None` zurück falls noch keine gültige Messung vorliegt.
    ///
    /// AGC source: `Comanche055/R31.agc`, RANGERD-Routine.
    fn read_range(&self) -> Option<u16>;
}

// ---------------------------------------------------------------------------
// Mock-Implementierung für Unit-Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
pub mod mock {
    use super::VhfRangingHal;

    /// Minimaler VHF-Ranging-Stub — liefert festen Entfernungswert.
    pub struct MockVhfRanging {
        pub available: bool,
        pub range_value: Option<u16>,
    }

    impl MockVhfRanging {
        pub const fn new() -> Self {
            Self {
                available: false,
                range_value: None,
            }
        }
    }

    impl VhfRangingHal for MockVhfRanging {
        fn is_available(&self) -> bool {
            self.available
        }

        fn request_range(&mut self) {
            // Stub: keine Aktion nötig — Wert wurde extern gesetzt.
        }

        fn read_range(&self) -> Option<u16> {
            self.range_value
        }
    }
}
