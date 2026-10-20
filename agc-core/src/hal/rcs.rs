/// RCS (Reaction Control System) Hardware-Abstraction-Layer.
///
/// Das CSM-RCS besteht aus 16 Thrustern in vier Quad-Gruppen (A/B/C/D),
/// je vier Thruster pro Quad. Die AGC steuert sie über Ausgabe-Kanal 5
/// (Quad A/B) und Kanal 6 (Quad C/D) als Bit-Felder.
///
/// # AGC-Kanalreferenzen
/// - Kanal **005** (octal): RCS Jet-Befehle Quad A (Bits 1–4) und Quad B
///   (Bits 5–8). 1 = Jet an, 0 = Jet aus.
/// - Kanal **006** (octal): RCS Jet-Befehle Quad C (Bits 1–4) und Quad D
///   (Bits 5–8).
///   Quelle: `Comanche055/JET_SELECTION_LOGIC.agc`, JETSLECT-Routine.
///
/// # Einheiten
/// - Jet-Befehle: Bit-Maske `u16` — Bits 1–8 gemäß AGC-Kanal-5/6-Format.
/// - Impuls-Dauer: `u16` Millisekunden (minimale AGC-Impulsbreite: 14 ms).
pub trait RcsHal {
    /// Sendet einen RCS-Jet-Befehl für Quad A+B (Kanal 5).
    ///
    /// `bits`: Bit 0 = Jet A1, Bit 1 = Jet A2, Bit 2 = Jet A3,
    ///         Bit 3 = Jet A4, Bit 4 = Jet B1 … Bit 7 = Jet B4.
    ///
    /// AGC source: `Comanche055/JET_SELECTION_LOGIC.agc`, JETSLECT-Routine.
    fn set_jets_ab(&mut self, bits: u8);

    /// Sendet einen RCS-Jet-Befehl für Quad C+D (Kanal 6).
    ///
    /// `bits`: Bit 0 = Jet C1 … Bit 7 = Jet D4 (analoges Format zu Kanal 5).
    ///
    /// AGC source: `Comanche055/JET_SELECTION_LOGIC.agc`, JETSLECT-Routine.
    fn set_jets_cd(&mut self, bits: u8);
}

// ---------------------------------------------------------------------------
// Mock-Implementierung für Unit-Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
pub mod mock {
    use super::RcsHal;

    /// Minimaler RCS-Stub — zeichnet die letzten Jet-Befehle auf.
    pub struct MockRcs {
        pub jets_ab: u8,
        pub jets_cd: u8,
    }

    impl MockRcs {
        pub const fn new() -> Self {
            Self {
                jets_ab: 0,
                jets_cd: 0,
            }
        }
    }

    impl RcsHal for MockRcs {
        fn set_jets_ab(&mut self, bits: u8) {
            self.jets_ab = bits;
        }

        fn set_jets_cd(&mut self, bits: u8) {
            self.jets_cd = bits;
        }
    }
}
