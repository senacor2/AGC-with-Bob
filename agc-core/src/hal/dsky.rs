/// DSKY (Display & Keyboard) Hardware-Abstraction-Layer.
///
/// Das DSKY ist die einzige Mensch-Maschine-Schnittstelle des AGC.
/// Es besitzt ein 7-Segment-Display (drei numerische Register R1/R2/R3,
/// Verb/Noun, Prog), Statuslichter sowie eine Tastatur.
///
/// # AGC-Kanalreferenzen
/// - Ausgabe-Kanal **010** (octal): Serien-Display-Daten (5 Bits pro
///   Segment-Gruppe, getaktet vom T4RUPT-Handler, DSPOUT-Routine).
///   Format: `AAAABCCCCCDDDDD` laut Kommentar in
///   `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, Zeile 367.
/// - Kanal **011** (octal): Warn- und Statuslichter (KEY RELEASE Bit 5,
///   VERB/NOUN FLASH Bit 6, OPERATOR ERROR Bit 7, UPLINK ACTY Bit 10).
/// - Eingangskanal **015** / **016** (octal): KEYRUPT1/KEYRUPT2 —
///   Tastatur-Interrupt-Quelle, Keycode 5 Bits.
///
/// # Einheiten
/// - Segment-Daten: Roh-`u16`-Worte gemäß Kanal-10-Format.
/// - Keycode: `u8` (0–31, AGC-Tastaturcode).
pub trait DskyHal {
    /// Schreibt ein vorformatiertes Kanal-10-Wort auf das Display.
    ///
    /// `word` enthält das 15-Bit-Display-Datenwort laut AGC-Kanal-10-Format.
    ///
    /// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`,
    /// DSPOUT / DSPIN-Routine.
    fn write_display_word(&mut self, word: u16);

    /// Setzt die Statuslichter über Kanal-11-Bits.
    ///
    /// `bits` entspricht direkt dem Kanal-11-Ausgabewort (Bit-1-basiert,
    /// wie im AGC).
    ///
    /// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`,
    /// Kanal-011-WOR/WAND-Operationen.
    fn write_status_lights(&mut self, bits: u16);

    /// Liest den nächsten Tastendruck aus dem DSKY-Puffer.
    ///
    /// Gibt `None` zurück wenn kein Tastendruck verfügbar.
    /// Keycode-Belegung: AGC-Tastatur (0–31).
    ///
    /// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`,
    /// KEYRUPT1 / KEYRUPT2.
    fn read_keycode(&mut self) -> Option<u8>;
}

// ---------------------------------------------------------------------------
// Mock-Implementierung für Unit-Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
pub mod mock {
    use super::DskyHal;

    /// Minimaler DSKY-Stub — puffert letztes Display-Wort, liefert
    /// vorkonfigurierte Tastendrücke.
    pub struct MockDsky {
        pub last_display_word: u16,
        pub last_status_bits: u16,
        /// Warteschlange simulierter Tastendrücke (FIFO, vorne = nächster).
        pub key_queue: [Option<u8>; 8],
        key_head: usize,
        key_len: usize,
    }

    impl MockDsky {
        pub const fn new() -> Self {
            Self {
                last_display_word: 0,
                last_status_bits: 0,
                key_queue: [None; 8],
                key_head: 0,
                key_len: 0,
            }
        }

        /// Fügt einen simulierten Tastendruck in die Warteschlange ein.
        pub fn push_key(&mut self, code: u8) {
            if self.key_len < self.key_queue.len() {
                let idx = (self.key_head + self.key_len) % self.key_queue.len();
                self.key_queue[idx] = Some(code);
                self.key_len += 1;
            }
        }
    }

    impl DskyHal for MockDsky {
        fn write_display_word(&mut self, word: u16) {
            self.last_display_word = word;
        }

        fn write_status_lights(&mut self, bits: u16) {
            self.last_status_bits = bits;
        }

        fn read_keycode(&mut self) -> Option<u8> {
            if self.key_len == 0 {
                return None;
            }
            let code = self.key_queue[self.key_head];
            self.key_head = (self.key_head + 1) % self.key_queue.len();
            self.key_len -= 1;
            code
        }
    }
}
