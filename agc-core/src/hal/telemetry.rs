/// Telemetry Hardware-Abstraction-Layer.
///
/// Der AGC kommuniziert bidirektional mit dem Boden über den
/// Downlink- (Telemetrie-Ausgabe) und Uplink-Kanal (Uplink-Befehle,
/// UPRUPT-Interrupt).
///
/// # AGC-Kanalreferenzen
/// - **Downlink** (Ausgabe): Kanal **034** (octal) und **035** — das
///   Downlink-Wort-Paar enthält 15-Bit-AGC-Worte im Super-Comutation-Format.
///   Quelle: `Comanche055/DOWNLINK_LISTS.agc`, DODOWNTM-Routine.
/// - **Uplink** (Eingabe): Kanal **020** (octal) — 16 Bit Uplink-Datenwort,
///   ausgelöst durch UPRUPT-Interrupt.
///   Quelle: `Comanche055/UPDATE_PROGRAM.agc`, UPRUPT-Handler.
///
/// # Einheiten
/// - Downlink-Wort: `u16` — rohes 15-Bit-AGC-Datenwort (Bit 15 = Vorzeichen).
/// - Uplink-Wort: `u16` — rohes 16-Bit-Uplink-Datenwort.
pub trait TelemetryHal {
    /// Sendet ein 15-Bit-Downlink-Datenwort zum Boden.
    ///
    /// `word`: rohes AGC-Downlink-Wort (Bit 15 = Vorzeichen, Bits 1–14 Daten).
    ///
    /// AGC source: `Comanche055/DOWNLINK_LISTS.agc`, DODOWNTM-Routine.
    fn send_downlink_word(&mut self, word: u16);

    /// Liest das nächste empfangene Uplink-Datenwort.
    ///
    /// Gibt `None` zurück falls kein Uplink-Wort verfügbar.
    ///
    /// AGC source: `Comanche055/UPDATE_PROGRAM.agc`, UPRUPT-Handler.
    fn read_uplink_word(&mut self) -> Option<u16>;
}

// ---------------------------------------------------------------------------
// Mock-Implementierung für Unit-Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
pub mod mock {
    use super::TelemetryHal;

    /// Minimaler Telemetrie-Stub — puffert gesendete Downlink-Worte,
    /// liefert vorkonfigurierte Uplink-Worte.
    pub struct MockTelemetry {
        /// Zuletzt gesendetes Downlink-Wort.
        pub last_downlink: u16,
        /// Downlink-Wort-Zähler (für Test-Assertions).
        pub downlink_count: u32,
        /// Warteschlange simulierter Uplink-Worte (FIFO).
        pub uplink_queue: [Option<u16>; 8],
        uplink_head: usize,
        uplink_len: usize,
    }

    impl MockTelemetry {
        pub const fn new() -> Self {
            Self {
                last_downlink: 0,
                downlink_count: 0,
                uplink_queue: [None; 8],
                uplink_head: 0,
                uplink_len: 0,
            }
        }

        /// Fügt ein simuliertes Uplink-Wort in die Warteschlange ein.
        pub fn push_uplink(&mut self, word: u16) {
            if self.uplink_len < self.uplink_queue.len() {
                let idx = (self.uplink_head + self.uplink_len) % self.uplink_queue.len();
                self.uplink_queue[idx] = Some(word);
                self.uplink_len += 1;
            }
        }
    }

    impl TelemetryHal for MockTelemetry {
        fn send_downlink_word(&mut self, word: u16) {
            self.last_downlink = word;
            self.downlink_count = self.downlink_count.saturating_add(1);
        }

        fn read_uplink_word(&mut self) -> Option<u16> {
            if self.uplink_len == 0 {
                return None;
            }
            let word = self.uplink_queue[self.uplink_head];
            self.uplink_head = (self.uplink_head + 1) % self.uplink_queue.len();
            self.uplink_len -= 1;
            word
        }
    }
}
