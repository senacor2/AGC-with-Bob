//! Alarm- und Abort-Infrastruktur.
//!
//! Entspricht `ALARM_AND_ABORT.agc` (Comanche055, pp. 1493–1496).
//!
//! # Konzept
//! Der AGC kennt zwei Fehlerstufen:
//!
//! | Stufe          | AGC-Äquivalent | Konsequenz                              |
//! |----------------|----------------|-----------------------------------------|
//! | Program Alarm  | `TC ALARM`     | Alarmcode speichern, Lampe an, weiter   |
//! | GOJAM / Abort  | `TC ABORT`     | Vollständiger Neustart via `panic!`     |
//!
//! In Rust löst [`gojam`] ein `panic!` aus, das bei `panic = "abort"` direkt in
//! einen Hardware-Reset (GOJAM) mündet. Es gibt weder `Result` noch Heap-Allokation.
//!
//! # AGC-Quelle
//! `Comanche055/ALARM_AND_ABORT.agc`

/// Maximale Anzahl gleichzeitig gespeicherter Program-Alarm-Codes.
///
/// Der AGC hatte drei FAILREG-Register (`FAILREG`, `FAILREG+1`, `FAILREG+2`).
pub const ALARM_QUEUE_DEPTH: usize = 3;

/// Interner Alarmzustand — wird als `static` in `agc-core` gehalten.
///
/// Enthält die zuletzt aufgetretenen Program-Alarm-Codes (FIFO, ältester zuerst).
/// Ein Eintrag von `0` bedeutet „leer".
pub struct AlarmState {
    codes: [u16; ALARM_QUEUE_DEPTH],
    count: usize,
}

impl AlarmState {
    /// Leerer Anfangszustand.
    pub const fn new() -> Self {
        Self {
            codes: [0; ALARM_QUEUE_DEPTH],
            count: 0,
        }
    }

    /// Speichert einen Alarmcode (Program Alarm, kein Neustart).
    ///
    /// Ist die Warteschlange voll, wird der älteste Eintrag überschrieben
    /// (entspricht dem FAILREG-Verhalten des AGC).
    pub fn push(&mut self, code: u16) {
        if self.count < ALARM_QUEUE_DEPTH {
            self.codes[self.count] = code;
            self.count += 1;
        } else {
            // Ältesten Eintrag verwerfen, nach links schieben
            self.codes[0] = self.codes[1];
            self.codes[1] = self.codes[2];
            self.codes[2] = code;
        }
    }

    /// Letzter gespeicherter Alarmcode, oder `None` falls leer.
    pub fn last(&self) -> Option<u16> {
        if self.count == 0 {
            None
        } else {
            Some(self.codes[self.count - 1])
        }
    }

    /// Alle gespeicherten Codes als Slice (ältester zuerst).
    pub fn as_slice(&self) -> &[u16] {
        &self.codes[..self.count]
    }

    /// Setzt den Alarmzustand zurück (Fresh Start / nach GOJAM).
    pub fn clear(&mut self) {
        self.codes = [0; ALARM_QUEUE_DEPTH];
        self.count = 0;
    }
}

impl Default for AlarmState {
    fn default() -> Self {
        Self::new()
    }
}

/// Löst einen nicht-abbrechenden Program Alarm aus.
///
/// Entspricht `TC ALARM` im AGC-Assembler. Speichert `code` in `state`,
/// schaltet konzeptuell die Alarm-Lampe an, setzt aber die Ausführung fort.
///
/// AGC source: `Comanche055/ALARM_AND_ABORT.agc`, `ALARM` routine.
#[inline]
pub fn raise(state: &mut AlarmState, code: u16) {
    state.push(code);
}

/// Löst einen sofortigen Neustart aus (GOJAM).
///
/// Entspricht `TC ABORT` im AGC-Assembler. In der Rust-Implementierung
/// wird `panic!` aufgerufen — bei `panic = "abort"` führt das zum
/// Hardware-Reset des Cortex-M7.
///
/// Diese Funktion kehrt **niemals** zurück.
///
/// AGC source: `Comanche055/ALARM_AND_ABORT.agc`, `ABORT` routine / GOJAM mechanism.
#[inline(never)]
#[cold]
pub fn gojam(code: u16) -> ! {
    // SAFETY-INVARIANT: panic = "abort" in agc-core — kein Unwinding,
    // kein Drop, direkter Hardware-Reset. Der Alarmcode ist im panic-String
    // sichtbar (z. B. im ITM-Trace oder RTT-Log).
    panic!("GOJAM: alarm code {:#06x}", code);
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{AlarmState, ALARM_QUEUE_DEPTH};

    #[test]
    fn new_state_is_empty() {
        let s = AlarmState::new();
        assert!(s.last().is_none());
        assert_eq!(s.as_slice().len(), 0);
    }

    #[test]
    fn push_single_code() {
        let mut s = AlarmState::new();
        s.push(0x0210);
        assert_eq!(s.last(), Some(0x0210));
        assert_eq!(s.as_slice(), &[0x0210]);
    }

    #[test]
    fn push_fills_queue() {
        let mut s = AlarmState::new();
        for i in 0..ALARM_QUEUE_DEPTH {
            s.push(i as u16 + 1);
        }
        assert_eq!(s.as_slice().len(), ALARM_QUEUE_DEPTH);
        assert_eq!(s.last(), Some(ALARM_QUEUE_DEPTH as u16));
    }

    #[test]
    fn push_overflow_drops_oldest() {
        let mut s = AlarmState::new();
        // Queue füllen: [1, 2, 3]
        s.push(1);
        s.push(2);
        s.push(3);
        // Überlauf: ältester (1) wird verworfen → [2, 3, 4]
        s.push(4);
        assert_eq!(s.as_slice(), &[2, 3, 4]);
        assert_eq!(s.last(), Some(4));
    }

    #[test]
    fn clear_resets_state() {
        let mut s = AlarmState::new();
        s.push(0xFFFF);
        s.clear();
        assert!(s.last().is_none());
        assert_eq!(s.as_slice().len(), 0);
    }

    #[test]
    fn raise_delegates_to_push() {
        let mut s = AlarmState::new();
        super::raise(&mut s, 0x1202);
        assert_eq!(s.last(), Some(0x1202));
    }
}
