//! Sim-DSKY: [`DskyHal`]-Implementierung für den Terminal-Simulator.
//!
//! `SimDsky` puffert alle Ausgaben von `agc-core` in einem gemeinsam genutzten
//! [`DskyState`]-Struct, das die TUI in Task 12 ausliest.  Tastatureingaben
//! werden über eine feste FIFO-Warteschlange injiziert.
//!
//! # Design-Entscheidung
//! Da `agc-core` im `no_std`-Kontext läuft und kein Locking kennt, werden
//! Display-Worte und Status-Bits in flachen Feldern gespeichert.  Die TUI
//! (Task 12) liest `DskyState` nach jedem Simulations-Tick aus (Single-Thread).

use agc_core::hal::DskyHal;

// ---------------------------------------------------------------------------
// Gemeinsamer Anzeigestatus
// ---------------------------------------------------------------------------

/// Puffer für alle Display-Ausgaben, die agc-core über das DSKY sendet.
///
/// Wird von `SimDsky` gefüllt und von der TUI (Task 12) gelesen.
#[derive(Clone, Copy, Debug, Default)]
pub struct DskyState {
    /// Zuletzt empfangenes Kanal-10-Display-Datenwort.
    pub last_display_word: u16,
    /// Zuletzt empfangene Kanal-11-Statuslichter.
    pub last_status_bits: u16,
    /// Anzahl bisher empfangener Display-Worte (zum Testen).
    pub display_count: u32,
}

// ---------------------------------------------------------------------------
// SimDsky
// ---------------------------------------------------------------------------

/// DSKY-HAL-Implementierung für den Simulator.
///
/// Puffert Display-Ausgaben in [`DskyState`] und liefert injizierte
/// Tastatureingaben via FIFO.
///
/// # Tastatur-FIFO
/// Feste Kapazität von 16 Keycodes.  Einträge werden mit [`SimDsky::push_key`]
/// hinzugefügt und von `agc-core` über [`DskyHal::read_keycode`] konsumiert.
pub struct SimDsky {
    /// Aktueller Anzeigestatus (wird von der TUI gelesen).
    pub state: DskyState,
    key_queue: [Option<u8>; 16],
    key_head: usize,
    key_len: usize,
}

impl SimDsky {
    /// Erstellt ein neues, leeres Sim-DSKY.
    pub fn new() -> Self {
        Self {
            state: DskyState::default(),
            key_queue: [None; 16],
            key_head: 0,
            key_len: 0,
        }
    }

    /// Fügt einen simulierten Tastendruck in die FIFO-Warteschlange ein.
    ///
    /// Keycode-Belegung: AGC-Tastaturcode 0–31.
    /// Falls die Warteschlange voll ist, wird der Tastendruck verworfen.
    pub fn push_key(&mut self, code: u8) {
        if self.key_len < self.key_queue.len() {
            let idx = (self.key_head + self.key_len) % self.key_queue.len();
            self.key_queue[idx] = Some(code);
            self.key_len += 1;
        }
    }

    /// Gibt zurück wie viele Tastendrücke noch in der Warteschlange sind.
    pub fn pending_keys(&self) -> usize {
        self.key_len
    }
}

impl Default for SimDsky {
    fn default() -> Self {
        Self::new()
    }
}

impl DskyHal for SimDsky {
    /// Speichert das Display-Datenwort und inkrementiert den Zähler.
    ///
    /// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`,
    /// DSPOUT / DSPIN routine.
    fn write_display_word(&mut self, word: u16) {
        self.state.last_display_word = word;
        self.state.display_count = self.state.display_count.saturating_add(1);
    }

    /// Speichert die Statuslichter.
    ///
    /// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`,
    /// Kanal-011-WOR/WAND operations.
    fn write_status_lights(&mut self, bits: u16) {
        self.state.last_status_bits = bits;
    }

    /// Liefert den nächsten Tastendruck aus der FIFO-Warteschlange.
    ///
    /// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`,
    /// KEYRUPT1 / KEYRUPT2.
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
