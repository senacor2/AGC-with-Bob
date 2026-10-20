//! Display-Zustandstypen und Protokollkern für das DSKY-PINBALL-System.
//!
//! Dieses Modul definiert alle Zustandstypen (Sub-Task 7.1) sowie den
//! zentralen [`DisplayState`]-Struct mit T4RUPT-Takt und HAL-Ausgabe
//! (Sub-Task 7.2).
//!
//! AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`,
//! PINBALL / DSPTAB / DSPOUT-Bereich.

// ---------------------------------------------------------------------------
// Sub-Task 7.1 — Newtypes
// ---------------------------------------------------------------------------

/// AGC Verb-Code (VERBREG), gültige Werte 0–99.
///
/// Kodiert den aktuell angezeigten oder angeforderten Verb.
///
/// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, VERBREG.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct VerbCode(u8);

impl VerbCode {
    /// Erstellt einen neuen `VerbCode`.  Gibt `None` zurück wenn `v > 99`.
    pub const fn new(v: u8) -> Option<Self> {
        if v <= 99 {
            Some(Self(v))
        } else {
            None
        }
    }

    /// Roher Wert des Verb-Codes.
    pub const fn val(self) -> u8 {
        self.0
    }
}

/// AGC Noun-Code (NOUNREG), gültige Werte 0–99.
///
/// Kodiert den aktuell angezeigten oder angeforderten Noun.
///
/// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, NOUNREG.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct NounCode(u8);

impl NounCode {
    /// Erstellt einen neuen `NounCode`.  Gibt `None` zurück wenn `n > 99`.
    pub const fn new(n: u8) -> Option<Self> {
        if n <= 99 {
            Some(Self(n))
        } else {
            None
        }
    }

    /// Roher Wert des Noun-Codes.
    pub const fn val(self) -> u8 {
        self.0
    }
}

/// AGC Major-Mode / Programm-Nummer (MODREG), gültige Werte 0–99.
///
/// Entspricht dem „PROG"-Anzeigefeld des DSKY.
///
/// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, MODREG.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct MajorMode(u8);

impl MajorMode {
    /// Erstellt einen neuen `MajorMode`.  Gibt `None` zurück wenn `m > 99`.
    pub const fn new(m: u8) -> Option<Self> {
        if m <= 99 {
            Some(Self(m))
        } else {
            None
        }
    }

    /// Roher Wert des Major-Mode.
    pub const fn val(self) -> u8 {
        self.0
    }
}

/// Roher AGC-Tastaturkode (KEYRUPT-Format), gültige Werte 0–31.
///
/// 5-Bit-Tastaturcode aus Kanal 015/016 (KEYRUPT1/KEYRUPT2).
///
/// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`,
/// KEYRUPT1 / KEYRUPT2-Interrupt-Handler.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct KeyCode(u8);

impl KeyCode {
    /// Erstellt einen neuen `KeyCode`.  Gibt `None` zurück wenn `k > 31`.
    pub const fn new(k: u8) -> Option<Self> {
        if k <= 31 {
            Some(Self(k))
        } else {
            None
        }
    }

    /// Roher Wert des Tastaturcodes.
    pub const fn val(self) -> u8 {
        self.0
    }
}

// ---------------------------------------------------------------------------
// Sub-Task 7.1 — DisplayRegister-Enum
// ---------------------------------------------------------------------------

/// Inhalt eines der drei DSKY-Anzeigeregister R1, R2 oder R3.
///
/// Das DSKY zeigt jeweils fünf Stellen pro Register an; dieses Enum
/// kodiert die semantische Bedeutung des Wertes.
///
/// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, DSPTAB.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum DisplayRegister {
    /// Register ist leer/nicht belegt.
    Blank,
    /// Oktalwert, 0–077777 (passt in `u16`).
    Octal(u16),
    /// Dezimalwert ±99999.
    Decimal(i32),
    /// Hochpräziser Dezimalwert: Ganzzahlanteil + Nachkommaanteil.
    DecimalHighPrec(i32, i32),
}

// ---------------------------------------------------------------------------
// Sub-Task 7.1 — StatusLights-Struct
// ---------------------------------------------------------------------------

/// Zustand aller DSKY-Statuslichter (Warnlampen).
///
/// Entspricht den Kanal-11-Bits des AGC.  Bit-Belegung (1-basiert, LSB = Bit 1):
///
/// | Feld             | Kanal-11-Bit |
/// |------------------|-------------|
/// | `key_release`    | Bit  5      |
/// | `verb_noun_flash`| Bit  6      |
/// | `operator_error` | Bit  7      |
/// | `uplink_acty`    | Bit 10      |
/// | `no_att`         | Bit 11      |
/// | `gimbal_lock`    | Bit 12      |
/// | `tracker`        | Bit 13      |
/// | `alt`            | Bit 14      |
/// | `vel`            | Bit 15      |
///
/// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`,
/// Kanal-011-WOR/WAND-Operationen.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct StatusLights {
    /// KEY RELEASE — Tastatur-Freigabe-Lampe.
    pub key_release: bool,
    /// VERB/NOUN FLASH — blinkendes Verb/Noun-Anzeigefeld.
    pub verb_noun_flash: bool,
    /// OPERATOR ERROR — Bedienerfehler-Lampe.
    pub operator_error: bool,
    /// UPLINK ACTY — Uplink-Aktivitäts-Lampe.
    pub uplink_acty: bool,
    /// NO ATT — Keine-Lage-Referenz-Lampe.
    pub no_att: bool,
    /// GIMBAL LOCK — Kardanschloss-Warnung.
    pub gimbal_lock: bool,
    /// TRACKER — Optischer Tracker aktiv.
    pub tracker: bool,
    /// ALT — Höhenanzeige aktiv.
    pub alt: bool,
    /// VEL — Geschwindigkeitsanzeige aktiv.
    pub vel: bool,
}

impl StatusLights {
    /// Alle Statuslichter ausgeschaltet.
    pub const fn new() -> Self {
        Self {
            key_release: false,
            verb_noun_flash: false,
            operator_error: false,
            uplink_acty: false,
            no_att: false,
            gimbal_lock: false,
            tracker: false,
            alt: false,
            vel: false,
        }
    }

    /// Serialisiert den Lampenstatus in ein AGC-Kanal-11-Wort.
    ///
    /// Bit N (1-basiert) = `1u16 << (N-1)`.
    ///
    /// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`,
    /// Kanal-011-Ausgabe.
    pub fn to_channel11_bits(&self) -> u16 {
        let mut bits: u16 = 0;
        if self.key_release {
            bits |= 1u16 << 4; // Bit 5
        }
        if self.verb_noun_flash {
            bits |= 1u16 << 5; // Bit 6
        }
        if self.operator_error {
            bits |= 1u16 << 6; // Bit 7
        }
        if self.uplink_acty {
            bits |= 1u16 << 9; // Bit 10
        }
        if self.no_att {
            bits |= 1u16 << 10; // Bit 11
        }
        if self.gimbal_lock {
            bits |= 1u16 << 11; // Bit 12
        }
        if self.tracker {
            bits |= 1u16 << 12; // Bit 13
        }
        if self.alt {
            bits |= 1u16 << 13; // Bit 14
        }
        if self.vel {
            bits |= 1u16 << 14; // Bit 15
        }
        bits
    }
}

impl Default for StatusLights {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Sub-Task 7.2 — DisplayState
// ---------------------------------------------------------------------------

/// Gesamter DSKY-Anzeigestatus (PINBALL-Erasable-Bereich).
///
/// Enthält alle Zustandsfelder, die der PINBALL-Subsystem im AGC in
/// Erasable Memory (VERBREG, NOUNREG, MODREG, DSPTAB, DSPLOCK) hält.
///
/// # Felder
/// - `verb`, `noun`, `major_mode`: Angezeigte Verb-/Noun-/Prog-Nummer
/// - `r1`, `r2`, `r3`: Inhalte der drei Anzeigeregister
/// - `lights`: Zustand der Statuslampen (Kanal 11)
/// - `dsp_lock`: DSPLOCK-Interlock (Tastatur gesperrt während interner Anforderung)
/// - `flash_counter`: T4RUPT-Ticks bis zum nächsten Flash-Toggle (0 = kein Blinken aktiv)
/// - `flash_state`: Gibt an ob das Blinken gerade läuft (`true` = aktiv)
///
/// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`,
/// DSPTAB-Register / DSPLOCK-Erasable.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct DisplayState {
    /// Aktuell angezeigter Verb-Code (VERBREG).
    pub verb: VerbCode,
    /// Aktuell angezeigter Noun-Code (NOUNREG).
    pub noun: NounCode,
    /// Aktuell angezeigter Major-Mode / Programm-Nummer (MODREG).
    pub major_mode: MajorMode,
    /// Anzeigeregister R1.
    pub r1: DisplayRegister,
    /// Anzeigeregister R2.
    pub r2: DisplayRegister,
    /// Anzeigeregister R3.
    pub r3: DisplayRegister,
    /// Zustand der neun Statuslampen (Kanal 11).
    pub lights: StatusLights,
    /// DSPLOCK-Interlock: `true` = Tastatur während interner Anforderung gesperrt.
    pub dsp_lock: bool,
    /// T4RUPT-Zähler für Verb/Noun-Blinken.
    /// Wird pro T4RUPT (≈10 ms) dekrementiert; bei Überlauf auf 0 wird
    /// `lights.verb_noun_flash` getoggelt und der Zähler zurückgesetzt.
    /// 0 bedeutet: Blinken ist nicht aktiv.
    pub flash_counter: u8,
    /// `true` solange das Blinken läuft (Flip-Flop für start/stop).
    pub flash_state: bool,
}

/// Anzahl T4RUPT-Ticks pro Blink-Halbperiode (25 × 10 ms = 250 ms).
///
/// Ergibt eine Blinkfrequenz von 500 ms (2 Hz) — AGC-typischer Wert für
/// VERB/NOUN FLASH.
///
/// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, T4RUPT.
const FLASH_HALF_PERIOD_TICKS: u8 = 25;

impl DisplayState {
    /// Initialer Zustand: alle Register leer, alle Lampen aus, kein Blinken.
    pub const fn new() -> Self {
        // Safety: VerbCode(0), NounCode(0), MajorMode(0) sind alle gültig (≤99).
        Self {
            verb: VerbCode(0),
            noun: NounCode(0),
            major_mode: MajorMode(0),
            r1: DisplayRegister::Blank,
            r2: DisplayRegister::Blank,
            r3: DisplayRegister::Blank,
            lights: StatusLights::new(),
            dsp_lock: false,
            flash_counter: 0,
            flash_state: false,
        }
    }
}

impl Default for DisplayState {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Sub-Task 7.2 — t4rupt_tick / start_flash / stop_flash
// ---------------------------------------------------------------------------

/// Wird pro T4RUPT-Interrupt (ca. alle 10 ms) aufgerufen.
///
/// Dekrementiert `flash_counter` und toggled `lights.verb_noun_flash`
/// beim Zählerüberlauf auf 0, sofern `flash_state == true`.
///
/// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`,
/// T4RUPT / DSPOUT-Routine.
pub fn t4rupt_tick(state: &mut DisplayState) {
    if state.flash_counter > 0 {
        state.flash_counter -= 1;
    } else if state.flash_state {
        // Zähler auf 0 und Blinken aktiv → Toggle + Zähler neu laden
        state.lights.verb_noun_flash = !state.lights.verb_noun_flash;
        state.flash_counter = FLASH_HALF_PERIOD_TICKS;
    }
}

/// Startet das VERB/NOUN-Blinken.
///
/// Setzt `flash_counter` auf die AGC-typische Halbperiode (25 T4RUPT-Ticks)
/// und markiert `flash_state = true`.
///
/// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`,
/// DSPOUT / NVSBWAIT Flash-Initialisierung.
pub fn start_flash(state: &mut DisplayState) {
    state.flash_counter = FLASH_HALF_PERIOD_TICKS;
    state.flash_state = true;
}

/// Stoppt das VERB/NOUN-Blinken und schaltet die Lampe aus.
///
/// Setzt `flash_state = false`, `flash_counter = 0` und
/// `lights.verb_noun_flash = false`.
///
/// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`,
/// DSPOUT / Verb/Noun-Flash-Reset.
pub fn stop_flash(state: &mut DisplayState) {
    state.flash_state = false;
    state.flash_counter = 0;
    state.lights.verb_noun_flash = false;
}

// ---------------------------------------------------------------------------
// Sub-Task 7.2 — encode_display_word / write_to_dsky
// ---------------------------------------------------------------------------

/// Kodiert den aktuellen `DisplayState` in ein vereinfachtes 16-Bit-Display-Wort.
///
/// Vereinfachtes Kanal-10-Format (keine echte 7-Segment-Kodierung — kommt in Task 12):
/// - Bits 14–8: Verb-Code (7 Bit)
/// - Bits  7–1: Noun-Code (7 Bit)
/// - Bit   0  : `flash_state` (1 = Blinken aktiv)
///
/// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, DSPOUT.
fn encode_display_word(state: &DisplayState) -> u16 {
    let verb_bits = (state.verb.val() as u16) & 0x7F;
    let noun_bits = (state.noun.val() as u16) & 0x7F;
    let flash_bit = state.flash_state as u16;
    (verb_bits << 8) | (noun_bits << 1) | flash_bit
}

/// Schreibt den vollständigen `DisplayState` über die DSKY-HAL aus.
///
/// Ruft nacheinander auf:
/// 1. `dsky.write_status_lights(state.lights.to_channel11_bits())`
/// 2. `dsky.write_display_word(encode_display_word(state))`
///
/// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`,
/// T4RUPT / DSPOUT-Routine.
pub fn write_to_dsky<H: crate::hal::dsky::DskyHal>(state: &DisplayState, dsky: &mut H) {
    dsky.write_status_lights(state.lights.to_channel11_bits());
    dsky.write_display_word(encode_display_word(state));
}

// ---------------------------------------------------------------------------
// Sub-Task 7.3 — AGC-Keycode-Konstanten
// ---------------------------------------------------------------------------

/// AGC-Tastaturkode: Ziffer 0.
///
/// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, KEYCODE-Tabelle.
pub const KEY_0: u8 = 0;
/// AGC-Tastaturkode: Ziffer 1.
pub const KEY_1: u8 = 1;
/// AGC-Tastaturkode: Ziffer 2.
pub const KEY_2: u8 = 2;
/// AGC-Tastaturkode: Ziffer 3.
pub const KEY_3: u8 = 3;
/// AGC-Tastaturkode: Ziffer 4.
pub const KEY_4: u8 = 4;
/// AGC-Tastaturkode: Ziffer 5.
pub const KEY_5: u8 = 5;
/// AGC-Tastaturkode: Ziffer 6.
pub const KEY_6: u8 = 6;
/// AGC-Tastaturkode: Ziffer 7.
pub const KEY_7: u8 = 7;
/// AGC-Tastaturkode: Ziffer 8.
pub const KEY_8: u8 = 8;
/// AGC-Tastaturkode: Ziffer 9.
pub const KEY_9: u8 = 9;
/// AGC-Tastaturkode: VERB-Taste (17).
pub const KEY_VERB: u8 = 17;
/// AGC-Tastaturkode: NOUN-Taste (18).
pub const KEY_NOUN: u8 = 18;
/// AGC-Tastaturkode: ENTER (14).
pub const KEY_ENTER: u8 = 14;
/// AGC-Tastaturkode: CLR — löscht laufende Eingabe (28).
pub const KEY_CLR: u8 = 28;
/// AGC-Tastaturkode: PRO / PROCEED = V33 (30).
pub const KEY_PRO: u8 = 30;
/// AGC-Tastaturkode: KEY RELEASE — gibt Tastatur wieder frei (24).
pub const KEY_KEY_REL: u8 = 24;
/// AGC-Tastaturkode: RSET / RESET (22).
pub const KEY_RSET: u8 = 22;
/// AGC-Tastaturkode: + (26).
pub const KEY_PLUS: u8 = 26;
/// AGC-Tastaturkode: − (27).
pub const KEY_MINUS: u8 = 27;

// ---------------------------------------------------------------------------
// Sub-Task 7.3 — InputMode / InputBuffer
// ---------------------------------------------------------------------------

/// Aktueller Eingabemodus des PINBALL-Tastaturautomaten.
///
/// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, CHARIN.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum InputMode {
    /// Kein laufender Eingabevorgang.
    Idle,
    /// Verb-Eingabe aktiv (nach KEY_VERB).
    Verb,
    /// Noun-Eingabe aktiv (nach KEY_NOUN).
    Noun,
}

/// Puffer für eine laufende zweistellige Verb- oder Noun-Eingabe.
///
/// Sammelt bis zu zwei Dezimalziffern (0–9), bevor sie per ENTER
/// als `VerbCode` oder `NounCode` übernommen werden.
///
/// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, CHARIN.
pub struct InputBuffer {
    digits: [u8; 2],
    count: u8,
    /// Welche Taste zuletzt gedrückt wurde (VERB / NOUN / keine).
    pub mode: InputMode,
}

impl InputBuffer {
    /// Leerer Puffer im Idle-Modus.
    pub const fn new() -> Self {
        Self {
            digits: [0; 2],
            count: 0,
            mode: InputMode::Idle,
        }
    }

    /// Fügt eine Dezimalziffer (0–9) in den Puffer ein.
    ///
    /// Gibt `false` zurück wenn der Puffer bereits voll ist (2 Ziffern).
    pub fn push_digit(&mut self, d: u8) -> bool {
        if self.count >= 2 {
            return false;
        }
        self.digits[self.count as usize] = d;
        self.count += 1;
        true
    }

    /// Konvertiert den Pufferinhalt in einen dezimalen `u8`-Wert.
    ///
    /// - 2 Ziffern: `10 * d[0] + d[1]`
    /// - 1 Ziffer:  `d[0]`
    /// - 0 Ziffern: `0`
    pub fn to_value(&self) -> u8 {
        match self.count {
            2 => 10 * self.digits[0] + self.digits[1],
            1 => self.digits[0],
            _ => 0,
        }
    }

    /// Leert den Puffer und setzt den Modus auf [`InputMode::Idle`].
    pub fn clear(&mut self) {
        self.digits = [0; 2];
        self.count = 0;
        self.mode = InputMode::Idle;
    }
}

impl Default for InputBuffer {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Sub-Task 7.3 — KeyEvent-Enum
// ---------------------------------------------------------------------------

/// Ergebnis eines einzelnen verarbeiteten Tastendrucks.
///
/// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, CHARIN.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum KeyEvent {
    /// Keine besondere Aktion (z. B. Zifferneingabe läuft noch).
    None,
    /// Verb-Code wurde per ENTER übernommen.
    VerbSet(VerbCode),
    /// Noun-Code wurde per ENTER übernommen.
    NounSet(NounCode),
    /// ENTER im Idle-Modus — "Accept" / PROCEED-Signal.
    Enter,
    /// PRO / PROCEED (V33).
    Proceed,
    /// Terminate — Anforderung abgebrochen.
    Terminate,
    /// Recycle — Anforderung wiederholt.
    Recycle,
    /// RSET — Alarmlampe gelöscht, Puffer geleert.
    Reset,
    /// OPERATOR ERROR ausgelöst.
    OperatorError,
}

// ---------------------------------------------------------------------------
// Sub-Task 7.3 — process_key
// ---------------------------------------------------------------------------

/// Verarbeitet einen einzelnen AGC-Tastendruck und aktualisiert `DisplayState`.
///
/// Implementiert den CHARIN-Zustandsautomaten (vereinfacht — kein vollständiger
/// PINBALL-Automat, aber korrekte Kernlogik für Ziffern, VERB/NOUN, ENTER, CLR,
/// PRO, KEY REL, RSET).
///
/// # Parameter
/// - `state`  — Aktueller DSKY-Anzeigestatus (wird ggf. modifiziert).
/// - `buf`    — Laufender Eingabepuffer (wird ggf. modifiziert).
/// - `key`    — Rohkeycode (AGC KEYRUPT-Format, 0–31).
/// - `alarm`  — Alarmzustand für unbekannte Keycodes.
///
/// # Rückgabe
/// Ein [`KeyEvent`]-Wert, der beschreibt was mit dem Tastendruck passiert ist.
///
/// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, CHARIN.
pub fn process_key(
    state: &mut DisplayState,
    buf: &mut InputBuffer,
    key: KeyCode,
    alarm: &mut crate::alarm::AlarmState,
) -> KeyEvent {
    match key.val() {
        KEY_VERB => {
            buf.clear();
            buf.mode = InputMode::Verb;
            KeyEvent::None
        }
        KEY_NOUN => {
            buf.clear();
            buf.mode = InputMode::Noun;
            KeyEvent::None
        }
        KEY_0..=KEY_9 => {
            if buf.mode == InputMode::Idle {
                state.lights.operator_error = true;
                return KeyEvent::OperatorError;
            }
            if !buf.push_digit(key.val()) {
                state.lights.operator_error = true;
                return KeyEvent::OperatorError;
            }
            KeyEvent::None
        }
        KEY_ENTER => match buf.mode {
            InputMode::Verb => {
                let val = buf.to_value();
                match VerbCode::new(val) {
                    Some(v) => {
                        state.verb = v;
                        buf.clear();
                        KeyEvent::VerbSet(v)
                    }
                    None => {
                        state.lights.operator_error = true;
                        KeyEvent::OperatorError
                    }
                }
            }
            InputMode::Noun => {
                let val = buf.to_value();
                match NounCode::new(val) {
                    Some(n) => {
                        state.noun = n;
                        buf.clear();
                        KeyEvent::NounSet(n)
                    }
                    None => {
                        state.lights.operator_error = true;
                        KeyEvent::OperatorError
                    }
                }
            }
            InputMode::Idle => KeyEvent::Enter,
        },
        KEY_CLR => {
            buf.clear();
            state.lights.operator_error = false;
            KeyEvent::None
        }
        KEY_PRO => KeyEvent::Proceed,
        KEY_KEY_REL => {
            state.lights.key_release = false;
            KeyEvent::None
        }
        KEY_RSET => {
            state.lights.operator_error = false;
            buf.clear();
            KeyEvent::Reset
        }
        _ => {
            crate::alarm::raise(alarm, 0x0700);
            state.lights.operator_error = true;
            KeyEvent::OperatorError
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hal::dsky::mock::MockDsky;

    // --- VerbCode / NounCode / MajorMode / KeyCode ---

    #[test]
    fn verb_code_new_valid() {
        assert_eq!(VerbCode::new(37), Some(VerbCode(37)));
    }

    #[test]
    fn verb_code_new_invalid() {
        assert_eq!(VerbCode::new(100), None);
    }

    #[test]
    fn verb_code_boundary() {
        assert_eq!(VerbCode::new(99), Some(VerbCode(99)));
        assert_eq!(VerbCode::new(0), Some(VerbCode(0)));
    }

    #[test]
    fn noun_code_new_valid() {
        assert_eq!(NounCode::new(15), Some(NounCode(15)));
    }

    #[test]
    fn noun_code_new_invalid() {
        assert_eq!(NounCode::new(100), None);
    }

    #[test]
    fn major_mode_new_valid() {
        assert_eq!(MajorMode::new(11), Some(MajorMode(11)));
    }

    #[test]
    fn major_mode_new_invalid() {
        assert_eq!(MajorMode::new(100), None);
    }

    #[test]
    fn key_code_new_valid() {
        assert_eq!(KeyCode::new(17), Some(KeyCode(17)));
    }

    #[test]
    fn key_code_new_invalid() {
        assert_eq!(KeyCode::new(32), None);
    }

    #[test]
    fn key_code_boundary() {
        assert_eq!(KeyCode::new(31), Some(KeyCode(31)));
        assert_eq!(KeyCode::new(0), Some(KeyCode(0)));
    }

    #[test]
    fn val_roundtrip() {
        assert_eq!(VerbCode::new(42).unwrap().val(), 42);
        assert_eq!(NounCode::new(9).unwrap().val(), 9);
        assert_eq!(MajorMode::new(0).unwrap().val(), 0);
        assert_eq!(KeyCode::new(31).unwrap().val(), 31);
    }

    // --- StatusLights::to_channel11_bits ---

    #[test]
    fn status_lights_default_zero() {
        assert_eq!(StatusLights::new().to_channel11_bits(), 0);
    }

    #[test]
    fn status_lights_to_channel11_key_release() {
        let mut l = StatusLights::new();
        l.key_release = true;
        // Bit 5 (1-basiert) = 1 << 4 = 0x0010
        assert_eq!(l.to_channel11_bits(), 1u16 << 4);
    }

    #[test]
    fn status_lights_to_channel11_verb_noun_flash() {
        let mut l = StatusLights::new();
        l.verb_noun_flash = true;
        assert_eq!(l.to_channel11_bits(), 1u16 << 5);
    }

    #[test]
    fn status_lights_to_channel11_all() {
        let l = StatusLights {
            key_release: true,
            verb_noun_flash: true,
            operator_error: true,
            uplink_acty: true,
            no_att: true,
            gimbal_lock: true,
            tracker: true,
            alt: true,
            vel: true,
        };
        let expected: u16 = (1 << 4)
            | (1 << 5)
            | (1 << 6)
            | (1 << 9)
            | (1 << 10)
            | (1 << 11)
            | (1 << 12)
            | (1 << 13)
            | (1 << 14);
        assert_eq!(l.to_channel11_bits(), expected);
    }

    // --- t4rupt_tick / start_flash / stop_flash ---

    #[test]
    fn t4rupt_tick_decrements_counter() {
        let mut state = DisplayState::new();
        start_flash(&mut state);
        assert_eq!(state.flash_counter, 25);
        t4rupt_tick(&mut state);
        assert_eq!(state.flash_counter, 24);
        assert!(!state.lights.verb_noun_flash); // noch kein Toggle
    }

    #[test]
    fn t4rupt_tick_toggles_flash_after_25_ticks() {
        let mut state = DisplayState::new();
        start_flash(&mut state);
        // Erste Halbperiode: 25 Decrements bis counter=0, dann beim nächsten Tick Toggle
        for _ in 0..25 {
            t4rupt_tick(&mut state);
        }
        // Nach 25 Ticks ist counter=0; nächster Tick löst Toggle aus
        assert_eq!(state.flash_counter, 0);
        t4rupt_tick(&mut state);
        assert!(state.lights.verb_noun_flash);
        assert_eq!(state.flash_counter, 25); // Zähler neu geladen
    }

    #[test]
    fn t4rupt_tick_double_toggle() {
        let mut state = DisplayState::new();
        start_flash(&mut state);
        // Erste Halbperiode: 25 Ticks + 1 Toggle-Tick
        for _ in 0..26 {
            t4rupt_tick(&mut state);
        }
        assert!(state.lights.verb_noun_flash);
        // Zweite Halbperiode: 25 Ticks + 1 Toggle-Tick
        for _ in 0..26 {
            t4rupt_tick(&mut state);
        }
        assert!(!state.lights.verb_noun_flash);
    }

    #[test]
    fn stop_flash_clears() {
        let mut state = DisplayState::new();
        start_flash(&mut state);
        stop_flash(&mut state);
        assert!(!state.flash_state);
        assert_eq!(state.flash_counter, 0);
        assert!(!state.lights.verb_noun_flash);
    }

    #[test]
    fn t4rupt_no_toggle_when_stopped() {
        let mut state = DisplayState::new();
        // Kein start_flash → kein Toggle, auch wenn counter zufällig 0
        for _ in 0..50 {
            t4rupt_tick(&mut state);
        }
        assert!(!state.lights.verb_noun_flash);
    }

    // --- write_to_dsky mit MockDsky ---

    #[test]
    fn write_to_dsky_calls_hal() {
        let mut state = DisplayState::new();
        // Verb 37, Noun 15 einstellen
        state.verb = VerbCode(37);
        state.noun = NounCode(15);
        state.lights.key_release = true;

        let mut mock = MockDsky::new();
        write_to_dsky(&state, &mut mock);

        // Status-Lights: key_release → Bit 5 = 1<<4
        assert_eq!(mock.last_status_bits, 1u16 << 4);

        // Display-Wort: verb=37 (0x25), noun=15 (0x0F), flash_state=false
        // Bits 14–8: 37 << 8 = 0x2500
        // Bits  7–1: 15 << 1 = 0x001E
        // Bit 0: 0
        let expected_word: u16 = (37u16 << 8) | (15u16 << 1);
        assert_eq!(mock.last_display_word, expected_word);
    }

    #[test]
    fn write_to_dsky_flash_bit() {
        let mut state = DisplayState::new();
        state.verb = VerbCode(1);
        state.noun = NounCode(1);
        start_flash(&mut state);
        // flash_state = true → Bit 0 gesetzt
        let mut mock = MockDsky::new();
        write_to_dsky(&state, &mut mock);
        assert_eq!(mock.last_display_word & 1, 1);
    }

    // --- Sub-Task 7.3: process_key ---

    fn make_key(k: u8) -> KeyCode {
        KeyCode::new(k).unwrap()
    }

    #[test]
    fn verb_entry_sequence() {
        let mut state = DisplayState::new();
        let mut buf = InputBuffer::new();
        let mut alarm = crate::alarm::AlarmState::new();

        process_key(&mut state, &mut buf, make_key(KEY_VERB), &mut alarm);
        process_key(&mut state, &mut buf, make_key(KEY_3), &mut alarm);
        process_key(&mut state, &mut buf, make_key(KEY_7), &mut alarm);
        let ev = process_key(&mut state, &mut buf, make_key(KEY_ENTER), &mut alarm);

        assert_eq!(ev, KeyEvent::VerbSet(VerbCode(37)));
        assert_eq!(state.verb, VerbCode(37));
        // Puffer muss danach leer/Idle sein
        assert_eq!(buf.mode, InputMode::Idle);
    }

    #[test]
    fn noun_entry_sequence() {
        let mut state = DisplayState::new();
        let mut buf = InputBuffer::new();
        let mut alarm = crate::alarm::AlarmState::new();

        process_key(&mut state, &mut buf, make_key(KEY_NOUN), &mut alarm);
        process_key(&mut state, &mut buf, make_key(KEY_1), &mut alarm);
        process_key(&mut state, &mut buf, make_key(KEY_8), &mut alarm);
        let ev = process_key(&mut state, &mut buf, make_key(KEY_ENTER), &mut alarm);

        assert_eq!(ev, KeyEvent::NounSet(NounCode(18)));
        assert_eq!(state.noun, NounCode(18));
    }

    #[test]
    fn digit_without_mode_is_error() {
        let mut state = DisplayState::new();
        let mut buf = InputBuffer::new();
        let mut alarm = crate::alarm::AlarmState::new();

        let ev = process_key(&mut state, &mut buf, make_key(KEY_1), &mut alarm);

        assert_eq!(ev, KeyEvent::OperatorError);
        assert!(state.lights.operator_error);
    }

    #[test]
    fn clr_resets_buffer_and_error() {
        let mut state = DisplayState::new();
        let mut buf = InputBuffer::new();
        let mut alarm = crate::alarm::AlarmState::new();

        // VERB → 5 → CLR → 3 → 7 → ENTER  ⟹  VerbSet(37)
        process_key(&mut state, &mut buf, make_key(KEY_VERB), &mut alarm);
        process_key(&mut state, &mut buf, make_key(KEY_5), &mut alarm);
        process_key(&mut state, &mut buf, make_key(KEY_CLR), &mut alarm);
        // Nach CLR: Puffer leer, Modus Idle → wir müssen erneut VERB drücken
        process_key(&mut state, &mut buf, make_key(KEY_VERB), &mut alarm);
        process_key(&mut state, &mut buf, make_key(KEY_3), &mut alarm);
        process_key(&mut state, &mut buf, make_key(KEY_7), &mut alarm);
        let ev = process_key(&mut state, &mut buf, make_key(KEY_ENTER), &mut alarm);

        assert_eq!(ev, KeyEvent::VerbSet(VerbCode(37)));
        assert!(!state.lights.operator_error);
    }

    #[test]
    fn invalid_verb_overflow_is_error() {
        // KEY_9 KEY_9 füllt den Puffer (Wert 99, gültig).
        // Ein dritter KEY_9 löst OperatorError aus (Puffer voll).
        let mut state = DisplayState::new();
        let mut buf = InputBuffer::new();
        let mut alarm = crate::alarm::AlarmState::new();

        process_key(&mut state, &mut buf, make_key(KEY_VERB), &mut alarm);
        process_key(&mut state, &mut buf, make_key(KEY_9), &mut alarm);
        process_key(&mut state, &mut buf, make_key(KEY_9), &mut alarm);
        let ev = process_key(&mut state, &mut buf, make_key(KEY_9), &mut alarm);

        assert_eq!(ev, KeyEvent::OperatorError);
        assert!(state.lights.operator_error);
    }
}
