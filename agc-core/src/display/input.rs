//! V25-Dateneingabe-Automat (PINBALL ENTPAS/STOREDAT).
//!
//! Dieser Automat akkumuliert Ziffern, Vorzeichen und verwaltet den
//! R1/R2/R3-Ladepuffer für V25 (Load Component). Nach dem ENTER bei R3
//! wird [`super::dispatcher::VerbDispatcher::accept_load`] aufgerufen.
//!
//! AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`,
//! ENTPAS / STOREDAT.

use crate::context::AgcContext;
use crate::display::dispatcher::VerbDispatcher;
use crate::display::verb_noun::{DisplayRegister, KEY_CLR, KEY_ENTER, KEY_MINUS, KEY_PLUS};

// ---------------------------------------------------------------------------
// InputLoadState
// ---------------------------------------------------------------------------

/// Zustand des V25-Dateneingabe-Automaten (PINBALL ENTPAS/STOREDAT).
///
/// Akkumuliert die Crew-Tastatureingabe für V25 (Load Component) —
/// bis zu 5 Dezimalziffern mit Vorzeichen pro Register.
/// Wird nach vollständigem R3-ENTER via [`VerbDispatcher::accept_load`]
/// an den Dispatcher übergeben.
///
/// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`,
/// ENTPAS / STOREDAT.
#[derive(Debug, Copy, Clone)]
pub struct InputLoadState {
    /// Zwischenpuffer für R1 (entspricht AGC XREG).
    pub r1: i32,
    /// Zwischenpuffer für R2 (entspricht AGC YREG).
    pub r2: i32,
    /// Zwischenpuffer für R3 (entspricht AGC ZREG).
    pub r3: i32,
    /// Index des nächsten zu befüllenden Lade-Registers (0=R1, 1=R2, 2=R3).
    pub reg_idx: u8,
    /// Akkumulierte Ziffern des aktuellen Eingabewerts (Absolutwert).
    pub num_buf: i32,
    /// Vorzeichen des aktuellen Zahlenpuffers: +1 oder −1.
    pub num_sign: i32,
    /// Anzahl der bereits eingetippten Ziffern (max. 5).
    pub num_digits: u8,
}

impl InputLoadState {
    /// Initialer Zustand: alle Register 0, Vorzeichen +1, leerer Puffer.
    pub const fn new() -> Self {
        Self {
            r1: 0,
            r2: 0,
            r3: 0,
            reg_idx: 0,
            num_buf: 0,
            num_sign: 1,
            num_digits: 0,
        }
    }

    /// Setzt den Ziffernpuffer zurück (für nächste Registereingabe).
    fn reset_buf(&mut self) {
        self.num_buf = 0;
        self.num_sign = 1;
        self.num_digits = 0;
    }
}

impl Default for InputLoadState {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// process_v25_key
// ---------------------------------------------------------------------------

/// Verarbeitet einen Tastendruck im V25-Dateneingabe-Modus.
///
/// Gibt `true` zurück wenn der Keycode konsumiert wurde und nicht an
/// [`crate::display::verb_noun::process_key`] weitergeleitet werden soll.
///
/// # Verhalten
/// - Ziffern 0–9: in `ctx.load_state.num_buf` akkumulieren (max. 5 Stellen; weiteres wird ignoriert).
/// - `KEY_PLUS` / `KEY_MINUS`: Vorzeichen setzen, Puffer zurücksetzen.
/// - `KEY_CLR`: Puffer und Vorzeichen zurücksetzen, Operator-Error löschen.
/// - `KEY_ENTER` (wenn mindestens 1 Ziffer): Wert in aktuelles Register schreiben,
///   `reg_idx` inkrementieren. Bei R3: [`VerbDispatcher::accept_load`] aufrufen,
///   `reg_idx` auf 0 zurücksetzen.
/// - `KEY_ENTER` (ohne vorherige Ziffern): **nicht** konsumiert → `false`.
/// - Alle anderen Tasten: nicht konsumiert → `false`.
///
/// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`,
/// ENTPAS / STOREDAT.
pub fn process_v25_key(key: u8, ctx: &mut AgcContext) -> bool {
    match key {
        // --- Vorzeichen ---
        KEY_PLUS => {
            ctx.load_state.num_sign = 1;
            ctx.load_state.num_buf = 0;
            ctx.load_state.num_digits = 0;
            ctx.display.lights.operator_error = false;
            true
        }
        KEY_MINUS => {
            ctx.load_state.num_sign = -1;
            ctx.load_state.num_buf = 0;
            ctx.load_state.num_digits = 0;
            ctx.display.lights.operator_error = false;
            true
        }
        // --- Ziffern 0–9 ---
        0..=9 => {
            if ctx.load_state.num_digits < 5 {
                ctx.load_state.num_buf = ctx.load_state.num_buf * 10 + key as i32;
                ctx.load_state.num_digits += 1;
                // Live-Anzeige im aktuellen Register
                let live =
                    DisplayRegister::Decimal(ctx.load_state.num_sign * ctx.load_state.num_buf);
                match ctx.load_state.reg_idx {
                    0 => ctx.display.r1 = live,
                    1 => ctx.display.r2 = live,
                    _ => ctx.display.r3 = live,
                }
            }
            // Overflow-Ziffern werden stillschweigend ignoriert (Taste konsumiert)
            true
        }
        // --- CLR ---
        KEY_CLR => {
            ctx.load_state.num_buf = 0;
            ctx.load_state.num_sign = 1;
            ctx.load_state.num_digits = 0;
            ctx.display.lights.operator_error = false;
            true
        }
        // --- ENTER mit mindestens einer Ziffer ---
        KEY_ENTER if ctx.load_state.num_digits > 0 => {
            let val = ctx.load_state.num_sign * ctx.load_state.num_buf;
            match ctx.load_state.reg_idx {
                0 => {
                    ctx.load_state.r1 = val;
                    ctx.display.r1 = DisplayRegister::Decimal(val);
                    ctx.load_state.reg_idx = 1;
                }
                1 => {
                    ctx.load_state.r2 = val;
                    ctx.display.r2 = DisplayRegister::Decimal(val);
                    ctx.load_state.reg_idx = 2;
                }
                _ => {
                    ctx.load_state.r3 = val;
                    ctx.display.r3 = DisplayRegister::Decimal(val);
                    // Alle drei Register befüllt → accept_load aufrufen
                    let (r1, r2, r3) = (ctx.load_state.r1, ctx.load_state.r2, ctx.load_state.r3);
                    VerbDispatcher::accept_load(r1, r2, r3, ctx);
                    ctx.load_state.reg_idx = 0;
                }
            }
            ctx.load_state.reset_buf();
            true
        }
        // --- Alle anderen Tasten (inkl. ENTER ohne Ziffern) ---
        _ => false,
    }
}

// ---------------------------------------------------------------------------
// Unit-Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::context::AgcContext;
    use crate::display::verb_noun::{DisplayRegister, NounCode, VerbCode};

    fn make_ctx_v25(noun_val: u8) -> AgcContext {
        let mut ctx = AgcContext::default();
        ctx.display.verb = VerbCode::new(25).unwrap();
        ctx.display.noun = NounCode::new(noun_val).unwrap();
        ctx.display.flash_state = true;
        ctx
    }

    // --- Zifferneingabe ---

    #[test]
    fn digit_accumulation() {
        let mut ctx = make_ctx_v25(33);

        // Tippe "1", "2", "3"
        assert!(process_v25_key(1, &mut ctx));
        assert!(process_v25_key(2, &mut ctx));
        assert!(process_v25_key(3, &mut ctx));

        assert_eq!(ctx.load_state.num_buf, 123);
        assert_eq!(ctx.load_state.num_digits, 3);
        assert_eq!(ctx.display.r1, DisplayRegister::Decimal(123));
    }

    // --- Vorzeichenwechsel ---

    #[test]
    fn sign_change() {
        let mut ctx = make_ctx_v25(33);

        assert!(process_v25_key(5, &mut ctx));
        assert!(process_v25_key(KEY_MINUS, &mut ctx));
        // Vorzeichenwechsel setzt Puffer zurück
        assert_eq!(ctx.load_state.num_buf, 0);
        assert_eq!(ctx.load_state.num_sign, -1);
        assert_eq!(ctx.load_state.num_digits, 0);

        assert!(process_v25_key(7, &mut ctx));
        assert_eq!(ctx.load_state.num_buf, 7);
        assert_eq!(ctx.display.r1, DisplayRegister::Decimal(-7));
    }

    // --- CLR ---

    #[test]
    fn clr_resets_buf() {
        let mut ctx = make_ctx_v25(33);

        assert!(process_v25_key(9, &mut ctx));
        assert!(process_v25_key(9, &mut ctx));
        assert!(process_v25_key(KEY_CLR, &mut ctx));

        assert_eq!(ctx.load_state.num_buf, 0);
        assert_eq!(ctx.load_state.num_sign, 1);
        assert_eq!(ctx.load_state.num_digits, 0);
        assert!(!ctx.display.lights.operator_error);
    }

    // --- ENTER-Kette R1→R2→R3 mit accept_load ---

    #[test]
    fn enter_chain_r1_r2_r3() {
        // N33 → TIG (HH:MM:SS) — accept_load setzt guidance.tig_s
        let mut ctx = make_ctx_v25(33);

        // R1: 1 ENTER
        assert!(process_v25_key(1, &mut ctx));
        assert!(process_v25_key(KEY_ENTER, &mut ctx));
        assert_eq!(ctx.load_state.r1, 1);
        assert_eq!(ctx.load_state.reg_idx, 1);

        // R2: 2 ENTER
        assert!(process_v25_key(2, &mut ctx));
        assert!(process_v25_key(KEY_ENTER, &mut ctx));
        assert_eq!(ctx.load_state.r2, 2);
        assert_eq!(ctx.load_state.reg_idx, 2);

        // R3: 3 ENTER → accept_load → tig_s = 1*3600 + 2*60 + 3 = 3723 s
        assert!(process_v25_key(3, &mut ctx));
        assert!(process_v25_key(KEY_ENTER, &mut ctx));
        assert_eq!(ctx.load_state.r3, 3);
        assert_eq!(ctx.load_state.reg_idx, 0);
        assert!(
            (ctx.guidance.tig_s - 3723.0).abs() < 1e-9,
            "tig_s={} != 3723",
            ctx.guidance.tig_s
        );
    }

    // --- Overflow: mehr als 5 Ziffern werden ignoriert ---

    #[test]
    fn overflow_ignored() {
        let mut ctx = make_ctx_v25(33);

        // 6 Ziffern eingeben: nur erste 5 zählen
        for d in 0..6_u8 {
            process_v25_key(d + 1, &mut ctx); // 1,2,3,4,5,(6 ignoriert)
        }
        assert_eq!(ctx.load_state.num_digits, 5);
        assert_eq!(ctx.load_state.num_buf, 12345);
    }

    // --- ENTER ohne vorherige Ziffern: nicht konsumiert ---

    #[test]
    fn enter_without_digits_not_consumed() {
        let mut ctx = make_ctx_v25(33);

        let consumed = process_v25_key(KEY_ENTER, &mut ctx);
        assert!(!consumed, "ENTER ohne Ziffern darf nicht konsumiert werden");
    }
}
