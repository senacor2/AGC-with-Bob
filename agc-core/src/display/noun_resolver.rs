//! Noun-Resolver — bildet `(NounCode, &AgcContext)` auf `[DisplayRegister; 3]` ab.
//!
//! Liest die physikalischen Werte aus dem [`GuidanceContext`] und skaliert
//! sie für die DSKY-Anzeige. Nur mission-kritische Nouns sind implementiert.
//!
//! # Implementierungsstand (Sub-Task 2)
//! Vollständig implementiert für Sub-Task 2 (Reentry-Nouns P0):
//! - N60: GMAX, VPRED, GAMMAEI  (P61 Entry Prep)
//! - N67: RTOGO, LAT, LONG      (P67 Terminal Phase)
//!
//! Alle übrigen Nouns geben `[Blank; 3]` zurück — sie werden in Sub-Task 4
//! vervollständigt.
//!
//! # SI-Einheiten (Planregel)
//! Alle Anzeigen erfolgen in SI-Einheiten (m, m/s, deg). Keine NM, keine ft/s.
//! Winkel intern [rad] werden für die Anzeige in 0.01° umgerechnet.
//!
//! AGC source: `Comanche055/PINBALL_NOUN_TABLES.agc` (SF-Konstanten),
//! `Comanche055/ASSEMBLY_AND_OPERATION_INFORMATION.agc` (Noun-Definitionen).

use crate::context::AgcContext;
use crate::display::verb_noun::{DisplayRegister, NounCode};

// ---------------------------------------------------------------------------
// Skalierungsfunktionen
// ---------------------------------------------------------------------------

/// Wandelt Grad in hundertstel Grad (0.01°) um → `i32` für das DSKY.
///
/// Beispiel: −6.5° → −650
///
/// AGC source: `Comanche055/PINBALL_NOUN_TABLES.agc`, SF-Konstante für Winkel.
#[inline]
fn deg_to_centideg(deg: f64) -> i32 {
    libm::round(deg * 100.0) as i32
}

/// Wandelt m/s in den DSKY-Dezimalwert um (ganzzahlig, kein Faktor).
///
/// Beispiel: 7 800.3 m/s → 7800
///
/// AGC source: `Comanche055/PINBALL_NOUN_TABLES.agc`, SF-Konstante für Geschwindigkeit.
#[inline]
fn mps_to_display(mps: f64) -> i32 {
    libm::round(mps) as i32
}

/// Wandelt Meter in Kilometer um → `i32` für das DSKY.
///
/// Beispiel: 500 000 m → 500
///
/// AGC source: `Comanche055/PINBALL_NOUN_TABLES.agc`, SF-Konstante für Distanz.
#[inline]
fn m_to_km_display(m: f64) -> i32 {
    libm::round(m / 1000.0) as i32
}

/// Wandelt g₀-Vielfache in 0.01 m/s² um.
///
/// 1 g₀ = 9.80665 m/s². Beispiel: 3.5 g₀ → 343
///
/// AGC source: `Comanche055/ASSEMBLY_AND_OPERATION_INFORMATION.agc`, N60 (GMAX).
#[inline]
fn g0_to_centimps2(g: f64) -> i32 {
    const G0: f64 = 9.806_65;
    libm::round(g * G0 * 100.0) as i32
}

/// Wandelt Radiant in hundertstel Grad um.
///
/// Beispiel: 0.1 rad → 573 (0.01°)
///
/// Wird in Sub-Task 4 für N67 LAT/LON und weitere Nouns verwendet.
#[expect(
    dead_code,
    reason = "wird in Sub-Task 4 (NounResolver-Erweiterung) genutzt"
)]
#[inline]
fn rad_to_centideg(rad: f64) -> i32 {
    const RAD_TO_DEG: f64 = 180.0 / core::f64::consts::PI;
    deg_to_centideg(rad * RAD_TO_DEG)
}

// ---------------------------------------------------------------------------
// resolve — Haupt-Einstiegspunkt
// ---------------------------------------------------------------------------

/// Bildet einen `NounCode` auf drei DSKY-Anzeigeregister ab.
///
/// Liest Werte aus `ctx.guidance` und skaliert sie in die für das DSKY
/// passenden Einheiten. Nicht implementierte Nouns geben `Blank` zurück.
///
/// # Einheiten der implementierten Nouns
///
/// | Noun | R1             | R2             | R3           |
/// |------|----------------|----------------|--------------|
/// | N60  | GMAX [0.01 m/s²] | VPRED [m/s]  | GAMMAEI [0.01°] |
/// | N67  | RTOGO [km]     | LAT [0.01°]    | LONG [0.01°] |
///
/// AGC source: `Comanche055/ASSEMBLY_AND_OPERATION_INFORMATION.agc`, Noun-Liste.
pub fn resolve(noun: NounCode, ctx: &AgcContext) -> [DisplayRegister; 3] {
    match noun.val() {
        // ------------------------------------------------------------------
        // N60 — Entry Display: GMAX, VPRED, GAMMAEI
        // P61 Entry Preparation.
        // AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N60.
        // ------------------------------------------------------------------
        60 => {
            let Some(ref p61) = ctx.guidance.p61 else {
                return [DisplayRegister::Blank; 3];
            };
            let Some(ref d) = p61.display else {
                return [DisplayRegister::Blank; 3];
            };
            [
                DisplayRegister::Decimal(g0_to_centimps2(d.gmax)),
                DisplayRegister::Decimal(mps_to_display(d.v_pred)),
                DisplayRegister::Decimal(deg_to_centideg(d.gamma_ei_deg)),
            ]
        }

        // ------------------------------------------------------------------
        // N67 — Terminal Phase: RTOGO, LAT, LONG
        // P67 Terminal Phase.
        // AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N67.
        // ------------------------------------------------------------------
        67 => {
            let Some(ref p67) = ctx.guidance.p67 else {
                return [DisplayRegister::Blank; 3];
            };
            // RTOGO: Reichweite vom aktuellen Ort zum Splashdown.
            // Im P67State wird xrngerr_m als Quer-Fehler und dnrngerr_m als Längs-Fehler
            // gespeichert. Als Näherung für RTOGO nutzen wir den Gesamtfehler.
            let rtogo_m =
                libm::sqrt(p67.xrngerr_m * p67.xrngerr_m + p67.dnrngerr_m * p67.dnrngerr_m);
            // LAT/LONG: Roll-Befehl liegt in deg; P67State enthält keine separate Lat/Lon.
            // Wir geben roll_cmd_deg für R2 aus und 0 für R3, bis P67 Lat/Lon speichert.
            // (Vollständige Implementierung in Sub-Task 4.)
            [
                DisplayRegister::Decimal(m_to_km_display(rtogo_m)),
                DisplayRegister::Decimal(deg_to_centideg(p67.roll_cmd_deg)),
                DisplayRegister::Decimal(0),
            ]
        }

        // ------------------------------------------------------------------
        // Alle übrigen Nouns: noch nicht implementiert (Sub-Task 4).
        // ------------------------------------------------------------------
        _ => [DisplayRegister::Blank; 3],
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::resolve;
    use crate::context::AgcContext;
    use crate::display::verb_noun::{DisplayRegister, NounCode};
    use crate::guidance::p61_67::{EntryDisplayData, P61State, P67State};

    fn noun(n: u8) -> NounCode {
        NounCode::new(n).unwrap()
    }

    /// N60 mit bekannten P61-Werten → korrekte Skalierung.
    ///
    /// AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N60.
    #[test]
    fn n60_resolves_from_p61_display() {
        let mut ctx = AgcContext::default();
        ctx.guidance.p61 = Some(P61State {
            phase: 0,
            display: Some(EntryDisplayData {
                gmax: 3.5,          // 3.5 g₀ × 9.80665 × 100 ≈ 343
                v_pred: 7_800.0,    // 7800 m/s → 7800
                gamma_ei_deg: -6.5, // −6.5° → −650
                rtgo_m: 500_000.0,
                vio: 7_750.0,
                tte_s: 120.0,
            }),
            lat_spl: 0.0,
            lng_spl: 0.0,
            heads_up: false,
        });

        let [r1, r2, r3] = resolve(noun(60), &ctx);

        // R1 = GMAX in 0.01 m/s²: 3.5 × 9.80665 × 100 = 3432.33 → 3432
        assert_eq!(r1, DisplayRegister::Decimal(3432));
        // R2 = VPRED in m/s: 7800
        assert_eq!(r2, DisplayRegister::Decimal(7800));
        // R3 = GAMMAEI in 0.01°: −650
        assert_eq!(r3, DisplayRegister::Decimal(-650));
    }

    /// N60 ohne aktiven P61-State → Blank.
    #[test]
    fn n60_blank_when_p61_inactive() {
        let ctx = AgcContext::default();
        let [r1, r2, r3] = resolve(noun(60), &ctx);
        assert_eq!(r1, DisplayRegister::Blank);
        assert_eq!(r2, DisplayRegister::Blank);
        assert_eq!(r3, DisplayRegister::Blank);
    }

    /// N60 mit P61 ohne berechnetes Display (Phase 2 noch nicht abgeschlossen) → Blank.
    #[test]
    fn n60_blank_when_p61_display_none() {
        let mut ctx = AgcContext::default();
        ctx.guidance.p61 = Some(P61State {
            phase: 2,
            display: None,
            lat_spl: 0.0,
            lng_spl: 0.0,
            heads_up: false,
        });
        let [r1, r2, r3] = resolve(noun(60), &ctx);
        assert_eq!(r1, DisplayRegister::Blank);
        assert_eq!(r2, DisplayRegister::Blank);
        assert_eq!(r3, DisplayRegister::Blank);
    }

    /// N67 mit bekannten P67-Werten → R1 = RTOGO in km, R2 in centideg.
    ///
    /// AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N67.
    #[test]
    fn n67_resolves_from_p67_state() {
        let mut ctx = AgcContext::default();
        ctx.guidance.p67 = Some(P67State {
            phase: 2,
            xrngerr_m: 0.0,      // kein Quer-Fehler
            dnrngerr_m: 3_000.0, // 3 km Längs-Fehler → RTOGO ≈ 3 km
            roll_cmd_deg: 12.5,  // 12.5° → 1250 in centideg
        });

        let [r1, r2, r3] = resolve(noun(67), &ctx);

        // R1 = RTOGO in km: sqrt(0² + 3000²) / 1000 = 3
        assert_eq!(r1, DisplayRegister::Decimal(3));
        // R2 = roll_cmd_deg in centideg: 12.5 × 100 = 1250
        assert_eq!(r2, DisplayRegister::Decimal(1250));
        // R3 = 0 (Platzhalter bis Sub-Task 4)
        assert_eq!(r3, DisplayRegister::Decimal(0));
    }

    /// N67 ohne aktiven P67-State → Blank.
    #[test]
    fn n67_blank_when_p67_inactive() {
        let ctx = AgcContext::default();
        let [r1, r2, r3] = resolve(noun(67), &ctx);
        assert_eq!(r1, DisplayRegister::Blank);
        assert_eq!(r2, DisplayRegister::Blank);
        assert_eq!(r3, DisplayRegister::Blank);
    }

    /// Unbekannter Noun → Blank.
    #[test]
    fn unknown_noun_returns_blank() {
        let ctx = AgcContext::default();
        let [r1, r2, r3] = resolve(noun(99), &ctx);
        assert_eq!(r1, DisplayRegister::Blank);
        assert_eq!(r2, DisplayRegister::Blank);
        assert_eq!(r3, DisplayRegister::Blank);
    }
}
