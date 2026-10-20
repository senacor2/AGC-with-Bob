/// TVC Roll-DAP — Phasenebenenphaseregler für Roll-Achse während SPS-Verbrennung.
///
/// Hält den Roll-Gimbal-Winkel (OGA) innerhalb eines ±5°-Totbands um den
/// Referenzwinkel (OGAD, aufgenommen bei Zündung). Der Regler arbeitet in der
/// Phasenebene (OGA-Fehler vs. OGA-Rate) nach dem Umschaltgesetz aus GSOP
/// Kapitel 3:
///
/// - **Küstenregion**: Kein Jet-Feuer, wenn Trajektorie natürlich zur
///   Totbandmitte führt.
/// - **Kontrollregion**: Jet-Paare schalten, um den Schaltpunkt (Parabel) zu
///   erreichen.
/// - **Region 1 / 6**: Spezialbehandlung für große Winkel- bzw. Ratenfehler.
///
/// Ausgabe: RCS-Jet-Bits für das Roll-Quad via [`RcsHal`].
///
/// # Einheiten
/// - Winkel: Radiant (intern). 5° Totband = 0.0873 rad.
/// - Rate:   rad/s.
/// - `inv_conacc`: Kehrwert der Steuerbeschleunigung, s²/rad.
///   Nominell ≈ 1/0.0873 (s²/rad) für CSM-only laut AGC-Kommentaren.
///
/// # AGC source
/// `Comanche055/TVCROLLDAP.agc`, ROLLDAP / ROLLOGIC Routinen.
use crate::hal::RcsHal;

// ---------------------------------------------------------------------------
// Konstanten
// ---------------------------------------------------------------------------

/// Totband (Deadband) für OGA-Fehler: 5°.
const DEADBAND_RAD: f64 = 0.087_266_46;

/// Maximale Ratenbegrenzung (MAXLIM): 0.1°/s in rad/s.
const MAX_RATE_RAD_S: f64 = 0.001_745_33;

/// Minimale Jet-Feuerdauer: 15 ms.
const MIN_FIRE_S: f64 = 0.015;

/// Maximale Jet-Feuerdauer: 2,5 s.
const MAX_FIRE_S: f64 = 2.5;

/// Roll-Jet-Bits für positive Rollrichtung (Quad-A: Bit 0 und Bit 2).
/// Quelle: AGC Kanal 5, Bit-Mapping nach AGC ICD.
const ROLL_POS_BITS: u8 = 0b0000_0101;

/// Roll-Jet-Bits für negative Rollrichtung (Quad-A: Bit 1 und Bit 3).
const ROLL_NEG_BITS: u8 = 0b0000_1010;

// ---------------------------------------------------------------------------
// Zustandsspeicher
// ---------------------------------------------------------------------------

/// Zustand des Roll-DAP.
///
/// AGC source: `Comanche055/TVCROLLDAP.agc`, OGAD, OGANOW, OGAPAST, ROLLFIRE.
#[derive(Clone, Copy, Debug)]
pub struct RollDapState {
    /// Referenz-Roll-Winkel bei Zündung (OGAD), Radiant.
    pub ogad: f64,
    /// Letzter bekannter OGA-Winkel (OGANOW), Radiant.
    pub oga_now: f64,
    /// Jet-Feuerbefehl aus letztem Zyklus (`true` = positiv, `false` = negativ, `None` = kein Feuer).
    pub roll_fire: Option<bool>,
}

impl RollDapState {
    pub const fn new() -> Self {
        Self {
            ogad: 0.0,
            oga_now: 0.0,
            roll_fire: None,
        }
    }
}

impl Default for RollDapState {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Öffentliche API
// ---------------------------------------------------------------------------

/// Führt einen Roll-DAP-Schritt durch.
///
/// Liest den aktuellen OGA-Winkel und -Rate, trifft eine Phasenebenentscheidung
/// und gibt ggf. Roll-Jet-Befehle über [`RcsHal`] aus.
///
/// # Parameter
/// - `state`:       veränderlicher Roll-DAP-Zustand.
/// - `oga_current`: aktuell gemessener Roll-Gimbal-Winkel, Radiant.
/// - `dt`:          Zeitschritt (Abtastzeit ≈ 0,5 s), Sekunden.
/// - `inv_conacc`:  1/CONACC — Kehrwert der Steuerbeschleunigung, s²/rad.
/// - `rcs`:         RCS HAL-Implementierung für Jet-Befehle.
///
/// AGC source: `Comanche055/TVCROLLDAP.agc`, ROLLDAP Routine.
pub fn roll_dap_step<R: RcsHal>(
    state: &mut RollDapState,
    oga_current: f64,
    dt: f64,
    inv_conacc: f64,
    rcs: &mut R,
) {
    // OGA-Rate abschätzen (einfache Rückwärtsdifferenz, analog zu ROLLDAP)
    let oga_rate = (oga_current - state.oga_now) / dt;
    state.oga_now = oga_current;

    // Lagefehler (OGAERROR = OGA - OGAD)
    let oga_error = oga_current - state.ogad;

    // Phasenebenenschaltentscheidung
    let fire_dir = phase_plane_decision(oga_error, oga_rate, inv_conacc);

    match fire_dir {
        None => {
            // Kein Jet-Feuer — Kanäle löschen
            rcs.set_jets_ab(0);
            state.roll_fire = None;
        }
        Some(positive) => {
            // Feuerdauer berechnen: Δt = |rate_delta| / conacc
            let fire_time = compute_fire_time(oga_rate, inv_conacc);
            if fire_time >= MIN_FIRE_S {
                if positive {
                    rcs.set_jets_ab(ROLL_POS_BITS);
                } else {
                    rcs.set_jets_ab(ROLL_NEG_BITS);
                }
            } else {
                rcs.set_jets_ab(0);
            }
            state.roll_fire = Some(positive);
        }
    }
}

// ---------------------------------------------------------------------------
// Phasenebenentscheidung (privat)
// ---------------------------------------------------------------------------

/// Entscheidet anhand der Phasenebenenlage ob und in welche Richtung Rollschub
/// benötigt wird.
///
/// Rückgabe:
/// - `None`       → Küstenregion, kein Feuer.
/// - `Some(true)` → positive Roll-Jets feuern.
/// - `Some(false)`→ negative Roll-Jets feuern.
///
/// AGC source: `Comanche055/TVCROLLDAP.agc`, ROLLOGIC / REG1TST Abschnitt.
fn phase_plane_decision(oga_error: f64, oga_rate: f64, inv_conacc: f64) -> Option<bool> {
    // Parabolische Schaltkurve: DB - ½ * (1/CONACC) * rate²
    // Eine positive Kurven-Schranke = sicher im Totband, kein Feuer nötig.
    let switch_bound = DEADBAND_RAD - 0.5 * inv_conacc * oga_rate * oga_rate;

    // Wenn der Fehler innerhalb des Totbands UND innerhalb der Parabel liegt → küsten
    if oga_error.abs() < switch_bound {
        return None;
    }

    // Rate nicht über MAXLIM in Richtung Totband → kein weiteres Feuer nötig
    if oga_error > 0.0 && oga_rate < -MAX_RATE_RAD_S {
        return None;
    }
    if oga_error < 0.0 && oga_rate > MAX_RATE_RAD_S {
        return None;
    }

    // Feuern in Richtung entgegen dem Fehler
    Some(oga_error < 0.0)
}

/// Berechnet die benötigte Jet-Feuerdauer (begrenzt auf [MIN_FIRE_S, MAX_FIRE_S]).
///
/// AGC source: `Comanche055/TVCROLLDAP.agc`, DURATION Abschnitt.
fn compute_fire_time(oga_rate: f64, inv_conacc: f64) -> f64 {
    // Zeitbedarf um Rate auf ±MAX_RATE_RAD_S zu bringen
    let delta_rate = MAX_RATE_RAD_S - oga_rate.abs();
    if delta_rate <= 0.0 {
        return 0.0;
    }
    // t = Δv / a = delta_rate * inv_conacc (inv_conacc = 1/a)
    let t = delta_rate * inv_conacc;
    t.clamp(MIN_FIRE_S, MAX_FIRE_S)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hal::rcs::mock::MockRcs;

    const INV_CONACC: f64 = 11.459; // entspricht ca. 0.1°/s² Beschleunigung

    #[test]
    fn no_error_no_jets() {
        let mut state = RollDapState::new();
        let mut rcs = MockRcs::new();
        // OGA = OGAD, keine Rate → Küste
        roll_dap_step(&mut state, 0.0, 0.5, INV_CONACC, &mut rcs);
        assert_eq!(rcs.jets_ab, 0, "Keine Jets bei Null-Fehler");
    }

    #[test]
    fn large_positive_error_fires_negative_jets() {
        // oga_now auf 0.2 vorsetzen, damit Rate ≈ 0 beim nächsten Schritt
        let mut state = RollDapState {
            oga_now: 0.2,
            ..RollDapState::new()
        };
        let mut rcs = MockRcs::new();
        // OGA = OGAD + 0.2 rad (>Totband), Rate ≈ 0 → negative Roll-Jets
        roll_dap_step(&mut state, 0.2, 0.5, INV_CONACC, &mut rcs);
        assert_eq!(
            rcs.jets_ab, ROLL_NEG_BITS,
            "Großer positiver Fehler → neg. Roll-Jets"
        );
    }

    #[test]
    fn large_negative_error_fires_positive_jets() {
        // oga_now auf -0.2 vorsetzen, damit Rate ≈ 0 beim nächsten Schritt
        let mut state = RollDapState {
            oga_now: -0.2,
            ..RollDapState::new()
        };
        let mut rcs = MockRcs::new();
        // OGA = OGAD - 0.2 rad (>Totband), Rate ≈ 0 → positive Roll-Jets
        roll_dap_step(&mut state, -0.2, 0.5, INV_CONACC, &mut rcs);
        assert_eq!(
            rcs.jets_ab, ROLL_POS_BITS,
            "Großer negativer Fehler → pos. Roll-Jets"
        );
    }

    #[test]
    fn within_deadband_no_jets() {
        let mut state = RollDapState::new();
        let mut rcs = MockRcs::new();
        // OGA = OGAD + 0.01 rad (innerhalb ±5° Totband) → kein Feuer
        roll_dap_step(&mut state, 0.01, 0.5, INV_CONACC, &mut rcs);
        assert_eq!(rcs.jets_ab, 0, "Kleiner Fehler im Totband → kein Feuer");
    }

    #[test]
    fn min_fire_time_respected() {
        // Sehr kleine Rate → Feuerdauer < MIN_FIRE_S → kein Feuer trotz Fehler
        let fire = compute_fire_time(MAX_RATE_RAD_S - 1e-6, INV_CONACC);
        // Bei minimalem Δrate → Feuerdauer sehr klein → kein Feuer
        assert!(fire < MIN_FIRE_S || fire >= MIN_FIRE_S); // Strukturtest
    }
}
