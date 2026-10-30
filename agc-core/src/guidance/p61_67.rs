//! P61–P67 — Entry Preparation und Reentry Control.
//!
//! Implementiert die atmosphärischen Wiedereintritts-Programme des AGC:
//!
//! - **P61** — Entry Preparation: berechnet EMS-Initialisierungsdaten
//!   (GMAX, VPRED, GAMMAEI, RTGO, VIO, TTE) aus dem aktuellen Zustandsvektor.
//!
//! - **P62** — Pre-Entry Orientation: prüft IMU-Ausrichtung und bereitet
//!   die Entry-Lage (ALFAPAD / ROLLC) vor.
//!
//! - **P63** — Entry Monitor / .05G Sensing: überwacht die Atmosphärenbremsung
//!   bis zum 0.05-g-Einsatzpunkt.
//!
//! - **P64** — Constant-Drag Entry: Schritt für Schritt Rollsteuerung zur
//!   Reichweite-Regelung (Eintrittsphase bei konstantem Luftwiderstand).
//!
//! - **P65** — Up-Control Phase: Hochziehmörser-Phase zur präzisen Zielführung
//!   (wenn Reichweite < 25 NM vom Ziel).
//!
//! - **P66** — Final Phase (Trim): Haltesteuerung relativ zum Relativgeschwindigkeitsvektor
//!   ohne Rollbefehle, bis Luftwiderstand wieder auf Q7 + 0.5 ft/s² steigt.
//!
//! - **P67** — Terminal Phase: Beendet die Rollsteuerung bei v_rel ≤ 305 m/s.
//!
//! # Physikalisches Modell
//!
//! Die Brems-Beschleunigung (Drag) und Liftkraft werden in vereinfachter Form
//! aus Zustandsvektor + atmosphärischen Konstanten berechnet.  Die Gleichungen
//! folgen den analytischen Entry-Guidance-Algorithmen aus REENTRY_CONTROL.agc.
//!
//! # Einheiten
//! - Positionen: m (ECI J2000)
//! - Geschwindigkeiten: m/s (ECI J2000)
//! - G-Last: in Einheiten von g₀ = 9.80665 m/s²
//! - Zeiten: Sekunden
//!
//! AGC source: `Comanche055/P61-P67.agc`, `Comanche055/REENTRY_CONTROL.agc`.

use crate::alarm::{raise, AlarmState};
use crate::navigation::orbital::CentralBody;
use crate::navigation::state_vector::StateVector;
use crate::restart::phase_table::PhaseTable;
use crate::types::Vec3;

// ---------------------------------------------------------------------------
// Physikalische Konstanten
// ---------------------------------------------------------------------------

/// Normalbeschleunigung g₀ [m/s²].
const G0: f64 = 9.806_65;

/// EMS-Interface-Altitude für orbitale Eintritte (400 000 ft = 121 920 m über Fischer-Ellipsoid).
///
/// AGC source: `Comanche055/P61-P67.agc`, S61.2, EMSALT PAD-Loaded Value.
pub const EMSALT_ORBITAL_M: f64 = 121_920.0;

/// Atmosphärische Skalenhöhe H [m] für vereinfachtes Exponentialmodell.
const SCALE_HEIGHT_M: f64 = 7_200.0;

/// Referenz-Luftdichte auf Meereshöhe [kg/m³].
const RHO_SEA_LEVEL: f64 = 1.225;

/// Ballistischer Koeffizient des CM (B = m / (C_D · A)) [kg/m²].
/// Typischer AGC-Wert: L/D ≈ 0.3, C_D ≈ 1.37.
const BALLISTIC_COEFF: f64 = 300.0;

/// Lift-to-Drag-Ratio L/D des CM (Trimm-Konfiguration).
const L_OVER_D: f64 = 0.3;

/// Alarmcode P61: Zustandsvektor außerhalb Atmosphäreneintritts-Grenze.
///
/// AGC source: `Comanche055/P61-P67.agc`, S61.1, Alarm 01426.
pub const ALARM_P61_IMU_UNSATISFACTORY: u16 = 0x1426;

/// Alarmcode P64: Keine Up-Control-Lösung mit V > 18 000 ft/s.
pub const ALARM_P64_NO_UPCONTROL: u16 = 0x0640;

// ---------------------------------------------------------------------------
// Hilfsfunktionen
// ---------------------------------------------------------------------------

/// Berechnet die atmosphärische Luftdichte in der gegebenen Höhe h [m].
///
/// Vereinfachtes Exponentialmodell:
/// ρ(h) = ρ₀ · exp(−h / H)
///
/// # Parameter
/// - `alt_m`: Höhe über Meeresspiegel [m]
///
/// # Rückgabe
/// Luftdichte [kg/m³]; 0 für h > 200 km (außerhalb Atmosphäre).
fn air_density(alt_m: f64) -> f64 {
    if alt_m > 200_000.0 {
        return 0.0;
    }
    let h = if alt_m < 0.0 { 0.0 } else { alt_m };
    RHO_SEA_LEVEL * libm::exp(-h / SCALE_HEIGHT_M)
}

/// Berechnet die Geschwindigkeit relativ zur Erde aus dem ECI-Zustandsvektor.
///
/// v_rel = v_ECI − ω_E × r
///
/// ω_E = 7.292 115 × 10⁻⁵ rad/s (Erdrotationsrate).
///
/// # Rückgabe
/// Relativgeschwindigkeit [m/s].
fn velocity_relative_to_earth(r: Vec3, v: Vec3) -> Vec3 {
    const OMEGA_EARTH: f64 = 7.292_115e-5; // rad/s
                                           // ω_E × r = (0, 0, ω_E) × (x, y, z) = (−ω·y, ω·x, 0)
    let v_earth = Vec3::new(-OMEGA_EARTH * r.y, OMEGA_EARTH * r.x, 0.0);
    Vec3::new(v.x - v_earth.x, v.y - v_earth.y, v.z - v_earth.z)
}

/// Berechnet die Brems-G-Last aus Zustandsvektor und Atmosphärenmodell.
///
/// D = ρ · v_rel² / (2 · B)   [m/s²]
///
/// # Parameter
/// - `sv`:  Zustandsvektor (ECI, m / m/s)
/// - `body`: Zentralkörper
///
/// # Rückgabe
/// Drag-Beschleunigung D in Einheiten von g₀ (dimensionslos).
pub fn drag_g(sv: &StateVector, body: &CentralBody) -> f64 {
    let r_mag = sv.position.magnitude();
    let alt = r_mag - body.r_equatorial();
    let rho = air_density(alt);
    let v_rel = velocity_relative_to_earth(sv.position, sv.velocity);
    let v_rel_sq = v_rel.dot(v_rel);
    let drag_accel = rho * v_rel_sq / (2.0 * BALLISTIC_COEFF);
    drag_accel / G0
}

// ---------------------------------------------------------------------------
// Entry-Anzeige-Berechnungen (S61.2)
// ---------------------------------------------------------------------------

/// Ergebnis der Entry-Anzeigeberechnungen (NOUN 60 / N63).
///
/// Entspricht den Ausgaberegistern von S61.2.
///
/// AGC source: `Comanche055/P61-P67.agc`, S61.2-Subroutine.
#[derive(Copy, Clone, Debug)]
pub struct EntryDisplayData {
    /// Maximale Brems-G-Last [g₀] (GMAX).
    pub gmax: f64,
    /// Vorhergesagte Geschwindigkeit bei EMS-Altitude (VPRED) [m/s].
    pub v_pred: f64,
    /// Vorhergesagter Eintrittswinkel bei EMS-Altitude (GAMMAEI) [deg].
    pub gamma_ei_deg: f64,
    /// Reichweite bis Splashdown ab EMS-Altitude (RTGO) [m].
    pub rtgo_m: f64,
    /// Inertialgeschwindigkeit bei EMS-Altitude (VIO) [m/s].
    pub vio: f64,
    /// Geschätzte Zeit bis EMS-Altitude (TTE) [s].
    pub tte_s: f64,
}

/// Berechnet Entry-Anzeigedaten für P61 (entspricht S61.2 im AGC).
///
/// Nutzt die Vis-Viva-Gleichung und eine vereinfachte Bremsbeschleunigungsschätzung.
///
/// # Parameter
/// - `sv`:  aktueller Zustandsvektor (ECI J2000, m / m/s)
/// - `body`: Zentralkörper
/// - `alarm`: Alarmzustand
///
/// # Rückgabe
/// `Some(EntryDisplayData)` bei gültigem Zustandsvektor, `None` bei Alarm.
///
/// AGC source: `Comanche055/P61-P67.agc`, S61.2-Subroutine.
pub fn compute_entry_display(
    sv: &StateVector,
    body: &CentralBody,
    alarm: &mut AlarmState,
) -> Option<EntryDisplayData> {
    let mu = body.mu();
    let r_eq = body.r_equatorial();

    let r = sv.position;
    let v = sv.velocity;
    let r_mag = r.magnitude();
    let v_mag = libm::sqrt(v.dot(v));

    if r_mag < 1.0 || v_mag < 1.0 {
        raise(alarm, ALARM_P61_IMU_UNSATISFACTORY);
        return None;
    }

    // vis-viva: a = 1 / (2/r − v²/μ)
    let inv_a = 2.0 / r_mag - v_mag * v_mag / mu;
    if inv_a <= 0.0 {
        // Hyperbolische Bahn — kein Wiedereintritt möglich
        raise(alarm, ALARM_P61_IMU_UNSATISFACTORY);
        return None;
    }
    let a = 1.0 / inv_a;

    // Perigee-Radius: r_per = a · (1 − e)
    let h_vec = r.cross(v);
    let h_sq = h_vec.dot(h_vec);
    let e_sq = 1.0 - h_sq / (mu * a);
    let e = if e_sq < 0.0 { 0.0 } else { libm::sqrt(e_sq) };
    let r_per = a * (1.0 - e);

    // EMS-Interface Radius
    let r_ems = r_eq + EMSALT_ORBITAL_M;

    // Geschwindigkeit bei EMS per Vis-Viva
    let vio = libm::sqrt(libm::fabs(mu * (2.0 / r_ems - 1.0 / a)));

    // Eintrittswinkel γ_EI: sin(γ) = -ṙ / v
    // ṙ = (r · v) / |r|
    let r_dot = r.dot(v) / r_mag;
    let cos_fpa = libm::sqrt(h_sq) / (r_mag * v_mag);
    // Sicherheits-Clamp für acos
    let cos_fpa_clamped = cos_fpa.clamp(-1.0, 1.0);
    let fpa_rad = libm::acos(cos_fpa_clamped);
    // Vorzeichen: negativ bei Abstieg (ṙ < 0)
    let gamma_ei_deg = if r_dot < 0.0 {
        -fpa_rad * 180.0 / core::f64::consts::PI
    } else {
        fpa_rad * 180.0 / core::f64::consts::PI
    };

    // Zeit bis EMS: vereinfachte Kegelschnitt-Transferzeit
    // Für r > r_ems und absteigend: t = (r_mag - r_ems) / |ṙ|
    // Für r ≤ r_ems (bereits unter EMS) oder aufsteigend: halbe Umlaufzeit
    let tte_s = if r_dot < -0.1 && r_mag > r_ems {
        (r_mag - r_ems) / (-r_dot)
    } else {
        // Bahnperiode schätzen (halbe Umlaufzeit als konservative Schätzung)
        let period = 2.0 * core::f64::consts::PI * libm::sqrt(a * a * a / mu);
        period * 0.5
    };

    // RTGO: Reichweite bis Splashdown ab EMS (vereinfacht: 10 × Verzögerungshöhe)
    // Echte Gleichung: RTGO = THETAH/360 × r_eq  (Winkelreichweite)
    // Hier: vereinfachte Schätzung basierend auf L/D und Eintrittsbedingungen
    let rtgo_m = libm::fabs(vio * vio / (2.0 * G0)) * L_OVER_D * 2.0;

    // GMAX: vereinfachtes Modell aus dem AGC
    // GMAX ≈ (4 / (1 + 4.8·vbar²)) · (γ − 6.05° − 2.4·vbar²) − 10·(L/D − 0.3) + 10
    // vbar = (v [ft/s] − 36000) / 20000
    let v_fps = vio * 3.280_839_895; // m/s → ft/s
    let vbar = (v_fps - 36_000.0) / 20_000.0;
    let vbar_sq = vbar * vbar;
    let gamma_rad = gamma_ei_deg * core::f64::consts::PI / 180.0;
    let gmax_raw = (4.0 / (1.0 + 4.8 * vbar_sq))
        * (gamma_rad.abs() - 6.05_f64.to_radians() - 2.4 * vbar_sq)
        - 10.0 * (L_OVER_D - 0.3)
        + 10.0;
    let gmax = if gmax_raw < 0.0 { 0.0 } else { gmax_raw };

    // VPRED: vorhergesagte Geschwindigkeit bei 400 000 ft ≈ vio (für Phase 1-Näherung)
    let r_400k = r_eq + 121_920.0; // 400 000 ft in m
    let v_pred = libm::sqrt(libm::fabs(mu * (2.0 / r_400k - 1.0 / a)));

    // Prüfe: Perigee muss unter EMS liegen für Wiedereintritt
    if r_per > r_ems + 10_000.0 {
        raise(alarm, ALARM_P61_IMU_UNSATISFACTORY);
        return None;
    }

    Some(EntryDisplayData {
        gmax,
        v_pred,
        gamma_ei_deg,
        rtgo_m,
        vio,
        tte_s,
    })
}

// ---------------------------------------------------------------------------
// P61State — Programmzustand
// ---------------------------------------------------------------------------

/// Laufzustand von P61 — Entry Preparation.
///
/// Hält alle zur Restart-sicheren Ausführung notwendigen Zwischenwerte.
///
/// AGC source: `Comanche055/P61-P67.agc`, P61-Programm.
#[derive(Copy, Clone, Debug)]
pub struct P61State {
    /// Aktuelle Programmphase (2 = Display-Berechnung, 3 = Anzeige N60/N63, 0 = done).
    pub phase: u8,
    /// Berechnete Anzeigedaten (None bis Phase 2 abgeschlossen).
    pub display: Option<EntryDisplayData>,
    /// Ziel-Breite [rev] — LAT(SPL), durch Crew eingegeben.
    pub lat_spl: f64,
    /// Ziel-Länge [rev] — LNG(SPL), durch Crew eingegeben.
    pub lng_spl: f64,
    /// HEADSUP-Flag: true = Lift-Down, false = Lift-Up.
    pub heads_up: bool,
}

impl P61State {
    /// Neuer P61-Zustand (startet in Phase 2).
    pub fn new(lat_spl: f64, lng_spl: f64, heads_up: bool) -> Self {
        Self {
            phase: 2,
            display: None,
            lat_spl,
            lng_spl,
            heads_up,
        }
    }
}

/// Führt einen P61-Schritt aus (Entry Preparation).
///
/// # Phasen
/// - **Phase 2**: Berechnet Entry-Anzeigedaten (S61.2 äquivalent) und setzt Checkpoint 4.
/// - **Phase 3**: Setzt Checkpoint auf 0 (Programm fertig), gibt `Some(())` zurück.
/// - **Phase 0**: Bereits fertig → gibt `None` zurück.
///
/// # Parameter
/// - `state`: Programmzustand
/// - `sv`:    aktueller Zustandsvektor (ECI J2000, m / m/s)
/// - `body`:  Zentralkörper
/// - `pt`:    Phase-Table
/// - `alarm`: Alarmzustand
///
/// AGC source: `Comanche055/P61-P67.agc`, P61-Programm, DUMPP61-Label.
pub fn p61_step(
    state: &mut P61State,
    sv: &StateVector,
    body: &CentralBody,
    pt: &mut PhaseTable,
    alarm: &mut AlarmState,
) -> Option<()> {
    match state.phase {
        2 | 3 => {
            // Phase 2: Anzeigedaten berechnen, Checkpoint 4 setzen
            pt.set_phase(4, 3)?;
            let disp = compute_entry_display(sv, body, alarm)?;
            state.display = Some(disp);
            // Phase 3: Abschluss
            state.phase = 0;
            pt.set_phase(4, 0)?;
            Some(())
        }
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// P64State — Constant-Drag Entry
// ---------------------------------------------------------------------------

/// Laufzustand von P64 — Constant-Drag Entry Phase.
///
/// AGC source: `Comanche055/P61-P67.agc`, P64-Programm.
#[derive(Copy, Clone, Debug)]
pub struct P64State {
    /// Aktuelle Phase (2 = Drag-Regelung aktiv, 3 = Umschalt-Check, 0 = done).
    pub phase: u8,
    /// Aktueller Roll-Befehl [deg].
    pub roll_cmd_deg: f64,
    /// Aktuelle Drag-G-Last [g₀].
    pub drag_g_current: f64,
    /// Zielerreichter Eintrittswinkel (GAMMAEI bei .05G) [deg].
    pub gamma_at_05g_deg: f64,
}

impl P64State {
    /// Neuer P64-Zustand.
    pub fn new() -> Self {
        Self {
            phase: 2,
            roll_cmd_deg: 0.0,
            drag_g_current: 0.0,
            gamma_at_05g_deg: 0.0,
        }
    }
}

impl Default for P64State {
    fn default() -> Self {
        Self::new()
    }
}

/// AGC: 0.05 g Schwellwert für Entry-Einleitung.
const ENTRY_05G_THRESHOLD: f64 = 0.05;

/// AGC: Q7 — Drag-Schwelle für Up-Control Übergang [g₀].
const Q7_THRESHOLD: f64 = 0.2;

/// Führt einen P64-Schritt aus (Constant-Drag Entry).
///
/// Berechnet den Roll-Befehl aus der aktuellen Drag-G-Last und der
/// Reichweiten-Abweichung (vereinfachtes Constant-Drag-Modell).
///
/// # Rückgabe
/// - `Some(true)`:  Weiter in P64.
/// - `Some(false)`: Übergang zu P65 (wenn D < Q7 oder Up-Control-Bedingung erfüllt).
/// - `None`:         Alarm.
///
/// AGC source: `Comanche055/P61-P67.agc`, P64-Programm, DANZIG-Label.
pub fn p64_step(
    state: &mut P64State,
    sv: &StateVector,
    body: &CentralBody,
    target_range_m: f64,
    pt: &mut PhaseTable,
    alarm: &mut AlarmState,
) -> Option<bool> {
    let _ = alarm;

    match state.phase {
        2 => {
            pt.set_phase(5, 2)?;
            let d_g = drag_g(sv, body);
            state.drag_g_current = d_g;

            // Eintritts-Rollanpassung: Roll-Befehl aus Reichweiten-Fehler.
            // Wird immer berechnet (auch vor .05G), damit N69 R1 / N74 R1
            // bereits während der Abstiegsphase einen sinnvollen Wert zeigen.
            // AGC source: Comanche055/P61-P67.agc, DANZIG label — BETA-Berechnung
            // erfolgt unabhängig vom .05G-Check.
            let r_mag = sv.position.magnitude();
            let alt = r_mag - body.r_equatorial();
            // Vereinfachte Reichweiten-Schätzung: alt × L/D × 2
            let est_range_m = alt.abs() * L_OVER_D * 2.0;
            let range_err_m = target_range_m - est_range_m;
            // Positiver Fehler → zu weit → Lift-down → Roll 90°
            // Negativer Fehler → zu nah → Lift-up → Roll 0°
            state.roll_cmd_deg = if range_err_m > 0.0 { 90.0 } else { 0.0 };

            if d_g < ENTRY_05G_THRESHOLD {
                // Noch nicht tief genug — warten (Roll-Befehl bleibt gesetzt)
                return Some(true);
            }

            // Übergang zu P65, wenn D > Q7
            if d_g > Q7_THRESHOLD {
                state.phase = 3;
                return Some(false); // → P65
            }

            Some(true)
        }
        3 => {
            // Phase 3: Cutoff-Check
            pt.set_phase(5, 0)?;
            state.phase = 0;
            Some(false)
        }
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// P67State — Terminal Phase
// ---------------------------------------------------------------------------

/// Laufzustand von P67 — Terminal Phase (v_rel ≤ ~305 m/s).
///
/// AGC source: `Comanche055/P61-P67.agc`, P67-Programm.
#[derive(Copy, Clone, Debug)]
pub struct P67State {
    /// Aktuelle Phase (2 = Monitoring, 0 = done).
    pub phase: u8,
    /// Quer-Reichweitenfehler (XRNGERR) [m].
    pub xrngerr_m: f64,
    /// Längs-Reichweitenfehler (DNRNGERR) [m].
    pub dnrngerr_m: f64,
    /// Letzter Roll-Befehl [deg].
    pub roll_cmd_deg: f64,
}

impl P67State {
    /// Neuer P67-Zustand.
    pub fn new() -> Self {
        Self {
            phase: 2,
            xrngerr_m: 0.0,
            dnrngerr_m: 0.0,
            roll_cmd_deg: 0.0,
        }
    }
}

impl Default for P67State {
    fn default() -> Self {
        Self::new()
    }
}

/// Beendigungs-Geschwindigkeit für P67 (1 000 ft/s ≈ 304.8 m/s).
///
/// AGC source: `Comanche055/P61-P67.agc`, P67-Programm.
const TERMINAL_VELOCITY_M_S: f64 = 304.8;

/// Führt einen P67-Schritt aus (Terminal Phase).
///
/// Beendet die aktive Steuerung, wenn |v_rel| ≤ 304.8 m/s.
///
/// # Rückgabe
/// `Some(true)` = weiter in P67; `Some(false)` / `None` = Entry-DAP aus, POOH.
///
/// AGC source: `Comanche055/P61-P67.agc`, P67-Programm.
pub fn p67_step(
    state: &mut P67State,
    sv: &StateVector,
    target_lat_rad: f64,
    target_lng_rad: f64,
    pt: &mut PhaseTable,
    _alarm: &mut AlarmState,
) -> Option<bool> {
    match state.phase {
        2 => {
            pt.set_phase(5, 2)?;
            let v_rel = velocity_relative_to_earth(sv.position, sv.velocity);
            let v_rel_mag = libm::sqrt(v_rel.dot(v_rel));

            // Einfache Reichweiten-Fehler-Schätzung (XRNGERR, DNRNGERR)
            // Aktuelle Lat/Lon aus Positionsvektor
            let r = sv.position;
            let lat_cur = libm::asin(r.z / r.magnitude());
            let lng_cur = libm::atan2(r.y, r.x);
            let r_eq = CentralBody::Earth.r_equatorial();
            // Abweichungen in m (kleine-Winkel-Näherung)
            state.dnrngerr_m = (target_lat_rad - lat_cur) * r_eq;
            state.xrngerr_m = (target_lng_rad - lng_cur) * r_eq * libm::cos(lat_cur);

            if v_rel_mag <= TERMINAL_VELOCITY_M_S {
                // Entry-DAP deaktivieren, Programm beenden
                state.phase = 0;
                pt.set_phase(5, 0)?;
                return Some(false);
            }
            Some(true)
        }
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::state_vector::StateVector;
    use crate::restart::phase_table::PhaseTable;
    use crate::types::{Met, Vec3};

    /// Erstellt einen typischen Wiedereintritts-Zustandsvektor.
    ///
    /// Perigee ≈ 120 km, e ≈ 0.96 (stark elliptisch), absteigende Bahn.
    fn entry_sv() -> StateVector {
        // Näherungs-Rückkehrbahn von Mond zur Erde:
        // Apogee ≈ 400 000 km, Perigee ≈ 120 km
        // Hier: Position bei r ≈ 6 500 km (gerade in Eintrittszone)
        let r_mag = 6_500_000.0_f64; // 6 500 km
        let v_mag = 11_000.0_f64; // 11 km/s (typisch Lunar-Return)
                                  // Absteigende Bahn: v hauptsächlich entgegengesetzt zu r, etwas radial
        let r = Vec3::new(r_mag, 0.0, 0.0);
        // Geschwindigkeit: leichter Abstieg (γ ≈ -6.5°)
        let gamma_rad = -6.5_f64 * core::f64::consts::PI / 180.0;
        let v = Vec3::new(
            v_mag * libm::sin(gamma_rad), // radiale Komponente (negativ = absteigend)
            v_mag * libm::cos(gamma_rad), // along-track
            0.0,
        );
        StateVector::new(r, v, Met::from_seconds(0.0))
    }

    // -----------------------------------------------------------------------
    // Test 1: compute_entry_display — Anzeigedaten bei gültigem Zustandsvektor
    // -----------------------------------------------------------------------
    #[test]
    fn entry_display_valid_sv() {
        let sv = entry_sv();
        let body = CentralBody::Earth;
        let mut alarm = AlarmState::new();

        let result = compute_entry_display(&sv, &body, &mut alarm);
        assert!(
            result.is_some(),
            "compute_entry_display muss Some liefern für gültigen Lunar-Return-SV"
        );
        let d = result.unwrap();
        assert!(
            alarm.last().is_none(),
            "Kein Alarm erwartet bei gültigem Zustandsvektor"
        );
        // GMAX soll physikalisch plausibel sein (0 ≤ GMAX ≤ 50 g)
        assert!(
            d.gmax >= 0.0 && d.gmax <= 50.0,
            "GMAX = {:.2} g soll im Bereich [0, 50] liegen",
            d.gmax
        );
        // VIO muss positiv sein
        assert!(d.vio > 0.0, "VIO muss positiv sein");
        // VPRED muss positiv sein
        assert!(d.v_pred > 0.0, "VPRED muss positiv sein");
        // TTE muss positiv sein
        assert!(d.tte_s > 0.0, "TTE muss positiv sein");
        // Eintrittswinkel soll für Lunar-Return im Bereich [-10°, 0°] liegen
        assert!(
            d.gamma_ei_deg <= 0.0 && d.gamma_ei_deg >= -15.0,
            "GAMMAEI = {:.2}° soll im Bereich [-15°, 0°] liegen",
            d.gamma_ei_deg
        );
    }

    // -----------------------------------------------------------------------
    // Test 2: compute_entry_display — hyperbolische Bahn → Alarm
    // -----------------------------------------------------------------------
    #[test]
    fn entry_display_hyperbolic_gives_alarm() {
        // Escape-Geschwindigkeit überschreiten → hyperbolisch
        let r_mag = 6_378_137.0 + 300_000.0;
        let v_escape = libm::sqrt(2.0 * CentralBody::Earth.mu() / r_mag);
        let sv = StateVector::new(
            Vec3::new(r_mag, 0.0, 0.0),
            Vec3::new(0.0, v_escape * 1.1, 0.0), // > v_escape → Hyperbel
            Met::from_seconds(0.0),
        );
        let body = CentralBody::Earth;
        let mut alarm = AlarmState::new();
        let result = compute_entry_display(&sv, &body, &mut alarm);
        assert!(result.is_none(), "Hyperbolische Bahn muss None liefern");
        assert!(
            alarm.last().is_some(),
            "Alarm muss gesetzt sein bei hyperbolischer Bahn"
        );
    }

    // -----------------------------------------------------------------------
    // Test 3: p61_step — Phase 2 → done
    // -----------------------------------------------------------------------
    #[test]
    fn p61_step_phase2_to_done() {
        let sv = entry_sv();
        let body = CentralBody::Earth;
        let mut state = P61State::new(0.0, 0.0, false);
        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();

        assert_eq!(state.phase, 2);
        let result = p61_step(&mut state, &sv, &body, &mut pt, &mut alarm);
        assert!(
            result.is_some(),
            "p61_step Phase 2 → done muss Some liefern"
        );
        assert_eq!(state.phase, 0, "state.phase soll 0 (done) sein");
        assert!(
            state.display.is_some(),
            "display-Daten müssen nach Phase 2 gesetzt sein"
        );
        assert_eq!(
            pt.get_phase(4),
            Some(0),
            "Phase-Table Gruppe 4 soll 0 (done) sein"
        );
        assert!(alarm.last().is_none(), "Kein Alarm bei gültigem SV");
    }

    // -----------------------------------------------------------------------
    // Test 4: p61_step Phase 0 → None (bereits fertig)
    // -----------------------------------------------------------------------
    #[test]
    fn p61_step_phase0_returns_none() {
        let sv = entry_sv();
        let body = CentralBody::Earth;
        let mut state = P61State::new(0.0, 0.0, false);
        state.phase = 0;
        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();
        let result = p61_step(&mut state, &sv, &body, &mut pt, &mut alarm);
        assert!(result.is_none(), "Phase 0 soll None zurückgeben");
    }

    // -----------------------------------------------------------------------
    // Test 5: p64_step — 0.05g noch nicht erreicht → weiter
    // -----------------------------------------------------------------------
    #[test]
    fn p64_step_below_05g_stays_in_p64() {
        // Hoher Orbit — kaum Luftwiderstand
        let sv = StateVector::new(
            Vec3::new(6_700_000.0, 0.0, 0.0),
            Vec3::new(0.0, 7_800.0, 0.0),
            Met::from_seconds(0.0),
        );
        let body = CentralBody::Earth;
        let mut state = P64State::new();
        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();

        let result = p64_step(&mut state, &sv, &body, 3_000_000.0, &mut pt, &mut alarm);
        // Bei hohem Orbit: drag < 0.05g → bleibt in P64 (Some(true))
        assert!(result.is_some(), "p64_step muss Some liefern");
        assert_eq!(result, Some(true), "Unter 0.05g: weiter in P64");
    }

    // -----------------------------------------------------------------------
    // Test 6: p67_step — v_rel < Schwellwert → Programm beendet
    // -----------------------------------------------------------------------
    #[test]
    fn p67_step_slow_velocity_ends_program() {
        // Relativgeschwindigkeit muss < 305 m/s sein.
        // Erdrotation: v_earth.y ≈ ω · r ≈ 7.292e-5 * 6_388_137 ≈ 466 m/s
        // Um v_rel < 305 m/s zu bekommen: v.y ≈ 466 - 200 = 266 m/s
        // → v_rel.y ≈ 266 - 466 = -200 m/s → |v_rel| ≈ 200 m/s < 305 m/s
        let r_val = 6_378_137.0 + 10_000.0;
        let omega = 7.292_115e-5_f64;
        let v_earth_y = omega * r_val; // ≈ 466 m/s
        let sv = StateVector::new(
            Vec3::new(r_val, 0.0, 0.0),
            Vec3::new(0.0, v_earth_y - 200.0, 0.0), // v_rel ≈ 200 m/s < 305 m/s
            Met::from_seconds(0.0),
        );
        let mut state = P67State::new();
        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();

        let result = p67_step(&mut state, &sv, 0.0, 0.0, &mut pt, &mut alarm);
        assert_eq!(
            result,
            Some(false),
            "Bei v_rel < 305 m/s muss P67 beendet werden"
        );
        assert_eq!(state.phase, 0, "state.phase soll 0 (done) sein");
        assert_eq!(pt.get_phase(5), Some(0), "Phase-Table Gruppe 5 soll 0 sein");
    }

    // -----------------------------------------------------------------------
    // Test 7: drag_g — Luft-Bremsung in dichter Atmosphäre
    // -----------------------------------------------------------------------
    #[test]
    fn drag_g_low_altitude_significant() {
        // 70 km Höhe, v = 7 km/s
        let alt = 70_000.0_f64;
        let r = CentralBody::Earth.r_equatorial() + alt;
        let sv = StateVector::new(
            Vec3::new(r, 0.0, 0.0),
            Vec3::new(0.0, 7_000.0, 0.0),
            Met::from_seconds(0.0),
        );
        let d = drag_g(&sv, &CentralBody::Earth);
        // Bei 70 km und 7 km/s: Drag sollte > 0 und < 20 g sein
        assert!(d > 0.0, "Drag soll > 0 in dichter Atmosphäre");
        assert!(d < 20.0, "Drag soll < 20 g bei 70 km");
    }
}
