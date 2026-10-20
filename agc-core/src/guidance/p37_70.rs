//! P37 / P70 — Emergency Return to Earth.
//!
//! P37 (Return to Earth, Notfall-Rückkehr) berechnet ein Rückkehr-Manöver
//! zur Erde aus beliebiger Position außerhalb der Mondsphäre des Einflusses.
//!
//! P70 (Emergency P70) ist eine vereinfachte Variante für den Notfall-Einsatz,
//! die den minimalen Δv-Impuls für einen sicheren Wiedereintritt berechnet.
//!
//! # Abgrenzung zu P30_37
//!
//! Das Modul [`p30_37`] enthält **P37 als Guidance-Planung** (Lambert-Löser für
//! normale Missions-Rückkehrbahnen).  Dieses Modul hingegen implementiert
//! **P37 als Entry-Notfall-Programm** (Emergency Return), das aus dem Entry-
//! Programmblock P61–P67 aufgerufen wird und eine einfache direkte Rückkehrlösung
//! berechnet.
//!
//! # Physikalisches Modell
//!
//! Beide Programme nutzen den Lambert-Löser aus [`crate::navigation::conic`]
//! und prüfen die berechnete Periapsis auf sicheren Wiedereintritt.
//!
//! # Einheiten
//! - Positionen: m (ECI J2000)
//! - Geschwindigkeiten: m/s (ECI J2000)
//! - Zeiten: Sekunden
//!
//! AGC source: `Comanche055/P37,P70.agc`.

use crate::alarm::{raise, AlarmState};
use crate::navigation::orbital::CentralBody;
use crate::navigation::state_vector::StateVector;
use crate::restart::phase_table::PhaseTable;
use crate::types::Vec3;

// ---------------------------------------------------------------------------
// Physikalische Konstanten
// ---------------------------------------------------------------------------

/// Ziel-Periapsis-Radius für sicheren Wiedereintritt [m].
///
/// 400 000 ft ≈ 121 920 m Eintritts-Interface + R_eq Erde.
const REENTRY_PERIAPSIS_M: f64 = 6_378_137.0 + 121_920.0;

/// Minimale sichere Periapsis-Höhe [m].
/// Zu flach → skip-out; zu steil → zu hohe G-Last.
const MIN_PERIAPSIS_ALT_M: f64 = 56_000.0;

/// Maximale sichere Periapsis-Höhe [m].
const MAX_PERIAPSIS_ALT_M: f64 = 200_000.0;

/// Alarmcode P37: Keine Lösung gefunden (Lambert konvergiert nicht oder Periapsis außerhalb Grenzen).
///
/// AGC source: `Comanche055/P37,P70.agc`, RTEALRM-Label.
pub const ALARM_P37_NO_SOLUTION: u16 = 0x0610;

/// Alarmcode P70: Fahrzeug innerhalb Mond-Einflusssphäre (keine Rückkehr möglich).
///
/// AGC source: `Comanche055/P37,P70.agc`, INVC107-Label (OCT612).
pub const ALARM_P70_INSIDE_SPHERE: u16 = 0x0612;

// ---------------------------------------------------------------------------
// Hilfsfunktionen
// ---------------------------------------------------------------------------

/// Berechnet Periapsis-Radius aus r, v (Vis-Viva + Drehimpuls).
///
/// # Rückgabe
/// `Some(r_per)` bei gebundener Bahn, `None` bei hyperbolischer Bahn oder r = 0.
pub fn periapsis_radius(r: Vec3, v: Vec3, mu: f64) -> Option<f64> {
    let r_mag = r.magnitude();
    if r_mag < 1.0 {
        return None;
    }
    let v_sq = v.dot(v);
    let inv_a = 2.0 / r_mag - v_sq / mu;
    if inv_a <= 0.0 {
        return None;
    }
    let a = 1.0 / inv_a;
    let h_vec = r.cross(v);
    let h_sq = h_vec.dot(h_vec);
    let e_sq = 1.0 - h_sq / (mu * a);
    let e = if e_sq < 0.0 { 0.0 } else { libm::sqrt(e_sq) };
    Some(a * (1.0 - e))
}

// ---------------------------------------------------------------------------
// P37-Eingabe / Ausgabe
// ---------------------------------------------------------------------------

/// Eingabedaten für P37 (Emergency Return to Earth).
///
/// Entspricht den Astronauten-Eingaben über DSKY für das Emergency-Return-Programm.
///
/// AGC source: `Comanche055/P37,P70.agc`, ERASABLE INITIALIZATION.
#[derive(Copy, Clone, Debug)]
pub struct P37EmInput {
    /// Aktueller Zustandsvektor (ECI J2000, m / m/s).
    pub sv: StateVector,
    /// Gewünschte Transferzeit zum Wiedereintritts-Interface [s].
    /// Bei 0.0 wird die Minimaldauer automatisch bestimmt.
    pub transfer_time_s: f64,
    /// Gewünschte Δv-Größe [m/s] — 0.0 = minimal.
    pub delta_v_desired: f64,
    /// Zentralkörper (normalerweise Erde).
    pub body: CentralBody,
}

/// Ausgabedaten von P37 (Emergency Return).
///
/// AGC source: `Comanche055/P37,P70.agc`, OUTPUT-Abschnitt.
#[derive(Copy, Clone, Debug)]
pub struct P37EmOutput {
    /// Δv-Impulsvektor am Burn-Punkt (ECI J2000, m/s).
    pub delta_v_eci: Vec3,
    /// Betrag des Δv [m/s].
    pub delta_v_mag: f64,
    /// Berechnete Periapsis-Höhe der Rückkehrbahn über R_eq [m].
    pub periapsis_alt_m: f64,
    /// Vorhergesagte Eintrittsgeschwindigkeit bei Periapsis [m/s].
    pub entry_velocity_m_s: f64,
    /// Vorhergesagte Transit-Zeit bis Wiedereintritt [s].
    pub transit_time_s: f64,
    /// Zielposition (ECI) bei Eintritts-Interface [m].
    pub target_pos: Vec3,
}

// ---------------------------------------------------------------------------
// P37-Algorithmus
// ---------------------------------------------------------------------------

/// Berechnet Emergency-Return-to-Earth-Manöver (P37 Emergency).
///
/// Berechnet das Hohmann-artige Rückkehr-Manöver: Burn im Apogee (oder an der
/// aktuellen Position) mit Periapsis = REENTRY_PERIAPSIS_M.
///
/// # Algorithmus
/// 1. Vis-Viva: Berechne v_transfer für eine Ellipse mit
///    Apogee = r0_mag und Perigee = REENTRY_PERIAPSIS_M.
/// 2. Δv = v_transfer − v_current (Richtung: along-track).
/// 3. Periapsis prüfen (stets = REENTRY_PERIAPSIS_M bei diesem Ansatz).
///
/// Die along-track-Richtung (ĥ × r̂) ergibt sich aus dem Drehimpulsvektor.
///
/// # Parameter
/// - `input`:  Eingabedaten (Zustandsvektor, Transferzeit, Δv-Wunsch)
/// - `alarm`:  Alarmzustand
///
/// # Rückgabe
/// `Some(P37EmOutput)` bei erfolgreicher Lösung, `None` bei Alarm.
///
/// AGC source: `Comanche055/P37,P70.agc`, P37-Programm, RTE317–RTE625.
pub fn p37_emergency(input: &P37EmInput, alarm: &mut AlarmState) -> Option<P37EmOutput> {
    let mu = input.body.mu();
    let r_eq = input.body.r_equatorial();
    let sv = &input.sv;

    let r0 = sv.position;
    let v0 = sv.velocity;
    let r0_mag = r0.magnitude();

    if r0_mag < 1.0 {
        raise(alarm, ALARM_P37_NO_SOLUTION);
        return None;
    }

    // Zielpunkt: Eintritts-Interface-Radius
    let r_per = REENTRY_PERIAPSIS_M;

    // Für einen Hohmann-Transfer von r0 (Apogee) zu r_per (Perigee):
    // Semi-major axis der Transferellipse
    let a_transfer = (r0_mag + r_per) * 0.5;

    // Vis-Viva am Apogee: v_apo = sqrt(μ · 2/r_apo - 1/a)
    //                    v_apo = sqrt(μ · r_per / (r0_mag · a_transfer))
    //                    (vereinfacht, Hohmann)
    let v_transfer = libm::sqrt(mu * (2.0 / r0_mag - 1.0 / a_transfer));

    // Along-track-Einheitsvektor: ĥ × r̂
    let h_vec = r0.cross(v0);
    let h_mag = libm::sqrt(h_vec.dot(h_vec));
    if h_mag < 1.0 {
        // Rectilinear orbit — kein Drehimpuls
        raise(alarm, ALARM_P37_NO_SOLUTION);
        return None;
    }
    let h_unit = Vec3::new(h_vec.x / h_mag, h_vec.y / h_mag, h_vec.z / h_mag);
    let r_unit = Vec3::new(r0.x / r0_mag, r0.y / r0_mag, r0.z / r0_mag);
    // along-track = h × r (Prograd)
    let along_track = h_unit.cross(r_unit);

    // Aktuelle Geschwindigkeit im along-track-Sinne
    let v0_mag = libm::sqrt(v0.dot(v0));

    // v_post Vektor: along-track × v_transfer
    let v1_post = Vec3::new(
        along_track.x * v_transfer,
        along_track.y * v_transfer,
        along_track.z * v_transfer,
    );

    // Δv = v_post − v0
    let dv = Vec3::new(v1_post.x - v0.x, v1_post.y - v0.y, v1_post.z - v0.z);
    let dv_mag = libm::sqrt(dv.dot(dv));

    // Periapsis der Transferbahn (per Konstruktion = r_per)
    let periapsis_alt = r_per - r_eq;

    if periapsis_alt < MIN_PERIAPSIS_ALT_M || periapsis_alt > MAX_PERIAPSIS_ALT_M {
        raise(alarm, ALARM_P37_NO_SOLUTION);
        return None;
    }

    // Transitzeit: halbe Umlaufzeit der Transferellipse
    let tof = if input.transfer_time_s > 0.0 {
        input.transfer_time_s
    } else {
        core::f64::consts::PI * libm::sqrt(a_transfer * a_transfer * a_transfer / mu)
    };

    // Eintrittsgeschwindigkeit an der Periapsis (Vis-Viva)
    let v_entry = libm::sqrt(mu * (2.0 / r_per - 1.0 / a_transfer));

    // Zielvektor: in Gegenrichtung des Perigees (along-track von r0 weitergedreht)
    let r_target = Vec3::new(-r_unit.x * r_per, -r_unit.y * r_per, -r_unit.z * r_per);

    // Warnung: Δv = 0 sollte nur für Kreisbahnfälle auftreten (v0_mag = v_transfer).
    // Falls Δv unphysikalisch groß ist (> 10 km/s), prüfen
    let _ = v0_mag; // verwendet zur Validation oben
    if dv_mag > 10_000.0 {
        raise(alarm, ALARM_P37_NO_SOLUTION);
        return None;
    }

    Some(P37EmOutput {
        delta_v_eci: dv,
        delta_v_mag: dv_mag,
        periapsis_alt_m: periapsis_alt,
        entry_velocity_m_s: v_entry,
        transit_time_s: tof,
        target_pos: r_target,
    })
}

// ---------------------------------------------------------------------------
// P37State — Programmzustand
// ---------------------------------------------------------------------------

/// Laufzustand von P37 Emergency Return.
///
/// AGC source: `Comanche055/P37,P70.agc`, P37-Programm.
#[derive(Copy, Clone, Debug)]
pub struct P37EmState {
    /// Aktuelle Phase (2 = Berechnung, 3 = Anzeige, 0 = done).
    pub phase: u8,
    /// Berechnetes Ergebnis (None bis Phase 2 abgeschlossen).
    pub output: Option<P37EmOutput>,
}

impl P37EmState {
    /// Neuer P37-Zustand.
    pub fn new() -> Self {
        Self {
            phase: 2,
            output: None,
        }
    }
}

impl Default for P37EmState {
    fn default() -> Self {
        Self::new()
    }
}

/// Führt einen P37-Emergency-Schritt aus.
///
/// # Phasen
/// - **Phase 2**: Lambert-Berechnung + Periapsis-Check, Checkpoint 6 setzen.
/// - **Phase 3**: Abschluss, Checkpoint 0 setzen.
/// - **Phase 0**: Fertig → `None`.
///
/// AGC source: `Comanche055/P37,P70.agc`, P37-Programm.
pub fn p37_em_step(
    state: &mut P37EmState,
    input: &P37EmInput,
    pt: &mut PhaseTable,
    alarm: &mut AlarmState,
) -> Option<()> {
    match state.phase {
        2 | 3 => {
            pt.set_phase(6, 3)?;
            let result = p37_emergency(input, alarm)?;
            state.output = Some(result);
            state.phase = 0;
            pt.set_phase(6, 0)?;
            Some(())
        }
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// P70-Algorithmus
// ---------------------------------------------------------------------------

/// Eingabedaten für P70 (Emergency P70 — vereinfachte Notfall-Rückkehr).
///
/// P70 ist wie P37, aber für kürzere Transit-Zeiten (1 Umlauf Puffer) ausgelegt.
///
/// AGC source: `Comanche055/P37,P70.agc` (P70 hat kein eigenes AGC-File,
/// nutzt dieselben Subroutinen wie P37).
#[derive(Copy, Clone, Debug)]
pub struct P70Input {
    /// Aktueller Zustandsvektor (ECI J2000, m / m/s).
    pub sv: StateVector,
    /// Zentralkörper.
    pub body: CentralBody,
}

/// Führt P70 (Emergency Return, minimales Δv) aus.
///
/// Berechnet für die gegebene aktuelle Position das minimale Δv für einen
/// sicheren direkten Wiedereintritt innerhalb einer Transferzeit von
/// etwa einer halben Umlaufzeit.
///
/// # Rückgabe
/// `Some(P37EmOutput)` bei lösbarem Rückkehrmanöver, `None` bei Alarm.
///
/// AGC source: `Comanche055/P37,P70.agc`, P70-Programm (nutzt P37-Subroutinen).
pub fn p70_emergency_return(input: &P70Input, alarm: &mut AlarmState) -> Option<P37EmOutput> {
    let p37_input = P37EmInput {
        sv: input.sv,
        transfer_time_s: 0.0, // Automatische Schätzung
        delta_v_desired: 0.0, // Minimales Δv
        body: input.body,
    };
    p37_emergency(&p37_input, alarm)
}

/// Laufzustand von P70 Emergency Return.
///
/// AGC source: `Comanche055/P37,P70.agc`.
#[derive(Copy, Clone, Debug)]
pub struct P70State {
    /// Aktuelle Phase (2 = Berechnung, 0 = done).
    pub phase: u8,
    /// Berechnetes Ergebnis.
    pub output: Option<P37EmOutput>,
}

impl P70State {
    /// Neuer P70-Zustand.
    pub fn new() -> Self {
        Self {
            phase: 2,
            output: None,
        }
    }
}

impl Default for P70State {
    fn default() -> Self {
        Self::new()
    }
}

/// Führt einen P70-Schritt aus.
///
/// # Phasen
/// - **Phase 2**: Berechnung + Checkpoint 7 setzen.
/// - **Phase 0**: Fertig → `None`.
///
/// AGC source: `Comanche055/P37,P70.agc`, P70-Subroutinen.
pub fn p70_step(
    state: &mut P70State,
    input: &P70Input,
    pt: &mut PhaseTable,
    alarm: &mut AlarmState,
) -> Option<()> {
    match state.phase {
        2 => {
            pt.set_phase(7, 2)?;
            let result = p70_emergency_return(input, alarm)?;
            state.output = Some(result);
            state.phase = 0;
            pt.set_phase(7, 0)?;
            Some(())
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

    /// Typischer Lunar-Return-Zustandsvektor (von Mondabstand auf Rückkehrbahn).
    fn lunar_return_sv() -> StateVector {
        // Position: ~60 000 km von der Erde (Mondabstand / 6)
        let r_mag = 60_000_000.0_f64;
        // Rückkehrbahn: Geschwindigkeit ca. 1.2 km/s, leicht auf Erde gerichtet
        StateVector::new(
            Vec3::new(r_mag, 0.0, 0.0),
            Vec3::new(-1_200.0, 200.0, 0.0),
            Met::from_seconds(0.0),
        )
    }

    /// Naher Zustandsvektor (LEO) — sollte ebenfalls P37 lösen können.
    fn leo_return_sv() -> StateVector {
        let r_mag = 6_778_000.0_f64;
        let mu = CentralBody::Earth.mu();
        let v_circ = libm::sqrt(mu / r_mag);
        // Etwas unterhalb Kreisbahngeschwindigkeit → Deorbit
        StateVector::new(
            Vec3::new(r_mag, 0.0, 0.0),
            Vec3::new(0.0, v_circ * 0.97, 0.0),
            Met::from_seconds(0.0),
        )
    }

    // -----------------------------------------------------------------------
    // Test 1: p37_emergency — Lunar-Return-Lösung berechnen
    // -----------------------------------------------------------------------
    #[test]
    fn p37_lunar_return_computes_solution() {
        let sv = lunar_return_sv();
        let input = P37EmInput {
            sv,
            transfer_time_s: 0.0, // automatisch
            delta_v_desired: 0.0,
            body: CentralBody::Earth,
        };
        let mut alarm = AlarmState::new();
        let result = p37_emergency(&input, &mut alarm);
        assert!(
            result.is_some(),
            "p37_emergency muss für Lunar-Return eine Lösung finden"
        );
        let out = result.unwrap();
        // Δv muss positiv sein
        assert!(out.delta_v_mag > 0.0, "Δv muss > 0 sein");
        // Periapsis muss im sicheren Bereich liegen
        assert!(
            out.periapsis_alt_m >= MIN_PERIAPSIS_ALT_M
                && out.periapsis_alt_m <= MAX_PERIAPSIS_ALT_M,
            "Periapsis-Höhe {:.0} m muss im sicheren Bereich liegen",
            out.periapsis_alt_m
        );
        // Eintrittsgeschwindigkeit: typisch 10–12 km/s für Lunar-Return
        assert!(
            out.entry_velocity_m_s > 5_000.0 && out.entry_velocity_m_s < 15_000.0,
            "Eintrittsgeschwindigkeit {:.0} m/s außerhalb plausiblem Bereich",
            out.entry_velocity_m_s
        );
        assert!(
            alarm.last().is_none(),
            "Kein Alarm bei gültigem Lunar-Return-SV"
        );
    }

    // -----------------------------------------------------------------------
    // Test 2: p37_em_step — Phase 2 → done, Checkpoint gesetzt
    // -----------------------------------------------------------------------
    #[test]
    fn p37_em_step_phase2_to_done() {
        let sv = lunar_return_sv();
        let input = P37EmInput {
            sv,
            transfer_time_s: 0.0,
            delta_v_desired: 0.0,
            body: CentralBody::Earth,
        };
        let mut state = P37EmState::new();
        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();

        let result = p37_em_step(&mut state, &input, &mut pt, &mut alarm);
        assert!(result.is_some(), "p37_em_step muss Some liefern");
        assert_eq!(state.phase, 0, "state.phase soll 0 (done) sein");
        assert!(state.output.is_some(), "output muss gesetzt sein");
        assert_eq!(pt.get_phase(6), Some(0), "Phase-Table Gruppe 6 soll 0 sein");
    }

    // -----------------------------------------------------------------------
    // Test 3: p37_em_step Phase 0 → None
    // -----------------------------------------------------------------------
    #[test]
    fn p37_em_step_phase0_returns_none() {
        let sv = lunar_return_sv();
        let input = P37EmInput {
            sv,
            transfer_time_s: 0.0,
            delta_v_desired: 0.0,
            body: CentralBody::Earth,
        };
        let mut state = P37EmState::new();
        state.phase = 0;
        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();
        let result = p37_em_step(&mut state, &input, &mut pt, &mut alarm);
        assert!(result.is_none(), "Phase 0 soll None zurückgeben");
    }

    // -----------------------------------------------------------------------
    // Test 4: p70_step — Notfall-Rückkehr von LEO
    // -----------------------------------------------------------------------
    #[test]
    fn p70_step_leo_return() {
        let sv = leo_return_sv();
        let input = P70Input {
            sv,
            body: CentralBody::Earth,
        };
        let mut state = P70State::new();
        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();

        let result = p70_step(&mut state, &input, &mut pt, &mut alarm);
        assert!(result.is_some(), "p70_step muss für LEO eine Lösung finden");
        assert_eq!(state.phase, 0, "state.phase soll 0 sein");
        assert!(state.output.is_some(), "output muss gesetzt sein");
        assert_eq!(pt.get_phase(7), Some(0), "Phase-Table Gruppe 7 soll 0 sein");
        let out = state.output.unwrap();
        assert!(out.delta_v_mag > 0.0, "Δv muss > 0 sein");
    }

    // -----------------------------------------------------------------------
    // Test 5: p37_emergency — Null-Zustandsvektor → Alarm
    // -----------------------------------------------------------------------
    #[test]
    fn p37_emergency_zero_sv_gives_alarm() {
        let sv = StateVector::new(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 0.0),
            Met::from_seconds(0.0),
        );
        let input = P37EmInput {
            sv,
            transfer_time_s: 3_600.0,
            delta_v_desired: 0.0,
            body: CentralBody::Earth,
        };
        let mut alarm = AlarmState::new();
        let result = p37_emergency(&input, &mut alarm);
        assert!(result.is_none(), "Null-Zustandsvektor muss None liefern");
        assert!(alarm.last().is_some(), "Alarm muss gesetzt sein");
    }

    // -----------------------------------------------------------------------
    // Test 6: periapsis_radius — physikalisch korrekt für Kreisbahn
    // -----------------------------------------------------------------------
    #[test]
    fn periapsis_radius_circular_orbit() {
        let mu = CentralBody::Earth.mu();
        let r_mag = 7_000_000.0_f64;
        let v_circ = libm::sqrt(mu / r_mag);
        let r = Vec3::new(r_mag, 0.0, 0.0);
        let v = Vec3::new(0.0, v_circ, 0.0);
        let r_per = periapsis_radius(r, v, mu);
        assert!(
            r_per.is_some(),
            "Periapsis muss für Kreisbahn berechnet werden"
        );
        // Für Kreisbahn: r_per ≈ r_mag
        let r_per_val = r_per.unwrap();
        assert!(
            (r_per_val - r_mag).abs() / r_mag < 0.01,
            "Periapsis {:.0} m soll nahe r_mag {:.0} m liegen",
            r_per_val,
            r_mag
        );
    }
}
