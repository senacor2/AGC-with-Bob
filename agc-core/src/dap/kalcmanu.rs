/// KALCMANU — Automatische Lagemanöver-Steuerung.
///
/// Berechnet inkrementelle CDU-Winkelschritte für ein automatisches Lagemanöver
/// von der aktuellen Lage zur Ziel-Lage, mit konfigurierbarer Manöverrate.
/// Neue Befehle werden jede Sekunde generiert (DAP-Abtastrate).
///
/// Die Ziel-CDU-Winkel werden aus der gewünschten Rotationsmatrix berechnet
/// (DCM → CDU, Funktion [`crate::navigation::alignment::dcm_to_cdu_angles`]).
///
/// # Manöverstrategien
/// | `ManeuverRate` | Winkelgeschwindigkeit  | AGC-RATEINDX |
/// |----------------|------------------------|--------------|
/// | `Slow`         | 0,05 °/s               | 0            |
/// | `Medium`       | 0,2 °/s                | 1            |
/// | `Fast`         | 0,5 °/s                | 2            |
/// | `VeryFast`     | 2,0 °/s                | 3            |
///
/// # AGC source
/// `Comanche055/KALCMANU_STEERING.agc`, NEWDELHI / NEWANGL / INCRDCDU Routinen.
use crate::alarm::{self, AlarmState};

// ---------------------------------------------------------------------------
// Manöver-Rate
// ---------------------------------------------------------------------------

/// Manöverwinkelgeschwindigkeit (entspricht ARATE-Tabelle im AGC).
///
/// AGC source: `Comanche055/KALCMANU_STEERING.agc`, ARATE Konstanten.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ManeuverRate {
    /// 0,05 °/s — ARATE[0].
    Slow,
    /// 0,2 °/s — ARATE[1].
    Medium,
    /// 0,5 °/s — ARATE[2].
    Fast,
    /// 2,0 °/s — ARATE[3].
    VeryFast,
}

impl ManeuverRate {
    /// Gibt die Rate in rad/s zurück.
    pub const fn rad_per_sec(self) -> f64 {
        match self {
            ManeuverRate::Slow => 8.727e-4,     // 0,05°/s
            ManeuverRate::Medium => 3.491e-3,   // 0,2°/s
            ManeuverRate::Fast => 8.727e-3,     // 0,5°/s
            ManeuverRate::VeryFast => 3.491e-2, // 2,0°/s
        }
    }
}

// ---------------------------------------------------------------------------
// Zustand
// ---------------------------------------------------------------------------

/// Zustand eines laufenden KALCMANU-Manövers.
///
/// AGC source: `Comanche055/KALCMANU_STEERING.agc`, BCDU / NCDU / DELCDUX
/// erasable variables.
#[derive(Clone, Copy, Debug)]
pub struct KalcmanuState {
    /// Ziel-CDU-Winkel [OGA, MGA, IGA], in CDU-Bits.
    pub target_cdu: [i16; 3],
    /// Startwinkel bei Manöververgabe, in CDU-Bits.
    pub start_cdu: [i16; 3],
    /// Inkrementelle Schritte pro Sekunde [OGA, MGA, IGA], in CDU-Bits.
    pub delta_cdu: [i16; 3],
    /// Verbleibende Manöverzeit in Sekunden.
    pub time_remaining: f64,
    /// Manöver aktiv.
    pub active: bool,
}

impl KalcmanuState {
    pub const fn new() -> Self {
        Self {
            target_cdu: [0; 3],
            start_cdu: [0; 3],
            delta_cdu: [0; 3],
            time_remaining: 0.0,
            active: false,
        }
    }
}

impl Default for KalcmanuState {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Öffentliche API
// ---------------------------------------------------------------------------

/// Initialisiert ein neues KALCMANU-Manöver.
///
/// Berechnet die CDU-Winkelschritte pro Sekunde aus dem Gesamt-Winkel und
/// der gewählten Manöverrate. Setzt `state.active = true`.
///
/// # Parameter
/// - `state`:       veränderlicher Manöver-Zustand.
/// - `current_cdu`: aktuelle CDU-Winkel [OGA, MGA, IGA] in CDU-Bits.
/// - `target_cdu`:  Ziel-CDU-Winkel [OGA, MGA, IGA] in CDU-Bits.
/// - `rate`:        gewünschte Manöverrate.
///
/// Gibt `None` zurück und ruft [`alarm::raise`] wenn die Ziel-CDU-Winkel
/// Gimbal-Lock implizieren (MGA |> 70°, d.h. |MGA| > 6371 CDU-Bits).
///
/// AGC source: `Comanche055/KALCMANU_STEERING.agc`, MANUSTAT Routine.
pub fn kalcmanu_start(
    alarm_state: &mut AlarmState,
    state: &mut KalcmanuState,
    current_cdu: [i16; 3],
    target_cdu: [i16; 3],
    rate: ManeuverRate,
) -> Option<()> {
    // Gimbal-Lock-Prüfung: |MGA| > 70° (bei 15-Bit-Vollkreis = 32768 Bits)
    // 70° = 70/360 * 32768 ≈ 6371 Bits
    const GIMBAL_LOCK_LIMIT: i16 = 6371;
    if target_cdu[1].abs() > GIMBAL_LOCK_LIMIT {
        alarm::raise(alarm_state, 0x0412); // Alarm 0412: Gimbal Lock im KALCMANU
        return None;
    }

    state.start_cdu = current_cdu;
    state.target_cdu = target_cdu;

    // Skalierungsfaktor: rad pro CDU-Bit = 2π / 32768
    const CDU_LSB_RAD: f64 = core::f64::consts::TAU / 32768.0;

    // Grössten Winkelunterschied bestimmen (für Manöverzeit)
    let mut max_delta_rad: f64 = 0.0;
    for i in 0..3 {
        let delta = (target_cdu[i].wrapping_sub(current_cdu[i])) as f64;
        let delta_rad = libm::fabs(delta) * CDU_LSB_RAD;
        if delta_rad > max_delta_rad {
            max_delta_rad = delta_rad;
        }
    }

    let rate_rad_s = rate.rad_per_sec();
    if max_delta_rad < 1e-9 {
        // Kein Manöver nötig
        state.active = false;
        return Some(());
    }

    let maneuver_time = max_delta_rad / rate_rad_s; // Sekunden

    // CDU-Inkremente pro Sekunde
    for i in 0..3 {
        let delta_bits = (target_cdu[i].wrapping_sub(current_cdu[i])) as f64;
        let step = delta_bits / maneuver_time;
        // Auf ±32767 begrenzen (CDU-Bits)
        state.delta_cdu[i] = libm::round(step).clamp(-32767.0, 32767.0) as i16;
    }

    state.time_remaining = maneuver_time;
    state.active = true;
    Some(())
}

/// Führt einen KALCMANU-Schritt durch (einmal pro Sekunde).
///
/// Gibt die neuen Soll-CDU-Winkel zurück, die dem Guidance-Programm als
/// Lageziel übergeben werden.  Setzt `active = false` wenn das Ziel erreicht.
///
/// AGC source: `Comanche055/KALCMANU_STEERING.agc`, NEWANGL / TMANUCHK Routinen.
pub fn kalcmanu_step(state: &mut KalcmanuState, dt: f64) -> Option<[i16; 3]> {
    if !state.active {
        return None;
    }

    state.time_remaining -= dt;

    if state.time_remaining <= 0.0 {
        // Manöver abgeschlossen — Zielwinkel direkt setzen
        state.active = false;
        return Some(state.target_cdu);
    }

    // Nächste CDU-Sollwerte = Start + Anteil der Gesamtdeltage der noch verbleibt
    let steps_done = libm::ceil(state.time_remaining / dt) as i16;
    let deltas: [i32; 3] = [
        (state.target_cdu[0].wrapping_sub(state.start_cdu[0])) as i32,
        (state.target_cdu[1].wrapping_sub(state.start_cdu[1])) as i32,
        (state.target_cdu[2].wrapping_sub(state.start_cdu[2])) as i32,
    ];
    let total_steps = state.delta_cdu.map(|d| d.max(1) as i32);
    let cmd_cdu = [
        state.start_cdu[0].wrapping_add(
            (deltas[0] * steps_done as i32 / total_steps[0]).clamp(-32768, 32767) as i16,
        ),
        state.start_cdu[1].wrapping_add(
            (deltas[1] * steps_done as i32 / total_steps[1]).clamp(-32768, 32767) as i16,
        ),
        state.start_cdu[2].wrapping_add(
            (deltas[2] * steps_done as i32 / total_steps[2]).clamp(-32768, 32767) as i16,
        ),
    ];

    Some(cmd_cdu)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kalcmanu_no_motion_if_at_target() {
        let mut alarm = alarm::AlarmState::new();
        let mut state = KalcmanuState::new();
        let result = kalcmanu_start(
            &mut alarm,
            &mut state,
            [0, 0, 0],
            [0, 0, 0],
            ManeuverRate::Medium,
        );
        assert!(result.is_some());
        assert!(!state.active, "Kein Manöver nötig wenn bereits am Ziel");
    }

    #[test]
    fn kalcmanu_reaches_target() {
        let mut alarm = alarm::AlarmState::new();
        let mut state = KalcmanuState::new();
        // 1000 CDU-Bits ≈ 11° auf OGA-Achse (keine Gimbal-Lock-Gefahr)
        kalcmanu_start(
            &mut alarm,
            &mut state,
            [0, 0, 0],
            [1000, 0, 0],
            ManeuverRate::Fast,
        )
        .unwrap();
        assert!(state.active);
        assert!(state.time_remaining > 0.0);
    }

    #[test]
    fn kalcmanu_gimbal_lock_rejected() {
        let mut alarm = alarm::AlarmState::new();
        let mut state = KalcmanuState::new();
        // |MGA| > 70° → Alarm, kein Manöver
        let result = kalcmanu_start(
            &mut alarm,
            &mut state,
            [0, 0, 0],
            [0, 7000, 0], // MGA > GIMBAL_LOCK_LIMIT
            ManeuverRate::Fast,
        );
        assert!(result.is_none(), "Gimbal-Lock muss abgelehnt werden");
    }

    #[test]
    fn maneuver_rate_values() {
        // Sicherstellen dass Ratenwerte positiv und aufsteigend sind
        let rates = [
            ManeuverRate::Slow.rad_per_sec(),
            ManeuverRate::Medium.rad_per_sec(),
            ManeuverRate::Fast.rad_per_sec(),
            ManeuverRate::VeryFast.rad_per_sec(),
        ];
        for i in 0..3 {
            assert!(
                rates[i] < rates[i + 1],
                "Manöverraten müssen aufsteigend sein"
            );
        }
    }
}
