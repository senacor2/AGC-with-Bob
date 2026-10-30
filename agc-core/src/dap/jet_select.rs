/// Jet-Selection-Logic für den RCS-CSM Autopiloten.
///
/// Mappt Translationsanforderungen (Pitch, Yaw, Roll) auf konkrete RCS-Jet-Bits
/// für Kanal 5 (Quad A/B) und Kanal 6 (Quad C/D). Die Logik entspricht den
/// Tabellen aus dem AGC-Quellcode: Für jede Kombinaton aus Translationsachse
/// und Richtung sind je zwei komplementäre Jet-Paare vorgesehen, damit kein
/// Nettodrehmoment entsteht (außer bei Roll-Befehlen).
///
/// # Jet-Nummerierung (AGC ICD, CSM RCS)
/// ```text
///  Quad A: A1..A4  → Kanal 5, Bits 0..3
///  Quad B: B1..B4  → Kanal 5, Bits 4..7
///  Quad C: C1..C4  → Kanal 6, Bits 0..3
///  Quad D: D1..D4  → Kanal 6, Bits 4..7
/// ```
///
/// # AGC source
/// `Comanche055/JET_SELECTION_LOGIC.agc`, JETSLECT / PWORD / YWORD / ZWORD
/// Routinen (Tabellen PCOM, YCOM, ZCOM).
use crate::hal::RcsHal;

// ---------------------------------------------------------------------------
// Jet-Befehlstabellen
// ---------------------------------------------------------------------------

/// Pitch+ (Nase hoch) — Kanal 5 und 6.
const PITCH_POS_AB: u8 = 0b0000_1001; // A1 + A4 (vorn-unten)
const PITCH_POS_CD: u8 = 0b0000_1001; // C1 + C4

/// Pitch− (Nase runter).
const PITCH_NEG_AB: u8 = 0b0110_0000; // B3 + B2
const PITCH_NEG_CD: u8 = 0b0110_0000; // D3 + D2

/// Yaw+ (Nase rechts).
const YAW_POS_AB: u8 = 0b0001_0100; // A3 + B1
const YAW_POS_CD: u8 = 0b0001_0100; // C3 + D1

/// Yaw− (Nase links).
const YAW_NEG_AB: u8 = 0b1000_0010; // B4 + A2
const YAW_NEG_CD: u8 = 0b1000_0010; // D4 + C2

/// Roll+ (Uhrzeigersinn aus Nase).
const ROLL_POS_AB: u8 = 0b0000_0101; // A1 + A3
const ROLL_POS_CD: u8 = 0b0000_0000; // keine C/D-Jets für Roll+

/// Roll− (Gegen-Uhrzeigersinn).
const ROLL_NEG_AB: u8 = 0b0000_1010; // A2 + A4
const ROLL_NEG_CD: u8 = 0b0000_0000;

// ---------------------------------------------------------------------------
// Öffentliche API
// ---------------------------------------------------------------------------

/// Beschreibt den Feuerbefehl für eine Rotationsachse.
///
/// Entspricht dem TAU-Wert (0 = kein Feuer, +1 = positiv, −1 = negativ)
/// aus dem AGC-Quellcode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AxisCmd {
    /// Kein Jet-Feuer für diese Achse.
    None,
    /// Positiver Rotationsimpuls (Pitch↑, Yaw→, Roll+CW).
    Positive,
    /// Negativer Rotationsimpuls.
    Negative,
}

/// Lagefehler-Schwellwertentscheidung für eine Körperachse.
///
/// Gibt [`AxisCmd`] zurück: Feuer wenn |`error`| > `deadband` und die Richtung
/// nicht bereits durch eine entsprechende Rate korrigiert wird.
///
/// # AGC source
/// `Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc`, KMATRIX Abschnitt.
pub fn axis_command(error: f64, rate: f64, deadband: f64) -> AxisCmd {
    if error.abs() <= deadband {
        return AxisCmd::None;
    }
    // Wenn Rate bereits in korrekter Richtung und > Grenzrate → küsten
    let correcting_rate = if error > 0.0 { rate < 0.0 } else { rate > 0.0 };
    if correcting_rate && rate.abs() > deadband * 0.5 {
        return AxisCmd::None;
    }
    if error > 0.0 {
        AxisCmd::Positive
    } else {
        AxisCmd::Negative
    }
}

/// Wählt RCS-Jets basierend auf Pitch-, Yaw- und Roll-Befehlen aus und schreibt
/// sie via [`RcsHal`].
///
/// Mehrere gleichzeitige Achsbefehle werden durch bitweises OR kombiniert.
///
/// # Parameter
/// - `pitch`:  [`AxisCmd`] für die Pitch-Achse.
/// - `yaw`:    [`AxisCmd`] für die Yaw-Achse.
/// - `roll`:   [`AxisCmd`] für die Roll-Achse.
/// - `rcs`:    RCS HAL für die Ausgabe.
///
/// AGC source: `Comanche055/JET_SELECTION_LOGIC.agc`, PWORD / YWORD / ZWORD
/// Tabellen und Ausgabe via CHAN 5 / CHAN 6.
/// Berechnet RCS-Jet-Bits ohne HAL-Zugriff.
///
/// Gibt `(jets_ab, jets_cd)` zurück — für den Waitlist-Task in `agc-core`,
/// der keine `&mut RcsHal`-Referenz halten kann.
///
/// AGC source: `Comanche055/JET_SELECTION_LOGIC.agc`, PWORD / YWORD / ZWORD.
pub fn select_jets_raw(pitch: AxisCmd, yaw: AxisCmd, roll: AxisCmd) -> (u8, u8) {
    let mut ab: u8 = 0;
    let mut cd: u8 = 0;

    match pitch {
        AxisCmd::Positive => {
            ab |= PITCH_POS_AB;
            cd |= PITCH_POS_CD;
        }
        AxisCmd::Negative => {
            ab |= PITCH_NEG_AB;
            cd |= PITCH_NEG_CD;
        }
        AxisCmd::None => {}
    }

    match yaw {
        AxisCmd::Positive => {
            ab |= YAW_POS_AB;
            cd |= YAW_POS_CD;
        }
        AxisCmd::Negative => {
            ab |= YAW_NEG_AB;
            cd |= YAW_NEG_CD;
        }
        AxisCmd::None => {}
    }

    match roll {
        AxisCmd::Positive => {
            ab |= ROLL_POS_AB;
            cd |= ROLL_POS_CD;
        }
        AxisCmd::Negative => {
            ab |= ROLL_NEG_AB;
            cd |= ROLL_NEG_CD;
        }
        AxisCmd::None => {}
    }

    (ab, cd)
}

/// Wählt RCS-Jets basierend auf Pitch-, Yaw- und Roll-Befehlen aus und schreibt
/// sie via [`RcsHal`].
///
/// Mehrere gleichzeitige Achsbefehle werden durch bitweises OR kombiniert.
///
/// # Parameter
/// - `pitch`:  [`AxisCmd`] für die Pitch-Achse.
/// - `yaw`:    [`AxisCmd`] für die Yaw-Achse.
/// - `roll`:   [`AxisCmd`] für die Roll-Achse.
/// - `rcs`:    RCS HAL für die Ausgabe.
///
/// AGC source: `Comanche055/JET_SELECTION_LOGIC.agc`, PWORD / YWORD / ZWORD
/// Tabellen und Ausgabe via CHAN 5 / CHAN 6.
pub fn select_jets<R: RcsHal>(pitch: AxisCmd, yaw: AxisCmd, roll: AxisCmd, rcs: &mut R) {
    let (ab, cd) = select_jets_raw(pitch, yaw, roll);
    rcs.set_jets_ab(ab);
    rcs.set_jets_cd(cd);
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hal::rcs::mock::MockRcs;

    const DB: f64 = 0.087_266_46; // 5° in rad

    #[test]
    fn no_commands_zero_jets() {
        let mut rcs = MockRcs::new();
        select_jets(AxisCmd::None, AxisCmd::None, AxisCmd::None, &mut rcs);
        assert_eq!(rcs.jets_ab, 0);
        assert_eq!(rcs.jets_cd, 0);
    }

    #[test]
    fn pitch_positive_fires_correct_jets() {
        let mut rcs = MockRcs::new();
        select_jets(AxisCmd::Positive, AxisCmd::None, AxisCmd::None, &mut rcs);
        assert_eq!(rcs.jets_ab, PITCH_POS_AB);
        assert_eq!(rcs.jets_cd, PITCH_POS_CD);
    }

    #[test]
    fn pitch_negative_fires_correct_jets() {
        let mut rcs = MockRcs::new();
        select_jets(AxisCmd::Negative, AxisCmd::None, AxisCmd::None, &mut rcs);
        assert_eq!(rcs.jets_ab, PITCH_NEG_AB);
        assert_eq!(rcs.jets_cd, PITCH_NEG_CD);
    }

    #[test]
    fn yaw_positive_fires_correct_jets() {
        let mut rcs = MockRcs::new();
        select_jets(AxisCmd::None, AxisCmd::Positive, AxisCmd::None, &mut rcs);
        assert_eq!(rcs.jets_ab, YAW_POS_AB);
        assert_eq!(rcs.jets_cd, YAW_POS_CD);
    }

    #[test]
    fn roll_positive_fires_correct_jets() {
        let mut rcs = MockRcs::new();
        select_jets(AxisCmd::None, AxisCmd::None, AxisCmd::Positive, &mut rcs);
        assert_eq!(rcs.jets_ab, ROLL_POS_AB);
        assert_eq!(rcs.jets_cd, ROLL_POS_CD);
    }

    #[test]
    fn combined_pitch_yaw() {
        let mut rcs = MockRcs::new();
        select_jets(
            AxisCmd::Positive,
            AxisCmd::Negative,
            AxisCmd::None,
            &mut rcs,
        );
        // Kombinierte Maske
        assert_eq!(rcs.jets_ab, PITCH_POS_AB | YAW_NEG_AB);
        assert_eq!(rcs.jets_cd, PITCH_POS_CD | YAW_NEG_CD);
    }

    // --- axis_command ---

    #[test]
    fn axis_cmd_no_error() {
        assert_eq!(axis_command(0.0, 0.0, DB), AxisCmd::None);
    }

    #[test]
    fn axis_cmd_small_error_within_deadband() {
        assert_eq!(axis_command(0.01, 0.0, DB), AxisCmd::None);
    }

    #[test]
    fn axis_cmd_large_positive_error() {
        assert_eq!(axis_command(0.2, 0.0, DB), AxisCmd::Positive);
    }

    #[test]
    fn axis_cmd_large_negative_error() {
        assert_eq!(axis_command(-0.2, 0.0, DB), AxisCmd::Negative);
    }

    #[test]
    fn axis_cmd_correcting_rate_coasts() {
        // Fehler positiv, Rate bereits stark negativ → küste
        assert_eq!(axis_command(0.2, -0.1, DB), AxisCmd::None);
    }
}
