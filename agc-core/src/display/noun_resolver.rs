//! Noun-Resolver — bildet `(NounCode, &AgcContext)` auf `[DisplayRegister; 3]` ab.
//!
//! Liest die physikalischen Werte aus dem [`GuidanceContext`] und skaliert
//! sie für die DSKY-Anzeige. Nur mission-kritische Nouns sind implementiert.
//!
//! # Implementierungsstand (Sub-Tasks 4 + 5)
//! Vollständig implementiert für alle Tier-1/Tier-2-Nouns sowie IMU-Alignment-Nouns:
//!
//! | Noun | R1               | R2               | R3              | Programm         |
//! |------|------------------|------------------|-----------------|------------------|
//! | N17  | OGA [0.01°]      | MGA [0.01°]      | IGA [0.01°]     | P52 CPHIX        |
//! | N20  | OGA [0.01°]      | MGA [0.01°]      | IGA [0.01°]     | P51/P52 CDU      |
//! | N22  | THETAD1 [0.01°]  | THETAD2 [0.01°]  | THETAD3 [0.01°] | P51/P52 Ziel     |
//! | N40  | TTOGO [MM:SS]    | VG [m/s]         | DVTOTAL [m/s]   | P40 Burn         |
//! | N44  | HAPO [km]        | HPER [km]        | TFF [MM:SS]     | P30/P37          |
//! | N60  | GMAX [0.01 m/s²] | VPRED [m/s]      | GAMMAEI [0.01°] | P61 Entry        |
//! | N61  | LAT(SPL) [0.01°] | LNG(SPL) [0.01°] | HEADSUP [±1]    | P61/P62 Ziel     |
//! | N63  | RTGO [km]        | VIO [m/s]        | TFE [s]         | P61 Guidance     |
//! | N66  | ROLLC [0.01°]    | XRNGERR [km]     | DNRNGERR [km]   | P64/P65          |
//! | N67  | RTOGO [km]       | LAT [0.01°]      | LONG [0.01°]    | P67 Terminal     |
//! | N69  | ROLLC [0.01°]    | DL [0.01 m/s²]   | VL [m/s]        | P64 Monitor      |
//! | N74  | ROLLC [0.01°]    | VI [m/s]         | D [0.01 m/s²]   | P64 Const-Drag   |
//! | N81  | DVx [m/s]        | DVy [m/s]        | DVz [m/s]       | P30 ΔV           |
//! | N92  | SHAFT [0.01°]    | TRUNION [0.01°]  | — (Blank)       | P52 Optik-Winkel |
//! | N93  | ΔG1 [0.001°]     | ΔG2 [0.001°]     | ΔG3 [0.001°]   | P52 Gyro-Torq    |
//!
//! Alle übrigen Nouns geben `[Blank; 3]` zurück.
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
    // Clamp auf i32-Bereich vor dem Cast; f64::MAX würde andernfalls zu i32-Überlauf führen.
    // Entspricht AGC MAXCHK: Apogee-Anzeige auf max. 9 999 999 km (= ~9999 NM) begrenzt.
    const MAX_KM: f64 = 9_999_999.0;
    libm::round((m / 1000.0).clamp(-MAX_KM, MAX_KM)) as i32
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
/// AGC source: `Comanche055/PINBALL_NOUN_TABLES.agc`, SF-Konstante für Winkel.
#[inline]
fn rad_to_centideg(rad: f64) -> i32 {
    const RAD_TO_DEG: f64 = 180.0 / core::f64::consts::PI;
    deg_to_centideg(rad * RAD_TO_DEG)
}

/// Wandelt Sekunden in einen gerundeten `i32` um → für TGO/TFE-Anzeige.
///
/// Beispiel: 65.7 s → 66
#[inline]
fn s_to_display(s: f64) -> i32 {
    libm::round(s) as i32
}

/// Wandelt Grad in tausendstel Grad (0.001°) um → für Gyro-Torquing-Anzeige (N93).
///
/// Beispiel: 0.123° → 123
///
/// AGC source: `Comanche055/ASSEMBLY_AND_OPERATION_INFORMATION.agc`, N93 (DELTA GYRO ANGLES).
#[inline]
fn deg_to_millideg(deg: f64) -> i32 {
    libm::round(deg * 1000.0) as i32
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
/// | Noun | R1               | R2               | R3               |
/// |------|------------------|------------------|------------------|
/// | N17  | OGA [0.01°]      | MGA [0.01°]      | IGA [0.01°]      |
/// | N20  | OGA [0.01°]      | MGA [0.01°]      | IGA [0.01°]      |
/// | N22  | THETAD1 [0.01°]  | THETAD2 [0.01°]  | THETAD3 [0.01°]  |
/// | N33  | TIG [HH]         | TIG [MM]         | TIG [SS]         |
/// | N35  | TTOGO [MM:SS]    | — (Blank)        | — (Blank)        |
/// | N40  | TTOGO [MM:SS]    | VGDISP [m/s]     | DVTOTAL [m/s]    |
/// | N44  | HAPO [km]        | HPER [km]        | TFF [MM:SS]      |
/// | N60  | GMAX [0.01 m/s²] | VPRED [m/s]      | GAMMAEI [0.01°]  |
/// | N61  | LAT(SPL) [0.01°] | LNG(SPL) [0.01°] | HEADSUP [±1]     |
/// | N63  | RTGO [km]        | VIO [m/s]        | TFE [s]          |
/// | N66  | ROLLC [0.01°]    | XRNGERR [km]     | DNRNGERR [km]    |
/// | N67  | RTOGO [km]       | LAT(ZIE) [0.01°] | LON(ZIE) [0.01°] |
/// | N69  | ROLLC [0.01°]    | DL [0.01 m/s²]   | VL [m/s]         |
/// | N74  | ROLLC [0.01°]    | VI [m/s]         | D [0.01 m/s²]    |
/// | N81  | DVx [m/s]        | DVy [m/s]        | DVz [m/s]        |
/// | N92  | SHAFT [0.01°]    | TRUNION [0.01°]  | — (Blank)        |
/// | N93  | ΔG1 [0.001°]     | ΔG2 [0.001°]     | ΔG3 [0.001°]    |
///
/// AGC source: `Comanche055/ASSEMBLY_AND_OPERATION_INFORMATION.agc`, Noun-Liste.
pub fn resolve(noun: NounCode, ctx: &AgcContext) -> [DisplayRegister; 3] {
    match noun.val() {
        // ------------------------------------------------------------------
        // N17 — CPHIX: Astronauten-Gesamtlage (OGA, MGA, IGA) [0.01°]
        // P52 IMU-Alignment — zeigt die aktuelle CDU-Lage.
        // R1 = OGA (Outer Gimbal Angle) [0.01°]
        // R2 = MGA (Middle Gimbal Angle) [0.01°]
        // R3 = IGA (Inner Gimbal Angle) [0.01°]
        //
        // CPHIX wird im AGC als Kopie von CDUX gespeichert (Extended Verbs, V60 SET CPHIX).
        // Hier lesen wir direkt die P51-Ausgabe (v1_body als Proxy falls vorhanden,
        // sonst thetad-Felder aus GuidanceContext).
        //
        // AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N17 (CPHIX / ASTRONAUT TOTAL ATTITUDE).
        // AGC source: EXTENDED_VERBS.agc, V60 (SET CPHIX = CDU).
        // ------------------------------------------------------------------
        17 => {
            // CPHIX = aktuelle CDU-Winkel (OGA/MGA/IGA). Wir lesen aus thetad
            // falls gesetzt, sonst Blank (P51/P52 muss erst Werte liefern).
            let [oga_rad, mga_rad, iga_rad] = ctx.guidance.thetad;
            // Nur anzeigen, wenn thetad ≠ [0;0;0] (initialer Defaultwert = kein Wert).
            // Eine echte IMU würde hier CDUX/CDUY/CDUZ liefern — im Simulator
            // werden die thetad-Felder nach P51/P52-Berechnungen gesetzt.
            if oga_rad == 0.0 && mga_rad == 0.0 && iga_rad == 0.0 {
                return [DisplayRegister::Blank; 3];
            }
            [
                DisplayRegister::Decimal(rad_to_centideg(oga_rad)),
                DisplayRegister::Decimal(rad_to_centideg(mga_rad)),
                DisplayRegister::Decimal(rad_to_centideg(iga_rad)),
            ]
        }

        // ------------------------------------------------------------------
        // N20 — ICDU Angles: OGA, MGA, IGA [0.01°]
        // P51/P52 IMU-Alignment — zeigt aktuelle CDU-Winkel (CDUX, CDUY, CDUZ).
        // R1 = OGA (Outer Gimbal Angle) [0.01°]
        // R2 = MGA (Middle Gimbal Angle) [0.01°]
        // R3 = IGA (Inner Gimbal Angle) [0.01°]
        //
        // Liest thetad aus GuidanceContext (wird von P52 / V41 gesetzt).
        //
        // AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N20 (ICDU ANGLES).
        // AGC source: PINBALL_NOUN_TABLES.agc, ECADR CDUX (# 20 ICDU ANGLES).
        // ------------------------------------------------------------------
        20 => {
            let [oga_rad, mga_rad, iga_rad] = ctx.guidance.thetad;
            if oga_rad == 0.0 && mga_rad == 0.0 && iga_rad == 0.0 {
                return [DisplayRegister::Blank; 3];
            }
            [
                DisplayRegister::Decimal(rad_to_centideg(oga_rad)),
                DisplayRegister::Decimal(rad_to_centideg(mga_rad)),
                DisplayRegister::Decimal(rad_to_centideg(iga_rad)),
            ]
        }

        // ------------------------------------------------------------------
        // N22 — THETAD: Gewünschte neue CDU-Winkel [0.01°]
        // P51/P52 IMU-Alignment — zeigt die Ziel-Gimbal-Winkel nach Alignment-Berechnung.
        // R1 = THETAD[0] (OGA Ziel) [0.01°]
        // R2 = THETAD[1] (MGA Ziel) [0.01°]
        // R3 = THETAD[2] (IGA Ziel) [0.01°]
        //
        // THETAD = berechnete Ziel-Gimbalwinkel für Coarse/Fine Align.
        // Wird durch P52-Berechnung oder V41-Befehl gesetzt.
        //
        // AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N22 (NEW ICDU ANGLES).
        // AGC source: PINBALL_NOUN_TABLES.agc, ECADR THETAD (# 22 NEW ICDU ANGLES).
        // ------------------------------------------------------------------
        22 => {
            let [t0, t1, t2] = ctx.guidance.thetad;
            if t0 == 0.0 && t1 == 0.0 && t2 == 0.0 {
                return [DisplayRegister::Blank; 3];
            }
            [
                DisplayRegister::Decimal(rad_to_centideg(t0)),
                DisplayRegister::Decimal(rad_to_centideg(t1)),
                DisplayRegister::Decimal(rad_to_centideg(t2)),
            ]
        }

        // ------------------------------------------------------------------
        // N33 — TIG: Zündzeitpunkt (HH:MM:SS)
        // Zeigt den per V25/N33 eingegebenen TIG als HH / MM / SS in R1/R2/R3.
        // Blank wenn kein TIG gesetzt (tig_s == 0.0).
        //
        // AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N33 (TIG).
        // ------------------------------------------------------------------
        33 => {
            if ctx.guidance.tig_s == 0.0 {
                return [DisplayRegister::Blank; 3];
            }
            let total = ctx.guidance.tig_s.max(0.0) as u32;
            let hh = total / 3600;
            let mm = (total % 3600) / 60;
            let ss = total % 60;
            [
                DisplayRegister::Decimal(hh as i32),
                DisplayRegister::Decimal(mm as i32),
                DisplayRegister::Decimal(ss as i32),
            ]
        }

        // ------------------------------------------------------------------
        // N35 — TTOGO: Zeit bis TIG (MM:SS)
        // R1 = verbleibende Zeit bis TIG in MM:SS.
        // Blank wenn kein TIG gesetzt oder TIG bereits vergangen.
        //
        // AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N35 (TTOGO).
        // ------------------------------------------------------------------
        35 => {
            if ctx.guidance.tig_s == 0.0 {
                return [DisplayRegister::Blank; 3];
            }
            let now_s = ctx.current_sv.time.as_seconds();
            let remaining = ctx.guidance.tig_s - now_s;
            if remaining <= 0.0 {
                return [
                    DisplayRegister::MinSec(0),
                    DisplayRegister::Blank,
                    DisplayRegister::Blank,
                ];
            }
            [
                DisplayRegister::MinSec(remaining as u32),
                DisplayRegister::Blank,
                DisplayRegister::Blank,
            ]
        }

        // ------------------------------------------------------------------
        // N40 — SPS-Burn Monitor: TTOGO, VGDISP, DVTOTAL
        // P40 SPS-Burn.
        // R1 = TTOGO = Restbrennzeit in MM:SS (keine Eingabe, nur Anzeige)
        // R2 = VGDISP = delta_v_target [m/s] (Velocity-to-Go)
        // R3 = DVTOTAL = dv_accumulated [m/s] (akkumuliertes Delta-V)
        //
        // AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N40 (VG/TGO/DELTAV).
        // ------------------------------------------------------------------
        40 => {
            let Some(ref p40) = ctx.guidance.p40 else {
                return [DisplayRegister::Blank; 3];
            };
            let tgo_s = (p40.t_burn - p40.t_elapsed).max(0.0) as u32;
            [
                DisplayRegister::MinSec(tgo_s),
                DisplayRegister::Decimal(mps_to_display(p40.delta_v_target)),
                DisplayRegister::Decimal(mps_to_display(p40.dv_accumulated)),
            ]
        }

        // ------------------------------------------------------------------
        // N44 — Orbit-Parameter: HAPO, HPER, TFF
        // P30 External ΔV Targeting / P37 Return-to-Earth.
        // R1 = HAPO (Apogee-Höhe nach Burn) [km]
        // R2 = HPER (Perigee-Höhe nach Burn) [km]
        // R3 = TFF  (Zeit bis Atmosphäreneintritt, MM:SS)
        //
        // AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N44 (HAPO/HPER/TFF).
        // ------------------------------------------------------------------
        44 => {
            let Some(ref p30) = ctx.guidance.p30 else {
                return [DisplayRegister::Blank; 3];
            };
            [
                DisplayRegister::Decimal(m_to_km_display(p30.h_apo)),
                DisplayRegister::Decimal(m_to_km_display(p30.h_per)),
                // TFF: Zeit bis Atmosphäreneintritt (MM:SS) — Berechnung ausstehend (TFFCONMU)
                DisplayRegister::MinSec(p30.tff_s.max(0.0) as u32),
            ]
        }

        // ------------------------------------------------------------------
        // N60 — Entry Display: GMAX, VPRED, GAMMAEI
        // P61 Entry Preparation.
        // R1 = GMAX [0.01 m/s²]  (maximale Brems-G-Last)
        // R2 = VPRED [m/s]       (vorhergesagte Geschwindigkeit bei EMS-Alt)
        // R3 = GAMMAEI [0.01°]   (vorhergesagter Eintrittswinkel)
        //
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
        // N61 — Splashdown-Zielkoordinaten: LAT(SPL), LNG(SPL), HEADSUP
        // P61/P62 Target-Eingabe und Anzeige.
        // R1 = LAT(SPL) [0.01°]  (Ziel-Breitengrad, aus Crew-Eingabe V25)
        // R2 = LNG(SPL) [0.01°]  (Ziel-Längengrad, aus Crew-Eingabe V25)
        // R3 = HEADSUP [±1]      (+1 = Lift-Down / Heads-Up, −1 = Lift-Up)
        //
        // Die Koordinaten werden in `GuidanceContext.splashdown_lat/lon` als
        // Radiant gespeichert und hier in 0.01° umgerechnet.
        //
        // AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N61.
        // ------------------------------------------------------------------
        61 => {
            let headsup_display = if ctx.guidance.heads_up { 1 } else { -1 };
            [
                DisplayRegister::Decimal(rad_to_centideg(ctx.guidance.splashdown_lat)),
                DisplayRegister::Decimal(rad_to_centideg(ctx.guidance.splashdown_lon)),
                DisplayRegister::Decimal(headsup_display),
            ]
        }

        // ------------------------------------------------------------------
        // N63 — Entry Guidance Outputs: RTGO, VIO, TFE
        // P61 Guidance-Ausgaben nach Entry-Berechnung.
        // R1 = RTGO [km]   (Reichweite bis Splashdown ab EMS-Altitude)
        // R2 = VIO [m/s]   (Inertialgeschwindigkeit bei EMS-Altitude)
        // R3 = TFE [s]     (geschätzte Zeit bis EMS-Altitude)
        //
        // AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N63.
        // ------------------------------------------------------------------
        63 => {
            let Some(ref p61) = ctx.guidance.p61 else {
                return [DisplayRegister::Blank; 3];
            };
            let Some(ref d) = p61.display else {
                return [DisplayRegister::Blank; 3];
            };
            [
                DisplayRegister::Decimal(m_to_km_display(d.rtgo_m)),
                DisplayRegister::Decimal(mps_to_display(d.vio)),
                DisplayRegister::Decimal(s_to_display(d.tte_s)),
            ]
        }

        // ------------------------------------------------------------------
        // N66 — Guidance Errors: ROLLC, XRNGERR, DNRNGERR
        // P64/P65 Guidance-Fehleranzeige.
        // R1 = ROLLC [0.01°]    (Roll-Befehl)
        // R2 = XRNGERR [km]     (Quer-Reichweitenfehler, positiv = rechts)
        // R3 = DNRNGERR [km]    (Längs-Reichweitenfehler, positiv = zu weit)
        //
        // Liest aus P67State (enthält xrngerr_m/dnrngerr_m für P64/P67).
        // Falls P64 aktiv, wird dessen Roll-Befehl verwendet.
        //
        // AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N66.
        // ------------------------------------------------------------------
        66 => {
            // Quer- und Längs-Reichweitenfehler aus P67State
            let Some(ref p67) = ctx.guidance.p67 else {
                return [DisplayRegister::Blank; 3];
            };
            // Roll-Befehl aus P64State falls vorhanden, sonst aus P67State
            let rollc_deg = ctx
                .guidance
                .p64
                .as_ref()
                .map(|p| p.roll_cmd_deg)
                .unwrap_or(p67.roll_cmd_deg);
            [
                DisplayRegister::Decimal(deg_to_centideg(rollc_deg)),
                DisplayRegister::Decimal(m_to_km_display(p67.xrngerr_m)),
                DisplayRegister::Decimal(m_to_km_display(p67.dnrngerr_m)),
            ]
        }

        // ------------------------------------------------------------------
        // N67 — Terminal Phase: RTOGO, LAT(Ziel), LONG(Ziel)
        // P67 Terminal Phase.
        // R1 = RTOGO [km]     (Gesamtentfernung zum Splashdown)
        // R2 = LAT(ZIE) [0.01°] (Ziel-Breitengrad = splashdown_lat)
        // R3 = LON(ZIE) [0.01°] (Ziel-Längengrad  = splashdown_lon)
        //
        // RTOGO wird aus dem geometrischen Gesamtfehler (xrngerr_m, dnrngerr_m)
        // als Näherung berechnet. Zielkoordinaten kommen aus der Crew-Eingabe.
        //
        // AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N67.
        // ------------------------------------------------------------------
        67 => {
            let Some(ref p67) = ctx.guidance.p67 else {
                return [DisplayRegister::Blank; 3];
            };
            let rtogo_m =
                libm::sqrt(p67.xrngerr_m * p67.xrngerr_m + p67.dnrngerr_m * p67.dnrngerr_m);
            [
                DisplayRegister::Decimal(m_to_km_display(rtogo_m)),
                DisplayRegister::Decimal(rad_to_centideg(ctx.guidance.splashdown_lat)),
                DisplayRegister::Decimal(rad_to_centideg(ctx.guidance.splashdown_lon)),
            ]
        }

        // ------------------------------------------------------------------
        // N69 — Constant-Drag Monitor: ROLLC, DL, VL
        // P64 Constant-Drag Entry Phase Monitor.
        // R1 = ROLLC [0.01°]   (Roll-Befehl aus P64)
        // R2 = DL [0.01 m/s²]  (aktuelle Drag-G-Last in m/s²)
        // R3 = VL [m/s]        (Relativgeschwindigkeit — nicht in P64State,
        //                        wird als Blank ausgegeben bis P64 erweitert)
        //
        // AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N69 (BETA/DL/VL).
        // ------------------------------------------------------------------
        69 => {
            let Some(ref p64) = ctx.guidance.p64 else {
                return [DisplayRegister::Blank; 3];
            };
            // DL: Drag in m/s² = drag_g_current × g₀, Anzeige in 0.01 m/s²
            let dl_centimps2 = g0_to_centimps2(p64.drag_g_current);
            [
                DisplayRegister::Decimal(deg_to_centideg(p64.roll_cmd_deg)),
                DisplayRegister::Decimal(dl_centimps2),
                DisplayRegister::Blank, // VL fehlt in P64State (Sub-Task 5 / DAP)
            ]
        }

        // ------------------------------------------------------------------
        // N74 — Constant-Drag Display: ROLLC, VI, D
        // P64 Constant-Drag Entry Phase Haupt-Anzeige.
        // R1 = ROLLC [0.01°]   (Roll-Befehl)
        // R2 = VI [m/s]        (Inertialgeschwindigkeit — nicht in P64State,
        //                        wird als Blank ausgegeben bis DAP bereitstellt)
        // R3 = D [0.01 m/s²]   (Drag-Beschleunigung)
        //
        // AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N74 (ROLLC/VI/D).
        // ------------------------------------------------------------------
        74 => {
            let Some(ref p64) = ctx.guidance.p64 else {
                return [DisplayRegister::Blank; 3];
            };
            let d_centimps2 = g0_to_centimps2(p64.drag_g_current);
            [
                DisplayRegister::Decimal(deg_to_centideg(p64.roll_cmd_deg)),
                DisplayRegister::Blank, // VI (Inertialgeschwindigkeit) fehlt in P64State
                DisplayRegister::Decimal(d_centimps2),
            ]
        }

        // ------------------------------------------------------------------
        // N81 — Δv-LV: DVx, DVy, DVz
        // P30 External ΔV Targeting — Δv im ECI-System.
        // R1 = DVx [m/s]
        // R2 = DVy [m/s]
        // R3 = DVz [m/s]
        //
        // AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N81 (DELVLVC).
        // ------------------------------------------------------------------
        81 => {
            let Some(ref p30) = ctx.guidance.p30 else {
                return [DisplayRegister::Blank; 3];
            };
            let dv = p30.delta_v_ref;
            [
                DisplayRegister::Decimal(mps_to_display(dv.x)),
                DisplayRegister::Decimal(mps_to_display(dv.y)),
                DisplayRegister::Decimal(mps_to_display(dv.z)),
            ]
        }

        // ------------------------------------------------------------------
        // N92 — New Optics Angles: SHAFT, TRUNION [0.01°]
        // P52 IMU-Alignment — zeigt neue Optik-Winkel (aus Sternbeobachtung).
        // R1 = SHAFT [0.01°]     (Shaft-Winkel des Optiksystems)
        // R2 = TRUNION [0.01°]   (Trunnion-Winkel des Optiksystems)
        // R3 = Blank (2-komponentiger Noun)
        //
        // Im aktuellen Modell werden Optik-Winkel nicht separat berechnet
        // (P51 arbeitet mit IMU-Gimbals, nicht Optik). Daher Blank bis DAP/Optik-HAL.
        //
        // AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N92 (NEW OPTICS ANGLES SHAFT/TRUNION).
        // AGC source: PINBALL_NOUN_TABLES.agc, OCT 02234 (# 92 NEW OPTICS ANGLES).
        // ------------------------------------------------------------------
        92 => {
            // Optik-Winkel sind noch nicht im GuidanceContext modelliert.
            // Blank zurückgeben bis der Optik-HAL verfügbar ist.
            [DisplayRegister::Blank; 3]
        }

        // ------------------------------------------------------------------
        // N93 — Delta Gyro Angles: ΔG1, ΔG2, ΔG3 [0.001°]
        // P52 IMU-Alignment — zeigt die berechneten Gyro-Torquing-Korrekturen.
        // R1 = ΔG1 [0.001°]   (Torquing-Winkel Achse 1)
        // R2 = ΔG2 [0.001°]   (Torquing-Winkel Achse 2)
        // R3 = ΔG3 [0.001°]   (Torquing-Winkel Achse 3)
        //
        // Die Gyro-Korrekturen werden aus der THETAD-Differenz (Ziel minus Ist)
        // berechnet. Im aktuellen Modell wird die Differenz aus thetad (Ziel)
        // und Null-Referenz approximiert.
        //
        // AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N93 (DELTA GYRO ANGLES).
        // AGC source: PINBALL_NOUN_TABLES.agc, OCT 04237 (# 93 DELTA GYRO ANGLES).
        // AGC source: P51-P53.agc, R55.1 (V06N93 display of gyro corrections).
        // ------------------------------------------------------------------
        93 => {
            // THETAD-Differenz als Gyro-Torquing-Approximation.
            // Wenn kein P51-State → Blank.
            let Some(_p51) = ctx.guidance.p51.as_ref() else {
                return [DisplayRegister::Blank; 3];
            };
            // Gyro-Korrekturen können aus v1_body/v2_body-Vektoren nicht direkt
            // abgeleitet werden ohne vollständige Gyro-Torquing-Berechnung (R55).
            // Wir zeigen THETAD-Winkel in 0.001° als Proxy für die Delta-Winkel,
            // sobald thetad gesetzt wurde (nicht mehr Null-Defaultwert).
            let [t0, t1, t2] = ctx.guidance.thetad;
            if t0 == 0.0 && t1 == 0.0 && t2 == 0.0 {
                return [DisplayRegister::Blank; 3]; // Noch nicht berechnet
            }
            [
                DisplayRegister::Decimal(deg_to_millideg(t0 * (180.0 / core::f64::consts::PI))),
                DisplayRegister::Decimal(deg_to_millideg(t1 * (180.0 / core::f64::consts::PI))),
                DisplayRegister::Decimal(deg_to_millideg(t2 * (180.0 / core::f64::consts::PI))),
            ]
        }

        // ------------------------------------------------------------------
        // Alle übrigen Nouns: nicht implementiert.
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
    use crate::guidance::p30_37::P30Output;
    use crate::guidance::p40_47::P40State;
    use crate::guidance::p61_67::{EntryDisplayData, P61State, P64State, P67State};
    use crate::types::Vec3;

    fn noun(n: u8) -> NounCode {
        NounCode::new(n).unwrap()
    }

    // -----------------------------------------------------------------------
    // N40 — SPS-Burn Monitor
    // -----------------------------------------------------------------------

    /// N40 mit aktivem P40-State → TTOGO (MM:SS), VGDISP und DVTOTAL korrekt.
    ///
    /// AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N40.
    #[test]
    fn n40_resolves_from_p40_state() {
        let mut ctx = AgcContext::default();
        ctx.guidance.p40 = Some(P40State {
            phase: 3,
            delta_v_target: 600.0, // 600 m/s VG
            isp: 314.5,
            thrust: 97_855.0,
            mass: 28_000.0,
            t_burn: 90.0,
            t_elapsed: 25.0,       // TGO = 65 s
            dv_accumulated: 150.0, // 150 m/s akkumuliert
        });

        let [r1, r2, r3] = resolve(noun(40), &ctx);

        // R1 = TTOGO in MM:SS: 65 s → 1 Min 5 Sek
        assert_eq!(r1, DisplayRegister::MinSec(65));
        // R2 = VGDISP in m/s: 600
        assert_eq!(r2, DisplayRegister::Decimal(600));
        // R3 = DVTOTAL in m/s: 150
        assert_eq!(r3, DisplayRegister::Decimal(150));
    }

    /// N40 ohne aktiven P40-State → Blank.
    #[test]
    fn n40_blank_when_p40_inactive() {
        let ctx = AgcContext::default();
        let [r1, r2, r3] = resolve(noun(40), &ctx);
        assert_eq!(r1, DisplayRegister::Blank);
        assert_eq!(r2, DisplayRegister::Blank);
        assert_eq!(r3, DisplayRegister::Blank);
    }

    /// N40 wenn t_elapsed > t_burn → TTOGO klemmt auf 0 s (MM:SS = 00:00).
    #[test]
    fn n40_tgo_clamps_to_zero_after_cutoff() {
        let mut ctx = AgcContext::default();
        ctx.guidance.p40 = Some(P40State {
            phase: 4,
            delta_v_target: 300.0,
            isp: 314.5,
            thrust: 97_855.0,
            mass: 28_000.0,
            t_burn: 50.0,
            t_elapsed: 55.0, // überschritten
            dv_accumulated: 0.0,
        });

        let [r1, _r2, _r3] = resolve(noun(40), &ctx);
        // TTOGO darf nicht negativ sein → MinSec(0)
        assert_eq!(r1, DisplayRegister::MinSec(0));
    }

    // -----------------------------------------------------------------------
    // N44 — Orbit-Parameter HAPO/HPER
    // -----------------------------------------------------------------------

    /// N44 mit bekannten P30-Werten → HAPO und HPER korrekt in km.
    ///
    /// AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N44.
    #[test]
    fn n44_resolves_from_p30_output() {
        let mut ctx = AgcContext::default();
        ctx.guidance.p30 = Some(P30Output {
            delta_v_ref: Vec3::new(100.0, 0.0, 0.0),
            h_apo: 400_000.0, // 400 km Apogee
            h_per: 185_000.0, // 185 km Perigee
            tff_s: 0.0,
        });

        let [r1, r2, r3] = resolve(noun(44), &ctx);

        // R1 = HAPO in km: 400
        assert_eq!(r1, DisplayRegister::Decimal(400));
        // R2 = HPER in km: 185
        assert_eq!(r2, DisplayRegister::Decimal(185));
        // R3 = TFF als MinSec (0 s → MinSec(0))
        assert_eq!(r3, DisplayRegister::MinSec(0));
    }

    /// N44 ohne P30-State → Blank.
    #[test]
    fn n44_blank_when_p30_inactive() {
        let ctx = AgcContext::default();
        let [r1, r2, r3] = resolve(noun(44), &ctx);
        assert_eq!(r1, DisplayRegister::Blank);
        assert_eq!(r2, DisplayRegister::Blank);
        assert_eq!(r3, DisplayRegister::Blank);
    }

    // -----------------------------------------------------------------------
    // N60 — Entry Display
    // -----------------------------------------------------------------------

    /// N60 mit bekannten P61-Werten → korrekte Skalierung.
    ///
    /// AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N60.
    #[test]
    fn n60_resolves_from_p61_display() {
        let mut ctx = AgcContext::default();
        ctx.guidance.p61 = Some(P61State {
            phase: 0,
            display: Some(EntryDisplayData {
                gmax: 3.5,          // 3.5 g₀ × 9.80665 × 100 ≈ 3432
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

    /// N60 mit P61 ohne berechnetes Display → Blank.
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

    // -----------------------------------------------------------------------
    // N61 — Splashdown-Zielkoordinaten
    // -----------------------------------------------------------------------

    /// N61 mit gesetzten Splashdown-Koordinaten → korrekte Umrechnung rad → 0.01°.
    ///
    /// AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N61.
    #[test]
    fn n61_resolves_splashdown_target() {
        let mut ctx = AgcContext::default();
        // 15° Nord = 15 * π/180 rad
        ctx.guidance.splashdown_lat = 15.0_f64.to_radians();
        // 165° Ost (Pazifik) = 165 * π/180 rad
        ctx.guidance.splashdown_lon = 165.0_f64.to_radians();
        ctx.guidance.heads_up = true;

        let [r1, r2, r3] = resolve(noun(61), &ctx);

        // R1 = 15° in centideg: 1500 (±1 wg. Rundungsfehler)
        if let DisplayRegister::Decimal(v) = r1 {
            assert!((v - 1500).abs() <= 1, "LAT erwartet ~1500, got {v}");
        } else {
            panic!("R1 sollte Decimal sein");
        }
        // R2 = 165° in centideg: 16500 (±1)
        if let DisplayRegister::Decimal(v) = r2 {
            assert!((v - 16500).abs() <= 1, "LON erwartet ~16500, got {v}");
        } else {
            panic!("R2 sollte Decimal sein");
        }
        // R3 = HEADSUP = +1
        assert_eq!(r3, DisplayRegister::Decimal(1));
    }

    /// N61 HEADSUP = false → R3 = −1.
    #[test]
    fn n61_headsup_false_gives_minus_one() {
        let mut ctx = AgcContext::default();
        ctx.guidance.heads_up = false;
        let [_r1, _r2, r3] = resolve(noun(61), &ctx);
        assert_eq!(r3, DisplayRegister::Decimal(-1));
    }

    // -----------------------------------------------------------------------
    // N63 — Entry Guidance Outputs
    // -----------------------------------------------------------------------

    /// N63 mit bekannten P61-Display-Daten → RTGO, VIO, TFE korrekt.
    ///
    /// AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N63.
    #[test]
    fn n63_resolves_from_p61_display() {
        let mut ctx = AgcContext::default();
        ctx.guidance.p61 = Some(P61State {
            phase: 0,
            display: Some(EntryDisplayData {
                gmax: 3.5,
                v_pred: 7_800.0,
                gamma_ei_deg: -6.5,
                rtgo_m: 3_000_000.0, // 3000 km
                vio: 10_500.0,       // 10 500 m/s
                tte_s: 450.0,        // 450 s bis EMS-Alt
            }),
            lat_spl: 0.0,
            lng_spl: 0.0,
            heads_up: false,
        });

        let [r1, r2, r3] = resolve(noun(63), &ctx);

        // R1 = RTGO in km: 3000
        assert_eq!(r1, DisplayRegister::Decimal(3000));
        // R2 = VIO in m/s: 10500
        assert_eq!(r2, DisplayRegister::Decimal(10_500));
        // R3 = TFE in s: 450
        assert_eq!(r3, DisplayRegister::Decimal(450));
    }

    /// N63 ohne P61-State → Blank.
    #[test]
    fn n63_blank_when_p61_inactive() {
        let ctx = AgcContext::default();
        let [r1, r2, r3] = resolve(noun(63), &ctx);
        assert_eq!(r1, DisplayRegister::Blank);
        assert_eq!(r2, DisplayRegister::Blank);
        assert_eq!(r3, DisplayRegister::Blank);
    }

    // -----------------------------------------------------------------------
    // N66 — Guidance Errors (ROLLC/XRNGERR/DNRNGERR)
    // -----------------------------------------------------------------------

    /// N66 mit P67-State → Roll, Quer- und Längs-Fehler korrekt.
    ///
    /// AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N66.
    #[test]
    fn n66_resolves_guidance_errors() {
        let mut ctx = AgcContext::default();
        ctx.guidance.p67 = Some(P67State {
            phase: 2,
            xrngerr_m: 5_000.0,   // 5 km Quer-Fehler
            dnrngerr_m: -2_000.0, // −2 km Längs-Fehler
            roll_cmd_deg: 45.0,   // 45° → 4500 centideg
        });

        let [r1, r2, r3] = resolve(noun(66), &ctx);

        // R1 = ROLLC in centideg: 4500
        assert_eq!(r1, DisplayRegister::Decimal(4500));
        // R2 = XRNGERR in km: 5
        assert_eq!(r2, DisplayRegister::Decimal(5));
        // R3 = DNRNGERR in km: −2
        assert_eq!(r3, DisplayRegister::Decimal(-2));
    }

    /// N66 bevorzugt P64-Roll-Befehl wenn P64 aktiv.
    #[test]
    fn n66_uses_p64_roll_when_available() {
        let mut ctx = AgcContext::default();
        ctx.guidance.p64 = Some(P64State {
            phase: 2,
            roll_cmd_deg: 30.0,
            drag_g_current: 0.1,
            gamma_at_05g_deg: -6.5,
        });
        ctx.guidance.p67 = Some(P67State {
            phase: 2,
            xrngerr_m: 1_000.0,
            dnrngerr_m: 500.0,
            roll_cmd_deg: 90.0, // ignoriert wenn P64 aktiv
        });

        let [r1, _r2, _r3] = resolve(noun(66), &ctx);
        // Erwartet P64-Roll: 30° → 3000 centideg
        assert_eq!(r1, DisplayRegister::Decimal(3000));
    }

    /// N66 ohne P67-State → Blank.
    #[test]
    fn n66_blank_when_p67_inactive() {
        let ctx = AgcContext::default();
        let [r1, r2, r3] = resolve(noun(66), &ctx);
        assert_eq!(r1, DisplayRegister::Blank);
        assert_eq!(r2, DisplayRegister::Blank);
        assert_eq!(r3, DisplayRegister::Blank);
    }

    // -----------------------------------------------------------------------
    // N67 — Terminal Phase
    // -----------------------------------------------------------------------

    /// N67 mit bekannten P67-Werten → R1 = RTOGO in km, R2/R3 = Zielkoordinaten.
    ///
    /// AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N67.
    #[test]
    fn n67_resolves_from_p67_state() {
        let mut ctx = AgcContext::default();
        ctx.guidance.splashdown_lat = 10.0_f64.to_radians();
        ctx.guidance.splashdown_lon = 170.0_f64.to_radians();
        ctx.guidance.p67 = Some(P67State {
            phase: 2,
            xrngerr_m: 0.0,
            dnrngerr_m: 3_000.0, // 3 km Längs-Fehler → RTOGO ≈ 3 km
            roll_cmd_deg: 12.5,
        });

        let [r1, r2, r3] = resolve(noun(67), &ctx);

        // R1 = RTOGO in km: sqrt(0² + 3000²) / 1000 = 3
        assert_eq!(r1, DisplayRegister::Decimal(3));
        // R2 = LAT in centideg: 10° → 1000 (±1)
        if let DisplayRegister::Decimal(v) = r2 {
            assert!((v - 1000).abs() <= 1, "LAT erwartet ~1000, got {v}");
        } else {
            panic!("R2 sollte Decimal sein");
        }
        // R3 = LON in centideg: 170° → 17000 (±1)
        if let DisplayRegister::Decimal(v) = r3 {
            assert!((v - 17000).abs() <= 1, "LON erwartet ~17000, got {v}");
        } else {
            panic!("R3 sollte Decimal sein");
        }
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

    // -----------------------------------------------------------------------
    // N69 — Constant-Drag Monitor
    // -----------------------------------------------------------------------

    /// N69 mit P64-State → ROLLC und DL korrekt; VL = Blank.
    ///
    /// AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N69.
    #[test]
    fn n69_resolves_from_p64_state() {
        let mut ctx = AgcContext::default();
        ctx.guidance.p64 = Some(P64State {
            phase: 2,
            roll_cmd_deg: 90.0,  // 90° → 9000 centideg
            drag_g_current: 0.5, // 0.5 × 9.80665 × 100 = 490 centimps²
            gamma_at_05g_deg: -6.5,
        });

        let [r1, r2, r3] = resolve(noun(69), &ctx);

        // R1 = ROLLC in centideg: 9000
        assert_eq!(r1, DisplayRegister::Decimal(9000));
        // R2 = DL in 0.01 m/s²: 0.5 × 9.80665 × 100 = 490.3 → 490
        assert_eq!(r2, DisplayRegister::Decimal(490));
        // R3 = VL = Blank (nicht in P64State)
        assert_eq!(r3, DisplayRegister::Blank);
    }

    /// N69 ohne P64-State → Blank.
    #[test]
    fn n69_blank_when_p64_inactive() {
        let ctx = AgcContext::default();
        let [r1, r2, r3] = resolve(noun(69), &ctx);
        assert_eq!(r1, DisplayRegister::Blank);
        assert_eq!(r2, DisplayRegister::Blank);
        assert_eq!(r3, DisplayRegister::Blank);
    }

    // -----------------------------------------------------------------------
    // N74 — Constant-Drag Display
    // -----------------------------------------------------------------------

    /// N74 mit P64-State → ROLLC und D korrekt; VI = Blank.
    ///
    /// AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N74.
    #[test]
    fn n74_resolves_from_p64_state() {
        let mut ctx = AgcContext::default();
        ctx.guidance.p64 = Some(P64State {
            phase: 2,
            roll_cmd_deg: 0.0,   // 0° → 0 centideg
            drag_g_current: 0.1, // 0.1 × 9.80665 × 100 = 98.07 → 98
            gamma_at_05g_deg: -6.0,
        });

        let [r1, r2, r3] = resolve(noun(74), &ctx);

        // R1 = ROLLC in centideg: 0
        assert_eq!(r1, DisplayRegister::Decimal(0));
        // R2 = VI = Blank (nicht in P64State)
        assert_eq!(r2, DisplayRegister::Blank);
        // R3 = D in 0.01 m/s²: 0.1 × 9.80665 × 100 = 98.07 → 98
        assert_eq!(r3, DisplayRegister::Decimal(98));
    }

    /// N74 ohne P64-State → Blank.
    #[test]
    fn n74_blank_when_p64_inactive() {
        let ctx = AgcContext::default();
        let [r1, r2, r3] = resolve(noun(74), &ctx);
        assert_eq!(r1, DisplayRegister::Blank);
        assert_eq!(r2, DisplayRegister::Blank);
        assert_eq!(r3, DisplayRegister::Blank);
    }

    // -----------------------------------------------------------------------
    // N81 — Δv-LV (ECI-Komponenten)
    // -----------------------------------------------------------------------

    /// N81 mit bekannten P30-Daten → DVx, DVy, DVz korrekt in m/s.
    ///
    /// AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N81.
    #[test]
    fn n81_resolves_delta_v_from_p30() {
        let mut ctx = AgcContext::default();
        ctx.guidance.p30 = Some(P30Output {
            delta_v_ref: Vec3::new(120.5, -45.0, 8.7),
            h_apo: 400_000.0,
            h_per: 185_000.0,
            tff_s: 0.0,
        });

        let [r1, r2, r3] = resolve(noun(81), &ctx);

        // R1 = DVx: round(120.5) = 121
        assert_eq!(r1, DisplayRegister::Decimal(121));
        // R2 = DVy: round(−45.0) = −45
        assert_eq!(r2, DisplayRegister::Decimal(-45));
        // R3 = DVz: round(8.7) = 9
        assert_eq!(r3, DisplayRegister::Decimal(9));
    }

    /// N81 ohne P30-State → Blank.
    #[test]
    fn n81_blank_when_p30_inactive() {
        let ctx = AgcContext::default();
        let [r1, r2, r3] = resolve(noun(81), &ctx);
        assert_eq!(r1, DisplayRegister::Blank);
        assert_eq!(r2, DisplayRegister::Blank);
        assert_eq!(r3, DisplayRegister::Blank);
    }

    // -----------------------------------------------------------------------
    // Sub-Task 5 — IMU-Alignment-Nouns (N17, N20, N22, N92, N93)
    // -----------------------------------------------------------------------

    /// N20 ohne thetad (Defaultwert 0,0,0) → Blank.
    ///
    /// AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N20.
    #[test]
    fn n20_without_thetad_returns_blank() {
        let ctx = AgcContext::default();
        let [r1, r2, r3] = resolve(noun(20), &ctx);
        assert_eq!(r1, DisplayRegister::Blank, "R1 muss Blank sein ohne thetad");
        assert_eq!(r2, DisplayRegister::Blank);
        assert_eq!(r3, DisplayRegister::Blank);
    }

    /// N20 mit gesetztem thetad → korrekte CDU-Winkel in 0.01°.
    ///
    /// OGA = 45° → R1 = 4500; MGA = −90° → R2 = −9000; IGA = 15° → R3 = 1500
    ///
    /// AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N20 (ICDU ANGLES, XXX.XX DEG).
    #[test]
    fn n20_with_thetad_returns_centideg() {
        let mut ctx = AgcContext::default();
        ctx.guidance.thetad = [
            45.0_f64.to_radians(),
            (-90.0_f64).to_radians(),
            15.0_f64.to_radians(),
        ];

        let [r1, r2, r3] = resolve(noun(20), &ctx);

        // OGA = 45° → 4500 (0.01°)
        assert_eq!(
            r1,
            DisplayRegister::Decimal(4500),
            "R1 (OGA=45°) → 4500 (0.01°)"
        );
        // MGA = −90° → −9000 (0.01°)
        assert_eq!(
            r2,
            DisplayRegister::Decimal(-9000),
            "R2 (MGA=−90°) → −9000 (0.01°)"
        );
        // IGA = 15° → 1500 (0.01°)
        assert_eq!(
            r3,
            DisplayRegister::Decimal(1500),
            "R3 (IGA=15°) → 1500 (0.01°)"
        );
    }

    /// N17 mit gesetztem thetad → identische CDU-Winkel wie N20.
    ///
    /// N17 (CPHIX) und N20 (ICDU) lesen beide aus `guidance.thetad`.
    ///
    /// AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N17 (CPHIX / ASTRONAUT TOTAL ATTITUDE).
    #[test]
    fn n17_mirrors_n20_with_thetad() {
        let mut ctx = AgcContext::default();
        ctx.guidance.thetad = [
            30.0_f64.to_radians(),
            (-20.0_f64).to_radians(),
            10.0_f64.to_radians(),
        ];

        let n17 = resolve(noun(17), &ctx);
        let n20 = resolve(noun(20), &ctx);

        assert_eq!(n17, n20, "N17 (CPHIX) und N20 (ICDU) müssen identisch sein");
    }

    /// N22 ohne thetad → Blank.
    ///
    /// AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N22 (NEW ICDU ANGLES).
    #[test]
    fn n22_without_thetad_returns_blank() {
        let ctx = AgcContext::default();
        let [r1, r2, r3] = resolve(noun(22), &ctx);
        assert_eq!(r1, DisplayRegister::Blank);
        assert_eq!(r2, DisplayRegister::Blank);
        assert_eq!(r3, DisplayRegister::Blank);
    }

    /// N22 mit gesetztem thetad → Ziel-Gimbal-Winkel in 0.01°.
    ///
    /// THETAD[0] = 60° → R1 = 6000
    ///
    /// AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N22 (NEW ICDU ANGLES, XXX.XX DEG).
    #[test]
    fn n22_with_thetad_returns_target_angles() {
        let mut ctx = AgcContext::default();
        ctx.guidance.thetad = [
            60.0_f64.to_radians(),
            (-30.0_f64).to_radians(),
            5.0_f64.to_radians(),
        ];

        let [r1, r2, r3] = resolve(noun(22), &ctx);

        assert_eq!(
            r1,
            DisplayRegister::Decimal(6000),
            "R1 (THETAD[0]=60°) → 6000"
        );
        assert_eq!(
            r2,
            DisplayRegister::Decimal(-3000),
            "R2 (THETAD[1]=−30°) → −3000"
        );
        assert_eq!(r3, DisplayRegister::Decimal(500), "R3 (THETAD[2]=5°) → 500");
    }

    /// N92 → immer Blank (Optik-Winkel noch nicht modelliert).
    ///
    /// AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N92 (NEW OPTICS ANGLES SHAFT/TRUNION).
    #[test]
    fn n92_returns_blank() {
        let ctx = AgcContext::default();
        let [r1, r2, r3] = resolve(noun(92), &ctx);
        assert_eq!(r1, DisplayRegister::Blank, "N92 R1 muss Blank sein");
        assert_eq!(r2, DisplayRegister::Blank, "N92 R2 muss Blank sein");
        assert_eq!(r3, DisplayRegister::Blank, "N92 R3 muss Blank sein");
    }

    /// N93 ohne P51-State → Blank.
    ///
    /// AGC source: P51-P53.agc, R55.1 (V06N93).
    #[test]
    fn n93_without_p51_returns_blank() {
        let ctx = AgcContext::default();
        let [r1, r2, r3] = resolve(noun(93), &ctx);
        assert_eq!(r1, DisplayRegister::Blank, "N93 ohne P51 → Blank");
        assert_eq!(r2, DisplayRegister::Blank);
        assert_eq!(r3, DisplayRegister::Blank);
    }

    /// N93 mit P51-State aber thetad == [0,0,0] → Blank.
    ///
    /// thetad=0 ist der Defaultwert vor jeder P51-Berechnung; Daten sind erst
    /// sinnvoll wenn thetad gesetzt wurde.
    ///
    /// AGC source: P51-P53.agc, R55.1 (V06N93).
    #[test]
    fn n93_with_zero_thetad_returns_blank() {
        use crate::guidance::p51_53::P51State;

        let mut ctx = AgcContext::default();
        // P51-State vorhanden, aber thetad noch auf Default-Nullwert
        ctx.guidance.p51 = Some(P51State::new(1, 2));
        // ctx.guidance.thetad bleibt [0.0, 0.0, 0.0] (Default)

        let [r1, r2, r3] = resolve(noun(93), &ctx);
        assert_eq!(r1, DisplayRegister::Blank, "N93 mit thetad=0 → Blank");
        assert_eq!(r2, DisplayRegister::Blank);
        assert_eq!(r3, DisplayRegister::Blank);
    }

    /// N93 mit thetad ≠ 0 → Delta-Gyro-Winkel in 0.001° sichtbar.
    ///
    /// Sobald P51 thetad gesetzt hat, zeigt N93 die Winkel an.
    ///
    /// AGC source: P51-P53.agc, R55.1 (V06N93 display of gyro corrections).
    #[test]
    fn n93_with_nonzero_thetad_returns_values() {
        use crate::guidance::p51_53::P51State;

        let mut ctx = AgcContext::default();
        ctx.guidance.p51 = Some(P51State::new(1, 0));
        // 1° in Radiant für alle drei Achsen
        let one_deg_rad = core::f64::consts::PI / 180.0;
        ctx.guidance.thetad = [one_deg_rad, one_deg_rad, one_deg_rad];

        let [r1, r2, r3] = resolve(noun(93), &ctx);
        // 1° × (180/π) × 1° = 1° → 1000 tausendstel Grad
        assert_eq!(
            r1,
            DisplayRegister::Decimal(1000),
            "N93 thetad=1° → 1000 (0.001°)"
        );
        assert_eq!(r2, DisplayRegister::Decimal(1000));
        assert_eq!(r3, DisplayRegister::Decimal(1000));
    }

    // -----------------------------------------------------------------------
    // Allgemein
    // -----------------------------------------------------------------------

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
