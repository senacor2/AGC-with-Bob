//! Reentry-Programmverkettung — automatische Übergänge P61 → P62 → P63 → P64 → P67.
//!
//! Im echten AGC läuft die Verkettung über den **SERVICER/AVERAGE-G**-Zyklus
//! (alle 2 s), der via `AVGEXIT` in `TARGETNG` → `SCALEPOP` → `GOTOADDR`
//! weiterläuft.  Der `GOTOADDR`-Zeiger springt je nach Reentry-Phase nach
//! `INITROLL` (P63), `HUNTEST`/`CONSTD` (P64) oder `PREDICT3` (P67).
//! Der Programmwechsel auf der DSKY (MODREG, MM-Anzeige) erfolgt dabei
//! automatisch über `NEWMODEX`-Aufrufe innerhalb desselben SERVICER-Jobs —
//! nicht über V37-Eingaben.
//!
//! Diese Funktion [`reentry_tick`] repliziert dieses Verhalten für alle
//! Plattformen (Sim und Bare-Metal).  Sie wird einmal pro Simulations-Tick
//! aufgerufen.
//!
//! # Übergangsbedingungen
//!
//! | Von | Nach | Bedingung |
//! |-----|------|-----------|
//! | P61 | P62  | Crew drückt PRO auf N63-Flash (`proceed_flag`) |
//! | P62 | P63  | sofort (FALL INTO P63, kein Wartezeit) |
//! | P62/P63 | P64 | Drag D ≥ 0.05 g (`.05GSW` setzen) |
//! | P64 | P67 | `p64_step` gibt `Some(false)` zurück (D > Q7 oder V < VFINAL) |
//!
//! # AGC-Quelle
//! `Comanche055/P61-P67.agc`, P61–P67-Programme.
//! `Comanche055/REENTRY_CONTROL.agc`, SCALEPOP/INITROLL/PREDICT3/P67.1.
//! `Comanche055/SERVICER207.agc`, AVGEXIT/READACCS-Zyklus.

use crate::context::AgcContext;
use crate::display::verb_noun::MajorMode;
use crate::guidance::p61_67::{drag_g, p61_step, p64_step, p67_step, P64State, P67State};
use crate::navigation::orbital::CentralBody;

// ---------------------------------------------------------------------------
// Öffentliche API
// ---------------------------------------------------------------------------

/// Führt einen Reentry-Steuerungs-Tick durch.
///
/// Muss einmal pro Simulations-Tick aufgerufen werden, wenn ein Entry-Programm
/// aktiv ist (P61–P67).  Implementiert:
///
/// 1. **P61-Aktualisierung**: Solange P61 aktiv ist, werden die Entry-Anzeigedaten
///    (GMAX, VPRED, GAMMAEI) laufend neu berechnet.
///
/// 2. **P61 → P62**: Wenn die Crew PRO drückt (`proceed_flag`), wechselt MODREG
///    sofort auf 62.  Im AGC fällt P61 nach dem zweiten N63-PROCEED direkt in
///    P62 durch (`.... THEN FALL INTO P62`, Zeile 162 in P61-P67.agc).
///    AGC source: `Comanche055/P61-P67.agc`, P61.2-Label / FALL INTO P62.
///
/// 3. **P62 → P63**: P62 ist in dieser Simulation ein Momentanzustand (keine
///    IMU-Neuausrichtung, kein CM/DAP).  MODREG wechselt sofort auf 63.
///    Im AGC: `TC P63` am Ende von P62 wenn `CMDAPMOD` bereits -1 oder +0.
///    AGC source: `Comanche055/P61-P67.agc`, P62.1-Label, letzter `TC P63`.
///
/// 4. **P63 → P64** (.05G-Sensing): Wenn P62 oder P63 aktiv ist und die
///    atmosphärische Bremsverzögerung 0.05 g überschreitet, wird P64
///    automatisch gestartet und MODREG auf 64 gesetzt.
///    AGC source: `Comanche055/REENTRY_CONTROL.agc`, `.05GSW`-Check in TARGETNG.
///
/// 5. **P64-Tick**: Solange P64 aktiv ist, wird `p64_step` aufgerufen, der den
///    Roll-Befehl und die Drag-G-Last aktualisiert.
///
/// 6. **P64 → P67**: Gibt `p64_step` `Some(false)` zurück (Übergang zu
///    PREDICT3 / P67), wird P67 automatisch gestartet und MODREG auf 67 gesetzt.
///    AGC source: `Comanche055/P61-P67.agc`, P67 — "ENTER VIA RTB FROM REENTRY CONTROL".
///
/// 7. **P67-Tick**: Solange P67 aktiv ist, werden XRNGERR und DNRNGERR
///    laufend aktualisiert.
///
/// # Parameter
/// - `ctx`: zentraler AGC-Zustand (Display, Guidance, Phase-Table, Alarme)
///
/// AGC source: `Comanche055/REENTRY_CONTROL.agc`, SCALEPOP-Sequenz.
/// AGC source: `Comanche055/SERVICER207.agc`, AVGEXIT/READACCS-Zyklus (2 s).
pub fn reentry_tick(ctx: &mut AgcContext) {
    let sv = ctx.current_sv;

    // -----------------------------------------------------------------------
    // 1. P61: Entry-Anzeigedaten laufend berechnen (S61.2-Schleife).
    //    Nach Phase 0 bleibt das letzte Ergebnis in p61.display stehen.
    //    AGC source: Comanche055/P61-P67.agc, DUMPP61-Label.
    // -----------------------------------------------------------------------
    if ctx.guidance.p61.as_ref().is_some_and(|p| p.phase != 0) {
        if let Some(ref mut p61) = ctx.guidance.p61 {
            let _ = p61_step(
                p61,
                &sv,
                &CentralBody::Earth,
                &mut ctx.phase_table,
                &mut ctx.alarms,
            );
        }
    }

    // -----------------------------------------------------------------------
    // 2. P61 → P62: PRO-Taste (proceed_flag) löst Übergang aus.
    //    Im AGC fällt P61 nach dem N63-PROCEED direkt in P62 durch.
    //    proceed_flag wird vom Dispatcher bei V33 gesetzt und hier konsumiert.
    //    AGC source: Comanche055/P61-P67.agc, P61.2-Label / "FALL INTO P62".
    // -----------------------------------------------------------------------
    if ctx.guidance.active_program == Some(61) && ctx.guidance.proceed_flag {
        ctx.guidance.proceed_flag = false;
        ctx.guidance.active_program = Some(62);
        if let Some(mm) = MajorMode::new(62) {
            ctx.display.major_mode = mm;
        }
    }

    // -----------------------------------------------------------------------
    // 3. P62 → P63: sofortiger Durchfall (kein Warten, kein IMU-Manöver
    //    in dieser Simulation).  Im AGC: `TC P63` am Ende von P62.1 wenn
    //    CMDAPMOD bereits im richtigen Zustand ist.
    //    AGC source: Comanche055/P61-P67.agc, P62.1-Label, letzter `TC P63`.
    // -----------------------------------------------------------------------
    if ctx.guidance.active_program == Some(62) {
        ctx.guidance.active_program = Some(63);
        if let Some(mm) = MajorMode::new(63) {
            ctx.display.major_mode = mm;
        }
    }

    // -----------------------------------------------------------------------
    // 4. P63 → P64: .05G-Sensing.
    //    Im AGC: SERVICER/TARGETNG prüft `.05GSW` nach jedem Average-G-Zyklus.
    //    Wenn D − 0.05G positiv: SET .05GSW → RTB zu P64 (NEWMODEX MM 64).
    //    Bedingung: aktives Programm ist 62 oder 63, P64 noch nicht aktiv.
    //    AGC source: Comanche055/REENTRY_CONTROL.agc, Zeile "DSU BMN .05G NO.05G".
    // -----------------------------------------------------------------------
    if matches!(ctx.guidance.active_program, Some(62) | Some(63)) && ctx.guidance.p64.is_none() {
        let d_g = drag_g(&sv, &CentralBody::Earth);
        if d_g >= 0.05 {
            // .05G überschritten → P64 automatisch starten (RTB NEWMODEX MM 64).
            ctx.guidance.p64 = Some(P64State::new());
            ctx.guidance.active_program = Some(64);
            if let Some(mm) = MajorMode::new(64) {
                ctx.display.major_mode = mm;
            }
        }
    }

    // -----------------------------------------------------------------------
    // 5. P64: Constant-Drag Rollsteuerung pro Tick.
    //    Haversine-Großkreis-Abstand zwischen aktuellem Ort und Splashdown.
    //    AGC source: Comanche055/P61-P67.agc, DANZIG-Label.
    // -----------------------------------------------------------------------
    let p64_transition = if ctx.guidance.p64.as_ref().is_some_and(|p| p.phase != 0) {
        let spl_lat = ctx.guidance.splashdown_lat;
        let spl_lon = ctx.guidance.splashdown_lon;
        let r = sv.position;
        let r_mag = r.magnitude();
        let lat_cur = libm::asin(r.z / r_mag);
        let lon_cur = libm::atan2(r.y, r.x);
        let dlat = spl_lat - lat_cur;
        let dlon = spl_lon - lon_cur;
        let a = libm::sin(dlat * 0.5) * libm::sin(dlat * 0.5)
            + libm::cos(lat_cur)
                * libm::cos(spl_lat)
                * libm::sin(dlon * 0.5)
                * libm::sin(dlon * 0.5);
        let c = 2.0 * libm::atan2(libm::sqrt(a), libm::sqrt(1.0 - a));
        let target_range_m = CentralBody::Earth.r_equatorial() * c;

        if let Some(ref mut p64) = ctx.guidance.p64 {
            p64_step(
                p64,
                &sv,
                &CentralBody::Earth,
                target_range_m,
                &mut ctx.phase_table,
                &mut ctx.alarms,
            )
        } else {
            None
        }
    } else {
        None
    };

    // -----------------------------------------------------------------------
    // 6. P64 → P67: automatischer Übergang wenn p64_step Some(false) zurückgibt.
    //    Bedingung im AGC: PREDICT3-Phase aktiv (V < VFINAL oder D > Q7).
    //    AGC source: Comanche055/P61-P67.agc, P67 — "ENTER VIA RTB FROM REENTRY CONTROL".
    // -----------------------------------------------------------------------
    if p64_transition == Some(false) && ctx.guidance.p67.is_none() {
        ctx.guidance.p67 = Some(P67State::new());
        ctx.guidance.active_program = Some(67);
        if let Some(mm) = MajorMode::new(67) {
            ctx.display.major_mode = mm;
        }
    }

    // -----------------------------------------------------------------------
    // 7. P67: Terminal-Phase — XRNGERR/DNRNGERR pro Tick aktualisieren.
    //    AGC source: Comanche055/P61-P67.agc, P67-Programm.
    // -----------------------------------------------------------------------
    if ctx.guidance.p67.as_ref().is_some_and(|p| p.phase != 0) {
        let spl_lat = ctx.guidance.splashdown_lat;
        let spl_lon = ctx.guidance.splashdown_lon;
        if let Some(ref mut p67) = ctx.guidance.p67 {
            let _ = p67_step(
                p67,
                &sv,
                spl_lat,
                spl_lon,
                &mut ctx.phase_table,
                &mut ctx.alarms,
            );
        }
    }
}
