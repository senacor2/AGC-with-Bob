/// RCS-DAP-Waitlist-Task — läuft alle 100 ms (10 Zentisekunden).
///
/// Dieser Task implementiert den T5RUPT-getriebenen DAP-Zyklus des echten AGC:
/// 1. CDU-Winkel aus `ctx.dap.cdu_angles` lesen (vom Sim-Loop befüllt).
/// 2. Lagefehler mit [`rcs_dap::rcs_dap_update`] berechnen.
/// 3. Achsbefehle mit [`jet_select::axis_command`] ableiten.
/// 4. RCS-Bits in `ctx.dap.jets_ab` / `ctx.dap.jets_cd` schreiben.
/// 5. Sich selbst nach 10 cs erneut in die Waitlist eintragen.
///
/// Der Sim-Loop liest `ctx.dap.jets_ab/cd` nach dem Waitlist-Tick und
/// übergibt sie an `SimRcs`.
///
/// # AGC source
/// `Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc`, RCSATT / SETT5 / T5PHASE1.
use crate::context::AgcContext;
use crate::dap::jet_select::{axis_command, select_jets_raw};
use crate::dap::rcs_dap::{rcs_dap_update_raw, DEADBAND_RAD};

// ---------------------------------------------------------------------------
// Zeitkonstante
// ---------------------------------------------------------------------------

/// DAP-Abtastperiode in Zentisekunden (100 ms).
///
/// AGC source: `Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc`, T5TVCDT (100 ms).
const DAP_PERIOD_CS: u32 = 10;

/// DAP-Abtastperiode in Sekunden — für Fehlerberechnung.
const DAP_DT_S: f64 = DAP_PERIOD_CS as f64 / 100.0;

// ---------------------------------------------------------------------------
// Öffentliche API
// ---------------------------------------------------------------------------

/// Trägt den RCS-DAP-Task einmalig in die Waitlist ein.
///
/// Sollte beim Simulatorstart (oder DAP-Aktivierung) genau einmal aufgerufen
/// werden.  `ctx.dap.task_scheduled` verhindert Doppel-Einträge.
///
/// AGC source: `Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc`, RCSATT / INITDAP.
pub fn schedule_dap(ctx: &mut AgcContext) {
    if !ctx.dap.task_scheduled {
        ctx.waitlist.call_after(DAP_PERIOD_CS, rcs_dap_task);
        ctx.dap.task_scheduled = true;
    }
}

// ---------------------------------------------------------------------------
// Waitlist-Task (privat — wird nur als Funktionszeiger verwendet)
// ---------------------------------------------------------------------------

/// RCS-DAP-Waitlist-Task.
///
/// AGC source: `Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc`, T5PHASE1 routine.
fn rcs_dap_task(ctx: &mut AgcContext) {
    // CDU-Winkel aus dem Erasable-Memory lesen (vom Sim-Loop geschrieben).
    let [cx, cy, cz] = ctx.dap.cdu_angles;

    // Lagefehler berechnen (ohne direkten IMU-HAL-Zugriff).
    let errors = rcs_dap_update_raw(&mut ctx.dap.rcs, cx, cy, cz, DAP_DT_S);

    // Achsbefehle aus Lagefehlern ableiten (Phasenebenenschwelle = DEADBAND).
    let pitch_cmd = axis_command(errors[0], 0.0, DEADBAND_RAD);
    let yaw_cmd = axis_command(errors[1], 0.0, DEADBAND_RAD);
    let roll_cmd = axis_command(errors[2], 0.0, DEADBAND_RAD);

    // RCS-Bits berechnen und in ctx.dap ablegen.
    let (ab, cd) = select_jets_raw(pitch_cmd, yaw_cmd, roll_cmd);
    ctx.dap.jets_ab = ab;
    ctx.dap.jets_cd = cd;

    // Selbst-Wiederholung: nach DAP_PERIOD_CS erneut feuern.
    ctx.waitlist.call_after(DAP_PERIOD_CS, rcs_dap_task);
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::context::AgcContext;

    #[test]
    fn schedule_dap_sets_flag() {
        let mut ctx = AgcContext::default();
        assert!(!ctx.dap.task_scheduled);
        schedule_dap(&mut ctx);
        assert!(
            ctx.dap.task_scheduled,
            "task_scheduled muss nach schedule_dap gesetzt sein"
        );
    }

    #[test]
    fn schedule_dap_idempotent() {
        let mut ctx = AgcContext::default();
        schedule_dap(&mut ctx);
        let count_after_first = ctx.waitlist.count();
        schedule_dap(&mut ctx);
        assert_eq!(
            ctx.waitlist.count(),
            count_after_first,
            "Doppeltes Einplanen darf Waitlist nicht verlängern"
        );
    }
}
