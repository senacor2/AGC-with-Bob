# Task 8c — Guidance: SPS-Burn (P40–P47)

## Überblick

Implementierung der SPS-Burn-Programme in `agc-core::guidance`. P40 führt den
SPS-Haupttriebwerksbrand durch (Ignition → Steering → Cutoff), P41 ist der
Gimbal-Motor-Test vor dem Burn, P47 ist das Post-Burn Trim-Manöver.

**Fidelity-Entscheidung**: 
- Die DSKY-Interaktion (V99-Bestätigung, N40/N85-Anzeige) und der TVC-DAP
  sind nicht Teil dieses Tasks — sie gehören zu Task 9 (DAP) und Task 12 (TUI).
- `P41` (Gimbal Motor Test) prüft die TVC-Gimbals durch; in Rust validieren
  wir nur, dass der `SpsHal`-Trait korrekt aufgerufen wird.
- `P47` (Trim Maneuver) berechnet die Korrektur aus akkumuliertem ΔIMU-Fehler.
- Cutoff-Kriterium: Restgeschwindigkeit (VGO) ≤ Impulsiv-Schwellwert (600 cs)
  → `burn_time()` bestimmt Brenndauer, Cutoff wenn `t_elapsed ≥ t_burn`.

**Scope**:
- P40: Ignition-Phase, Steuerung, Cutoff, Post-Burn State
- P41: Gimbal-Motor-Test (synthetisch: pitch ±2°, yaw ±2°, zurück zu 0°)
- P47: Trim-Manöver (Δv-Restfehler aus ΔIMU-Messung)
- Phase-Table-Gruppe 4 für Burn-Phasen
- `SpsHal` aus `hal/sps.rs` für Triebwerkssteuerung

**Nicht in diesem Task**:
- TVC-DAP / Gimbal-Steering-Regelkreis (Task 9)
- DSKY-Anzeige (Task 12)
- Ullage-RCS-Befehle (Task 9 / RCS-HAL)
- PHASCHNG / Restart-Vorbereitung für CLOCKTASK/CLOKTASK (Zeit-Hardware)

---

## Sub-Task 1 — `guidance::p40_47` implementieren

**Status**: `[x] done`

**Intent**: Drei Guidance-Programme für die SPS-Burn-Sequenz, aufbauend auf
`navigation::powered_flight::burn_time()` und `hal::sps::SpsHal`.

**Expected Outcomes**:
- `agc-core/src/guidance/p40_47.rs`

### P40 (SPS Burn)

- **`P40State`-Struct**:
  - `phase: u8` — Phase 2 = Pre-Ignition, 3 = Ignition/Burn, 4 = Cutoff, 0 = done
  - `delta_v_target: f64` — gewünschte Δv-Größe [m/s]
  - `isp: f64` — spez. Impuls [s] (SPS: 314.5 s)
  - `thrust: f64` — Schub [N] (SPS: 97 855 N)
  - `mass: f64` — Startmasse [kg]
  - `t_burn: f64` — berechnete Brenndauer [s] (aus `burn_time()`)
  - `t_elapsed: f64` — seit Ignition verstrichene Zeit [s]

- **`p40_step(state, sps, pt, alarm, dt) -> Option<()>`**:
  - Phase 2: `burn_time(delta_v, isp, thrust, mass)` → `t_burn`, Alarm 0x0440
    bei `None`; `sps.set_engine(true)`, Phase-Table Gruppe 4 Checkpoint 3
  - Phase 3: `t_elapsed += dt`, `sps.set_engine(true)`,
    Cutoff wenn `t_elapsed >= t_burn` → Phase 4
  - Phase 4: `sps.set_engine(false)`, Phase-Table Gruppe 4 auf 0; `phase = 0`
  - AGC source: `P40-P47.agc`, P40CSM / IGNITION / ENGINOFF / SPSOFF-Routinen

### P41 (Gimbal Motor Test)

- **`P41State`-Struct**:
  - `phase: u8` — Phase 2 = Pitch-Test, 3 = Yaw-Test, 4 = Trim, 0 = done
  - `step: u8` — Schritt innerhalb der Phase (0–2: +2°, -2°, 0°)

- **`p41_step(state, sps, pt, alarm) -> Option<()>`**:
  - Phase 2: Pitch-Sequenz +84, -84, 0 LSB an `sps.set_tvc_pitch()`
  - Phase 3: Yaw-Sequenz +84, -84, 0 LSB an `sps.set_tvc_yaw()`
  - Phase 4: Trim-Werte auf 0 setzen, `phase = 0`
  - AGC source: `P40-P47.agc`, S40.6 GIMDTEST-Routine

### P47 (Trim Maneuver)

- **`P47State`-Struct**:
  - `phase: u8` — Phase 2 = Δv berechnen, 0 = done
  - `delta_v_imu: f64` — akkumuliertes Δv aus IMU-Messung [m/s]
  - `delta_v_ref: f64` — Δv-Sollwert aus dem Burn [m/s]

- **`p47_step(state, pt, alarm) -> Option<f64>`**:
  - Phase 2: Restfehler `dv_trim = delta_v_ref - delta_v_imu`, `phase = 0`
  - Rückgabe: `Some(dv_trim)` (positiv = Nachzünden nötig)
  - AGC source: `P40-P47.agc`, P47CSM / CALCN83-Routine

**Todo List**:
1. `P40State`, `P41State`, `P47State` Structs mit Doc-Comments anlegen
2. `p40_step()` Phase 2→3→4→0 implementieren (burn_time + SpsHal)
3. `p41_step()` Phase 2→3→4→0 (Gimbal-Sequenz)
4. `p47_step()` Phase 2→0 (Trim-Fehler-Berechnung)
5. `pub mod p40_47` in `guidance/mod.rs` ergänzen
6. Clippy sauber

**Relevant Context**:
- `agc-core/src/navigation/powered_flight.rs` — `burn_time()`
- `agc-core/src/hal/sps.rs` — `SpsHal`, `MockSps`
- `agc-core/src/restart/phase_table.rs` — Gruppe 4
- `input/virtualagc/Comanche055/P40-P47.agc`

---

## Sub-Task 2 — Unit-Tests

**Status**: `[x] done`

**Intent**: Burn-Korrektheit, Cutoff, Gimbal-Test und Trim mit synthetischen Inputs.

**Expected Outcomes**:
- Test: **Burn nominal** — gültige Inputs, Phase 2 → 3 → 4 → 0;
  nach Cutoff: `sps.engine_on() == false`
- Test: **Burn cutoff** — `dt = t_burn + 0.1` in einer Phase-3-Iteration → Cutoff
- Test: **Ungültige Parameter** — `delta_v = 0` → `None` + Alarm 0x0440
- Test: **P41 Gimbal-Sequenz** — drei Schritte Pitch, drei Schritte Yaw,
  jedes Mal korrekte Werte an `MockSps`
- Test: **P47 Trim** — `delta_v_imu = 95.0`, `delta_v_ref = 100.0` → `Some(5.0)`
- Test: **Restart Phase 3** — `state.phase = 3`, Phase-Table Gruppe 4 = 3,
  Burn setzt sich korrekt fort

**Todo List**:
1. Test `p40_burn_nominal`: voller Zyklus Phase 2→0
2. Test `p40_burn_cutoff_timing`: Cutoff nach genau t_burn
3. Test `p40_invalid_params`: delta_v=0 → None + Alarm
4. Test `p41_gimbal_sequence`: Pitch→Yaw→Trim korrekte Werte
5. Test `p47_trim_residual`: Restfehler korrekt berechnet
6. Test `p40_restart_phase3`: State mit Phase 3 + t_elapsed=5.0 → Burn fortgesetzt

---

## Sub-Task 3 — Integration, Plan-Update & Git-Commit

**Status**: `[x] done`

**Expected Outcomes**:
- `cargo fmt && cargo clippy -- -D warnings && cargo test` grün
- `cargo build --target thumbv7em-none-eabihf -p agc-core` grün
- `docs/target-architecture-plan.md`: Task 8c auf `[x] done`
- Git-Commit: `Task 8c — Guidance SPS-Burn: P40, P41, P47 implementiert`

---

## Abhängigkeiten

```
Sub-Task 1: p40_47.rs implementieren
    └─► Sub-Task 2: Unit-Tests
            └─► Sub-Task 3: Commit
```
