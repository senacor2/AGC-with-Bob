# Task 8d — Guidance: Rendezvous-Programme (P20–P25, P30–P37, P32–P33)

## Überblick

Implementierung der Rendezvous-Navigations- und Manöver-Programme in
`agc-core::guidance`. Die Programme nutzen `navigation::measurement`
(Kalman-Filter) für Radar-Update und `navigation::conic` (Kepler-Propagation,
Lambert-Solver) für Manöver-Planung.

**Fidelity-Entscheidung**:
- DSKY-Interaktion (V06N33/V06N81-Anzeige), Optik-Steuerung (R52) und
  Rendezvous-Radar-Hardware (LM-only) sind nicht Teil dieses Tasks.
- CM-Rendezvous verwendet VHF-Ranging (`VhfRangingHal`) und optisches
  Tracking (LOS-Winkel aus Positionsvektoren).
- P37 (Return-to-Earth) liefert ein Lambert-Δv mit Periapsis-Check.
- P22 (Landmark Tracking) berechnet die MGA (Middle Gimbal Angle) aus
  Positions-/Geschwindigkeitsvektor — kein Optik-Hardware-Aufruf.

**Scope**:
- P20: Rendezvous-Navigations-Schleife (Zustandsvektor-Update via Kalman)
- P22: Landmark Tracking (MGA-Berechnung aus r×v Querschnitt)
- P30: External ΔV Targeting (Apogee/Perigee + DELVSIN aus TIG + DELVSLV)
- P37: Return-to-Earth (Lambert-Manöver zur Rückkehrbahn)
- P32: CSI — Co-Elliptic Sequence Initiation (Δv_CSI in Horizontalebene)
- P33: CDH — Constant Delta Height (Δv_CDH zur Höhennivellierung)
- Phase-Table-Gruppe 6 für Rendezvous-Phasen
- Unit-Tests mit synthetischen Inputs

**Nicht in diesem Task**:
- Optik-Hardware R52 (Sextant/Telescope Shaft/Trunnion)
- RCS/TVC DAP (Task 9)
- DSKY-Anzeige (Task 12)
- TPI-Manöver P34/P35 (außerhalb Scope laut Plan)

---

## Sub-Task 1 — `guidance::p20_25` implementieren

**Status**: `[x] done`
**Implementiert in**: `agc-core/src/guidance/p20_25.rs`

**Intent**: P20 aktualisiert den Zustandsvektor via Kalman-Messung
(VHF-Ranging). P22 berechnet MGA aus dem aktuellen Zustandsvektor.

**Expected Outcomes**:
- `agc-core/src/guidance/p20_25.rs`

### P20 (Rendezvous Navigation)

- **`P20State`-Struct**:
  - `phase: u8` — Phase 2 = Initialisierung, 3 = Tracking-Schleife, 0 = done
  - `sv_active: StateVector` — aktueller aktiver Zustandsvektor
  - `mark_count: u16` — Tracking-Mark-Zähler
  - `delta_x: Vec3` — Positions-Fehlervektor [m]

- **`p20_step(state, ranging, pt, alarm, body) -> Option<()>`**:
  - Phase 2: `delta_x = Vec3::zero()`, `mark_count = 0`, Phase-Table Gruppe 6
    Checkpoint 3
  - Phase 3: VHF-Ranging lesen (`ranging.read_range()`), Kalman-Update
    (`incorporate_measurement`), `mark_count++`, Phase-Table Gruppe 6 Checkpoint 3
  - Phase 0: `None` (done / idle)
  - AGC-Quelle: `P20-P25.agc`, PROG20-Routine

### P22 (Landmark Tracking / MGA)

- **`p22_mga(sv) -> f64`**: Berechnet MGA (Middle Gimbal Angle) aus
  Zustandsvektor: `arccos(|(v × r) / |v × r|| · ê_z_refsmmat)`
  — vereinfacht: arccos(|dot(unit(v × r), [0,0,1])|)
  - AGC-Quelle: `P20-P25.agc`, PROG22-Routine

---

## Sub-Task 2 — `guidance::p30_37` implementieren

**Status**: `[x] done`
**Implementiert in**: `agc-core/src/guidance/p30_37.rs`

**Intent**: P30 berechnet Apogee/Perigee und DELVSIN für ein externes
Δv-Manöver. P37 berechnet den Lambert-Transfer zur Rückkehrbahn.

**Expected Outcomes**:
- `agc-core/src/guidance/p30_37.rs`

### P30 (External ΔV Targeting)

- **`P30Input`-Struct**: `sv_tig`, `delta_v_lv`, `body`
- **`P30Output`-Struct**: `delta_v_ref: Vec3`, `h_apo: f64`, `h_per: f64`
- **`p30_compute(input, alarm) -> Option<P30Output>`**:
  - Propagiert SV zum TIG (Kepler), rotiert DELVSLV in ECI via local-vertical
    Rahmen, berechnet Post-Burn SV, ermittelt Apogee/Perigee via
    `orbital_elements()`
  - AGC-Quelle: `P30,P37.agc`, P30-Routine + S30.1-Subroutine

### P37 (Return to Earth)

- **`P37Input`-Struct**: `sv_current`, `t_return`, `body`
- **`P37Output`-Struct**: `delta_v: Vec3`, `h_per: f64`
- **`p37_compute(input, alarm) -> Option<P37Output>`**:
  - Lambert-Solver von aktueller Position zu Periapsis-Target auf
    Rückkehrbahn (r_target = 6 471 000 m ≈ 100 km altitude over Earth)
  - Periapsis-Check: h_per ≥ 60 km sonst Alarm 0x0370
  - AGC-Quelle: `P30,P37.agc`, P37-Routine

---

## Sub-Task 3 — `guidance::p32_33` implementieren

**Status**: `[x] done`
**Implementiert in**: `agc-core/src/guidance/p32_33.rs`

**Intent**: P32 (CSI) berechnet das Co-Elliptic-Initiation-Δv in der
Horizontalebene des aktiven Fahrzeugs. P33 (CDH) berechnet das
Constant-Delta-Height-Manöver zur Höhennivellierung.

**Expected Outcomes**:
- `agc-core/src/guidance/p32_33.rs`

### P32 (CSI — Co-Elliptic Sequence Initiation)

- **`CsiInput`-Struct**: `sv_active`, `sv_passive`, `t_csi`, `body`
- **`CsiOutput`-Struct**: `delta_v_csi: Vec3`, `t1_to_t2: f64`
- **`csi_compute(input, alarm) -> Option<CsiOutput>`**:
  - Propagiert beide Fahrzeuge zu T_CSI (Kepler)
  - Δv_CSI = Δv in der Horizontalebene (r̂ × (r̂ × v̂) Komponente entfernen)
  - t1_to_t2 = T_CDH − T_CSI (halbiertes Periapsis-Intervall)
  - Alarm 0x0320 wenn t1_to_t2 < 600 s (10 min AGC-Minimum)
  - AGC-Quelle: `P32-P33,_P72-P73.agc`, P32-Routine + CSI/A-Subroutine

### P33 (CDH — Constant Delta Height)

- **`CdhInput`-Struct**: `sv_active_cdh`, `sv_passive_cdh`, `body`
- **`CdhOutput`-Struct**: `delta_v_cdh: Vec3`, `diff_alt: f64`
- **`cdh_compute(input, alarm) -> Option<CdhOutput>`**:
  - diff_alt = |r_passive| − |r_active| (Höhendifferenz)
  - Δv_CDH: Geschwindigkeit angleichen, damit `diff_alt` konstant bleibt
    → radiale Komponente von (v_passive − v_active) annullieren
  - Alarm 0x0330 wenn diff_alt > 92 600 m (50 NM, AGC-Grenze)
  - AGC-Quelle: `P32-P33,_P72-P73.agc`, P33-Routine + CDHMVR-Subroutine

---

## Sub-Task 4 — Unit-Tests

**Status**: `[x] done`
**Ergebnis**: 15 neue Tests (6× p20_25, 5× p30_37, 5× p32_33), alle grün.

**Expected Outcomes**:
- Tests für P20, P22, P30, P37, P32, P33 in den jeweiligen Modulen

---

## Sub-Task 5 — Integration, Plan-Update & Git-Commit

**Status**: `[x] done`

**Ergebnis**:
- `cargo fmt && cargo clippy -- -D warnings && cargo test` ✓ (192 Tests, 0 Failures)
- `cargo build --target thumbv7em-none-eabihf -p agc-core` ✓
- `docs/target-architecture-plan.md`: Task 8d auf `[x] done` gesetzt
- `docs/task8d-rendezvous-plan.md`: aktualisiert
- Git-Commit: `Task 8d — Guidance Rendezvous: P20, P22, P30, P32, P33, P37 implementiert`

---

## Abhängigkeiten

```
Sub-Task 1: p20_25.rs implementieren
    └─► Sub-Task 2: p30_37.rs implementieren
            └─► Sub-Task 3: p32_33.rs implementieren
                    └─► Sub-Task 4: Unit-Tests
                            └─► Sub-Task 5: Commit
```
