# Task 8a — Guidance: Orbit-Programme (P11, P38–P39)

## Überblick

Implementierung der ersten Guidance-Programme in `agc-core::guidance`. P11 überwacht
die Earth Orbit Insertion (Aufstiegsphase), P38/P39 planen Rendezvous-Manöver im
stabilen Orbit. Diese drei Programme validieren das Executive-Integration-Pattern
und das Phase-Table-Checkpoint-Muster für alle späteren P-Programme.

**Scope**: Nur die navigationsmathematische Kernlogik — kein Attitude-Control (DAP),
keine reale Pitch/Roll-Polynomial-Steuerung (Saturn-V-spezifisch, zu komplex für
ersten Schritt), kein DSKY-Anzeigetext (nur State-Update an `DskyHal`-Trait).

**Nicht in diesem Task**:
- Gimbal-Lock-Avoidance (Task 9 / DAP)
- Polynomial-basierte Pitch/Roll-Befehle an Saturn V (`POWRSERS`/`NORMLIZE`)
- DSKY-TUI-Darstellung (Task 12)

---

## Sub-Task 1 — Guidance-Modul-Skelett & `GuidanceProgram`-Trait

**Status**: `[x] done`

**Intent**: Gemeinsame Abstraktion für alle P-Programme anlegen. Definiert die
Schnittstelle, die Executive-Jobs, Phase-Table und HAL-Traits verbindet.

**Expected Outcomes**:
- `agc-core/src/guidance/mod.rs` mit `pub trait GuidanceProgram`
- Trait-Methoden: `run_phase(phase: u8, sv: &StateVector, pt: &mut PhaseTable) -> Option<()>`
- `agc-core/src/lib.rs`: `pub mod guidance` ergänzt
- Clippy sauber, `cargo build --target thumbv7em-none-eabihf -p agc-core` grün

**Todo List**:
1. `agc-core/src/guidance/mod.rs` anlegen mit `GuidanceProgram`-Trait-Definition
2. `pub mod guidance` in `agc-core/src/lib.rs` ergänzen
3. Clippy + embedded-Build validieren

**Relevant Context**:
- `agc-core/src/executive/mod.rs` — `Executive::schedule(priority, fn())`
- `agc-core/src/restart/phase_table.rs` — `PhaseTable::set_phase(group, phase)`
- `agc-core/src/navigation/state_vector.rs` — `StateVector`

---

## Sub-Task 2 — P11: Earth Orbit Insertion Monitor

**Status**: `[x] done`

**Intent**: P11 überwacht nach Liftoff die Aufstiegsbahn. In Rust: Zustandsvektor
propagieren, Höhe/Geschwindigkeit berechnen, Display-State aktualisieren.
Polynomial-Steuerung (Saturn-V-spezifisch) und Attitude-Error-Matrix werden
**nicht** implementiert — sie gehören zu DAP (Task 9) bzw. sind Saturn-V-Hardware.

**Expected Outcomes**:
- `agc-core/src/guidance/p11.rs`
- `P11State`-Struct mit `liftoff_met: Met`, `phase: u8`
- Funktion `p11_step(state: &mut P11State, sv: &StateVector, pt: &mut PhaseTable) -> Option<()>`
  - Phase 2: Zustandsvektor initialisieren, `kepler_step` für erste Propagation
  - Phase 3: Höhe/Radialgeschwindigkeit aus `sv` berechnen, Display-Update
  - `AlarmState` wird durchgereicht
- Phase-Table-Gruppe 3 für Restart-Sicherheit
- Unit-Tests:
  - Normaldurchlauf Phase 2→3→done
  - Simulierter Restart in Phase 3 (Phase-Table auf 3, erneuter Aufruf)

**Relevant AGC-Routinen** (als Kommentar-Referenz im Code):
- `P11.agc`: `REP11S`/`REP11SA` (Restart-Eintrag), `MATRXJOB` (RN/VN-Init), `S11.1` (Vi/HDOT/H)
- `CONIC_SUBROUTINES.agc`: `KEPRTN` → `kepler_step()`
- `VHHDOT`: Höhe und Radialgeschwindigkeit — entspricht `sv.position.magnitude() - R_EARTH`

**Todo List**:
1. `agc-core/src/guidance/p11.rs` anlegen
2. `P11State`-Struct mit Doc-Comment (Einheiten, AGC-Quelle)
3. `p11_step()` mit Phase-2-Initialisierung (Kepler-Propagation, Met-Berechnung)
4. Phase-3-Logik: Höhe aus `|r| - R_EARTH`, Radialgeschwindigkeit `r·v / |r|`
5. Phase-Table-Checkpoints (Gruppe 3, Phasen 2/3)
6. Unit-Tests: Normaldurchlauf + Restart-Szenario
7. `pub mod p11` in `guidance/mod.rs` ergänzen
8. Clippy + Tests grün

**Relevant Context**:
- `input/virtualagc/Comanche055/P11.agc`
- `agc-core/src/navigation/conic.rs` — `kepler_step()`
- `agc-core/src/navigation/orbital.rs` — `CentralBody::Earth`, `propagate()`
- `agc-core/src/alarm/mod.rs` — `AlarmState`

---

## Sub-Task 3 — P38: Stable Orbit Insertion (SOI)

**Status**: `[x] done`

**Intent**: P38 berechnet das SOI-Manöver — Transfer-Bahn vom aktiven Fahrzeug
zum passiven Fahrzeug. Kern: Lambert-Solver für Δv-Berechnung.

**Expected Outcomes**:
- `agc-core/src/guidance/p38_39.rs`
- `P38Input`-Struct: `tig: Met`, `tof: f64` (Transfer-Zeit in Sekunden), `sv_active: StateVector`, `sv_passive: StateVector`
- `P38Output`-Struct: `delta_v_soi: Vec3` (Δv am Burn-Punkt, ECI m/s), `delta_v_intercept: Vec3`
- Funktion `p38_compute(input: &P38Input, alarm: &mut AlarmState) -> Option<P38Output>`
  - Passivfahrzeug zu `tig + tof` propagieren → `r_target`
  - `lambert_solver(r_active_at_tig, r_target, tof, mu, prograde=true)` aufrufen
  - Δv_SOI = `v_lambert_initial - v_active_at_tig`
  - Δv_Intercept = `v_passive_at_intercept - v_lambert_final`
- Unit-Tests:
  - Kreisbahn-Transfer (Hohmann): analytisch bekanntes Δv → Abweichung < 1 m/s
  - Lambert `None` bei 180°-Transfer (Singularität) → `None`-Propagation nach oben

**Relevant AGC-Routinen** (als Kommentar-Referenz im Code):
- `STABLE_ORBIT_-_P38-P39.agc`: `MAINRTNE`, `PREC/TT`, `TIMETHET`
- `CONIC_SUBROUTINES.agc`: `LAMBERT` → `lambert_solver()`

**Todo List**:
1. `P38Input` und `P38Output` Structs mit Doc-Comments
2. `p38_compute()` implementieren (Lambert-Aufruf, Δv-Berechnung)
3. Unit-Tests: Hohmann-Transfer + 180°-Singularität
4. Clippy + Tests grün

**Relevant Context**:
- `input/virtualagc/Comanche055/STABLE_ORBIT_-_P38-P39.agc`
- `agc-core/src/navigation/conic.rs` — `lambert_solver()`, `kepler_step()`
- `agc-core/src/navigation/orbital.rs` — `CentralBody`, `propagate()`

---

## Sub-Task 4 — P39: Midcourse Correction

**Status**: `[x] done`

**Intent**: P39 berechnet eine Bahnkorrektur zwischen SOI und Intercept, basierend
auf der aktuellen (gemessenen) statt der propagierten Position des Passivfahrzeugs.

**Expected Outcomes**:
- `P39Input`-Struct: `sv_active: StateVector`, `sv_passive_measured: StateVector`, `tof_remaining: f64`
- `P39Output`-Struct: `delta_v_correction: Vec3`
- Funktion `p39_compute(input: &P39Input, alarm: &mut AlarmState) -> Option<P39Output>`
  - Lambert mit aktueller Passivfahrzeug-Position
  - Δv_Korrektur = neue SOI-Bahn - geplante SOI-Bahn
- Unit-Tests:
  - Keine Korrektur nötig (Messung = Propagation) → `|Δv| ≈ 0`
  - Kleine Abweichung → Korrektur in sinnvoller Größenordnung

**Todo List**:
1. `P39Input`/`P39Output`-Structs ergänzen in `p38_39.rs`
2. `p39_compute()` implementieren
3. Unit-Tests: Nullkorrektur + kleine Abweichung
4. Clippy + Tests grün

**Relevant Context**:
- `input/virtualagc/Comanche055/STABLE_ORBIT_-_P38-P39.agc`
- `agc-core/src/navigation/conic.rs` — `lambert_solver()`

---

## Sub-Task 5 — Integration, Validierung & Git-Commit

**Status**: `[x] done`

**Intent**: Alle drei Programme durch den Executive laufen lassen (als Jobs), Plan-Status
aktualisieren und Commit erstellen.

**Expected Outcomes**:
- `agc-core/src/guidance/mod.rs` exportiert `p11`, `p38_39`
- Integrations-Test: `p11_step` → Executive-Job → Phase-Table-Checkpoint
- `cargo fmt && cargo clippy -- -D warnings && cargo test` grün
- `cargo build --target thumbv7em-none-eabihf -p agc-core` grün
- Task 8a in `docs/target-architecture-plan.md` auf `[x] done` gesetzt
- Git-Commit: `Task 8a — Guidance Orbit-Programme: P11, P38, P39 implementiert`

**Todo List**:
1. Integrations-Test in `guidance/mod.rs` oder separater Testdatei:
   - P11-Job über Executive ausführen, Phase-Table-Zustand danach prüfen
2. Alle Tests grün, Clippy sauber, embedded-Build grün
3. Plan-Status in `docs/target-architecture-plan.md` aktualisieren
4. Git-Commit

**Relevant Context**:
- `agc-core/src/executive/mod.rs` — `Executive::schedule()`
- `agc-core/src/restart/phase_table.rs` — `PhaseTable`
- `docs/target-architecture-plan.md` § Task 8a

---

## Abhängigkeiten

```
Sub-Task 1 (Skelett)
    └─► Sub-Task 2 (P11)
    └─► Sub-Task 3 (P38)
            └─► Sub-Task 4 (P39)
                    └─► Sub-Task 5 (Integration + Commit)
```

Sub-Tasks 2 und 3 können parallel implementiert werden — sie teilen nur den
`GuidanceProgram`-Trait aus Sub-Task 1, haben aber keine gegenseitige Abhängigkeit.
