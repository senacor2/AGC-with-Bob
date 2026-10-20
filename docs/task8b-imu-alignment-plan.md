# Task 8b — Guidance: IMU-Alignment (P51–P53)

## Überblick

Implementierung der IMU-Alignment-Programme in `agc-core::guidance`. P51/P53
(Initial Platform Alignment / Gyro Torquing) bestimmen die REFSMMAT-Rotationsmatrix
aus zwei Sternmessungen. P52 richtet die IMU auf eine neue Orientierung aus
(Reorientation). Diese Programme bauen auf den vorhandenen Modulen
`navigation::alignment` (TRIAD) und `navigation::star_tables` auf.

**Fidelity-Entscheidung**: P53 ist im AGC ein Alias für P51 (`P53 EQUALS P51`).
In Rust wird P53 als Typ-Alias auf dieselbe Funktion abgebildet.

**Scope**:
- P51/P53: Sternmessung (synthetisch via IMU-Gimbal-Winkel) → REFSMMAT via TRIAD
- P52: Neue Zielorientierung berechnen (Nominal Orientation) + Gimbal-Winkel ausgeben
- CDU-Winkel-Konversionshilfsfunktionen in `navigation::alignment` ergänzen
- Phase-Table-Checkpoints (Restart-Sicherheit)

**Nicht in diesem Task**:
- Hardware-Level Coarse/Fine-Align (IMUCOARS/IMUFINE → reale Gimbal-Motoren,
  kein Rust-Gegenstück ohne echte Hardware)
- Stern-Okklusionstest (Erde/Sonne/Mond-Schatten → Task 9 / DAP-Geometrie)
- DSKY-TUI-Darstellung (Task 12)
- Optik-Geometrie (SXTNB, Sextant Shaft/Trunnion-Mechanik → Task 9)

---

## Sub-Task 1 — `navigation::alignment` erweitern

**Status**: `[ ] pending`

**Intent**: Die vorhandene `alignment.rs` besitzt nur `triad_align()`. Für P51/P52
werden drei weitere Hilfsfunktionen benötigt, die CDU-Winkel verarbeiten und
Sternwinkel-Plausibilität prüfen.

**Expected Outcomes**:
- `cdu_to_rad(cdu: i16) -> f64` — CDU-LSB → Radiant
  - 1 LSB = 2π / 32768 rad (= 360° / 2^15)
  - AGC source: `Comanche055/IMU_CALIBRATION_AND_ALIGNMENT.agc`, CDUTRIG routine
- `gimbal_to_vec(oga: i16, mga: i16, iga: i16) -> Vec3` — CDU-Gimbal-Winkel
  der Stable Member → Einheitsvektor in Body-Frame (Z-Achse des SM)
  - Entspricht der Stable-Member-Z-Achse nach Euler-Rotation (Inner→Middle→Outer)
  - Reihenfolge: R_z(oga) · R_x(mga) · R_z(iga), Z-Achse extrahieren
  - AGC source: `Comanche055/INFLIGHT_ALIGNMENT_ROUTINES.agc`, CALCSMSC routine
- `star_angle_valid(v1: Vec3, v2: Vec3) -> bool` — prüft ob der Winkel zwischen
  zwei Sternvektoren im gültigen Bereich liegt (40° – 66°, AGC PICAPAR-Kriterium)
  - AGC source: `Comanche055/P51-P53.agc`, R56/PICAPAR routine
- Alle drei Funktionen mit vollständigen Doc-Comments (Einheiten, AGC-Quelle)
- Unit-Tests für alle drei Funktionen

**Todo List**:
1. `cdu_to_rad()` implementieren und testen (Grenzwerte: 0→0.0, 32767→≈2π, -16384→-π)
2. `gimbal_to_vec()` implementieren: Euler-Rotation aufstellen, Z-Achse extrahieren
3. `star_angle_valid()` implementieren: `dot(v1.normalize, v2.normalize)` → arccos → Bereichsprüfung
4. Unit-Tests: `cdu_to_rad` Roundtrip, `gimbal_to_vec` Identität (alle 0 → [0,0,1] oder [0,0,1] je Konvention), `star_angle_valid` Grenzfälle
5. Clippy sauber

**Relevant Context**:
- `agc-core/src/navigation/alignment.rs` (bestehende Datei erweitern)
- `input/virtualagc/Comanche055/INFLIGHT_ALIGNMENT_ROUTINES.agc` (CALCSMSC)
- `input/virtualagc/Comanche055/P51-P53.agc` (PICAPAR-Winkelbereich)

---

## Sub-Task 2 — `guidance::p51_53` implementieren

**Status**: `[ ] pending`

**Intent**: P51 führt das vollständige Alignment durch: zwei Sternbeobachtungen
→ TRIAD → REFSMMAT. P52 berechnet die Zielorientierung aus Positions-/
Geschwindigkeitsvektor. P53 ist identisch mit P51.

**Expected Outcomes**:
- `agc-core/src/guidance/p51_53.rs`
- **`Refsmmat`-Newtype** für die 3×3-Rotationsmatrix (Body→Ref), mit Doc-Comment
- **`P51State`-Struct**: zwei Sternbeobachtungen (Body-Vektoren), Phase, Sternindizes
- **`p51_step(state, imu, pt, alarm) -> Option<Refsmmat>`**:
  - Phase 2: Erste Sternmessung via `gimbal_to_vec(imu.read_gimbal_angles())`, Phase-Table Gruppe 5
  - Phase 3: Zweite Sternmessung, `star_angle_valid()` prüfen (Alarm 0x0351 bei ungültig)
  - Phase 4: `triad_align(v1_body, v2_body, v1_ref, v2_ref)` → REFSMMAT
  - Phase-Table Checkpoints in Gruppe 5 (Phasen 2/3/4)
  - Referenzsternvektoren aus `star_tables::get_star()`
- **`p52_reorient(sv, pt, alarm) -> Option<[i16; 3]>`**: Ziel-REFSMMAT aus
  Positions-/Geschwindigkeitsvektor berechnen, als CDU-Gimbal-Winkel zurückgeben
  - Nominale Orientierung: X-Achse = -r̂ (lokale Vertikal), Z-Achse = (r×v)×r
  - Gimbal-Winkel aus Zielrotationsmatrix berechnen
  - AGC source: `P51-P53.agc` S52.3-Routine
- **`pub use p51_step as p53_step;`** (P53 = P51)
- `pub mod p51_53` in `guidance/mod.rs` ergänzen

**Todo List**:
1. `Refsmmat`-Newtype mit Doc-Comment anlegen
2. `P51State` mit Sternindizes und Messungen definieren
3. `p51_step()` Phase 2→3→4 implementieren
4. `p52_reorient()` implementieren
5. `pub mod p51_53` in `guidance/mod.rs`
6. Clippy sauber

**Relevant Context**:
- `agc-core/src/navigation/alignment.rs` — `triad_align()`, `gimbal_to_vec()`
- `agc-core/src/navigation/star_tables.rs` — `get_star()`
- `agc-core/src/hal/imu.rs` — `ImuHal`, `MockImu`
- `agc-core/src/restart/phase_table.rs` — Gruppe 5
- `input/virtualagc/Comanche055/P51-P53.agc`

---

## Sub-Task 3 — Unit-Tests

**Status**: `[ ] pending`

**Intent**: Alignment-Korrektheit mit synthetischen Sternmessungen verifizieren.

**Expected Outcomes**:
- Test: **Exaktes Alignment** — synthetische Messungen ohne Rauschen → REFSMMAT = Identität
  (Sternvektoren im Body-Frame = Sternvektoren im Ref-Frame → Kein Fehler)
- Test: **Bekannte Rotation** — Body um 90° um Z rotiert → REFSMMAT entspricht R_z(90°)
- Test: **Restart-Simulation** — Phase-Table manuell auf Phase 3 setzen, erste Messung
  bereits in State → zweite Messung korrekt fortgesetzt
- Test: **Ungültiger Sternwinkel** — Sterne zu nah beieinander (<40°) → `None` + Alarm 0x0351
- Test: **IMU nicht operational** — `imu.operational = false` → sofortiges `None` + Alarm
- Alle Tests in `#[cfg(test)]`-Block in `p51_53.rs`

**Todo List**:
1. Test `identity_alignment`: Stern 1 und Stern 2 im Body = Katalogwerte → Identitätsmatrix
2. Test `known_rotation`: Sternvektoren um 90° rotiert → REFSMMAT prüfen
3. Test `restart_resume`: State mit Phase 3 + erster Messung → zweite Messung → REFSMMAT
4. Test `angle_too_small_alarm`: Sterne < 40° apart → None
5. Test `imu_not_operational`: operational=false → None
6. Alle Tests grün

**Relevant Context**:
- `agc-core/src/hal/imu.rs` — `mock::MockImu`
- `agc-core/src/navigation/star_tables.rs` — Stern 1 und Stern 2 für bekannte Winkel

---

## Sub-Task 4 — Integration, Plan-Update & Git-Commit

**Status**: `[ ] pending`

**Intent**: Abschlussvalidierung, Plan-Status aktualisieren, Commit erstellen.

**Expected Outcomes**:
- `cargo fmt && cargo clippy -- -D warnings && cargo test` grün
- `cargo build --target thumbv7em-none-eabihf -p agc-core` grün
- `docs/target-architecture-plan.md`: Task 8b auf `[x] done`
- Git-Commit: `Task 8b — Guidance IMU-Alignment: P51, P52, P53 implementiert`

**Todo List**:
1. Alle Tests grün, Clippy sauber, embedded-Build grün
2. `docs/target-architecture-plan.md` Task 8b auf `[x] done` setzen
3. Git-Commit

---

## Abhängigkeiten

```
Sub-Task 1: alignment.rs erweitern
    └─► Sub-Task 2: p51_53.rs implementieren
            └─► Sub-Task 3: Unit-Tests
                    └─► Sub-Task 4: Commit
```
