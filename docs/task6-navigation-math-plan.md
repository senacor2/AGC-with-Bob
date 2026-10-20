# Task 6 — Navigationsmathematik: Implementierungsplan

> **Scope**: `agc-core/src/navigation/` + `tests/reference/`  
> **Ziel**: Alle Kern-Navigationsroutinen als saubere `f64`-Rust-Funktionen, vollständig getestet und mit AGC-Cross-References dokumentiert.

---

## Überblick

Die Navigation ist die mathematische Herzschicht von Comanche055. Jede Funktion entspricht einer oder mehreren AGC-Routinen aus dem CHIEFTAN-Segment. Da `agc-core` `no_std` ist, steht nur `libm` für Mathematik zur Verfügung (keine `std::f64`-Methoden direkt – aber da wir auf dem Host testen, können wir `f64` intrinsics via `#[cfg(test)]` nutzen; in `no_std` wird `libm` aufgerufen).

**Wichtig**: `libm`-Funktionen heißen `libm::sqrt(x)`, `libm::sin(x)` etc. – nicht `x.sqrt()`. In `#[cfg(test)]`-Blöcken, die auf dem Host laufen, darf `f64::sqrt()` verwendet werden, da `std` dort verfügbar ist.

---

## Abhängigkeiten zu anderen Tasks

- Task 3 (Newtypes) ✅ – `Vec3`, `Met`, `DeltaV`, `CduAngle` vorhanden
- Task 4 (Alarm) ✅ – `alarm::raise()` für ungültige Eingaben nutzbar
- Task 5 (Executive) ✅ – kein direkter Einfluss auf Navigation-Math

---

## Sub-Tasks

### Sub-Task 6.0 — Referenz-Skript & Testkonstanten anlegen

**Status**: `[x] done`

**Intent**: Das Python-Skript erzeugt hochwertige Referenzwerte für Kepler-Propagation, Hohmann-Transfer und Mond-Ephemeris. Die Werte werden direkt als `f64`-Konstanten in die Rust-Testdateien übertragen — das Skript selbst belegt nur die Herkunft.

**Expected Outcomes**:
- `tests/reference/generate_nav_refs.py` existiert und ist ausführbar
- Kommentare im Skript nennen Quellenangaben (poliastro / Vallado)
- Werte für Kepler-Test (Kreisbahn, 1 Periode), Hohmann-Δv und Mond-Position (GET = 0) dokumentiert

**Todo List**:
1. Verzeichnis `tests/reference/` anlegen
2. `generate_nav_refs.py` schreiben:
   - Kreisbahn LEO (r = 6 678 000 m, μ = 3.986004418e14) — Propagation eine volle Periode, Endposition = Startposition
   - Hohmann-Transfer LEO→GTO: Δv1, Δv2 analytisch berechnen
   - Mond-Position GET=0 (Näherung via einfachem Polynom-Modell oder JPL-Näherung aus Vallado)
3. Skript einmalig ausführen, Ausgabe sichten
4. Zahlenwerte als Kommentare in die jeweiligen Rust-Testdateien eintragen (noch leer, werden in späteren Sub-Tasks befüllt)

**Relevant Context**:
- `agc-core/Cargo.toml` — nur `libm` als Dependency
- Vallado "Fundamentals of Astrodynamics and Applications", Algorithm 8 (Kepler), Algorithm 61 (Hohmann)

---

### Sub-Task 6.1 — StateVector-Typ (`state_vector.rs`)

**Status**: `[x] done`

**Intent**: Der zentrale Datencontainer für eine Bahn-Zustandsgröße. Muss `Copy` und `no_std`-kompatibel sein.

**Expected Outcomes**:
- `agc-core/src/navigation/state_vector.rs` mit `StateVector { position: Vec3, velocity: Vec3, time: Met }`
- Dokumentiert: Einheiten (m, m/s, Sekunden seit TEPHEM), AGC-Kontext
- Einfacher Konstruktor und `#[derive(Copy, Clone, Debug)]`
- Kleiner Unit-Test: Konstruktion und Feldwerte

**Todo List**:
1. `agc-core/src/navigation/state_vector.rs` anlegen
2. `StateVector`-Struct definieren mit `Vec3` (aus `crate::types`) und `Met`
3. Dokumentations-Kommentar: Einheiten, Skalierung, AGC-Segment (CHIEFTAN)
4. `const ZERO`-Instanz anlegen
5. Unit-Test: Konstruktion, Feldwerte prüfen

**Relevant Context**:
- `agc-core/src/types/vec3.rs` — `Vec3` bereits vorhanden
- `agc-core/src/types/met.rs` — `Met` bereits vorhanden
- Plan-Abschnitt §agc-core/navigation/state_vector.rs

---

### Sub-Task 6.2 — Kepler-Propagation (`conic.rs`)

**Status**: `[x] done`

**Intent**: `kepler_step()` propagiert einen Zustandsvektor über eine Zeit-Δt entlang einer Kegelschnittbahn (Zweikörperproblem). Das ist die Kernfunktion aller Orbit-Berechnungen. `lambert()` löst das Lambert-Problem (Verbindungsbahn zwischen zwei Punkten in gegebener Zeit).

**Expected Outcomes**:
- `kepler_step(sv: StateVector, dt: f64, mu: f64) -> StateVector`
- `lambert(r1: Vec3, r2: Vec3, tof: f64, mu: f64) -> Option<(Vec3, Vec3)>` — gibt `None` zurück wenn keine Lösung
- Beide Funktionen mit AGC-Cross-Reference (`CONIC_SUBROUTINES.agc`, KEPLER-Routine)
- Test Stufe A: Kreisbahn, eine Periode → `|r1 - r0| < 1 m`
- Test Stufe A: Hohmann-Δv analytisch verifiziert
- Test Stufe B: Referenzwert aus `generate_nav_refs.py` als Kommentar

**Algorithmus-Hinweis für `kepler_step`**:
- Universal-Variable-Methode (Battin/Vallado): konvergiert für alle Kegelschnitttypen
- Stumpff-Funktionen `c2(ψ)`, `c3(ψ)` benötigt
- Newton-Iteration für universale Variable χ
- Kein Fallback auf Spezialfälle nötig (hyperbolisch via negative ψ abgedeckt)

**Algorithmus-Hinweis für `lambert`**:
- **Nur Single-Revolution** (ausreichend für alle Apollo-Manöver)
- Izzo/Lancaster-Blanchard-Methode, begrenzte Iterationstiefe, `None` bei Nicht-Konvergenz

**Todo List**:
1. `agc-core/src/navigation/conic.rs` anlegen
2. Hilfsfunktionen `stumpff_c2(psi: f64) -> f64` und `stumpff_c3(psi: f64) -> f64` implementieren
3. `kepler_step()` implementieren (Universal-Variable-Methode, Newton-Iteration)
4. `lambert()` implementieren (Lancaster-Blanchard oder Izzo, max. Iterationen begrenzt)
5. Tests: Kreisbahn eine Periode, Hohmann-Δv, Stufe-B-Wert als Kommentar

**Relevant Context**:
- `input/virtualagc/Comanche055/CONIC_SUBROUTINES.agc` — KEPLER, KEPPREP, DELTIME
- Vallado Algorithm 8 (Kepler Universal Variable), Algorithm 55 (Lambert)

---

### Sub-Task 6.3 — Orbital-Propagation mit J2 (`orbital.rs`)

**Status**: `[x] done`

**Intent**: Erweitert `kepler_step()` um den J2-Störterm der Erde (Abplattung) und Mond-Gravitation für Mondnähe. Dies entspricht `ORBITAL_INTEGRATION.agc` / KEPPREP.

**Expected Outcomes**:
- `propagate(sv: StateVector, dt: f64, body: CentralBody) -> StateVector`
- `CentralBody`-Enum: `Earth`, `Moon` (unterschiedliche μ und J2)
- J2-Beschleunigung als separater Term addiert (kein voll numerischer Integrator — einfache erste Ordnung)
- Test Stufe B: RAAN-Drift einer polaren LEO-Bahn über einen Orbit stimmt mit Referenz überein

**Todo List**:
1. `agc-core/src/navigation/orbital.rs` anlegen
2. `CentralBody`-Enum und zugehörige Konstanten (μ, J2, R_eq) definieren
3. `j2_acceleration(pos: Vec3, body: CentralBody) -> Vec3` — J2-Störbeschleunigung
4. `propagate()`: `kepler_step` + J2-Korrektur (Störungsansatz erster Ordnung)
5. Tests: J2-freie Propagation = kepler_step, J2-RAAN-Drift plausibel

**Relevant Context**:
- `input/virtualagc/Comanche055/ORBITAL_INTEGRATION.agc` — KEPPREP, KEPPROP
- Vallado §9.6 (J2 perturbation)

---

### Sub-Task 6.4 — Kalman-Filter / W-Matrix (`measurement.rs`)

**Status**: `[x] done`

**Intent**: Minimale Implementierung des Kalman-artigen Filters aus `MEASUREMENT_INCORPORATION.agc`. Nimmt eine Tracking-Messung (Abstand oder Winkel) und aktualisiert den Zustandsvektor-Bias (DELTAX). **Nur 3D-Fall** (Positionsvektor) — die volle 6D/9D-W-Matrix kommt in Task 8.

**Expected Outcomes**:
- `incorporate_measurement(obs: f64, h_row: Vec3, w: f64, delta_x: Vec3) -> Vec3` — gibt aktualisiertes DELTAX zurück (3D, Einheit m)
- Dokumentiert: H-Matrix-Zeile, W-Gewicht, Ausgabeeinheit m
- Test: Analytische Konvergenz — nach N Iterationen mit konsistenten Messungen Bias < Schwellwert

**Todo List**:
1. `agc-core/src/navigation/measurement.rs` anlegen
2. Skalare Kalman-Update-Formel implementieren (vereinfacht 3D)
3. Dokumentation: AGC-Cross-Reference MEASUREMENT_INCORPORATION.agc, INCORP1-Routine
4. Unit-Test: Konvergenz nach 5 konsistenten synthetischen Messungen

**Relevant Context**:
- `input/virtualagc/Comanche055/MEASUREMENT_INCORPORATION.agc` — INCORP1, W-Matrix

---

### Sub-Task 6.5 — Ephemeris (`ephemeris.rs`)

**Status**: `[x] done`

**Intent**: Analytische Näherungsformeln für Mond- und Sonnenposition relativ zur Erde als Funktion der Zeit. Entspricht `LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc` (LSPOS, LUNPOS, LUNVEL, SOLPOS). **Koeffizienten aus Vallado** (vereinfachte Polynome, ausreichend für Simulation).

**Expected Outcomes**:
- `moon_position(get_centisecs: f64) -> Vec3` — Mondposition in m (ECI, J2000)
- `moon_velocity(get_centisecs: f64) -> Vec3` — Mondgeschwindigkeit in m/s
- `sun_position(get_centisecs: f64) -> Vec3` — Sonnenposition in m (ECI, J2000)
- Polynomische Vallado-Näherung (degree 9 für Mond), keine JPL-Daten
- Test Stufe B: Mondposition bei GET=0 gegen Referenzwert aus `generate_nav_refs.py`

**Todo List**:
1. `agc-core/src/navigation/ephemeris.rs` anlegen
2. Polynomial-Koeffizienten als Konstanten definieren (Mond 9. Grad, Sonne vereinfacht)
3. `moon_position()`, `moon_velocity()`, `sun_position()` implementieren
4. Tests: Mond-Distanz zur Erde plausibel (384 400 km ± 10%), Stufe-B-Wert als Kommentar

**Relevant Context**:
- `input/virtualagc/Comanche055/LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc` — LSPOS, LUNPOS, SOLPOS, LUNVEL
- Vallado §5.4 (Approximate Planetary Positions)

---

### Sub-Task 6.6 — IMU-Ausrichtung (`alignment.rs`)

**Status**: `[x] done`

**Intent**: Berechnet die Gimbal-Winkel aus zwei Sternvektor-Messungen (TRIAD-Methode). Entspricht dem Kern von `INFLIGHT_ALIGNMENT_ROUTINES.agc`.

**Expected Outcomes**:
- `triad_align(v1_body: Vec3, v2_body: Vec3, v1_ref: Vec3, v2_ref: Vec3) -> Option<[[f64; 3]; 3]>` — gibt Rotationsmatrix zurück
- Test Stufe A: Bekannte Sternpaare (Arcturus + Vega) aus `star_tables.rs` → Identitätsmatrix bei korrekter Eingabe

**Todo List**:
1. `agc-core/src/navigation/alignment.rs` anlegen
2. TRIAD-Algorithmus implementieren
3. Tests: Identitäts-Rotation, orthogonale Matrix-Prüfung

**Relevant Context**:
- Shuster & Oh "Three-Axis Attitude Determination from Vector Observations" (1981)
- AGC: INFLIGHT_ALIGNMENT_ROUTINES.agc (INFLY-Segment, Comanche055)

---

### Sub-Task 6.7 — Powered-Flight-Subroutinen (`powered_flight.rs`)

**Status**: `[x] done`

**Intent**: Brennzeit-Integration für SPS-Burns. Gibt Δv und Brenndauer für gegebene Masse, Schub und gewünschtes Δv zurück. Entspricht `POWERED_FLIGHT_SUBROUTINES.agc`.

**Expected Outcomes**:
- `burn_time(delta_v: f64, isp: f64, thrust: f64, mass: f64) -> (f64, f64)` — (Brenndauer in s, Endmasse in kg)
- Verwendet Tsiolkovsky-Gleichung
- CDU-Trigonometrie: `cdu_to_sin_cos(angle: CduAngle) -> (f64, f64)`
- Test Stufe A: Bekannte SPS-Parameter (Isp = 314 s, Schub = 97,855 N) → analytisch prüfbar

**Todo List**:
1. `agc-core/src/navigation/powered_flight.rs` anlegen
2. `burn_time()` via Tsiolkovsky-Gleichung implementieren
3. `cdu_to_sin_cos()` für CDU-Winkelumrechnung (nutzt `CduAngle.to_radians()`)
4. Tests: Tsiolkovsky analytisch, CDU-Trigonometrie gegen bekannte Winkel

**Relevant Context**:
- `input/virtualagc/Comanche055/POWERED_FLIGHT_SUBROUTINES.agc` — CDUTRIG, CDUTRIG1
- `agc-core/src/types/cdu_angle.rs` — `CduAngle` mit `to_radians()`

---

### Sub-Task 6.8 — Restliche Navigationsmodule

**Status**: `[x] done`

**Intent**: Die drei verbleibenden Module sind klar abgegrenzt und weniger komplex: freie Flugzeit auf Kegelschnitt, Geokoordinaten-Umrechnung, Sterntabellen.

**Expected Outcomes**:
- `time_of_fall.rs`: `time_of_free_fall(sv: StateVector, r_terminal: f64, mu: f64) -> Option<f64>` — Flugzeit in Sekunden
- `lat_lon.rs`: `radius_to_lat_lon_alt(pos: Vec3, r_eq: f64) -> (f64, f64, f64)` — Breite (rad), Länge (rad), Höhe (m)
- `star_tables.rs`: Array `STARS: [Vec3; 37]` — Unit-Vektoren der 37 AGC-Navigationssterne (J2000)
- Tests: Bekannte Punkte (Nordpol → lat=π/2), Sternvektor-Magnitudes ≈ 1.0

**Todo List**:
1. `time_of_fall.rs` anlegen — Newton-Iteration auf Kepler-Bahn für Endradius
2. `lat_lon.rs` anlegen — sphärische Koordinatenumrechnung + Höhenformel
3. `star_tables.rs` anlegen — 37 Sternvektoren aus `STAR_TABLES.agc` abtippen
4. Tests für alle drei Module

**Relevant Context**:
- `input/virtualagc/Comanche055/TIME_OF_FREE_FALL.agc` — TFFCONIC, CALC/TFF
- `input/virtualagc/Comanche055/LATITUDE_LONGITUDE_SUBROUTINES.agc` — LAT-LONG
- `input/virtualagc/Comanche055/STAR_TABLES.agc` — 37 Sterndaten (3 Words à B-1)

---

### Sub-Task 6.9 — Modul-Integration & Abschluss

**Status**: `[x] done`

**Intent**: Alle Navigation-Module in `lib.rs` einbinden, Clippy-Clean herstellen, embedded-Build verifizieren, git commit.

**Expected Outcomes**:
- `agc-core/src/navigation/mod.rs` re-exportiert alle öffentlichen Typen
- `agc-core/src/lib.rs` enthält `pub mod navigation`
- `cargo test -p agc-core` grün
- `cargo clippy -p agc-core -- -D warnings` sauber
- `cargo build --target thumbv7em-none-eabihf -p agc-core` erfolgreich
- git commit: `Task 6 — Navigationsmathematik: navigation-Modul implementiert`

**Todo List**:
1. `agc-core/src/navigation/mod.rs` mit allen Sub-Modul-Deklarationen anlegen
2. `pub mod navigation` in `agc-core/src/lib.rs` eintragen
3. `cargo test -p agc-core` ausführen, alle Fehler beheben
4. `cargo clippy -p agc-core -- -D warnings` ausführen, alle Warnungen beheben
5. `cargo build --target thumbv7em-none-eabihf -p agc-core` prüfen
6. git commit

---

## Implementierungsreihenfolge

```
6.0 (Referenz-Skript) → 6.1 (StateVector) → 6.2 (Kepler/Lambert)
    → 6.3 (Orbital J2) → 6.4 (Kalman) → 6.5 (Ephemeris)
    → 6.6 (Alignment) → 6.7 (Powered Flight) → 6.8 (Rest)
    → 6.9 (Integration & Commit)
```

6.2 (conic) kann parallel zu 6.1 vorbereitet werden; 6.3–6.8 sind jeweils unabhängig voneinander, aber alle benötigen 6.1 (StateVector).

---

## Kritische Konventionen

| Regel | Begründung |
|---|---|
| `libm::sin(x)` statt `x.sin()` in Produktionscode | `no_std` ohne `std`-Float-Methoden |
| `Option` zurückgeben bei Nicht-Konvergenz | Kein `panic` im Flugcode (GOJAM) |
| Jede Funktion: AGC-Cross-Reference-Kommentar | AGENTS.md-Pflicht |
| Einheiten immer in Doc-Comment | AGENTS.md-Pflicht |
| `f64` überall (kein ones-complement) | AGENTS.md: Hardware-Artefakt nicht replizieren |
| Keine Structs ohne `Copy + Clone` wo sinnvoll | Statisches Speicher-Budget |
