# AGC Simulator — Demonstrierbare Use Cases

## Überblick

**Ziel**: Den AGC-Simulator mit einem vollständigen Satz demonstrierbarer Use Cases ausstatten, die die bereits implementierten Guidance-Programme zeigen. Jeder Use Case ist:

- **Interaktiv ausführbar**: Schritt-für-Schritt-Checkliste für die Crew (DSKY-Tasteneingaben)
- **Automatisiert verifizierbar**: Integrationstests, die denselben Ablauf maschinell prüfen
- **Vorbefüllbar**: Ein Scenario Loader setzt den physikalischen Startzustand (Position/Velocity/Zeit) und optionale Navigationsdaten, damit die Demo ohne lange Vorlaufzeit beginnt

**Scope**: Nur bereits implementierte Programme werden genutzt — P30, P40, P51/P52, P61–P67. Kein neuer Guidance-Code.

**Abgrenzung**: P37 (Return-to-Earth Emergency) und P62/P63/P65 sind im Dispatcher registriert, aber ihre internen Berechnungen sind Stubs. Sie werden in den Use Cases vorbereitet, aber nicht als vollständige Demos beschrieben.

---

## Architektur-Überblick

```
agc-sim/src/scenarios.rs      ← neu: Scenario-Loader (Textdatei → SimState)
agc-sim/src/main.rs           ← CLI-Flag --scenario <datei>
agc-sim/src/sim_loop.rs       ← '@'-Taste öffnet Dateinamen-Eingabefeld
agc-core/src/display/
  dispatcher.rs               ← accept_load() für N22 erweitern
docs/use-cases/               ← Markdown-Checklisten pro Use Case
agc-sim/tests/                ← neue Integrationstests pro Use Case
```

---

## Sub-Tasks

---

### Sub-Task 1 — Scenario Loader: Modul und Dateiformat

**Status**: [x] done — commit `2e5cd8d`

**Intent**: Ein neues Modul `agc-sim/src/scenarios.rs` anlegen, das Szenarien aus einfachen Textdateien (Key=Value) liest und den `SimState`-Bahnzustand setzt. Dies ist die Grundlage für alle Use Cases: Ohne vorbereitete Ausgangslage wären die Szenarien nicht reproduzierbar startbar.

**Expected Outcomes**:
- `scenarios.rs` definiert `ScenarioData { position, velocity, time_s }` und `load_scenario_file(path: &str) -> Result<ScenarioData, String>`
- Textdatei-Format: `pos_x_m`, `pos_y_m`, `pos_z_m`, `vel_x_ms`, `vel_y_ms`, `vel_z_ms`, `time_s` als Key=Value-Zeilen (Kommentare mit `#`)
- Vordefinierte Szenario-Konstanten als `.scenario`-Dateien unter `docs/scenarios/`: `leo_circ.scenario`, `tei_pre.scenario`, `entry_pre.scenario`
- `SimState::load_scenario(data: &ScenarioData)` überschreibt `state.traj` und setzt `state.ctx` auf Default zurück

**Todo List**:
1. `agc-sim/src/scenarios.rs` anlegen mit `ScenarioData`-Struct und `load_scenario_file()`
2. Parser implementieren: Key=Value-Zeilen, `#`-Kommentare überspringen, fehlende Felder als Fehler
3. `SimState::load_scenario(&ScenarioData)` in `sim_loop.rs` hinzufügen
4. Drei `.scenario`-Dateien unter `docs/scenarios/` anlegen (LEO, TEI-Pre, Entry-Pre)
5. `agc-sim/src/lib.rs` und `mod.rs` um `scenarios`-Modul ergänzen
6. Unit-Tests für Parser in `scenarios.rs` (gültige Datei, fehlendes Feld, ungültiger Wert)

**Relevant Context**:
- `agc-sim/src/sim_loop.rs:92–115` — `SimState::new()`, zeigt welche Felder zurückgesetzt werden
- `agc-physics/src/trajectory.rs:41–48` — `State { position, velocity, time_s }` ist der Typ für `traj`
- Format: einfache `no_std`-kompatible Strings — Parser sitzt nur in `agc-sim` (std erlaubt)

---

### Sub-Task 2 — CLI-Flag `--scenario` und '@'-Taste in der TUI

**Status**: [x] done — commit `2e5cd8d`

**Intent**: Den Scenario Loader aus Sub-Task 1 in der TUI und per CLI erreichbar machen. Das CLI-Flag ermöglicht automatisierte Tests und CI-Läufe; die '@'-Taste die interaktive Nutzung während einer Demo.

**Expected Outcomes**:
- `cargo run -p agc-sim -- --scenario docs/scenarios/tei_pre.scenario` startet den Simulator mit vorbeladenem Bahnzustand
- '@'-Taste in der TUI öffnet ein einzeiliges Texteingabefeld (Dateiname), ENTER lädt das Szenario, ESC bricht ab
- Fehlerhafte Pfade zeigen eine einzeilige Fehlermeldung im TUI-Footer (keine Panik, kein Absturz)
- Die TUI-Hilfszeile zeigt `@: load scenario` als Tastenbelegung

**Todo List**:
1. `main.rs` um `clap`-Argument (oder manuelles `std::env::args()`-Parsing) für `--scenario <path>` ergänzen
2. Bei gesetztem Flag: `load_scenario_file()` aufrufen und `SimState::load_scenario()` vor dem Event-Loop aufrufen
3. In `sim_loop.rs` (event_loop): '@'-Taste in `map_key()` als neue Variante erkennen (kein AGC-Keycode, nur TUI-intern)
4. `SimState` um `scenario_input: Option<String>` ergänzen (Eingabemodus für Dateinamen)
5. TUI-Render-Funktion: wenn `scenario_input.is_some()` → einzeiliges Eingabefeld über dem DSKY-Widget zeichnen
6. ENTER im Eingabemodus: `load_scenario_file()` → `SimState::load_scenario()` oder Fehlermeldung in `status_msg`
7. ESC im Eingabemodus: `scenario_input = None` setzen
8. TUI-Hilfszeile um `@ load scenario` ergänzen

**Relevant Context**:
- `agc-sim/src/sim_loop.rs:410–443` — `map_key()`, hier '@' ergänzen
- `agc-sim/src/sim_loop.rs:472–507` — `event_loop()`, hier Eingabemodus-Logik
- `agc-sim/src/tui/` — Render-Funktion, hier Eingabefeld-Widget
- `agc-sim/src/main.rs` — Einstiegspunkt für CLI-Flag

---

### Sub-Task 3 — `accept_load()` für N22 und N93 erweitern

**Status**: [x] done — commit `2e5cd8d`

**Intent**: `accept_load()` in `dispatcher.rs` unterstützt bisher nur N61 (Splashdown-Koordinaten). Für den IMU-Alignment-Use-Case (P51/P52) müssen N22 (THETAD — Ziel-Gimbal-Winkel) und N93 (Delta-Gyro-Winkel) ebenfalls per V25-Crew-Eingabe befüllt werden können. Ohne diese Erweiterung bleiben die Eingaben ohne Wirkung.

**Expected Outcomes**:
- `accept_load()` schreibt bei aktivem **N22** die drei Werte (Centideg → Rad) in `ctx.guidance.thetad`
- Nach `V25 N22 ENTR <OGA> ENTR <MGA> ENTR <IGA> ENTR` sind die Gimbal-Zielwinkel gesetzt
- `V06 N20 ENTR` zeigt danach dieselben Werte in R1–R3 an
- `accept_load()` schreibt bei aktivem **N93** die drei Werte (Millideg → Rad) in `ctx.guidance.thetad` (Gyro-Torquing-Approximation — identisches Zielfeld wie N22, da der Simulator thetad als Proxy nutzt)
- Nach `V25 N93 ENTR <ΔG1> ENTR <ΔG2> ENTR <ΔG3> ENTR` kann P52 den alignment_cmd-Pfad abschließen
- Unit-Tests verifizieren die Runde: N22-Eingabe → `thetad` → N20-Anzeige; N93-Eingabe → `thetad` → N93-Anzeige

**Todo List**:
1. In `dispatcher.rs:accept_load()` einen `22 =>`-Arm ergänzen analog zu N61 — Centideg (0.01°) → Radiant via `CENTIDEG_TO_RAD`
2. In `dispatcher.rs:accept_load()` einen `93 =>`-Arm ergänzen — Millideg (0.001°) → Radiant via `MILLIDEG_TO_RAD = π / 180_000.0`
3. Für beide Arme: `ctx.guidance.thetad` schreiben, Flash stoppen
4. AGC-Querverweise in Doc-Kommentare ergänzen: `PINBALL_NOUN_TABLES.agc` N22 (THETAD), N93 (DELTA GYRO ANGLES)
5. Test N22: `V25 N22` Sequenz → prüfe `ctx.guidance.thetad` und `V06 N20` nicht Blank
6. Test N93: `V25 N93` Sequenz → prüfe `ctx.guidance.thetad` und `V06 N93` zeigt Werte (wenn p51 gesetzt)

**Relevant Context**:
- `agc-core/src/display/dispatcher.rs:490–508` — accept_load(), hier N22- und N93-Arme einfügen
- `agc-core/src/context.rs:134` — `thetad: [f64; 3]`
- `agc-core/src/display/noun_resolver.rs:188–198` — N20 liest aus `thetad`, zeigt Blank wenn `[0,0,0]`
- `agc-core/src/display/noun_resolver.rs:490–508` — N93 liest aus `thetad`, benötigt zusätzlich `p51.is_some()`
- Konstante `CENTIDEG_TO_RAD = π / 18_000.0` bereits in accept_load() definiert (Zeile 498)
- Neue Konstante `MILLIDEG_TO_RAD = π / 180_000.0` lokal in accept_load() definieren

---

### Sub-Task 4 — Use Case UC-1: IMU-Alignment (P51 → P52)

**Status**: [x] done — commit `2e5cd8d`

**Intent**: Den vollständigen IMU-Alignment-Ablauf als Integrationstests und Crew-Checkliste dokumentieren und verifizieren. Dies ist der einfachste eigenständige Use Case ohne vorherige Guidance-Programme.

**Expected Outcomes**:
- Integrationstest `uc1_imu_alignment` in `agc-sim/tests/uc1_imu_alignment.rs`:
  - Szenario `leo_circ.scenario` laden
  - `V37E51E` → PROG=51 prüfen
  - `V41E` → V25 N22 blinkt (CoarseAlign-Prompt)
  - `V25 N22` Werte eingeben (thetad ≠ 0) → N20 zeigt CDU-Winkel
  - `V37E52E` → PROG=52 prüfen
  - `V42E` → V25 N93 blinkt (FineAlign-Prompt)
  - `V33E` (PRO) → alignment_cmd = FineAlign
  - `V06 N93 ENTR` → R1–R3 nicht Blank wenn thetad gesetzt
- Markdown-Checkliste `docs/use-cases/uc1-imu-alignment.md` mit Tasteneingabe-Sequenz

**Todo List**:
1. Szenariodatei `docs/scenarios/leo_circ.scenario` aus Sub-Task 1 nutzen (oder vorab anlegen)
2. Testdatei `agc-sim/tests/uc1_imu_alignment.rs` anlegen
3. Testschritte implementieren (press_keys-Muster aus dispatcher_integration.rs)
4. Assertions: `major_mode`, `active_program`, `thetad`, `alignment_cmd`, Display-Register
5. Markdown-Checkliste mit DSKY-Tasten-Notation erstellen

**Relevant Context**:
- `agc-core/src/guidance/p51_53.rs:49–61` — P51State-Felder
- `agc-sim/tests/dispatcher_integration.rs:37–42` — `press_keys()` Hilfsfunktion (ins gemeinsame Test-Modul extrahieren oder kopieren)
- `agc-core/src/display/dispatcher.rs:421–455` — CoarseAlign/FineAlign-Prompt-Logik

---

### Sub-Task 5 — Use Case UC-2: TEI-Vorbereitung (P30 → P40)

**Status**: [x] done — commit `2e5cd8d`

**Intent**: Den Trans-Earth-Injection-Burn als Use Case abbilden: P30 berechnet den Burn-Plan, P40 führt ihn aus. Das ist der kernige Guidance-Demo-Fall für interplanetare Manöver.

**Expected Outcomes**:
- Szenariodatei `docs/scenarios/tei_pre.scenario` mit Mondorbit-Bahnzustand (110 km Höhe, Kreisbahn, s. Scenario-Daten am Ende des Plans)
- Integrationstest `uc2_tei_burn.rs`:
  - Szenario laden → `V37E30E` → PROG=30
  - `V06 N44 ENTR` → HAPO/HPER/TFF in R1–R3 (nicht Blank, da P30State gesetzt)
  - `V06 N81 ENTR` → DVx/DVy/DVz in R1–R3
  - `V37E40E` → PROG=40, N40 zeigt TTOGO/VG/DVTOTAL
  - `V16 N40 ENTR` → Monitor-Modus aktualisiert R1–R3 pro Sekunde
- Markdown-Checkliste `docs/use-cases/uc2-tei-burn.md`

**Todo List**:
1. `tei_pre.scenario` aus den berechneten Werten (Mondorbit 110 km, MU_MOON = 4.904869e12, v ≈ 1630.3 m/s) anlegen — s. Abschnitt „Konkrete Szenario-Bahnvektoren"
2. Testdatei `agc-sim/tests/uc2_tei_burn.rs` anlegen
3. Scenario über `SimState::load_scenario()` in den Test integrieren
4. Testschritte: V37 N30 → V06 N44 → V06 N81 → V37 N40 → V16 N40
5. Assertions auf Display-Register (nicht Blank, plausible Werte)
6. Markdown-Checkliste erstellen

**Relevant Context**:
- `agc-core/src/guidance/p30_37.rs:135–168` — P30State-Felder; `p30_fn()` wird von `program_p30()` aufgerufen
- `agc-core/src/guidance/p40_47.rs:83–100` — P40State-Felder
- `agc-core/src/display/noun_resolver.rs:246–265` — N44 (HAPO/HPER/TFF)
- `agc-physics/src/bodies.rs` — MU_MOON, R_MOON (Physik-Konstanten)
- *The Apollo Guidance Computer*, Chapter 4 „Command Module entry" (PDF S. 357–374) — Entry-Referenzwerte

---

### Sub-Task 6 — Use Case UC-3: Entry (P61 → P64 → P67)

**Status**: [x] done — commit `2e5cd8d`

**Intent**: Den Wiedereintritts-Ablauf als Use Case abbilden: P61 bereitet den Eintritt vor (Splashdown-Ziel eingeben), P64 übernimmt die Constant-Drag-Phase, P67 zeigt den terminalen Anflug. Das ist der visuell reichhaltigste Use Case mit mehreren N-Nummern.

**Expected Outcomes**:
- Szenariodatei `docs/scenarios/entry_pre.scenario` mit Apollo-11-EI-Bahnzustand (122 km Höhe, v ≈ 10 900 m/s, γ = −6.5°, s. Scenario-Daten am Ende des Plans)
- Integrationstest `uc3_entry.rs`:
  - Szenario laden → `V37E61E` → PROG=61
  - `V25 N61 ENTR` → Splashdown-Koordinaten eingeben → `V06 N61 ENTR` zeigt sie
  - `V06 N60 ENTR` → GMAX/VPRED/GAMMAEI (wenn P61State.display gesetzt)
  - `V37E64E` → PROG=64, `V06 N69 ENTR` zeigt ROLLC/DL
  - `V37E67E` → PROG=67, `V06 N67 ENTR` zeigt RTOGO/LAT/LON
  - `V16 N66 ENTR` → Monitor-Modus für Guidance-Fehler
- Markdown-Checkliste `docs/use-cases/uc3-entry.md`

**Todo List**:
1. `entry_pre.scenario` aus Apollo-11-EI-Werten anlegen (pos_x = 6 493 000 m, vel_x = 10 834 m/s, vel_y = −1 238 m/s) — s. Abschnitt „Konkrete Szenario-Bahnvektoren"
2. Testdatei `agc-sim/tests/uc3_entry.rs` anlegen
3. Testschritte: V37 N61 → V25 N61 (Koordinaten) → V06 N60 → V37 N64 → V37 N67 → V16 N66
4. Assertions auf `splashdown_lat/lon`, `active_program`, Display-Register
5. Markdown-Checkliste erstellen

**Relevant Context**:
- `agc-core/src/guidance/p61_67.rs:286–297` — P61State
- `agc-core/src/display/dispatcher.rs:490–508` — accept_load() N61
- `agc-core/src/display/noun_resolver.rs:291–432` — N60–N74, N67

---

### Sub-Task 7 — Gemeinsame Test-Infrastruktur und Checklisten-Format

**Status**: [x] done — commit `2e5cd8d`

**Intent**: Die in Sub-Tasks 4–6 doppelt auftauchenden Test-Helfer (press_keys, scenario-load-Wrapper) in ein gemeinsames Test-Hilfsmodul auslagern, damit die Use-Case-Tests DRY bleiben. Außerdem ein einheitliches Checklisten-Format für alle Markdown-Dokumente festlegen.

**Expected Outcomes**:
- `agc-sim/tests/test_utils.rs` mit `press_keys()`, `load_scenario_state()` (gibt fertigen `SimState` zurück)
- Alle Use-Case-Tests nutzen `test_utils` statt eigener Kopien
- `docs/use-cases/README.md` beschreibt das Checklisten-Format (Konventionen: Tasten in `code`, Noun/Verb als `VNN`/`VNN`, erwartete Display-Werte in Tabelle)

**Todo List**:
1. `agc-sim/tests/test_utils.rs` anlegen mit `press_keys()` und `load_scenario_state()`
2. Bestehende `dispatcher_integration.rs`-Tests auf `test_utils` umstellen
3. `docs/use-cases/README.md` mit Format-Konventionen anlegen
4. `docs/use-cases/`-Verzeichnis mit Index-Übersicht der drei Use Cases anlegen

**Relevant Context**:
- `agc-sim/tests/dispatcher_integration.rs:37–42` — aktuelles `press_keys()`-Muster

---

### Sub-Task 8 — Automatische Zentralkörper-Auswahl im Physik-Integrator

**Status**: [x] done — commit `2e5cd8d`

**Intent**: [`sim_loop.rs:359`](agc-sim/src/sim_loop.rs:359) verwendet hartkodiert `StepParams::earth_ballistic()` — damit rechnet der RK4-Integrator immer mit μ_Erde und R_Erde. Für das `tei_pre.scenario` (Mondorbit, r ≈ 1 847 km) führt das dazu, dass die Gravitationsbeschleunigung um Faktor 81 zu groß ist und die Bahn in Sekunden kollabiert. Sub-Task 8 ergänzt eine automatische Zentralkörper-Erkennung: der Integrator wählt `Body::Moon` wenn der Bahnradius kleiner als eine Schwelle ist, die eindeutig in Mondnähe liegt.

**Expected Outcomes**:
- `SimState` speichert den aktiven `StepParams` (oder zumindest `Body`) als Feld, statt ihn pro Tick neu zu erzeugen
- Beim Laden eines Scenarios über `SimState::load_scenario()` wird `Body` automatisch aus `pos.magnitude()` abgeleitet:
  - `pos.magnitude() < 3_000_000 m` → `Body::Moon` (Mondnähe; Mondumlaufbahnen liegen bei 1 738–2 500 km Radius)
  - sonst → `Body::Earth`
- `body_radius_m` in `StepParams` wird entsprechend auf `R_MOON` bzw. `R_EARTH` gesetzt
- Der `tei_pre.scenario`-Mondorbit bleibt unter RK4 stabil (Energieerhaltung < 0.01 % über eine Umlaufzeit)
- Bestehende LEO- und Entry-Tests laufen weiterhin grün
- Ein neuer Test `moon_orbit_stable` in `agc-sim/tests/` oder `agc-physics/tests/` verifiziert Energieerhaltung für `Body::Moon` bei r = 1 847 km über eine volle Mondorbit-Periode (~2 h)

**Todo List**:
1. `SimState` um ein Feld `step_body: Body` ergänzen (Default: `Body::Earth`)
2. `SimState::load_scenario()` aus Sub-Task 1 um automatische `Body`-Ableitung erweitern: `if data.pos.magnitude() < 3_000_000.0 { Body::Moon } else { Body::Earth }`
3. In `sim_tick()`: `StepParams` aus `state.step_body` bauen statt `StepParams::earth_ballistic()` hartzukodieren — `body_radius_m` ebenfalls aus `state.step_body.radius()` ableiten
4. Test `moon_orbit_stable`: Moonorbit-Scenario laden, 100 Ticks propagieren, Energieerhaltung prüfen (relative Abweichung < 0.1 %)
5. Sicherstellen, dass Aerodynamik-Schwelle (< 120 km) für `Body::Moon` nicht fälschlicherweise feuert — `body_radius_m = R_MOON` in `StepParams` ist dafür bereits der richtige Mechanismus (Höhe = `|r| − R_MOON`)

**Relevant Context**:
- `agc-sim/src/sim_loop.rs:359` — Stelle, wo `StepParams::earth_ballistic()` hartkodiert ist
- `agc-physics/src/trajectory.rs:82–92` — `StepParams::earth_ballistic()`, hier analog `moon_ballistic()` ergänzen oder `body`-Parameter aus `SimState` übergeben
- `agc-physics/src/bodies.rs:52–87` — `Body`-Enum mit `mu()`, `j2()`, `radius()`
- Schwellwert 3 000 000 m (3 000 km): deutlich über dem maximalen Mondorbitradius (~2 500 km) und weit unter dem minimalen Erdorbitradius (~6 371 km) — kein Überlappungsrisiko

---

## Abhängigkeiten zwischen Sub-Tasks

```
Sub-Task 1 (Scenario Loader)
    ├──→ Sub-Task 2 (CLI + TUI '@')
    └──→ Sub-Task 8 (Zentralkörper-Auswahl)   ← parallel zu Sub-Task 2 umsetzbar
Sub-Task 3 (accept_load N22/N93)               ← unabhängig von Sub-Task 1
    ↓
Sub-Task 7 (Test-Infrastruktur)                ← nach Sub-Tasks 1, 2, 3, 8
    ↓
Sub-Task 4 (UC-1 IMU)
Sub-Task 5 (UC-2 TEI)                          ← alle drei parallel nach Sub-Task 7
Sub-Task 6 (UC-3 Entry)
```

Sub-Task 8 muss vor Sub-Task 5 (UC-2 TEI) abgeschlossen sein, da erst mit korrektem Zentralkörper die interaktive TEI-Demo sinnvoll ist.

---

## Konkrete Szenario-Bahnvektoren

### leo_circ.scenario — Erdorbit 400 km (Kreisbahn)

Quelle: `agc-sim/src/sim_loop.rs` (bestehender Startzustand); Physik-Konstanten aus `agc-physics/src/bodies.rs`.

```
r        = 6_371_000 + 400_000 = 6_771_000 m
v_circ   = sqrt(3.986004418e14 / 6_771_000) ≈ 7_668.3 m/s

pos_x_m  = 6771000.0
pos_y_m  = 0.0
pos_z_m  = 0.0
vel_x_ms = 0.0
vel_y_ms = 7668.3
vel_z_ms = 0.0
time_s   = 0.0
```

### tei_pre.scenario — Mondorbit 110 km vor TEI-Burn (Kreisbahn)

Quelle: Physik-Konstanten aus `agc-physics/src/bodies.rs` (MU_MOON = 4.904_869_5e12 m³/s², R_MOON = 1_737_400 m). Apollo-11-typische Höhe von ~110 km (vgl. Vallado, *Fundamentals of Astrodynamics*, §5.4).

```
r        = 1_737_400 + 110_000 = 1_847_400 m
v_circ   = sqrt(4.904869e12 / 1_847_400) ≈ 1_630.3 m/s

pos_x_m  = 1847400.0
pos_y_m  = 0.0
pos_z_m  = 0.0
vel_x_ms = 0.0
vel_y_ms = 1630.3
vel_z_ms = 0.0
time_s   = 0.0
```

Hinweis: Der Simulator verwendet ECI mit Erde als Zentrum. Für das Scenario wird der Mond-μ-Wert als Platzhalter genutzt — der Gravity-Driver wird später um Zentralkörper-Auswahl erweitert. Für Demo-Zwecke reicht es, den Orbit in korrekter Geometrie darzustellen.

### entry_pre.scenario — Wiedereintritt bei 122 km Höhe (Entry Interface, Apollo-11)

Quelle: Apollo-11-Referenzwerte für den Entry Interface (EI): v ≈ 10 900 m/s (inertial), γ = −6.5° unter Horizontale. Beschrieben in *The Apollo Guidance Computer: Architecture and Operation*, Chapter 4 „Command Module entry" (Book S. 341–358 · PDF S. 357–374).

```
γ        = −6.5° = −0.1134 rad
r        = 6_371_000 + 122_000 = 6_493_000 m

pos_x_m  = 6493000.0
pos_y_m  = 0.0
pos_z_m  = 0.0
vel_x_ms = 10834.0     (v × cos(γ) = 10_900 × cos(−0.1134))
vel_y_ms = -1238.0     (v × sin(γ) = 10_900 × sin(−0.1134))
vel_z_ms = 0.0
time_s   = 0.0
```

---

## Scenario-Dateiformat (Referenz)

```
# AGC Scenario File
# Einheiten: m (Position), m/s (Velocity), s (Mission Elapsed Time)

pos_x_m   = 6771000.0
pos_y_m   = 0.0
pos_z_m   = 0.0
vel_x_ms  = 0.0
vel_y_ms  = 7668.0
vel_z_ms  = 0.0
time_s    = 0.0
```

Optionale Felder (für spätere Erweiterungen, werden ignoriert wenn unbekannt):
```
thetad_0_deg = 0.0
thetad_1_deg = 0.0
thetad_2_deg = 0.0
```

---

## DSKY-Tastaturnotation (für Checklisten)

| Zeichen | Taste |
|---------|-------|
| `V`     | VERB  |
| `N`     | NOUN  |
| `E`     | ENTER |
| `P`     | PRO   |
| `R`     | RSET  |
| `C`     | CLR   |
| `+`     | PLUS  |
| `-`     | MINUS |
| `0`–`9` | Ziffern |

Beispiel: `V37E61E` = VERB 3 7 ENTER 6 1 ENTER (Programm 61 starten)
