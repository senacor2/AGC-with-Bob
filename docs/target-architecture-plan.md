# Zielsystem-Architektur — Plan

> **Scope**: Comanche055 (Command Module, Earth-to-Moon-and-back)  
> **Zielplattform**: STM32 Nucleo-F767ZI (Cortex-M7, `thumbv7em-none-eabihf`)  
> **Simulator**: Zweistufig — Stufe 1 `agc-core` allein, Stufe 2 vollständiger Terminal-Simulator

---

## Überblick

Das System besteht aus **drei Rust-Crates** in einem Cargo-Workspace:

| Crate | `no_std` | Zweck |
|---|---|---|
| `agc-core` | ja (kein Heap) | Vollständige Flug-Logik: Executive, Waitlist, Guidance-Programme, Navigationsmathematik. HAL als Traits. |
| `agc-physics` | ja (kein Heap) | Physikalisches Simulationsmodell: Trajektorie, IMU-Integration, VHF-Ranging-Simulation, Aerodynamik. Wiederverwendbar für Hardware-in-the-Loop. |
| `agc-sim` | nein (std) | Terminal-Simulator: ratatui/crossterm DSKY-TUI, HAL-Implementierungen über `agc-physics`, Integrationstests. |

**Stufe 1** (agc-core allein): Unit-Tests auf dem Host laufen gegen Mock-HAL-Implementierungen direkt im Test-Code. Kein I/O, kein Heap.

**Stufe 2** (agc-sim): `agc-sim` bindet `agc-core` und `agc-physics` ein, implementiert die HAL-Traits mit echten simulierten Sensordaten und zeigt eine interaktive DSKY-TUI.

---

## Komponentenschnitt

### agc-core

```
agc-core/
  src/
    lib.rs               # no_std, crate-Root, re-exportiert öffentliche API
    executive/           # Kooperativer Job-Scheduler (EXECUTIVE.agc)
      mod.rs
      job_table.rs       # Statische Job-Tabelle, Prioritäten 0–31
      waitlist.rs        # Zeitgesteuerte Tasks (WAITLIST.agc), max. 9 Einträge
    hal/                 # HAL-Trait-Definitionen (keine Implementierungen)
      mod.rs
      imu.rs             # Trait ImuHal: lese CDU-Winkel, PIPA-Pulse
      dsky.rs            # Trait DskyHal: zeige Register, lese Tasten
      rcs.rs             # Trait RcsHal: feuere Thruster-Pulse
      sps.rs             # Trait SpsHal: Gimbal-Befehle, Engine-On/Off
      vhf_ranging.rs     # Trait VhfRangingHal: optionaler VHF-Entfernungsmesser (V87/V88)
                         # Nicht Rendezvous-Radar — das ist LM-only (Luminary)
      telemetry.rs       # Trait TelemetryHal: Down/Uplink-Kanal
    alarm/               # Alarm & Abort (ALARM_AND_ABORT.agc)
      mod.rs             # alarm::raise(code), abort::gojam()
    restart/             # Phase-Table & Restart (PHASE_TABLE_MAINTENANCE.agc)
      mod.rs
      phase_table.rs     # Statische Phase-Table, Checkpoint-Pattern
    navigation/          # Navigationsmathematik (CHIEFTAN-Segment)
      mod.rs
      state_vector.rs    # StateVector-Newtype (Pos Vec3, Vel Vec3, Zeit)
      conic.rs           # Kepler, Lambert (CONIC_SUBROUTINES.agc)
      orbital.rs         # Zustandsvektor-Propagation (ORBITAL_INTEGRATION.agc)
      measurement.rs     # Kalman-Filter / W-Matrix (MEASUREMENT_INCORPORATION.agc)
      ephemeris.rs       # Mond/Erde-Orientierung (PLANETARY_INERTIAL_ORIENTATION.agc)
      alignment.rs       # IMU-Sternmessung (INFLIGHT_ALIGNMENT_ROUTINES.agc)
      powered_flight.rs  # Δv, Brennzeit (POWERED_FLIGHT_SUBROUTINES.agc)
      time_of_fall.rs    # Flugzeit auf Kegelschnitt (TIME_OF_FREE_FALL.agc)
      lat_lon.rs         # Geo-Koordinaten (LATITUDE_LONGITUDE_SUBROUTINES.agc)
      star_tables.rs     # 37 Navigationssterne (STAR_TABLES.agc)
    guidance/            # Guidance-Programme (TROUBLE + COMEKISS-Segmente)
      mod.rs
      p11.rs             # P11 Earth Orbit Insertion
      p20_25.rs          # P20–P25 Rendezvous Navigation
      p30_37.rs          # P30, P37 Mid-Course / Return-to-Earth
      p32_33.rs          # P32–P33, P72–P73 Rendezvous Manöver
      p38_39.rs          # P38–P39 Stable Orbit
      p40_47.rs          # P40–P47 SPS Burn
      p51_53.rs          # P51–P53 IMU Alignment
      p61_67.rs          # P61–P67 Entry / Reentry
      p37_70.rs          # P37, P70 Emergency Return
    dap/                 # Digital Autopilots (TVCDAPS-Segment)
      mod.rs
      tvc.rs             # TVC-DAP: Pitch/Yaw Triebwerks-Vektorsteuerung
      tvc_roll.rs        # TVC Roll DAP
      rcs_dap.rs         # RCS-CSM Digital Autopilot
      jet_select.rs      # Jet Selection Logic
      kalcmanu.rs        # KALCMANU Attitude Manöver
      gimbal_lock.rs     # Gimbal-Lock-Vermeidung
    display/             # DSKY-Protokoll (PINBALL_GAME*.agc)
      mod.rs
      verb_noun.rs       # Verb/Noun-Tabellen, Dispatch
      display_iface.rs   # DISPLAY_INTERFACE_ROUTINES.agc
    imu/                 # IMU-Betrieb (COMAID-Segment)
      mod.rs
      t4rupt.rs          # T4RUPT-Handler: CDU-Abfrage, Zustandsupdate
      compensation.rs    # IMU_COMPENSATION_PACKAGE.agc
      mode_switch.rs     # IMU_MODE_SWITCHING_ROUTINES.agc
    types/               # Gemeinsame Newtypes
      mod.rs
      vec3.rs            # Vec3 (f64 × 3), keine std-Deps
      angles.rs          # CduAngle (i16, 360°/2^15 rev), rad/deg-Konversion
      met.rs             # Met (Mission Elapsed Time, f64 Sekunden)
      delta_v.rs         # DeltaV (f64 m/s)
```

### agc-physics

```
agc-physics/
  src/
    lib.rs               # no_std-kompatibel, kein Heap
    trajectory.rs        # RK4-Integrator für Zwei-Körper + Störterme
    imu_model.rs         # Simuliertes IMU: integriert Beschleunigungen → PIPA-Pulse
    vhf_ranging.rs       # VHF-Ranging-Simulation: euklidische CM-LM-Distanz + Messrauschen
                         # Kein Radar-Modell — das CM nutzt VHF-Laufzeit, kein Radargerät
    gravity.rs           # Gravitationsmodell: Erde (J2), Mond (Punkt)
    bodies.rs            # Physikalische Konstanten: μ_Erde, μ_Mond, R_Erde
    aero.rs              # Aerodynamikmodell: Drag + Lift für CM-Reentry (Apollo-Kapselprofil)
    atmosphere.rs        # Atmosphärenmodell: NRLMSISE-00 vereinfacht (Dichte vs. Höhe)
```

### agc-sim

```
agc-sim/
  src/
    main.rs              # Entry Point: TUI starten, agc-core initialisieren
    hal/                 # HAL-Implementierungen über agc-physics
      imu_hal.rs         # ImuHal impl: agc-physics::ImuModel → CDU-Pulse
      dsky_hal.rs        # DskyHal impl: ratatui-Widgets ↔ agc-core Display-State
      rcs_hal.rs         # RcsHal impl: Pulse → agc-physics Kraft-Vektoren
      sps_hal.rs         # SpsHal impl: Gimbal-Befehle → agc-physics Schub
      vhf_ranging_hal.rs # VhfRangingHal impl: agc-physics::VhfRanging → VHFRANGE-Register
      telemetry_hal.rs   # TelemetryHal impl: Log-Datei / stdout
    tui/                 # Terminal-UI (ratatui/crossterm)
      mod.rs
      dsky_widget.rs     # Visuelles DSKY-Panel
      state_display.rs   # Zustandsvektor-Anzeige (Pos, Vel, Zeit)
      alarm_display.rs   # Alarm-Log-Panel
    sim_loop.rs          # Simulations-Hauptschleife: tick agc-core + agc-physics
  tests/
    integration/         # End-to-End-Szenario-Tests (VirtualAGC-Referenzdaten)
```

---

## Zentrale Anwendungsfälle

### UC-1: Navigationsmathematik verifizieren (Stufe 1)

**Akteur**: Entwickler / CI  
**Vorbedingung**: `agc-core` kompiliert für Host und `thumbv7em-none-eabihf`  
**Ablauf**:
1. Unit-Test initialisiert `StateVector` mit Referenz-Startbedingungen aus VirtualAGC-Run.
2. Test ruft `conic::kepler_step()` oder `orbital::propagate()` direkt auf.
3. Ergebnis wird gegen tabellierten VirtualAGC-Referenzwert verglichen (Toleranz: ≤ 1 m Position, ≤ 1 mm/s Geschwindigkeit).

**Erfolgskriterium**: Alle Tests grün auf Host. `cargo build --target thumbv7em-none-eabihf` erfolgreich.

---

### UC-2: Guidance-Programm im Mock-HAL testen (Stufe 1)

**Akteur**: Entwickler / CI  
**Vorbedingung**: HAL-Traits in `agc-core::hal` definiert  
**Ablauf**:
1. Unit-Test implementiert HAL-Traits als Stubs (z.B. `MockImu` liefert feste PIPA-Counts).
2. Executive wird mit einem P-Programm gestartet (z.B. P11).
3. Test prüft, ob das P-Programm korrekte DSKY-Displays anfordert und die richtigen DAP-Befehle ausgibt.

**Erfolgskriterium**: P-Programm durchläuft alle Phasen ohne Alarm; Outputs entsprechen Erwartungswerten.

---

### UC-3: Vollständige Mission im Terminal-Simulator (Stufe 2)

**Akteur**: Benutzer  
**Vorbedingung**: `agc-sim` kompiliert; `agc-physics` liefert Trajektorie  
**Ablauf**:
1. `cargo run -p agc-sim` startet TUI mit DSKY-Panel, Zustandsanzeige und Alarm-Log.
2. Benutzer drückt Tasten auf der simulierten DSKY (Verb/Noun-Eingaben).
3. `agc-core` Executive verarbeitet Eingaben, ruft Guidance-Programme auf.
4. `agc-physics` propagiert Trajektorie, liefert IMU-Pulse und VHF-Ranging-Abstände.
5. DAP schickt Thruster-Befehle → `agc-physics` aktualisiert Bahnzustand.
6. DSKY-TUI zeigt aktuellen Zustand in Echtzeit.

**Erfolgskriterium**: TLI-Burn (P11 → P37) läuft ohne Programmabbruch; angezeigte Trajektorie stimmt mit VirtualAGC-Referenz überein.

---

### UC-4: IMU-Alignment durchführen (Stufe 2)

**Akteur**: Benutzer  
**Ablauf**:
1. Benutzer startet P52 (IMU-Realignment) über DSKY.
2. `agc-core` fragt `ImuHal::read_star_angle()` ab (implementiert via `agc-physics`).
3. `alignment::triad()` berechnet neue Plattformorientierung.
4. `ImuHal::coarse_align()` + `fine_align()` werden aufgerufen.
5. DSKY zeigt Alignment-Fehler (NOUN 05).

**Erfolgskriterium**: IMU-Drift nach Alignment unter Schwellwert.

---

### UC-5: GOJAM-Restart-Stabilität (Stufe 1 + 2)

**Akteur**: Entwickler / CI  
**Ablauf**:
1. Test startet mehrstufige Berechnung (z.B. P40 SPS-Burn-Sequenz).
2. Nach Phase 2 wird ein simulierter Panic ausgelöst (`alarm::gojam()`).
3. System startet mit `fresh_start::restart()` neu.
4. Phase Table wird ausgelesen; Berechnung wird ab Phase 2 fortgesetzt.

**Erfolgskriterium**: Nach Restart korrekte Wiederaufnahme; Ergebnis identisch mit unterbrechungsfreiem Durchlauf.

---

### UC-6: Reentry-Steuerung (P61–P67) (Stufe 2)

**Akteur**: Benutzer  
**Ablauf**:
1. Benutzer startet P61 (Entry Preparation) via DSKY.
2. `agc-core` berechnet Entry-Zielparameter (Landezone, Eintrittswinkel).
3. Bei atmosphärischem Eintritt übernimmt `CM_ENTRY_DIGITAL_AUTOPILOT` die Lageregelung.
4. `agc-physics` simuliert aerodynamische Kräfte (vereinfacht: Ballistic-Drag-Modell).
5. DSKY zeigt G-Last, Entfernung zur Landezone (NOUN 60/61).

**Erfolgskriterium**: Landezone-Abweichung innerhalb spezifizierten Fehlerradius (Referenz: Apollo 11 Ist-Landung).

---

## Sub-Tasks

### Task 1 — Cargo-Workspace und Crate-Skelette aufsetzen

**Status**: `[x] done`

**Intent**: Fundament legen. Alle drei Crates müssen kompilieren (leer), Workspace-Konfiguration ist korrekt, `agc-core` baut für Host und `thumbv7em-none-eabihf`.

**Expected Outcomes**:
- `Cargo.toml` (Workspace) mit Members `agc-core`, `agc-physics`, `agc-sim`
- `agc-core/Cargo.toml`: `no_std`, `panic = "abort"`, kein `std`-Feature
- `agc-physics/Cargo.toml`: `no_std`-kompatibel (optional `std`-Feature für Tests)
- `agc-sim/Cargo.toml`: `std`, Abhängigkeiten `ratatui`, `crossterm`
- `cargo build --target thumbv7em-none-eabihf -p agc-core` grün
- `cargo build -p agc-sim` grün
- `cargo test` grün (keine Tests, aber kompiliert)

**Todo List**:
1. `Cargo.toml` (Workspace) anlegen
2. `agc-core` Crate-Skeleton mit `lib.rs` (`#![no_std]`)
3. `agc-physics` Crate-Skeleton mit `lib.rs`
4. `agc-sim` Crate-Skeleton mit `main.rs`
5. `.cargo/config.toml` mit Default-Target `thumbv7em-none-eabihf` für `agc-core`
6. Validierung: `cargo build`, `cargo clippy -- -D warnings`, `cargo test`

**Relevant Context**: `AGENTS.md` §Build and Test, §no_std Constraints

---

### Task 2 — HAL-Trait-Definitionen in agc-core

**Status**: `[x] done`

**Intent**: Die Hardware-Abstraktionsschicht als Trait-Grenzen definieren. Alle Guidance-Programme und DAP-Code referenzieren ausschließlich diese Traits — nie konkrete Hardware.

**Expected Outcomes**:
- `agc-core::hal` Modul mit Traits: `ImuHal`, `DskyHal`, `RcsHal`, `SpsHal`, `VhfRangingHal`, `TelemetryHal`
- Hinweis im Modulkommentar: Das CM hat keinen Rendezvous-Radar (LM-only). `VhfRangingHal` bildet das VHF-Ranging (V87/V88, `RANGERD`-Routine) ab — optional aktivierbar per Flag.
- Jeder Trait dokumentiert: Eingangseinheiten, Ausgabeeinheiten, AGC-Kanalreferenz
- `MockHal` Implementierungen in `#[cfg(test)]`-Blöcken für Unit-Tests

**Todo List**:
1. `agc-core/src/hal/mod.rs` mit Trait-Definitionen
2. Jeder Trait in eigenem Sub-Modul
3. `#[cfg(test)] mod mock` mit minimalen Test-Stubs
4. Clippy + Tests grün

**Relevant Context**: `docs/comanche055-architecture.md` §Hardware-Schnittstellen, `AGENTS.md` §Architecture Rules

---

### Task 3 — Gemeinsame Newtypes (types-Modul)

**Status**: `[ ] pending`

**Intent**: Physikalische Einheiten als Newtypes definieren, um Verwechslungen von Skalierungen zu verhindern. Jede Größe trägt ihre Einheit im Typ.

**Expected Outcomes**:
- `Vec3` (f64 × 3), `CduAngle` (i16, 1 LSB = 360°/2^15), `Met` (f64 Sekunden seit TEPHEM), `DeltaV` (f64 m/s)
- Alle Newtypes mit struct-level Doc-Comment: Einheit, Skalierung, AGC-Quelle
- Arithmetik-Implementierungen nur wo AGC-Fidelity es erlaubt

**Todo List**:
1. `agc-core/src/types/` Modul anlegen
2. `Vec3`, `CduAngle`, `Met`, `DeltaV` implementieren
3. Unit-Tests für Konversionen (rad ↔ CduAngle, Winkel-Wrapping)
4. Clippy + Tests grün

**Relevant Context**: `AGENTS.md` §Numeric Types, §Documentation Requirements

---

### Task 4 — Alarm & Phase Table

**Status**: `[ ] pending`

**Intent**: Grundinfrastruktur für Fehlerbehandlung und Restart-Sicherheit. Muss vor allen anderen agc-core-Modulen stehen, da jedes Modul `alarm::raise` verwenden kann.

**Expected Outcomes**:
- `agc-core::alarm`: `raise(code: u16)`, `gojam()` — kein `Result`, kein Heap
- `agc-core::restart::PhaseTable`: statische Tabelle, `set_phase(group, phase)`, `get_phase(group)`
- Dokumentiertes Checkpoint-Muster mit Beispiel
- Unit-Tests: Phase setzen, Restart simulieren, korrekte Phase lesen

**Todo List**:
1. `agc-core/src/alarm/mod.rs`
2. `agc-core/src/restart/phase_table.rs`
3. Unit-Tests für Phase-Table-Roundtrip
4. Clippy + Tests grün

**Relevant Context**: `docs/comanche055-architecture.md` §Alarm- und Restart-Mechanismus, `AGENTS.md` §Error Handling Split

---

### Task 5 — Executive & Waitlist

**Status**: `[ ] pending`

**Intent**: Den kooperativen Scheduler implementieren. Alle Guidance-Programme laufen als Jobs unter dem Executive.

**Expected Outcomes**:
- `agc-core::executive`: statische Job-Tabelle (max. 8 Jobs), Prioritätssteuerung, `schedule(priority, job_fn)`, `run_highest()`
- `agc-core::executive::waitlist`: statische 9-Einträge-Liste, `call_after(centiseconds, task_fn)`, Tick-Update
- Kein Heap, kein Blocking
- Unit-Tests: Job-Scheduling nach Priorität, Waitlist-Timing

**Todo List**:
1. `agc-core/src/executive/job_table.rs`
2. `agc-core/src/executive/waitlist.rs`
3. Unit-Tests: Prioritäts-Reihenfolge, Waitlist-Abfeuern nach N Ticks
4. Clippy + Tests grün

**Relevant Context**: `docs/comanche055-architecture.md` §Executive/Waitlist, `AGENTS.md` §Architecture Rules

---

### Task 6 — Navigationsmathematik

**Status**: `[ ] pending`

**Intent**: Die mathematische Kernschicht. Jede Funktion entspricht einer AGC-Routine und trägt die Pflicht-Cross-Reference.

**Expected Outcomes**:
- Alle Module unter `agc-core::navigation` implementiert
- Jede öffentliche Funktion dokumentiert: Eingangseinheiten, Ausgabeeinheiten, AGC-Quelldatei + Routine
- Math-Tests enthalten je einen Referenzwert aus VirtualAGC-Referenz-Run

**Todo List**:
1. `state_vector.rs`, `conic.rs` (Kepler, Lambert)
2. `orbital.rs` (Propagation mit J2-Term)
3. `measurement.rs` (Kalman-Filter, W-Matrix 6D/9D)
4. `ephemeris.rs`, `alignment.rs`, `powered_flight.rs`
5. `time_of_fall.rs`, `lat_lon.rs`, `star_tables.rs`
6. Tests mit VirtualAGC-Referenzwerten für jeden Schritt
7. Clippy + Tests grün

**Relevant Context**: `docs/comanche055-architecture.md` §Navigationsmathematik, `input/virtualagc/Comanche055/CONIC_SUBROUTINES.agc`

---

### Task 7 — DSKY-Protokoll & Display-Interface

**Status**: `[ ] pending`

**Intent**: Verb/Noun-Mechanismus und Display-State als agc-core-internes Protokoll. Keine Darstellung (die ist in agc-sim), nur Zustandsverwaltung und Dispatch.

**Expected Outcomes**:
- `agc-core::display`: Verb/Noun-Tabellen, Display-Register-State (R1/R2/R3), Anforderungs-Queue an `DskyHal`
- Keine TUI-Abhängigkeiten in agc-core

**Todo List**:
1. `agc-core/src/display/verb_noun.rs`
2. `agc-core/src/display/display_iface.rs`
3. Unit-Tests mit MockDsky
4. Clippy + Tests grün

**Relevant Context**: `docs/comanche055-architecture.md` §COMAID/PINBALL

---

### Task 8a — Guidance: Orbit-Programme (P11, P38–P39)

**Status**: `[ ] pending`

**Intent**: Einstieg in die Guidance-Programme — die Orbit-Programme sind am wenigsten komplex und haben keine Abhängigkeit auf Rendezvous- oder Entry-Logik. Validiert das Executive-Integration-Pattern für alle späteren P-Programme.

**Expected Outcomes**:
- `agc-core::guidance::p11` — Earth Orbit Insertion Überwachung
- `agc-core::guidance::p38_39` — Stable Orbit halten
- Beide Programme registrieren sich korrekt beim Executive
- Phase-Table-Logik (Restart-Stabilität) getestet
- Unit-Tests mit `MockHal`

**Todo List**:
1. `agc-core/src/guidance/mod.rs` mit gemeinsamem Trait `GuidanceProgram`
2. `p11.rs` implementieren
3. `p38_39.rs` implementieren
4. Unit-Tests: Normaldurchlauf + simulierter Restart in Phase 2
5. Clippy + Tests grün

**Relevant Context**: `docs/comanche055-architecture.md` §TROUBLE/COMEKISS, `input/virtualagc/Comanche055/P11.agc`, `STABLE_ORBIT_-_P38-P39.agc`

---

### Task 8b — Guidance: IMU-Alignment (P51–P53)

**Status**: `[ ] pending`

**Intent**: IMU-Neuausrichtung per Sternmessung. Baut auf `agc-core::navigation::alignment` und `agc-core::navigation::star_tables` auf. Schnittstelle zu `ImuHal::coarse_align()` + `fine_align()`.

**Expected Outcomes**:
- `agc-core::guidance::p51_53`
- P51 (Initial Platform Alignment), P52 (Realignment), P53 (Gyro Torquing) implementiert
- DSKY-Interaktion: NOUN 05 (Sternwinkel-Anzeige), NOUN 71 (Torquing-Winkel)
- Unit-Tests mit `MockImu` (synthetische Sternwinkel)

**Todo List**:
1. `p51_53.rs` implementieren
2. Integration mit `navigation::alignment` und `navigation::star_tables`
3. Unit-Tests: Alignment-Fehler unter Schwellwert nach synthetischer Messung
4. Clippy + Tests grün

**Relevant Context**: `input/virtualagc/Comanche055/P51-P53.agc`, `INFLIGHT_ALIGNMENT_ROUTINES.agc`

---

### Task 8c — Guidance: SPS-Burn (P40–P47)

**Status**: `[ ] pending`

**Intent**: SPS-Haupttriebwerk-Brennsequenz. Nutzt `navigation::powered_flight` für Δv-Berechnung und `dap::tvc` für Triebwerks-Vektorsteuerung während des Burns.

**Expected Outcomes**:
- `agc-core::guidance::p40_47`
- P40 (SPS Burn), P41 (Gimbal Motor Test), P47 (Trim Maneuver)
- Cutoff-Kriterien korrekt implementiert
- Phase-Table für Burn-Phasen (Ignition → Cutoff → Trim)
- Unit-Tests: Δv-Ziel erreicht, Cutoff zum richtigen Zeitpunkt

**Todo List**:
1. `p40_47.rs` implementieren
2. Integration mit `navigation::powered_flight`, `dap::tvc`
3. Unit-Tests mit synthetischem Burn (bekanntes Δv)
4. Clippy + Tests grün

**Relevant Context**: `input/virtualagc/Comanche055/P40-P47.agc`, `POWERED_FLIGHT_SUBROUTINES.agc`

---

### Task 8d — Guidance: Rendezvous-Programme (P20–P25, P30–P37, P32–P33)

**Status**: `[ ] pending`

**Intent**: Rendezvous-Navigations- und Manöver-Programme. Nutzen `navigation::measurement` (Kalman-Filter) für Radar-Tracking und `navigation::conic` für Manöver-Planung.

**Expected Outcomes**:
- `agc-core::guidance::p20_25` — Rendezvous Navigation (Radar-Tracking)
- `agc-core::guidance::p30_37` — Mid-Course Correction, Return-to-Earth
- `agc-core::guidance::p32_33` — CDH + TPI Rendezvous-Manöver
- Unit-Tests mit synthetischen Radar-Messungen

**Todo List**:
1. `p20_25.rs` implementieren (P20 Navigation, P22 Landmark Tracking)
2. `p30_37.rs` implementieren (P30 External ΔV, P37 Return-to-Earth)
3. `p32_33.rs` implementieren (P32 CDH, P33 TPI)
4. Unit-Tests mit MockRadar (synthetische Range/Range-Rate)
5. Clippy + Tests grün

**Relevant Context**: `input/virtualagc/Comanche055/P20-P25.agc`, `P30,P37.agc`, `P32-P33,_P72-P73.agc`

---

### Task 8e — Guidance: Entry-Programme (P61–P67, P37, P70)

**Status**: `[ ] pending`

**Intent**: Atmosphärischer Wiedereintritt und Notfall-Rückkehr. Nutzt `agc-physics::aero` für aerodynamische Kräfte und `dap::rcs_dap` für Entry-Lageregelung.

**Expected Outcomes**:
- `agc-core::guidance::p61_67` — Entry Preparation + Reentry Control
- `agc-core::guidance::p37_70` — Emergency Return
- NOUN 60/61 (G-Last, Landezonen-Entfernung) korrekt angezeigt
- Unit-Tests mit simulierten Entry-Bedingungen

**Todo List**:
1. `p61_67.rs` implementieren
2. `p37_70.rs` implementieren
3. Integration mit `dap::rcs_dap` (Entry DAP)
4. Unit-Tests: Landezone-Abweichung innerhalb Toleranz
5. Clippy + Tests grün

**Relevant Context**: `input/virtualagc/Comanche055/P61-P67.agc`, `REENTRY_CONTROL.agc`, `CM_ENTRY_DIGITAL_AUTOPILOT.agc`

---

### Task 9 — Digitale Autopiloten (DAP)

**Status**: `[ ] pending`

**Intent**: TVC-DAP, RCS-DAP und Jet Selection Logic implementieren.

**Expected Outcomes**:
- `agc-core::dap`: TVC, Roll, RCS, Jet-Selection, KALCMANU, Gimbal-Lock-Schutz
- DAP-Ausgaben gehen ausschließlich über `RcsHal` und `SpsHal`

**Todo List**:
1. `tvc.rs`, `tvc_roll.rs`
2. `rcs_dap.rs`, `jet_select.rs`
3. `kalcmanu.rs`, `gimbal_lock.rs`
4. Unit-Tests mit simulierten Lagefehlern
5. Clippy + Tests grün

**Relevant Context**: `docs/comanche055-architecture.md` §TVCDAPS

---

### Task 10 — agc-physics: Physikalisches Simulationsmodell

**Status**: `[ ] pending`

**Intent**: Trajectory-Propagation, IMU-Modell, Radar-Modell und Aerodynamik als eigenständiges, no_std-kompatibles Crate. Wiederverwendbar für spätere Hardware-in-the-Loop-Tests.

**Expected Outcomes**:
- `agc-physics::trajectory`: RK4-Integrator mit Gravitationsmodell (Erde J2, Mond Punkt)
- `agc-physics::imu_model`: integriert Beschleunigungen, liefert PIPA-Pulse (i16)
- `agc-physics::vhf_ranging`: euklidische CM-LM-Distanz + Messrauschen (kein Radargerät — VHF-Laufzeitmessung)
- `agc-physics::atmosphere`: Luftdichte vs. Höhe (vereinfachtes NRLMSISE-00-Modell)
- `agc-physics::aero`: Drag + Lift für Apollo-CM-Kapselprofil (Cd, Cl als Funktion von Machzahl und Anstellwinkel)
- Tests gegen analytische Keplerlösungen; Reentry-Test gegen Apollo-11-Ist-Trajektorie

**Todo List**:
1. `bodies.rs`, `gravity.rs` (Konstanten und Gravitationsmodell)
2. `trajectory.rs` (RK4-Integrator, reine Gravitation)
3. `atmosphere.rs` (Dichte-Tabelle, exponentielles Modell)
4. `aero.rs` (Drag/Lift-Koeffizienten, Kraftvektor-Berechnung)
5. `trajectory.rs` um Aerodynamik erweitern (Reentry-Pfad)
6. `imu_model.rs`
7. `vhf_ranging.rs` (CM-LM-Distanz aus Trajektorie, kein Radar-Modell nötig)
8. Tests: Kreisbahn-Energie-Erhaltung, Reentry-Landezone
9. Clippy + Tests grün

---

### Task 11 — agc-sim: HAL-Implementierungen

**Status**: `[ ] pending`

**Intent**: Brücke zwischen agc-physics und agc-core: die HAL-Traits aus agc-core werden hier mit echten Simulationsdaten implementiert.

**Expected Outcomes**:
- Alle HAL-Traits implementiert in `agc-sim::hal`
- `ImuHal` liefert PIPA-Pulse aus `agc-physics::ImuModel`
- `DskyHal` schreibt in TUI-State, liest Tasteneingaben aus TUI-Event-Queue
- `RcsHal`, `SpsHal` geben Kräfte an `agc-physics::Trajectory` weiter
- `VhfRangingHal` liest CM-LM-Distanz aus `agc-physics::VhfRanging` (nur aktiv wenn V87 gesetzt)

**Todo List**:
1. `agc-sim/src/hal/` mit allen HAL-Implementierungen
2. Verbindung `agc-physics` ↔ HAL ↔ `agc-core`
3. Integrationstests mit einfacher Kreisbahn
4. Clippy + Tests grün

---

### Task 12 — agc-sim: DSKY-TUI

**Status**: `[ ] pending`

**Intent**: Visuelle DSKY-Darstellung im Terminal mit ratatui/crossterm. Tasten werden auf Verb/Noun-Events gemappt.

**Expected Outcomes**:
- DSKY-Panel: PROG, VERB, NOUN, R1/R2/R3-Displays, Warn-LEDs
- Tastatur-Mapping: numerische Tasten, ENTR, CLR, PRO, KEY REL, RSET
- Alarm-Log-Panel (rechts)
- Zustandsvektor-Panel (Pos, Vel, Met)
- `cargo run -p agc-sim` startet die TUI

**Todo List**:
1. `agc-sim/src/tui/dsky_widget.rs`
2. `agc-sim/src/tui/state_display.rs`, `alarm_display.rs`
3. `agc-sim/src/sim_loop.rs` — Hauptschleife: tick agc-core + agc-physics + TUI-Render
4. Integrations-Smoke-Test: Start, V37E (P00), kein Crash
5. Clippy + Tests grün

---

### Task 13 — End-to-End Integrationstest: TLI-Sequenz

**Status**: `[ ] pending`

**Intent**: Der Beweis, dass das System als Ganzes funktioniert: P11 → P37 ohne menschliche Eingabe, Trajektorie gegen VirtualAGC-Referenz verifiziert.

**Expected Outcomes**:
- Automatisierter Integrationstest in `agc-sim/tests/integration/`
- Sequenz: Init → P11 (Orbit-Insertion) → P37 (Return-to-Earth) läuft durch
- Zustandsvektor am Ende: Abweichung von VirtualAGC-Referenz ≤ definierter Toleranz
- Test läuft in CI ohne menschliche Eingabe

**Todo List**:
1. Referenzdaten aus VirtualAGC-Szenario extrahieren
2. Integrations-Test-Harness aufsetzen (kein TUI, nur Sim-Loop + Assertions)
3. Test P11 → P37 schreiben
4. Toleranzwerte dokumentieren
5. Clippy + Tests grün

---

## Abhängigkeiten zwischen Tasks

```
Task 1 (Workspace)
  ├── Task 2 (HAL-Traits)
  ├── Task 3 (Newtypes)
  │     └── Task 4 (Alarm + Phase Table)
  │           └── Task 5 (Executive + Waitlist)
  │                 ├── Task 6 (Navigationsmathematik)
  │                 │     ├── Task 8a (Orbit-Programme P11, P38-P39)
  │                 │     ├── Task 8b (IMU-Alignment P51-P53)
  │                 │     ├── Task 8c (SPS-Burn P40-P47)
  │                 │     │     └── Task 9 (DAP)
  │                 │     ├── Task 8d (Rendezvous P20-P37, P32-P33)
  │                 │     └── Task 8e (Entry P61-P67, P37, P70)
  │                 │           └── [benötigt Task 9 + Task 10 aero]
  │                 └── Task 7 (DSKY-Protokoll)
  └── Task 10 (agc-physics inkl. Aerodynamik)
        └── Task 11 (HAL-Impl agc-sim)
              └── Task 12 (DSKY-TUI)
                    └── Task 13 (E2E-Test)

Task 9 (DAP) und Task 11 (HAL-Impl) sind Voraussetzung für Task 13.
Tasks 2, 3 und 10 können direkt nach Task 1 parallel beginnen.
Tasks 8a–8d sind unabhängig voneinander (nach Task 6 + Task 5).
Task 8e benötigt zusätzlich Task 9 und Task 10 (agc-physics::aero).
```
