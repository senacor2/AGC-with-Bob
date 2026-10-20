# Task 14 — Verb-Dispatcher und Noun-Resolver

## Überblick

**Ziel**: Implementierung eines `VerbDispatcher`-Moduls und eines `NounResolver`-Moduls
in `agc-core`, die die bestehende DSKY-Infrastruktur (Task 7) mit den Guidance-Programmen
(Task 8a–8e) verbinden.

**Kontext**: `process_key()` liefert bereits `KeyEvent::VerbSet(v)` und `NounSet(n)`.
Die Guidance-Programme (P30, P37, P40–P47, P61–P67 usw.) sind mathematisch vollständig.
Bisher fehlt die Verdrahtung: wer empfängt `V37 ENTER 61 ENTER` und ruft `p61_step()` über
den Executive auf? Wer befüllt R1/R2/R3 wenn V06N60 aktiv ist?

**Scope**: Nur die im Rahmen von TEI, LOI und Reentry tatsächlich benötigten Verben und
Nouns werden implementiert. Vollständige 100-Verb- / 100-Noun-Abdeckung ist explizit
**kein Ziel**.

**Nicht in diesem Task**:
- Vollständige Simulation aller 99 Verbs / 99 Nouns
- DSKY-Uplink-Verarbeitung (Task separat)
- Load-Verben (V21–V25) für Crew-Dateneingabe über DSKY (nur TEI/LOI/Reentry-kritische)
- V69 (Cause Restart), V35 (Test Lights), V91 (Display Bank Sum)

**Missionsrelevanz-Priorisierung**:

| Priorität | Mission-Manöver | Kritische Programme | Kritische Verb/Noun-Paare |
|-----------|----------------|--------------------|-----------------------------|
| P0 | Reentry (P61–P67) | P61, P64, P67 | V06N60, V06N61, V06N63, V16N66, V16N67, V06N74, V16N69 |
| P1 | TEI / LOI (P30, P40) | P30, P37, P40 | V37N00, V06N33, V06N81, V16N40, V06N44, V06N42 |
| P2 | IMU Alignment (P51–P53) | P52, P51 | V37N00, V06N20, V06N92, V06N93, V41N20 |
| P3 | Orbit Verifikation | P11, P38 | V06N62, V06N44, V16N44 |
| P4 | Allg. Status-Display | — | V06N36 (AGC-Zeit), V06N43 (Lat/Lon/Alt) |

---

## Architektur

Der AGC-VERBFAN ist tabellenbasiert: `VERBFAN TC C(VERBTAB+VERBREG)`.
In Rust wird dieser Ansatz als **zwei kooperierende Tabellen** umgesetzt:

1. **`VerbDispatcher`** — bildet `VerbCode` auf einen Handler-Typ ab (Programm starten,
   Display setzen, Extended-Verb ausführen). Statische Dispatch-Tabelle als `[VerbAction; 100]`.

2. **`NounResolver`** — bildet `(VerbCode, NounCode)` auf die physikalischen Werte der
   aktuell laufenden Guidance-Programme ab und befüllt `DisplayState.r1/r2/r3`.
   Liest aus dem `GuidanceContext` (Aggregat aller Programmzustände).

**Kein zentraler `match`-Riese**: Handlers sind Funktionszeiger `fn(&mut AgcContext)`.
Kein Heap, keine Allokation — alles statisch.

**Datenfluss**:

```
Tastatur-Interrupt
    │
    ▼
process_key(keycode, &mut DisplayState, &mut InputBuffer)
    │ → KeyEvent::VerbSet(v)   → buf.verb = v, Flash starten
    │ → KeyEvent::NounSet(n)   → buf.noun = n
    │ → KeyEvent::Enter        → VerbDispatcher::dispatch(verb, noun, ctx)
    │ → KeyEvent::Proceed      → laufendem Programm signalisieren
    │ → KeyEvent::Terminate    → laufendem Programm signalisieren
    ▼
VerbDispatcher::dispatch(verb, noun, &mut AgcContext)
    │ → V37: ctx.executive.schedule(p##_job)
    │ → V06/V16: NounResolver::resolve(noun, ctx) → DisplayState.r1/r2/r3
    │ → V33/V34/V32: ProRstState setzen
    ▼
Executive::run_highest() — startet P-Programm-Job
    ▼
p##_step() — aktualisiert GuidanceContext
    ▼
NounResolver::resolve() — liest GuidanceContext → DisplayState
    ▼
write_to_dsky() — sendet Display-Word an HAL
```

---

## Sub-Task 1 — `AgcContext` und `GuidanceContext`

**Status**: `[x] done`

**Intent**: Zentrales Aggregat-Struct `AgcContext` anlegen, das alle bisher separat
übergebenen Zustände bündelt: `DisplayState`, `Executive`, `PhaseTable`, und den neuen
`GuidanceContext`. `GuidanceContext` ist ein Aggregat der Zustands-Structs aller
Guidance-Programme (z. B. `Option<P61State>`, `Option<P40State>`).  
Dieses Struct ermöglicht es dem Dispatcher, auf alle Programmzustände zuzugreifen.

**Expected Outcomes**:
- `agc-core/src/context.rs` mit `pub struct AgcContext` und `pub struct GuidanceContext`
- `GuidanceContext` enthält: `active_program: Option<u8>`, `p30: Option<P30Output>`,
  `p40: Option<P40State>`, `p61: Option<P61State>` (alle mission-kritischen Programme)
- `pub mod context` in `agc-core/src/lib.rs`
- Clippy sauber, `cargo build --target thumbv7em-none-eabihf -p agc-core` grün

**Todo List**:
1. `agc-core/src/context.rs` anlegen mit `GuidanceContext`-Struct-Definition
2. `GuidanceContext` mit allen TEI/LOI/Reentry-relevanten Programmzuständen befüllen
3. `AgcContext`-Struct anlegen: `display: DisplayState`, `exec: Executive`,
   `phase_table: PhaseTable`, `guidance: GuidanceContext`
4. `pub mod context` in `lib.rs` ergänzen
5. Clippy + embedded-Build validieren

**Relevant Context**:
- `agc-core/src/display/verb_noun.rs` — `DisplayState`
- `agc-core/src/executive/mod.rs` — `Executive`
- `agc-core/src/restart/phase_table.rs` — `PhaseTable`
- `agc-core/src/guidance/p40_47.rs`, `p61_67.rs`, `p30_37.rs`

---

## Sub-Task 2 — `VerbDispatcher`: Priorität P0 (Reentry V06/V16)

**Status**: `[x] done`

**Intent**: Kernstück für Reentry: Dispatch der Display-Verben V06 (einmalige Anzeige)
und V16 (Monitor = Wiederholanzeige). Diese Verben bilden zusammen mit den Reentry-Nouns
(N60–N69, N74) die DSKY-Anzeige für P61–P67.

**Expected Outcomes**:
- `agc-core/src/display/dispatcher.rs` neu angelegt
- `pub enum VerbAction`: `DisplayOnce`, `MonitorDisplay`, `ProgramChange(u8)`, `Noop`
- `pub struct VerbDispatcher` mit statischer Tabelle `[VerbAction; 40]` (V00–V39)
- `VerbDispatcher::dispatch(verb, noun, ctx)` implementiert für V06 und V16:
  - Setzt `ctx.display.verb` und `ctx.display.noun`
  - Ruft `NounResolver::resolve(noun, ctx)` auf (Sub-Task 4)
  - Für V16: setzt `ctx.display.flash_state = false` (kein Blinken bei Monitor)
- Tests: V06N60 → R1/R2/R3 aus `P61State` befüllt; V16N67 → Monitor-Flag gesetzt

**Todo List**:
1. `agc-core/src/display/dispatcher.rs` anlegen
2. `VerbAction`-Enum definieren
3. Statische Tabelle `VERB_TABLE: [VerbAction; 40]` für V00–V39
4. `dispatch()` für V06 (DisplayOnce) und V16 (MonitorDisplay) implementieren
5. `pub mod dispatcher` in `display/mod.rs` ergänzen
6. Unit-Tests für V06N60, V16N67
7. Clippy + embedded-Build

**Relevant Context**:
- `input/virtualagc/Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc` — VERBFAN
- `input/virtualagc/Comanche055/ASSEMBLY_AND_OPERATION_INFORMATION.agc` — Verb-Liste pp. 5–7
- `agc-core/src/display/verb_noun.rs` — `VerbCode`, `DisplayState`

---

## Sub-Task 3 — `VerbDispatcher`: V37 (Programm-Wechsel), V25/N61 (Load), V33/V34/V32

**Status**: `[ ] pending`

**Intent**: V37 (Change Program) ist das Herzstück jedes Missionsstarts. Astronaut
drückt `VERB 37 ENTER [61] ENTER` um P61 zu starten. Der Dispatcher muss daraufhin
den Executive-Job für das entsprechende Guidance-Programm einplanen und `MODREG` setzen.
Außerdem: **V25/N61** (Load 3 Komponenten → Ziel-Koordinaten für Reentry) — ohne
eingebbare Zielkoordinaten ist P61 operativ sinnlos.
Sowie V33 (Proceed), V34 (Terminate), V32 (Recycle) als Crew-Antworten auf VERB/NOUN-Blinken.

**Expected Outcomes**:
- `VerbDispatcher::dispatch()` erweitert für V37:
  - Liest Noun als Programm-Nummer (N = major mode)
  - Mappt auf `fn job(&mut AgcContext)` aus statischer Tabelle `PROGRAM_TABLE`
  - Ruft `ctx.exec.schedule(priority, job)` auf
  - Setzt `ctx.display.major_mode = MajorMode::new(program_nr)`
  - Stoppt Flash
- V25/N61: Crew gibt LAT(SPL), LNG(SPL), HEADSUP über R1/R2/R3 ein
  - `dispatch()` für V25 → setzt `display.verb = V25`, startet Flash (Eingabeaufforderung)
  - `KeyEvent::Enter` mit V25 aktiv → liest R1/R2/R3 aus `InputBuffer`, schreibt in
    `ctx.guidance.p61_target` (Lat [rad], Lon [rad], Headsup [bool])
- V33 (Proceed) → setzt `ctx.guidance.proceed_flag = true`
- V34 (Terminate) → setzt `ctx.guidance.terminate_flag = true`
- V32 (Recycle) → setzt `ctx.guidance.recycle_flag = true`
- Statische Tabelle `PROGRAM_TABLE: [Option<fn(&mut AgcContext)>; 100]` (sparse, nur
  mission-kritische eingetragen: P30, P37, P40, P51, P52, P61, P62, P63, P64, P65, P67)
- Tests:
  - V37 + N61 + Enter → P61-Job in Executive eingeplant; MODREG = 61
  - V25 + N61 + Ziffern + Enter → `p61_target` mit korrekten Werten befüllt
  - V33 → `proceed_flag` gesetzt
  - V34 → `terminate_flag` gesetzt

**Todo List**:
1. `PROGRAM_TABLE` statische Tabelle anlegen (sparse, alle nicht-kritischen = `None`)
2. `dispatch()` für V37 mit Tabellen-Lookup implementieren
3. Signalisierungs-Flags in `GuidanceContext` ergänzen (`proceed`, `terminate`, `recycle`)
4. `p61_target` Felder in `GuidanceContext`: `splashdown_lat: f64`, `splashdown_lon: f64`, `heads_up: bool`
5. V25 in `dispatch()` implementieren (Flash starten, Enter-Handler für N61-Eingabe)
6. V33/V34/V32 in `dispatch()` implementieren (Flag setzen)
7. Unit-Tests: V37N61-Sequenz, V25N61-Eingabesequenz, V33/V34
8. Clippy + embedded-Build

**Relevant Context**:
- `input/virtualagc/Comanche055/FRESH_START_AND_RESTART.agc` — V37-Logik pp. 819–920
- `input/virtualagc/Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc` — V25/V33/V34/V32
- `input/virtualagc/Comanche055/ASSEMBLY_AND_OPERATION_INFORMATION.agc` — N61 (LAT/LNG/HEADSUP)
- `agc-core/src/executive/mod.rs` — `Executive::schedule()`
- `agc-core/src/display/verb_noun.rs` — `InputBuffer`

---

## Sub-Task 4 — `NounResolver`: Priorität P0 (Reentry N60–N69, N74)

**Status**: `[ ] pending`

**Intent**: Der Noun-Resolver liest die physikalischen Werte aus dem `GuidanceContext`
und befüllt `DisplayState.r1/r2/r3` mit skalierten Dezimalwerten. Priorität haben die
Reentry-Nouns, da P61–P67 mathematisch fertig ist und nur die Anbindung fehlt.

**Skalierung**: Alle Anzeigen und Eingaben erfolgen in **SI-Einheiten** (m, m/s, deg).
Die historische Verwendung von US-Einheiten (ft/sec, NM) ist obsolet.
Interne Bogenmaß-Winkel (rad) werden in Grad umgerechnet. Keine NM, keine ft/sec.

**Expected Outcomes**:
- `agc-core/src/display/noun_resolver.rs` neu angelegt
- `pub fn resolve(noun: NounCode, ctx: &AgcContext) -> [DisplayRegister; 3]`
- Implementiert für: N40, N44, N60, N61, N63, N64, N66, N67, N68, N69, N74, N80, N81
- Einheiten: Geschwindigkeiten in m/s (ganzzahlig), Distanzen in km (ganzzahlig), Winkel in 0.01°
- `DisplayRegister::Decimal(i32)` befüllt (keine Octal-Darstellung für mission-kritische Nouns)
- Tests: `resolve(N60, ctx_with_p61_state)` → R1 = GMAX (0.01 m/s²), R2 = VPRED in m/s,
  R3 = GAMMAEI in 0.01°

**Todo List**:
1. `agc-core/src/display/noun_resolver.rs` anlegen
2. `resolve()` für Reentry-Nouns N60–N69, N74 implementieren
3. Skalierungsfunktionen: `rad_to_centideg(f64) -> i32`, `mps_to_display(f64) -> i32`, `km_to_display(f64) -> i32`
4. N40 (TIG/VG), N44 (HAPO/HPER/TFF), N81 (Δv-LV) für TEI/LOI ergänzen
5. `pub mod noun_resolver` in `display/mod.rs`
6. Unit-Tests mit bekannten Referenzwerten
7. Clippy + embedded-Build

**Relevant Context**:
- `input/virtualagc/Comanche055/ASSEMBLY_AND_OPERATION_INFORMATION.agc` — Noun-Liste pp. 8–13
- `input/virtualagc/Comanche055/PINBALL_NOUN_TABLES.agc` — SF-Konstanten
- `agc-core/src/display/verb_noun.rs` — `DisplayRegister`
- `agc-core/src/guidance/p61_67.rs` — `P61State`, `P64State`, `P67State`
- `agc-core/src/guidance/p30_37.rs` — `P30Output`
- `agc-core/src/guidance/p40_47.rs` — `P40State`

---

## Sub-Task 5 — V41 (Coarse Align) und IMU-Alignment-Verben (Priorität P2)

**Status**: `[ ] pending`

**Intent**: Für P51/P52 (IMU Alignment) werden Extended Verbs V41 (Coarse Align CDU),
V42 (Fine Align IMU), und Display-Nouns N20 (ICDU Angles), N92 (New Optics Angles),
N93 (Delta Gyro Angles) benötigt.

**Expected Outcomes**:
- `VerbDispatcher` erweitert um Extended Verbs (V40+): `VERB_TABLE` auf Größe 60 erweitert
  oder zweite Tabelle `EXT_VERB_TABLE: [VerbAction; 60]` (V40–V99)
- V41N20: Coarse-Align → `ctx.guidance.alignment_cmd = CoarseAlign`
- V42: Fine-Align → `ctx.guidance.alignment_cmd = FineAlign`
- `NounResolver` erweitert: N17 (CPHIX), N20 (CDU-Winkel), N22 (THETAD), N92, N93
- Tests: V41N20 → alignment_cmd korrekt gesetzt

**Todo List**:
1. `VERB_TABLE` / `EXT_VERB_TABLE` für V40–V45 ergänzen
2. `AlignmentCommand`-Enum in `GuidanceContext`
3. V41/V42-Handler implementieren
4. `NounResolver` für N17, N20, N22, N92, N93 ergänzen
5. Unit-Tests
6. Clippy + embedded-Build

**Relevant Context**:
- `input/virtualagc/Comanche055/EXTENDED_VERBS.agc` — VBZERO, VBCOARK, IMUFINEK
- `agc-core/src/guidance/p51_53.rs`

---

## Sub-Task 6 — Simulator-Integration und End-to-End-Test

**Status**: `[ ] pending`

**Intent**: `agc-sim` verdrahtet den Dispatcher mit dem bestehenden DSKY-TUI-Loop.
Keyboard-Events aus dem TUI werden über `process_key()` → `VerbDispatcher::dispatch()`
geleitet. End-to-End-Szenario: P61-Sequenz im Simulator durchspielen.

**Expected Outcomes**:
- `agc-sim/src/sim_loop.rs`: `VerbDispatcher` und `NounResolver` eingebunden
- DSKY-TUI zeigt nach `V37 ENTER 61 ENTER`: PROG = 61, VERB = 06, NOUN = 60, R1–R3 = Reentry-Daten
- V16N67 Monitor-Modus: R1–R3 werden pro T4RUPT-Tick aktualisiert
- Integration-Test in `agc-sim/tests/` oder `agc-core/tests/`: TEI-Sequenz
  (V37→P30→Δv-Anzeige→V37→P40→Burn-Monitor)
- `cargo fmt && cargo clippy -- -D warnings && cargo test` grün
- `cargo build --target thumbv7em-none-eabihf -p agc-core` grün

**Todo List**:
1. `AgcContext` in `sim_loop.rs` anlegen und initialisieren
2. Keyboard-Event-Loop: `process_key()` → `VerbDispatcher::dispatch(ctx)`
3. T4RUPT-Tick: Monitor-Modus (`V16`) → `NounResolver::resolve()` → Display aktualisieren
4. Integration-Test: P61-Startsequenz, Display-Werte prüfen
5. Optional: TEI/LOI-Sequenz als separater Integration-Test
6. Plan-Status aktualisieren, Git-Commit

**Relevant Context**:
- `agc-sim/src/sim_loop.rs`
- `agc-sim/src/hal/dsky.rs` — `SimDsky`
- `agc-sim/src/tui/dsky_widget.rs`

---

## Priorisierung: Wichtigkeit nach Missionsdurchführbarkeit

### Tier 1 — Reentry ohne diese Verben/Nouns nicht möglich (P61–P67)

| Verb | Noun | Bedeutung [SI-Einheiten] | P-Programm |
|------|------|--------------------------|------------|
| V06 | N60 | GMAX [m/s²], VPRED [m/s], GAMMAEI [0.01°] | P61 (Entry Prep) |
| V06 | N61 | LAT(SPL) [0.01°], LNG(SPL) [0.01°], HEADSUP [±1] | P61/P62 (Target input) |
| V06 | N63 | RTGO [km], VIO [m/s], TFE [s] | P61 (Guidance outputs) |
| V37 | N00 | Change Program → P61 | Dispatcher |
| V06 | N74 | ROLLC [0.01°], VI [m/s], D [m/s²] | P64 (Constant-drag) |
| V16 | N69 | BETA [0.01°], DL [m/s²], VL [m/s] | P64 (Monitor) |
| V16 | N66 | ROLLC [0.01°], XRNGERR [km], DNRNGERR [km] | P64/P65 (Guidance) |
| V16 | N67 | RTOGO [km], LAT [0.01°], LONG [0.01°] | P67 (Terminal) |

### Tier 2 — TEI/LOI ohne diese Verben/Nouns nicht möglich (P30, P37, P40)

| Verb | Noun | Bedeutung | P-Programm |
|------|------|-----------|------------|
| V37 | N00 | Change Program → P30/P40 | Dispatcher |
| V06 | N33 | TIG (Ignition Time) | P30 (External ΔV) |
| V06 | N81 | Δv (LV) | P30 |
| V16 | N40 | TIG/VG/DVAC (Burn Monitor) | P40 (SPS Burn) |
| V06 | N44 | HAPO, HPER, TFF | P30/P37 |
| V06 | N42 | APOGEE, PERIGEE, ΔV | P37 |
| V33 | — | Proceed (crew accept) | P30/P40 |
| V34 | — | Terminate | Alle |

### Tier 3 — IMU Alignment (P51–P52)

| Verb | Noun | Bedeutung | P-Programm |
|------|------|-----------|------------|
| V37 | N00 | Change Program → P52 | Dispatcher |
| V06 | N20 | ICDU Angles | P52 |
| V41 | N20 | Coarse Align | P52 |
| V42 | — | Fine Align IMU | P52 |
| V06 | N92 | New Optics Angles | P52 |
| V06 | N93 | Delta Gyro Angles | P52 |

### Tier 4 — Allgemeine Status-Verben

| Verb | Noun | Bedeutung |
|------|------|-----------|
| V06 | N36 | AGC-Zeit (HRS/MIN/SEC) |
| V06 | N43 | Lat/Lon/Alt |
| V06 | N62 | VI, HDOT, H (Orbit-Überwachung) |
| V16 | N44 | HAPO/HPER/TFF Monitor |

---

## Abhängigkeiten

```
Sub-Task 1: AgcContext
    └─► Sub-Task 2: VerbDispatcher V06/V16
    │       └─► Sub-Task 4: NounResolver Reentry
    └─► Sub-Task 3: VerbDispatcher V37/V33/V34
    └─► Sub-Task 5: Extended Verbs V41/V42
            └─► Sub-Task 6: Simulator-Integration
```

Sub-Tasks 2 und 3 können nach Sub-Task 1 parallel implementiert werden.
Sub-Task 4 hängt von Sub-Task 2 (Display-Verb-Dispatch) ab.
Sub-Task 5 hängt von Sub-Tasks 2 und 3 ab.
Sub-Task 6 hängt von allen vorherigen ab.
