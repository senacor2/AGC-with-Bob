# Task 7 — DSKY-Protokoll & Display-Interface: Implementierungsplan

> **Scope**: `agc-core/src/display/` (zwei Module: `verb_noun`, `display_iface`)  
> **Ziel**: Verb/Noun-Zustandsverwaltung und Display-Request-Dispatch als reine `no_std`-Logik — **keine TUI**, keine Darstellung, nur der Protokoll-Kern.

---

## Überblick

Das DSKY-Protokoll in Comanche055 besteht aus zwei Schichten:

1. **PINBALL** (`PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`): Verarbeitet Tastendrücke (Keycode → Verb/Noun-Zustandsautomat), baut den Display-Puffer (`DSPTAB`) auf und schreibt ihn via Kanal 10 raus.

2. **DISPLAY INTERFACE ROUTINES** (`DISPLAY_INTERFACE_ROUTINES.agc`): Prioritäts-Display-System — interne Programme fordern Displays an (Normal/Priority/Mark), die in einer Warteschlange verwaltet und sequenziell abgearbeitet werden.

In Rust übersetzen wir **nicht** den AGC-Bytecode, sondern die *Konzepte*:

| AGC-Konzept | Rust-Abstraktion |
|---|---|
| VERBREG / NOUNREG / MODREG | Felder in `DisplayState` |
| DSPTAB (11 Wörter, Kanal-10-Format) | `[u16; 11]` in `DisplayState` |
| DSPLOCK (Interlock) | `bool` in `DisplayState` |
| Display-Anforderungswarteschlange | `[DisplayRequest; 4]` statisches Array |
| NVSUB-Einstieg (internal call) | `fn request_display(...)` |
| Tastendruck-Verarbeitung | `fn process_key(...)` |
| Kanal-10-Serialisierung | `fn encode_channel10(...)` |

---

## Abhängigkeiten

- Task 2 (HAL) ✅ — `DskyHal`-Trait vorhanden mit `write_display_word`, `write_status_lights`, `read_keycode`
- Task 4 (Alarm) ✅ — `AlarmState` + `alarm::raise()` vorhanden
- Task 5 (Executive) ✅ — kein direkter Einfluss; Display-State wird durch Executive-Jobs aktualisiert

---

## Sub-Tasks

### Sub-Task 7.1 — Display-Zustandstypen (`verb_noun.rs`)

**Status**: `[x] done`

**Intent**: Alle Zustandstypen des PINBALL-Systems als saubere Rust-Typen definieren: Verb, Noun, Major-Mode (Prog), Display-Register R1/R2/R3, Status-Lights, Keycode. Kein Verhalten in diesem Modul — nur Datentypen und deren Konversionen.

**Expected Outcomes**:
- `VerbCode(u8)` — Newtype, gültige Werte 0–99 (AGC-Verben)
- `NounCode(u8)` — Newtype, gültige Werte 0–99 (AGC-Nomen)
- `MajorMode(u8)` — Major-Mode-Nummer (Programm-Code, MODREG, 0–99)
- `KeyCode(u8)` — Rohkeycode 0–31 (AGC-Tastaturkodierung)
- `DisplayRegister` — Enum: `Blank`, `Octal(u16)`, `Decimal(i32)`, `DecimalHighPrec(i32, i32)` — repräsentiert einen der drei 5-stelligen Anzeigebereiche R1/R2/R3
- `StatusLights` — bitfield-ähnliche Struktur: `key_release: bool`, `verb_noun_flash: bool`, `operator_error: bool`, `uplink_acty: bool`, `no_att: bool`, `gimbal_lock: bool`, `tracker: bool`, `alt: bool`, `vel: bool`
- Alle Typen: `Copy, Clone, Debug, PartialEq`
- Unit-Tests: Konstruktion, Gleichheitsvergleich

**Todo List**:
1. `agc-core/src/display/verb_noun.rs` anlegen
2. `VerbCode`, `NounCode`, `MajorMode`, `KeyCode` als Newtypes mit Dokumentation implementieren
3. `DisplayRegister`-Enum implementieren
4. `StatusLights`-Struct implementieren (Felder als `bool`)
5. Methoden: `StatusLights::to_channel11_bits() -> u16` — serialisiert Lampen-Status in AGC-Kanal-11-Format
6. Unit-Tests für alle Typen

**Relevant Context**:
- `input/virtualagc/Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc` pp. 307–389, Kanal-11-Bits
- Kanal-11-Bit-Belegung (aus `hal/dsky.rs`): KEY RELEASE=Bit5, VERB/NOUN FLASH=Bit6, OPERATOR ERROR=Bit7, UPLINK ACTY=Bit10

---

### Sub-Task 7.2 — Display-State & Kanal-10-Kodierung (`verb_noun.rs` Fortsetzung)

**Status**: `[x] done`

**Intent**: Der zentrale `DisplayState`-Struct hält den gesamten DSKY-Zustand. `encode_channel10()` serialisiert ihn in das AGC-Kanal-10-Format (DSPTAB), das dann via `DskyHal::write_display_word()` ausgegeben wird.

**Expected Outcomes**:
- `DisplayState` struct mit allen PINBALL-Erasable-Feldern:
  - `verb: VerbCode`, `noun: NounCode`, `major_mode: MajorMode`
  - `r1: DisplayRegister`, `r2: DisplayRegister`, `r3: DisplayRegister`
  - `lights: StatusLights`
  - `dsp_lock: bool` — DSPLOCK-Interlock (Keyboard gesperrt während interner Anforderung)
  - `flash_counter: u8` — T4RUPT-Zähler (0 = kein Blinken; > 0 wird pro T4RUPT dekrementiert; bei Erreichen von 0 toggled VERB/NOUN-FLASH-Bit)
  - `flash_state: bool` — aktueller An/Aus-Zustand des Blinkens
- `DisplayState::new() -> Self` — `const`-Konstruktor mit Default-Werten
- `fn t4rupt_tick(state: &mut DisplayState)` — wird pro T4RUPT (10 ms) aufgerufen; dekrementiert `flash_counter`, toggled `flash_state` und aktualisiert `lights.verb_noun_flash` beim Zählerüberlauf
- `fn write_to_dsky<H: DskyHal>(state: &DisplayState, dsky: &mut H)` — schreibt alle geänderten DSPTAB-Segmente via `write_display_word` und `write_status_lights`
- Kanal-10-Kodierung: Die AGC DSPTAB-Register kodieren 5-stellige 7-Segment-Daten als Nibble-Arrays. Für diese Implementierung wird eine vereinfachte Kodierung verwendet: `encode_r1r2r3_segment(reg: &DisplayRegister, pos: u8) -> u16` — gibt das Kanal-10-Wort für ein Register zurück
- Unit-Tests: `write_to_dsky` mit `MockDsky`, Prüfung dass `write_display_word` aufgerufen wird

**Kanal-10-Format-Hinweis**:
Das AGC-Kanal-10-Format ist ein 15-Bit-Serialisierungsprotokoll mit Segment-Gruppen. Für Task 7 implementieren wir eine **vereinfachte Kodierung**: Jedes Register wird als gepackter Binärwert ausgegeben (keine echte 7-Segment-Dekodierung — das ist Aufgabe von `agc-sim`). Die echte 7-Segment-Kodierung kommt in Task 12 (DSKY-TUI).

**Todo List**:
1. `DisplayState`-Struct in `verb_noun.rs` ergänzen (mit `flash_counter: u8` und `flash_state: bool`)
2. `DisplayState::new()` als `const fn` anlegen
3. `fn t4rupt_tick()` implementieren (Flash-Zähler dekrementieren, Bit togglen)
4. `fn write_to_dsky<H: DskyHal>()` implementieren
5. Vereinfachte Kanal-10-Kodierung implementieren (Rohdaten, keine 7-Segment)
6. Tests: `t4rupt_tick` toggled Flash-Bit nach N Ticks; `write_to_dsky` mit `MockDsky`

**Relevant Context**:
- `agc-core/src/hal/dsky.rs` — `DskyHal`-Trait + `MockDsky`
- `input/virtualagc/Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc` pp. 365–375 (DSPTAB-Format, DSPOUT-Routine)

---

### Sub-Task 7.3 — Tastendruck-Verarbeitung (`verb_noun.rs` Fortsetzung)

**Status**: `[x] done`

**Intent**: Der PINBALL-Zustandsautomat verarbeitet Tastendrücke und aktualisiert `DisplayState`. Im AGC ist das die CHARIN-Routine. Wir implementieren die Kernlogik: Ziffern sammeln, VERB/NOUN-Taste erkennen, ENTER/PROCEED/TERMINATE.

**Expected Outcomes**:
- `InputBuffer` — interner Puffer für laufende Verb/Noun-Eingabe (max. 2 Ziffern)
- `fn process_key(state: &mut DisplayState, buf: &mut InputBuffer, key: KeyCode, alarm: &mut AlarmState) -> KeyEvent`
- `KeyEvent`-Enum: `None`, `VerbNounSet`, `Enter`, `Proceed`, `Terminate`, `Recycle`, `Reset`, `OperatorError`
- AGC-Tastencode-Mapping:
  - 0–9: Ziffern (Keycode 0=0, 1=1, ... 9=9)
  - 17: VERB-Taste
  - 18: NOUN-Taste
  - 14: ENTER
  - 28: CLR (löscht laufende Eingabe)
  - 30: PRO (PROCEED = V33)
  - 24: KEY REL
  - 22: RSET (Reset)
  - 31: TPA (nur für Erweiterungen)
- Tests: VERB+Ziffer+Ziffer+ENTER → VerbCode gesetzt; falsche Reihenfolge → OperatorError; CLR löscht Puffer

**Todo List**:
1. `InputBuffer`-Struct in `verb_noun.rs` anlegen
2. `KeyEvent`-Enum anlegen
3. AGC-Keycode-Konstanten als `const u8`-Werte definieren
4. `process_key()` implementieren (Zustandsautomat)
5. Tests: Normaler Eingabefluss, CLR, Fehler-Erkennung

**Relevant Context**:
- `input/virtualagc/Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc` pp. 307–389 (CHARIN, KEYCODE-Tabelle)
- AGC Key-Codes aus `hal/dsky.rs`-Kommentaren

---

### Sub-Task 7.4 — Display-Request-System (`display_iface.rs`)

**Status**: `[x] done`

**Intent**: Interne Programme rufen `request_display()` auf — ähnlich wie `NVSUB` im AGC. Anforderungen werden in einer statischen Prioritäts-Queue gepuffert und durch `service_display_queue()` abgearbeitet (einmal pro Executive-Tick aufzurufen).

**Expected Outcomes**:
- `DisplayPriority`-Enum: `Normal`, `Priority` (Priority-Displays unterbrechen Normal-Displays)
- `DisplayRequest`-Struct: `verb: VerbCode`, `noun: NounCode`, `priority: DisplayPriority`
- `DisplayQueue` — **AGC-getreu: genau 2 Slots**: ein `priority_slot: Option<DisplayRequest>` + ein `normal_slot: Option<DisplayRequest>`
  - `push(req: DisplayRequest)` — Priority-Request belegt `priority_slot` (überschreibt ggf. vorherigen); Normal-Request belegt `normal_slot`
  - `pop() -> Option<DisplayRequest>` — Priority-Slot zuerst, dann Normal-Slot
  - `is_empty() -> bool`
- `fn request_display(state: &mut DisplayState, queue: &mut DisplayQueue, req: DisplayRequest)` — entspricht NVSUB im AGC
- `fn service_display_queue(state: &mut DisplayState, queue: &mut DisplayQueue)` — verarbeitet nächste Anforderung aus Queue, setzt VERBREG/NOUNREG
- Tests: Normal-Requests in Reihenfolge; Priority-Request verdrängt aktuellen Normal-Request; Queue-Voll-Verhalten

**Todo List**:
1. `agc-core/src/display/display_iface.rs` anlegen
2. `DisplayPriority`, `DisplayRequest`, `DisplayQueue` implementieren
3. `request_display()` und `service_display_queue()` implementieren
4. Tests: Prioritätslogik, FIFO, Queue-Overflow (silently dropped + Alarm)

**Relevant Context**:
- `input/virtualagc/Comanche055/DISPLAY_INTERFACE_ROUTINES.agc` pp. 1452–1484 (NVSUB, NVSBWAIT, MAKEPLAY, Priority-System)
- `input/virtualagc/Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc` — NVSUB-Aufrufkonventionen

---

### Sub-Task 7.5 — Modul-Integration & Abschluss

**Status**: `[x] done`

**Intent**: Alle Display-Module in `lib.rs` einbinden, Clippy-Clean, embedded-Build, git commit.

**Expected Outcomes**:
- `agc-core/src/display/mod.rs` mit `pub mod verb_noun; pub mod display_iface;` und Re-Exports
- `pub mod display;` in `agc-core/src/lib.rs`
- `cargo test -p agc-core` grün (alle bisherigen Tests + neue Display-Tests)
- `cargo clippy -p agc-core -- -D warnings` sauber
- `cargo build --target thumbv7em-none-eabihf -p agc-core` erfolgreich
- git commit: `Task 7 — DSKY-Protokoll: display-Modul implementiert`

**Todo List**:
1. `agc-core/src/display/mod.rs` anlegen
2. `pub mod display;` in `lib.rs` eintragen
3. `cargo test -p agc-core` — alle grün
4. `cargo clippy -p agc-core -- -D warnings` — sauber
5. `cargo build --target thumbv7em-none-eabihf -p agc-core` — grün
6. Plan-Datei auf `[x] done` setzen
7. git commit

---

## Implementierungsreihenfolge

```
7.1 (Typen) → 7.2 (DisplayState + Kanal-10) → 7.3 (Tastendruck-Automat)
    → 7.4 (Display-Request-Queue) → 7.5 (Integration)
```

7.1 und 7.2 sind eng gekoppelt (beide in `verb_noun.rs`). 7.3 baut auf 7.1/7.2 auf. 7.4 ist weitgehend unabhängig von 7.3 (nur `DisplayState` aus 7.2 benötigt).

---

## Kritische Konventionen

| Regel | Begründung |
|---|---|
| Kein `std`, kein Heap | `no_std`-Constraint |
| `DisplayQueue` mit 2 Slots (1 Priority + 1 Normal) | AGC-Treue + kein `Vec` |
| `MockDsky` aus `hal::dsky::mock` nutzen | Kein neuer Mock nötig |
| AGC-Keycodes als `const u8` | Lesbarkeit + keine magic numbers |
| Kanal-10 vereinfacht (keine 7-Segment) | 7-Segment kommt in Task 12 |
| `DskyHal` nur in `write_to_dsky` aufrufen | Trennung Display-State ↔ HAL |
| `flash_counter` als T4RUPT-Zähler, kein bool | AGC-Treue: Blinken ist hardware-getaktet |
| Kein `unwrap()`/`expect()` im Flugcode | GOJAM-Sicherheit |
