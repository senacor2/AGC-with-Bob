# Plan: Architecture-Cleanup — Domain-Logik aus agc-sim nach agc-core verlagern

## Überblick

Fünf Stellen in `agc-sim` enthalten AGC-Domain-Logik, die architektonisch in
`agc-core` gehört. Die Grundregel des Projekts lautet:

> Domain logic (navigation, guidance, DAP) is strictly separate from the HAL
> and the Executive scheduler.

Konkret: alles, was auf der echten STM32-Zielplattform ohne `sim_loop` laufen
muss, darf nicht in `agc-sim` implementiert sein. Die fünf Probleme sind:

| # | Fundstelle | Beschreibung | Schwere |
|---|-----------|--------------|---------|
| A1 | `sim_loop.rs:247` | V16-Monitor-Tick-Steuerung im sim_loop | bricht auf Embedded |
| A2 | `sim_loop.rs:559` | TIG-Trigger P40 im sim_loop | bricht auf Embedded |
| B1 | `sim_loop.rs:171` | Mond/Erde-Schwelle dupliziert | Drift-Risiko |
| C1 | `sim_loop.rs:269` | V25-Eingabe-Automat (7 State-Felder) | bricht auf Embedded |
| C2 | `sim_loop.rs:511` | PRO/V34-Alignment-Noun-Check dreifach | Duplikat |

Die Sub-Tasks beheben die Probleme in einer Reihenfolge, die sicherstellt,
dass der Simulator nach jedem Schritt kompiliert und alle Tests grün bleiben.

**Scope**: Ausschließlich die beschriebenen fünf Verschiebungen — keine neuen
Features, keine anderen Refactorings.

**Nicht in Scope**: Vollständige Navigation/IMU-Integration in den Burn-Vektor
(bleibt separater Task).

---

## Sub-Task 1 — Zentralkörper-Konstante aus agc-sim entfernen (B1)

**Status**: [ ] pending

### Intent
Die Schwellwertkonstante `3_000_000.0 m` und die Mond/Erde-Entscheidung
existieren identisch in `sim_loop.rs:load_scenario()` und in
`dispatcher.rs:program_p30()`. Bei einer Änderung der Schwelle driften die
beiden Implementierungen auseinander: Physik-Integrator und
Guidance-Programm rechnen in unterschiedlichen Gravitationsfeldern.

Die Single Source of Truth ist `agc-core`. `agc-sim` soll den Wert von dort
lesen, nicht selbst berechnen.

### Expected Outcomes
- `MOON_THRESHOLD_M` ist genau einmal definiert — als `pub const` in
  `agc-core` (z. B. `agc_core::navigation::orbital`).
- `sim_loop.rs:load_scenario()` importiert und verwendet diese Konstante.
- `dispatcher.rs:program_p30()` verwendet dieselbe Konstante.
- `cargo test` und `cargo clippy -- -D warnings` bleiben grün.

### Todo-Liste
1. `pub const MOON_THRESHOLD_M: f64` in `agc-core/src/navigation/orbital.rs`
   definieren und dokumentieren (Einheit, Herkunft).
2. `dispatcher.rs` auf diese Konstante umstellen (lokale Definition entfernen).
3. `sim_loop.rs:load_scenario()` auf diese Konstante umstellen (lokale
   Definition entfernen).
4. Sicherstellen, dass beide Nutzungsstellen denselben Wert referenzieren.

### Relevant Context
- `agc-core/src/display/dispatcher.rs`, Funktion `program_p30()`, Zeile ~200
- `agc-sim/src/sim_loop.rs`, Methode `load_scenario()`, Zeile ~171
- `agc-core/src/navigation/orbital.rs` (Zielort der Konstante)

---

## Sub-Task 2 — V25-Eingabe-Automat nach agc-core verlagern (C1)

**Status**: [ ] pending

### Intent
Der V25-Dateneingabe-Automat (Akkumulation von Ziffern, Vorzeichen, Puffer-
verwaltung, ENTER-Handling) ist eine Emulation des AGC PINBALL-Eingabe-
automaten (ENTPAS/STOREDAT in `PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`). Auf
dem echten AGC läuft dieser Automat im AGC selbst; die Tastatur liefert nur
Rohcodes.

Die sieben `SimState`-Felder `load_r1`, `load_r2`, `load_r3`, `load_reg_idx`,
`load_num_buf`, `load_num_sign`, `load_num_digits` sind ausschließlich Teil
dieses Automaten und gehören nach `agc-core`.

### Expected Outcomes
- Ein neues Modul `agc-core/src/display/input.rs` enthält das Struct
  `InputLoadState` (7 Felder) sowie die Automat-Funktion `process_v25_key()`.
- `InputLoadState` wird in `AgcContext` als Feld `load_state: InputLoadState`
  aufgenommen.
- Die gesamte Automat-Logik aus `sim_loop.rs` (Zeilen 269–358 und die
  Behandlung von `KeyEvent::NounSet` im Flash-V25-Zweig) ist nach
  `agc-core/src/display/input.rs` verschoben.
- `sim_loop.rs` delegiert bei V25-Flash-Modus nur noch an
  `process_v25_key()` — keine eigenen `load_num_*`-Felder mehr im `SimState`.
- Das Verhalten (Ziffernakkumulation, Vorzeichenwechsel, CLR, ENTER-Kette)
  ist durch Unit-Tests in `agc-core/src/display/input.rs` abgedeckt.
- `cargo test` und `cargo clippy -- -D warnings` bleiben grün.

### Todo-Liste
1. Neues Modul `agc-core/src/display/input.rs` anlegen; in
   `agc-core/src/display/mod.rs` einbinden.
2. `InputLoadState`-Struct in `input.rs` definieren
   (7 Felder, `Default`-Impl, `const fn new()`).
3. `InputLoadState` als Feld `load_state` in `AgcContext` aufnehmen.
4. Automat-Logik `process_v25_key()` in `input.rs` implementieren
   (identisches Verhalten zur bisherigen `sim_loop.rs`-Logik).
5. Unit-Tests für den Automaten in `input.rs` schreiben.
6. `SimState` um die sieben Einzelfelder bereinigen; stattdessen
   `ctx.load_state` verwenden.
7. `sim_loop.rs` V25-Block durch Aufruf von `process_v25_key()` ersetzen.

### Relevant Context
- `agc-sim/src/sim_loop.rs`, Zeilen 269–358 (V25-Dateneingabe-Block)
- `agc-sim/src/sim_loop.rs`, Zeilen 439–464 (NounSet im Flash-V25-Zweig)
- `agc-core/src/display/mod.rs` (hier neues Modul `input` einbinden)
- `agc-core/src/context.rs` (AgcContext — hier wird `load_state` aufgenommen)
- AGC source: `PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, ENTPAS / STOREDAT

---

## Sub-Task 3 — PRO/V34 Alignment-Noun-Check in Dispatcher konsolidieren (C2)

**Status**: [ ] pending

### Intent
Die Logik „PRO bei Flash+V25+N22 → `alignment_cmd = CoarseAlign`" und
„V34 bei Flash+V25+N22/N93 → Flash stoppen" ist dreifach im `sim_loop.rs`
implementiert (Enter-Zweig, Proceed-Zweig, Terminate-Zweig). Dieselbe Logik
gehört in `VerbDispatcher::accept_load()` in `agc-core`, wo die
V33/V34-Dispatcher-Handler bereits sitzen.

### Expected Outcomes
- `VerbDispatcher::accept_load()` behandelt N22 und N93 vollständig:
  bei N22 → `alignment_cmd = CoarseAlign`, bei N93 → `alignment_cmd = FineAlign`,
  Flash stoppen in beiden Fällen.
- Die redundanten Noun-22/93-Checks in `sim_loop.rs` für `KeyEvent::Enter`,
  `KeyEvent::Proceed` und `KeyEvent::Terminate` entfallen.
- `sim_loop.rs` ruft für PRO und Enter im V25-Flash-Modus nur noch
  `VerbDispatcher::accept_load()` auf, ohne Noun-spezifische Verzweigung.
- V34 (Terminate) im Flash-Modus: `sim_loop.rs` ruft `stop_flash()` auf
  — die Alignment-Cmd-Prüfung entfällt dort.
- Bestehende Tests für CoarseAlign/FineAlign bleiben grün.

### Todo-Liste
1. In `VerbDispatcher::accept_load()` die Noun-22- und Noun-93-Zweige
   vervollständigen: `alignment_cmd` setzen und `stop_flash()` aufrufen
   (Logik aus den drei `sim_loop.rs`-Zweigen zusammenführen).
2. In `sim_loop.rs` die redundanten Noun-22/93-Blöcke aus `KeyEvent::Enter`,
   `KeyEvent::Proceed` und `KeyEvent::Terminate` entfernen.
3. Sicherstellen, dass der V34-Terminate-Zweig nur noch `stop_flash()` und
   `terminate_flag` setzt (kein Alignment-Check mehr nötig, da dieser in
   `accept_load` liegt).
4. Unit-Test in `agc-core` für accept_load mit N22/N93 ergänzen.

### Relevant Context
- `agc-core/src/display/dispatcher.rs`, `accept_load()`, Zeile ~538
- `agc-sim/src/sim_loop.rs`, `KeyEvent::Enter` Zeilen 466–504
- `agc-sim/src/sim_loop.rs`, `KeyEvent::Proceed` Zeilen 506–529
- `agc-sim/src/sim_loop.rs`, `KeyEvent::Terminate` Zeilen 531–543
- AGC source: `EXTENDED_VERBS.agc`, VBCOARK / IMUFINEK

---

## Sub-Task 4 — Waitlist auf `fn(&mut AgcContext)` umstellen, in AgcContext aufnehmen, V16-Monitor verlagern (A1)

**Status**: [ ] pending

### Intent
Der V16-Monitor aktualisiert das Display 1× pro Sekunde — im `sim_loop.rs`
durch Tick-Zählung (`state.tick.is_multiple_of(100)`). Auf embedded gibt es
keinen `sim_loop`; die Aktualisierungsrate würde dort fehlen.

Der echte AGC verwendet dafür die Waitlist: MONITOR plant sich selbst nach
jeder Ausführung erneut per `call_after` ein. Die `Waitlist` existiert in
`agc-core`, ist aber noch nicht in `AgcContext` aufgenommen, und ihre
Task-Funktionen haben aktuell keine Parameter (`fn()`).

Dieser Sub-Task erweitert `WaitEntry::task_fn` von `fn()` auf
`fn(&mut AgcContext)`. Das ist möglich ohne Heap oder Closures — ein
`fn(&mut T)` ist weiterhin ein einfacher Funktionszeiger.

**Borrow-Lösung**: `Waitlist` sitzt in `AgcContext`. Die aufrufende Funktion
`tick_with_ctx` ist eine **freie Funktion** (kein `impl Waitlist`-Aufruf),
die `ctx` als einzigen Parameter nimmt und via `core::mem::take()` die
Waitlist temporär aus `ctx` herauslöst:

```
pub fn tick_with_ctx(ctx: &mut AgcContext, ticks: u32) -> usize {
    let mut wl = core::mem::take(&mut ctx.waitlist);  // ① Waitlist herausnehmen
    // ... Zeitberechnung und Listenmanipulation auf wl ...
    // Für jeden fälligen Task:
    //   f(ctx);   // ctx.waitlist ist leer (Default) → kein Borrow-Konflikt
    //             // f() darf ctx.waitlist.call_after() aufrufen (landet in ctx.waitlist)
    // ② Nach allen Task-Aufrufen: verbleibende wl-Einträge in ctx.waitlist zusammenführen
    merge_into(&mut ctx.waitlist, wl);                // ③ zurückführen
}
```

Wichtig: Tasks, die sich selbst wieder einplanen (`call_after` in `f(ctx)`),
schreiben in `ctx.waitlist` (das leer ist). Nach dem Durchlauf aller fälligen
Tasks müssen die verbleibenden `wl`-Einträge (noch nicht fällige Tasks) mit
den neu eingetragenen `ctx.waitlist`-Einträgen zusammengeführt werden.
Da beide Sets sortiert nach Zeit sind, ist das eine O(n)-Merge-Operation ohne
Heap. Kein `unsafe`, kein `RefCell`.

### Expected Outcomes
- `WaitEntry::task_fn` hat den Typ `Option<fn(&mut AgcContext)>`.
- Freie Funktion `tick_with_ctx(ctx: &mut AgcContext, ticks: u32) -> usize`
  in `waitlist.rs`, implementiert mit `core::mem::take()` + Merge.
- `AgcContext` hat ein neues Feld `waitlist: Waitlist`.
- Der V16-Handler im Dispatcher trägt beim Aktivieren einen Waitlist-Task
  ein (100 cs = 1 s), der `noun_resolver::resolve()` aufruft, Display
  aktualisiert, und sich selbst via `ctx.waitlist.call_after(100, ...)` neu
  einträgt (selbst-wiederholend).
- `sim_loop.rs:sim_tick()` ersetzt die V16-Prüfung durch
  `tick_with_ctx(&mut state.ctx, 1)` — keine
  `state.tick.is_multiple_of(100)`-Prüfung mehr für den Monitor.
- `state.tick` im `SimState` darf weiter existieren (COMP-ACTY-Blinken).
- Alle bestehenden Waitlist-Unit-Tests laufen weiter (Testfunktionen erhalten
  `_ctx: &mut AgcContext` als Dummy-Parameter).
- `cargo test` und `cargo clippy -- -D warnings` bleiben grün.

### Todo-Liste
1. `WaitEntry::task_fn` von `Option<fn()>` auf `Option<fn(&mut AgcContext)>`
   ändern; `call_after`-Signatur entsprechend anpassen.
2. Freie Funktion `tick_with_ctx(ctx: &mut AgcContext, ticks: u32) -> usize`
   in `waitlist.rs` implementieren:
   a. `let mut wl = core::mem::take(&mut ctx.waitlist)` — Waitlist herausnehmen.
   b. Zeitberechnung und Listenmanipulation auf `wl` (identisch zu bisherigem
      `tick()`-Körper, aber fällige `fn`-Zeiger zunächst in ein lokales
      Array `[Option<fn(&mut AgcContext)>; MAX_TASKS]` sammeln, bevor
      Tasks ausgeführt werden).
   c. `ctx.waitlist = wl` — nicht-fällige Resteinträge zurückschreiben.
   d. Gesammelte fällige Tasks nacheinander mit `f(ctx)` aufrufen. Tasks die
      sich selbst einplanen schreiben in `ctx.waitlist`; diese Einträge
      bleiben nach dem Durchlauf erhalten.
3. Alle bestehenden Waitlist-Unit-Tests auf die neue Signatur anpassen
   (`fn task_x(_ctx: &mut AgcContext) { ... }`).
4. `waitlist: Waitlist` zu `AgcContext` hinzufügen; `Default`-Impl anpassen.
5. V16-Dispatcher-Handler: nach dem ersten `resolve()`-Aufruf einen
   `fn(&mut AgcContext)`-Task in `ctx.waitlist` eintragen (100 cs), der
   `resolve()` aufruft, Display aktualisiert, und sich selbst neu einträgt.
6. `sim_loop.rs:sim_tick()`: V16-Tick-Prüfung durch
   `tick_with_ctx(&mut state.ctx, 1)` ersetzen.
7. Unit-Test: V16 aktivieren, 100 `tick_with_ctx`-Aufrufe simulieren,
   Display-Update nachweisen.

### Relevant Context
- `agc-core/src/executive/waitlist.rs` (WaitEntry, Waitlist — Typ- und API-Änderung)
- `agc-core/src/context.rs` (AgcContext — Feld `waitlist`, braucht `use Waitlist`)
- `agc-core/src/display/dispatcher.rs`, V16-Handler (Zeile ~429)
- `agc-core/src/display/noun_resolver.rs` (`resolve`-Funktion)
- `agc-sim/src/sim_loop.rs`, Zeilen 244–253

---

## Sub-Task 5 — TIG-Trigger P40 nach agc-core verlagern (A2)

**Status**: [ ] pending

### Intent
Der TIG-Trigger in `sim_loop.rs:559` entscheidet, wann das SPS-Triebwerk
zündet. Das ist pure Domain-Logik von P40. Auf embedded gibt es keinen
`sim_loop` — der Burn würde nie starten.

Der echte AGC implementiert den TIG-Check über die Waitlist: P40 plant beim
Start einen Task ein, der zum TIG feuert (TIGBLNK→TIGAVEG→TIG-5→TIGNOW).

In unserer Rust-Implementierung nutzt Sub-Task 5 die in Sub-Task 4 eingeführte
`fn(&mut AgcContext)`-Waitlist: `program_p40()` trägt beim Start einen
Waitlist-Task ein, der zum TIG-Zeitpunkt direkt `p40_step()` aufruft
(Phase 2→3: Zündung). Kein Flag-Pattern nötig — der Task hat direkten
Zugriff auf `ctx`.

Zusätzlich wird der laufende Burn (Phase 3) verdrahtet: `sim_loop.rs:sim_tick()`
ruft pro Tick `p40_step(dt)` auf, solange `p40.phase == 3`.

### Expected Outcomes
- `program_p40()` im Dispatcher berechnet den Delta-cs bis zum TIG
  (`(tig_s - now_s) * 100`) und trägt einen `fn(&mut AgcContext)`-Task ein,
  der `p40_step()` (Phase 2→3) direkt aufruft.
- Falls TIG bereits vergangen oder nicht gesetzt (tig_s == 0.0): Verzögerung
  1 cs (Zündung im nächsten Tick).
- `sim_loop.rs:sim_tick()` ruft nach `tick_with_ctx()` zusätzlich —
  falls `p40.phase == 3` — `p40_step(dt)` auf (Burn-Akkumulation).
- Der bisherige TIG-Trigger-Block (`sim_loop.rs:559–583`) entfällt.
- Der Burn startet korrekt auch dann, wenn die Crew P40 manuell via `V37E40E`
  vor dem TIG gestartet hat.
- **Acceptance-Kriterium UC-2**: `uc2_tei_burn.rs` enthält einen Test der
  nach Ablauf der TIG-Ticks prüft, dass `p40.dv_accumulated > 0.0`
  (DVTOTAL wächst während des Burns).
- `cargo test` und `cargo clippy -- -D warnings` bleiben grün.

### Todo-Liste
1. In `program_p40()`: TIG-Delta in cs berechnen
   (`((tig_s - now_s) * 100.0) as u32`) und einen `fn(&mut AgcContext)`-Task
   in `ctx.waitlist` eintragen, der `p40_step()` mit `dt=0.0` aufruft
   (Phase 2→3: Brennzeit berechnen und Triebwerk zünden).
   `now_s` = `ctx.current_sv.time.as_seconds()`.
2. Falls `tig_s == 0.0` oder `tig_s <= now_s`: Verzögerung 1 cs.
3. In `sim_loop.rs:sim_tick()`: wenn `p40.phase == 3` → `p40_step(dt)` pro
   Tick aufrufen (Burn-Akkumulation, Phase 3).
4. Den alten TIG-Trigger-Block (Zeilen 559–583) aus `sim_loop.rs` entfernen.
5. UC-2-Integrationstest `uc2_tei_burn.rs` um Test `uc2_burn_dvtotal_grows`
   ergänzen: TIG auf `now + 1 cs` setzen, genügend Ticks via `sim_tick()`
   simulieren, `p40.dv_accumulated > 0.0` prüfen.
6. Use-Case-Dokument `docs/use-cases/uc2-tei-burn.md` anpassen: den Hinweis
   zu Sub-Task 7 entfernen, korrekte automatische Zündlogik dokumentieren.

### Relevant Context
- `agc-sim/src/sim_loop.rs`, Zeilen 559–583 (zu entfernender TIG-Trigger)
- `agc-core/src/guidance/p40_47.rs`, `p40_step()`, `P40State`
- `agc-core/src/display/dispatcher.rs`, `program_p40()`, Zeile ~236
- `agc-core/src/context.rs`, `GuidanceContext`
- `agc-sim/tests/uc2_tei_burn.rs` (UC-2-Integrationstest)
- AGC source: `Comanche055/P40-P47.agc`, TIGBLNK / TIGAVEG / TIGNOW

---

## Abhängigkeiten zwischen Sub-Tasks

Sub-Task 4 muss vor Sub-Task 5 abgeschlossen sein, weil Sub-Task 5 die in
Sub-Task 4 eingeführten `fn(&mut AgcContext)`-Waitlist-Tasks und
`tick_with_ctx()` sowie `ctx.waitlist` voraussetzt.

Sub-Tasks 1, 2 und 3 sind voneinander unabhängig und können in beliebiger
Reihenfolge bearbeitet werden.

Empfohlene Reihenfolge: 1 → 2 → 3 → 4 → 5
