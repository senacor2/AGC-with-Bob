# Task 15 — DSKY-Implementierung: Abweichungen beheben

## Überblick

Dieser Task behebt die in der Analyse (Abgleich mit `docs/dsky-verb-noun-reference.md`)
identifizierten Abweichungen zwischen der Rust-Implementierung und dem AGC-Comanche055-Quellcode.
Jeder Subtask ist eine in sich abgeschlossene, commitfähige Änderung.

**Nicht in Scope:** Nouns und Verbs, die bewusst als `Noop`/`Blank` markiert sind
(d.h. noch nicht implementierte Funktionalität). Es werden ausschließlich Stellen
korrigiert, bei denen vorhandener Code falsch arbeitet.

---

## Sub-Task 15.1 — Live-Anzeige der Verb/Noun-Eingabe während des Tippens

**Status:** `[ ] pending`

### Intent
Im echten DSKY werden die eingegebenen Ziffern sofort in den Verb- bzw.
Noun-Segmenten angezeigt, während die Crew tippt — nicht erst nach ENTER.
Die AGC-Routine `NUM`/`DSPIN` schreibt jede Ziffer direkt in `DSPTAB`.

In der Rust-Implementierung schreibt `process_key()` bei einem Zifferndruck
nur in `InputBuffer.digits`. `state.verb` und `state.noun` bleiben bis ENTER
unverändert — das DSKY-Widget zeigt daher die alten Werte, bis ENTER gedrückt wird.

### Expected Outcomes
- Nach Drücken von `V 3` zeigt das VERB-Display `03`.
- Nach Drücken von `V 3 7` zeigt das VERB-Display `37`.
- Nach Drücken von `N 6` zeigt das NOUN-Display `06`.
- Das Verhalten nach ENTER ist identisch zum bisherigen (kein Regressionsrisiko).

### Todo List
1. In `process_key()` im `KEY_0..=KEY_9`-Zweig: nach `buf.push_digit()` den
   aktuellen Pufferwert (`buf.to_value()`) sofort in `state.verb` (Verb-Modus)
   bzw. `state.noun` (Noun-Modus) schreiben.
2. Unit-Test hinzufügen: nach einer einzelnen Ziffer im Verb-Modus zeigt
   `state.verb` den Teilwert; nach zwei Ziffern den vollständigen Wert.

### Relevant Context
- `agc-core/src/display/verb_noun.rs`, `process_key()`, `KEY_0..=KEY_9`-Zweig (~Zeile 585)
- AGC source: `PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, `NUM`/`DSPIN`-Routine

---

## Sub-Task 15.2 — V37: zweistufiger Eingabeablauf (Flash → Programmnummer → Start)

**Status:** `[x] done`

### Intent
V37 (`MMCHANG`) verlangt im AGC einen zweistufigen Ablauf:
1. Erster ENTER nach V37 → `REQMM` → Flash einschalten, Cursor auf Noun-Feld,
   warten auf 2 Dezimalziffern.
2. Crew tippt Programmnummer → zweiter ENTER → Programm starten.

Die aktuelle Implementierung dispatcht V37 beim `NounSet`-Event mit dem
vorher eingegeben Noun als Programmnummer. Damit fehlt der Flash-Prompt nach
dem ersten ENTER, und die Programmnummer wird konzeptuell falsch als Noun
statt als eigenständige numerische Eingabe behandelt.

### Expected Outcomes
- Nach `V 3 7 ENTER` ist `display.flash_state == true` (Flash aktiv).
- Noch kein Programm gestartet (`active_program == None`).
- Nach `N 6 1 ENTER` (Eingabe der Programmnummer über das Noun-Feld) wird
  das Programm gestartet, Flash gestoppt, `major_mode == 61`.
- Bestehende Integrationstests für V37+Programm-Sequenzen bleiben grün.

### Todo List
1. In `dispatcher.rs`: neue `VerbAction::RequestProgram`-Variante hinzufügen.
2. `VERB_TABLE[37]` auf `VerbAction::RequestProgram` setzen (ersetzt `Noop`).
3. In `dispatch()`: den bisherigen `if v == 37`-Sonderfall entfernen.
   Stattdessen `RequestProgram`-Arm im `match action`-Block: nur Flash einschalten.
4. In `dispatch()`: vor dem Tabellen-Lookup einen neuen Zweig ergänzen —
   wenn `v == 37` und `noun.val() != 0`, `dispatch_v37()` aufrufen und zurückkehren.
   (Dieser Zweig feuert beim zweiten Aufruf via `NounSet`.)
5. In `sim_loop.rs`: V37 zur Sofort-Dispatch-Liste beim `VerbSet`-Event
   hinzufügen (`matches!(v_raw, 32..=34 | 37)`).
6. Neuen Integrationstest hinzufügen: `v37_first_enter_activates_flash` —
   prüft dass nach `V 3 7 ENTER` Flash aktiv und kein Programm gestartet ist.

### Relevant Context
- `agc-core/src/display/dispatcher.rs`, `VerbAction`, `VERB_TABLE`, `dispatch()`,
  `dispatch_v37()` (~Zeilen 29–97, 109–134, 332–344, 483–516)
- `agc-sim/src/sim_loop.rs`, `VerbSet`-Zweig (~Zeile 165–173)
- `agc-sim/tests/dispatcher_integration.rs`, bestehende V37-Tests
- AGC source: `PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, `MMCHANG`/`REQMM` (~Zeilen 2806–2851)

---

## Sub-Task 15.3 — N93: Phase-Check-Bedingung korrigieren

**Status:** `[x] done`

### Intent
In `noun_resolver.rs` zeigt N93 (Delta-Gyro-Winkel) Daten wenn `p51.phase == 0`
und gibt `Blank` zurück wenn `p51.phase != 0`. Das ist invertiert: Phase 0 ist
der Anfangszustand vor jedem P51-Lauf. Daten sind erst sinnvoll, wenn P51
abgeschlossen ist.

Da die vollständige P51-Gyro-Torquing-Berechnung (R55) noch nicht implementiert
ist, zeigt N93 derzeit `thetad` als Proxy. Die Bedingung soll so korrigiert werden,
dass N93 `Blank` zurückgibt solange P51 nicht gestartet hat oder thetad noch
den Nullvektor enthält.

### Expected Outcomes
- N93 gibt `Blank` zurück wenn `p51` nicht gesetzt ist.
- N93 gibt `Blank` zurück wenn `thetad == [0.0; 0.0; 0.0]`.
- N93 zeigt die skalierten `thetad`-Werte wenn P51 aktiv ist und `thetad != 0`.
- Bestehende N93-Tests werden angepasst und bleiben grün.

### Todo List
1. In `noun_resolver.rs`, N93-Zweig: Bedingung `if p51.phase != 0` durch
   eine Prüfung auf leeres `thetad` ersetzen:
   `if thetad == [0.0, 0.0, 0.0] { return Blank }`.
   Den `p51.phase`-Check entfernen (er war konzeptuell falsch und nicht
   notwendig für den aktuellen Proxy-Ansatz).
2. Betroffene Unit-Tests in `noun_resolver.rs` anpassen:
   - Test „N93 ohne P51-State → Blank" bleibt unverändert.
   - Test „N93 mit P51 in Phase ≠ 0 → Blank" wird zu „N93 mit thetad=0 → Blank".
   - Neuen Test hinzufügen: „N93 mit P51 aktiv und thetad ≠ 0 → Werte sichtbar".

### Relevant Context
- `agc-core/src/display/noun_resolver.rs`, N93-Zweig (~Zeile 489–508)
- AGC source: `ASSEMBLY_AND_OPERATION_INFORMATION.agc`, N93 (DELTA GYRO ANGLES)

---

## Sub-Task 15.4 — Monitor-Rate: 1 Hz statt 100 Hz

**Status:** `[x] done`

### Intent
Der AGC aktualisiert Monitor-Anzeigen (V11–V17) einmal pro Sekunde (100 Ticks
à 10 ms). In `sim_tick()` wird der NounResolver bei `verb == 16` aktuell jeden
10-ms-Tick aufgerufen (100 Hz). Das entspricht nicht dem AGC-Verhalten und
erzeugt unnötige Rechenarbeit.

### Expected Outcomes
- NounResolver wird bei aktivem V16 nur jeden 100. Tick aufgerufen (≈ 1 Hz).
- Das angezeigte Ergebnis ändert sich für den Nutzer nicht sichtbar.
- Kein bestehender Test bricht.

### Todo List
1. In `sim_loop.rs`, im Monitor-Block (`if state.ctx.display.verb.val() == 16`):
   Aufruf mit `state.tick % 100 == 0` gaten.
2. Kommentar ergänzen: „1 Hz = 100 Ticks à 10 ms, entspricht AGC MONITOR-Zyklus".

### Relevant Context
- `agc-sim/src/sim_loop.rs`, Monitor-Block (~Zeile 128–138)
- AGC source: `PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, `MONITOR`-Routine

---

## Sub-Task 15.5 — accept_load: Eingabepuffer statt Display-Register lesen (iterativ)

**Status:** `[ ] pending`

### Intent
`VerbDispatcher::accept_load()` erhält in `sim_loop.rs` die Werte aus den
aktuell angezeigten `DisplayRegister`-Feldern (`display.r1/r2/r3`). Das ist
falsch: die Register enthalten den vom NounResolver berechneten Anzeigewert,
nicht die Crew-Eingabe.

Der AGC liest bei Load-Verben die Werte aus `XREG`/`YREG`/`ZREG` — den rohen
Eingabepuffern. Dieser Subtask führt einen dedizierten Dateneingabe-Puffer in
`SimState` ein und verdrahtet ihn iterativ:

**Iteration A** — Puffer einführen und Enter-Pfad umverdrahten
**Iteration B** — Zifferneingabe in den Puffer leiten (InputMode::Data)

### Expected Outcomes
- `SimState` hält drei `i32`-Felder `load_r1`, `load_r2`, `load_r3` für
  die laufende Crew-Dateneingabe (0 initialisiert).
- Nach Iteration A: `accept_load` erhält Werte aus `load_rX`, nicht aus
  `display.rX`. Da `load_rX` initial 0 sind, ist das Verhalten für N61
  weiterhin korrekt solange keine anderen Werte vorher gesetzt wurden.
- Nach Iteration B: Eingaben im `InputMode::Data`-Modus landen direkt in
  `load_rX` und werden auch live im Display gespiegelt (R1 → R2 → R3
  sequenziell nach je einem ENTER).
- Hilfsfunktion `display_reg_to_i32` wird nach Iteration B entfernt.
- Mindestens ein Test prüft den korrekten Eingabepfad.

### Todo List — Iteration A: Puffer + Enter-Umverdrahtung
1. `SimState` in `sim_loop.rs` um drei `i32`-Felder erweitern:
   `load_r1: i32`, `load_r2: i32`, `load_r3: i32` (alle `0`).
2. `SimState::new()` und `Default` anpassen.
3. Im `KeyEvent::Enter`-Zweig (Zeile 184–191): statt
   `display_reg_to_i32(state.ctx.display.r1/r2/r3)` die neuen Felder
   `state.load_r1/r2/r3` übergeben.

### Todo List — Iteration B: Dateneingabe-Modus
4. Neuen `InputMode::Data { register: u8 }` in `verb_noun.rs` hinzufügen
   (register = 0 → R1, 1 → R2, 2 → R3). Alternativ als separates Feld in
   `SimState` wenn die Erweiterung von `InputMode` zu viel Ripple erzeugt.
5. Bei V25-Dispatch (`LoadData`-Arm, `dispatcher.rs` Zeile 389): zusätzlich
   ein Signal setzen, das `sim_loop.rs` mitteilt, dass der nächste
   Zifferneingabe-Block für `load_r1` bestimmt ist.
6. Im `KeyEvent::NounSet`-Zweig in `sim_loop.rs` (Zeile 175): wenn
   `flash_state == true` und `verb == 25`, den Noun-Wert als Datenwert für
   das aktuelle Register (`load_r1` → `load_r2` → `load_r3`) übernehmen
   und im Register-Index weiterschalten.
7. `display_reg_to_i32` entfernen.
8. Test: V25 N61 mit drei separaten Noun-Eingaben → `accept_load` erhält
   korrekte Werte aus `load_r1/r2/r3`, nicht aus Display-Registern.

### Relevant Context
- `agc-sim/src/sim_loop.rs`, `SimState` (Zeilen 59–78), `KeyEvent::Enter`
  (Zeilen 181–191), `KeyEvent::NounSet` (Zeilen 175–180),
  `display_reg_to_i32` (Zeile 233)
- `agc-core/src/display/dispatcher.rs`, `accept_load()` (Zeilen 440–481),
  `LoadData`-Arm (Zeile 389)
- `agc-core/src/display/verb_noun.rs`, `InputMode` (Zeile 448), `InputBuffer`
  (Zeilen 463–511)
- AGC source: `PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, `NVSBWAIT`/`XREG`/`YREG`/`ZREG`

---

## Sub-Task 15.6 — V41/V42: Load-Prompt vor Alignment-Ausführung

**Status:** `[ ] pending`

### Intent
Der AGC zeigt vor der Ausführung von V41 (Coarse Align) und V42 (Fine Align)
einen blinkenden Lade-Prompt an, damit die Crew die Zielwinkel eingeben kann:
- V41 → blinkend V25 N22 (neue ICDU-Winkel laden)
- V42 → blinkend V25 N93 (Delta-Gyro-Winkel laden)

Aktuell setzt die Rust-Implementierung `alignment_cmd` sofort ohne
Crew-Eingabe-Prompt. Bei V41 wird zusätzlich der NounResolver aufgerufen
(Zeile 420), der für N20/N22 `Blank` zurückgibt wenn `thetad == [0,0,0]` —
es gibt also nichts anzuzeigen, bevor die Crew Winkel eingegeben hat.

Bei V42 wird der NounResolver gar nicht aufgerufen (kein Noun-Display nach
Fine-Align-Befehl).

Der `alignment_cmd` wird von P51/P52 asynchron konsumiert. Der Prompt-Mechanismus
muss daher so umgesetzt werden, dass `alignment_cmd` erst gesetzt wird,
nachdem die Crew die Eingabe mit V33 (Proceed) oder ENTER bestätigt hat.

### Expected Outcomes
- Nach V41 N20 ENTER: Flash ein, Anzeige V25 N22 (wartet auf Crew-Eingabe).
- Nach V41 mit Crew-Eingabe (neue ICDU-Winkel) + ENTER/Proceed:
  `alignment_cmd = CoarseAlign` gesetzt, Flash aus.
- Nach V42 N93 ENTER: Flash ein, Anzeige V25 N93 (wartet auf Gyro-Winkel).
- Nach V42 mit Eingabe + ENTER/Proceed: `alignment_cmd = FineAlign` gesetzt.
- Bei Terminate (V34): `alignment_cmd` bleibt `None`, Flash aus.
- `thetad` wird durch die Crew-Eingabe befüllt bevor `alignment_cmd` gesetzt wird.

### Todo List
1. In `dispatcher.rs`, `CoarseAlign`-Arm (Zeile 413): `alignment_cmd` **nicht**
   sofort setzen. Stattdessen: Flash einschalten und Verb/Noun auf V25/N22
   setzen (blinkender Lade-Prompt). `alignment_cmd` wird erst in Schritt 3 gesetzt.
2. In `dispatcher.rs`, `FineAlign`-Arm (Zeile 428): analog — Flash ein,
   Verb/Noun auf V25/N93 setzen.
3. In `sim_loop.rs`, `KeyEvent::Proceed`-Zweig (Zeile 193) **und**
   `KeyEvent::Enter`-Zweig (Zeile 181): wenn `flash_state == true` und das
   aktive Verb V25 ist und der aktive Noun N22 oder N93 ist,
   `alignment_cmd` entsprechend setzen und Flash stoppen. Beide Eingabewege
   lösen den Alignment-Schritt aus (AGC: sowohl ENTER als auch V33/PRO).
4. In `sim_loop.rs`, `KeyEvent::Terminate`-Zweig (Zeile 196): wenn
   Alignment-Prompt aktiv war, Flash stoppen ohne `alignment_cmd` zu setzen.
5. Tests ergänzen:
   - V41: nach V41-Enter ist Flash aktiv, `alignment_cmd == None`.
   - V41: nach Proceed ist `alignment_cmd == CoarseAlign`, Flash aus.
   - V41: nach Enter (statt Proceed) ist `alignment_cmd == CoarseAlign`, Flash aus.
   - V42: analog für FineAlign/N93.

### Relevant Context
- `agc-core/src/display/dispatcher.rs`, `CoarseAlign`-Arm (Zeilen 411–424),
  `FineAlign`-Arm (Zeilen 426–434)
- `agc-sim/src/sim_loop.rs`, `KeyEvent::Proceed` (Zeile 193),
  `KeyEvent::Terminate` (Zeile 196)
- AGC source: `EXTENDED_VERBS.agc`, `VBCOARK` (~Zeilen 250–278),
  `IMUFINEK` (~Zeilen 347–377)

---

## Sub-Task 15.7 — N40: R1/R2-Belegung korrigieren (VG/TGO → TTOGO/VGDISP)

**Status:** `[x] done`

### Intent
Die AGC-Referenz definiert N40 als Mixed Noun mit:
- R1: `TTOGO` — Zeit bis Zündung/Abbruch im Format MM:SS (kein Laden, nur Anzeige)
- R2: `VGDISP` — VG (Velocity-to-Go) in ft/s
- R3: `DVTOTAL` — akkumuliertes Delta-V in ft/s

Die aktuelle Implementierung zeigt:
- R1: `p40.delta_v_target` [m/s] — das ist `VGDISP`, nicht `TTOGO`
- R2: `p40.t_burn - p40.t_elapsed` [s] — das ist `TGO`, nicht `VGDISP`
- R3: `Blank`

Die Belegung R1/R2 ist vertauscht und inhaltlich falsch benannt. Da `P40State`
kein `TTOGO`-Feld im Zeitformat MM:SS enthält, muss `P40State` um ein Feld
`ttogo_s: f64` erweitert oder die vorhandene Restbrennzeit (`t_burn - t_elapsed`)
als TTOGO interpretiert und korrekt zugeordnet werden.

N40 hat außerdem `NO LOAD, DEC ONLY` — die Anzeige darf nur im Dezimalformat
erfolgen. R3 (`DVTOTAL`) fehlt komplett; dafür muss `P40State` ein Feld
`dv_accumulated: f64` erhalten.

Da N40 R1 originalgetreues `XXBXX`-Format (MM:SS, B = Leerzeichen) verwendet,
wird in diesem Subtask das MM:SS-Display-Widget implementiert. Das betrifft
`segment7.rs` (neue Formatierungsfunktion) und `DisplayRegister` (neue Variante
`MinSec(u32)`).

### Expected Outcomes
- `DisplayRegister` erhält eine neue Variante `MinSec(u32)` für Sekunden-Rohwerte.
  Das Widget rendert sie als `MM B SS` (B = Leerzeichen, je zweistellig,
  ohne Vorzeichen — AGC-Format `XXBXX`).
- N40 R1 zeigt TTOGO als `MM B SS` (originalgetreu).
- N40 R2 zeigt `VGDISP` (Velocity-to-Go = `delta_v_target`) in m/s.
- N40 R3 zeigt `DVTOTAL` (akkumuliertes Delta-V) in m/s.
- `P40State` erhält ein neues Feld `dv_accumulated: f64` (initialisiert mit 0.0).
- Bestehende N40-Tests werden auf die korrigierten Positionen angepasst.

### Todo List
1. `DisplayRegister` in `verb_noun.rs` um `MinSec(u32)` erweitern.
   Der Wert ist die Gesamtzeit in Sekunden; Darstellung: Minuten = Wert / 60,
   Sekunden = Wert % 60, je zweistellig, getrennt durch Leerzeichen (B).
   AGC-Format: `XXBXX` (max. 59 Min 59 Sek, darüber klemmen).
2. In `segment7.rs` eine Funktion `format_min_sec(total_s: u32)` hinzufügen,
   die `(top, mid, bot)` im `XXBXX`-Format zurückgibt:
   zwei Ziffern Minuten, ein Leerfeld (Blank-Segment), zwei Ziffern Sekunden.
3. In `dsky_widget.rs`, `render_register()`: neuen `MinSec`-Arm im `match reg`-Block
   ergänzen, der `format_min_sec()` aufruft.
4. `P40State` in `p40_47.rs` um `dv_accumulated: f64` erweitern (initialisiert `0.0`).
5. In `noun_resolver.rs`, N40-Zweig (Zeilen 234–244):
   - R1 ← `DisplayRegister::MinSec((tgo).max(0.0) as u32)` (TTOGO)
   - R2 ← `mps_to_display(p40.delta_v_target)` (VGDISP)
   - R3 ← `mps_to_display(p40.dv_accumulated)` (DVTOTAL)
6. Bestehende N40-Tests in `noun_resolver.rs` anpassen.
7. Unit-Test für `format_min_sec`: 0 s → `00 00`, 90 s → `01 30`,
   3599 s → `59 59`, 3600 s → `59 59` (geklemmt).

### Relevant Context
- `agc-core/src/display/noun_resolver.rs`, N40-Zweig (Zeilen 234–244)
- `agc-core/src/guidance/p40_47.rs`, `P40State` (Zeilen 83–98)
- `docs/dsky-verb-noun-reference.md`, N40 (Mixed Noun, NO LOAD, DEC ONLY)
- AGC source: `ASSEMBLY_AND_OPERATION_INFORMATION.agc`, N40

---

## Sub-Task 15.8 — N44: R3 (TFF) ergänzen

**Status:** `[x] done`

### Intent
N44 definiert R3 als `TFF` (Time to Free Fall, MM:SS — Zeit bis zum
Atmosphäreneintritt). Das Feld fehlt in `P30Output` komplett; R3 wird
daher als `Blank` angezeigt.

`P30Output` muss um `tff_s: f64` erweitert werden. Die Berechnung des TFF
(Conic-Subroutine `TFFCONMU`/`TFFRP`) ist in `agc-core/src/navigation/`
bereits teilweise vorhanden; der Wert muss aus dem P30-Berechnungspfad in
`P30Output` übernommen werden.

Da die vollständige Conic-Berechnung ggf. noch nicht TFF liefert, ist eine
Übergangslösung akzeptabel: `tff_s` wird mit `0.0` initialisiert und kann
später durch den Berechnungspfad befüllt werden. Entscheidend ist, dass
N44 R3 das Feld anzeigt (auch wenn es 0 ist) statt Blank.

### Expected Outcomes
- `P30Output` enthält ein Feld `tff_s: f64`.
- N44 R3 zeigt `tff_s` in Sekunden (als Dezimalwert, nicht MM:SS; Kommentar
  dokumentiert ausstehende Formatierung).
- N44 R3 ist nicht mehr `Blank`.
- Bestehender N44-Test wird angepasst (R3-Erwartung von Blank auf Decimal 0).

### Todo List
1. `P30Output` in `p30_37.rs` um `tff_s: f64` erweitern (initialisiert `0.0`).
   Alle Konstruktoren / `Default`-Impls anpassen.
2. In `noun_resolver.rs`, N44-Zweig (Zeilen 255–264):
   - R3 ← `DisplayRegister::MinSec(p30.tff_s.max(0.0) as u32)`
   (nutzt die in 15.7 eingeführte `MinSec`-Variante)
3. N44-Tests in `noun_resolver.rs` anpassen: R3 ist nun `MinSec(0)` statt `Blank`.

**Hinweis:** 15.8 setzt die `MinSec`-Variante aus 15.7 voraus und muss danach
umgesetzt werden.

### Relevant Context
- `agc-core/src/display/noun_resolver.rs`, N44-Zweig (Zeilen 255–264)
- `agc-core/src/guidance/p30_37.rs`, `P30Output` (Zeilen 155–162)
- `docs/dsky-verb-noun-reference.md`, N44 (TFF = R3, NO LOAD, DEC ONLY)
- AGC source: `ASSEMBLY_AND_OPERATION_INFORMATION.agc`, N44

---

## Abhängigkeiten zwischen Subtasks

```
15.1 (Live-Anzeige)      — unabhängig
15.2 (V37 zweistufig)    — unabhängig; baut auf 15.1 auf
15.3 (N93 Phase)         — unabhängig
15.4 (Monitor-Rate)      — unabhängig
15.5 (accept_load)       — baut auf 15.1 auf; Iteration A vor Iteration B
15.6 (V41/V42 Prompt)    — baut auf 15.5 Iteration B auf (Dateneingabe-Mechanismus)
15.7 (N40 R1/R2 + MMSS)  — unabhängig; führt DisplayRegister::MinSec ein
15.8 (N44 R3 TFF)        — setzt 15.7 voraus (MinSec-Variante)
```

Empfohlene Reihenfolge: **15.1 → 15.2 → 15.3 → 15.4 → 15.7 → 15.8 → 15.5A → 15.5B → 15.6**

15.7 und 15.8 sind reine Datenmodell- und Anzeige-Korrekturen und können
vorgezogen werden, da sie keine Abhängigkeit zu den Eingabepfad-Subtasks haben.
15.8 muss nach 15.7 kommen, da es `DisplayRegister::MinSec` verwendet.
