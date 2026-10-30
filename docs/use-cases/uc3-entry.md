# UC-3: Entry (P61 → P64 → P65 → P66 → P67)

**Szenario**: `docs/scenarios/entry_p61.scenario` (Trans-Earth Coast, EI − 60 min, Apollo-11-Referenz)
**Integrationstest**: `agc-sim/tests/uc3_entry.rs`
**Programme**: P61 (Entry Preparation), P64 (Constant-Drag Phase), P65 (Up-Control), P66 (Trim/Ballistik), P67 (Terminal Phase)

---

## Überblick

Apollo 11 ging **direkt aus der Trans-Earth-Coast-Bahn in den Wiedereintritt** —
kein Earth Orbit Insertion vor dem Reentry. Die gesamte Rückreise vom Mond war
eine einzige, ~59-stündige freie Keplerbahn mit dem Perigäum tief in der Atmosphäre.

Der Ablauf aus AGC-Sicht:

| Programm | Start | Bedingung |
|---|---|---|
| **P61** | Manuell `V37E61E` | ~60 min vor EI, auf Trans-Earth-Bahn |
| **P62** | Automatisch | Folgt auf P61 nach PROCEED |
| **P63** | Automatisch | Folgt auf P62; hält Entry-Lage bis .05G |
| **P64** | **Automatisch** | Wenn Bremsverzögerung ≥ 0.05 g (~90 s nach EI) |
| **P65** | **Automatisch** | Wenn Restreichweite < 25 NM zum Ziel (GOTOUPSY) |
| **P66** | **Automatisch** | Wenn D < Q7 (≈ 6 ft/s²) in UPCONTRL-Phase (KEP) |
| **P67** | **Automatisch** | Wenn D > Q7F+KDMIN (≈ 6.5 ft/s²) in P66 (PREFINAL) |

> **Wichtig**: Die Crew startet **nur P61** per V37. P62–P67
> werden vom AGC-internen SERVICER/REENTRY_CONTROL-Zyklus automatisch ausgelöst —
> analog zu `NEWMODEX`-Aufrufen via RTB in `REENTRY_CONTROL.agc`.
>
> AGC source: `Comanche055/REENTRY_CONTROL.agc`, SCALEPOP/INITROLL/GOTOUPSY/UPCONTRL/KEP/PREFINAL/PREDICT3.

---

## Szenario laden

```sh
cargo run -p agc-sim -- --scenario docs/scenarios/entry_p61.scenario
```

Das Szenario startet **60 Minuten vor dem Entry Interface**:

| Parameter | Wert |
|---|---|
| Höhe | 17 126 km |
| Geschwindigkeit | 5 485 m/s (inertial) |
| MET `time_s` | −3 600 s (= EI − 60 min) |
| Bahn | Trans-Earth Coast (e = 0.938, a = 103 973 km) |

> Der lokale Flugpfadwinkel beträgt hier −57° — das ist kein Fehler. Auf der
> stark exzentrischen Rückkehrbahn vom Mond ist die Bahn nahe dem Perigäum
> sehr steil. Der EI-Winkel γ = −6.5° wird erst kurz vor dem Entry Interface
> bei 122 km Höhe erreicht, wenn die Atmosphäre abbremst.

---

## Timing-Übersicht (Apollo-11-Referenz)

| Zeit | Ereignis | Eingabe |
|---|---|---|
| t = −3 600 s (EI − 60 min) | **Szenario-Start** | Simulator laden |
| t ≈ −3 600 s | **P61 starten** | `V37E61E` |
| t ≈ −3 600 s | Splashdown-Koords eingeben | `V25N61E` + Werte |
| t ≈ −3 600 s | EMS-Daten prüfen | `V06N60E`, `V06N63E` |
| t ≈ −600 s (EI − 10 min) | CM/SM-Separation (P62) | PRO-Taste |
| **t = 0 (EI)** | Entry Interface | P63 aktiv, 122 km |
| t ≈ +90 s | .05G → **P64 automatisch** | (kein Eingriff) |
| t ≈ +10 min | Restreichweite < 25 NM → **P65 automatisch** | (kein Eingriff) |
| t ≈ +10–12 min | D < Q7 → **P66 automatisch** (Trim) | (kein Eingriff) |
| t ≈ +12–15 min | D > Q7+0.5 → **P67 automatisch** | (kein Eingriff) |
| t ≈ +15 min | V ≤ 1 000 fps, Fallschirm | P67 fertig |

---

## Checkliste

### Schritt 1: P61 starten (Entry Preparation)

Direkt nach dem Laden des Szenarios:

**Eingabe**: `V37E61E`
= VERB → 3 → 7 → ENTER → 6 → 1 → ENTER

**Erwartet**:
| Anzeige | Wert |
|---------|------|
| PROG | 61 |

---

### Schritt 2: Splashdown-Koordinaten eingeben (V25 N61)

**Eingabe**: `V25N61E` dann Werte
= VERB → 2 → 5 → NOUN → 6 → 1 → ENTER

Beispiel Apollo-11 Splashdown (Pazifik):
```
LAT(SPL) =  +1354   (= +13.54°,  in 0.01°)
LNG(SPL) = -16942   (= -169.42°, in 0.01°)
HEADSUP  =     +1   (Lift-Down = heads-up)
```

Tasteneingabe:
```
+ 1 3 5 4 E   ← LAT(SPL) = +1354 centideg
- 1 6 9 4 2 E ← LNG(SPL) = -16942 centideg
+ 1 E         ← HEADSUP = 1
```

**Erwartet**:
- Flash erlischt
- `splashdown_lat` und `splashdown_lon` intern gesetzt

---

### Schritt 3: Eintrittsparameter prüfen (V06 N60)

**Eingabe**: `V06N60E`

**Erwartet**:
| Register | Bedeutung | Einheit | Referenzwert Apollo 11 |
|----------|-----------|---------|------------------------|
| R1 | GMAX (max. Bremslast) | 0.01 g | ≈ 600–1300 (= 6–13 g) |
| R2 | VPRED (Geschwindigkeit bei EI) | ft/s | ≈ 36 200 |
| R3 | GAMMAEI (Eintrittswinkel) | 0.01° | ≈ −650 (= −6.50°) |

> **Hinweis zu GMAX**: Das AGC-Modell (GMAXCALC, `P61-P67.agc` Z. 808) enthält einen konstanten
> Additionsterm von 10 g (KR3 = 1000 B-14). Der tatsächliche Peak bei Apollo 11 betrug ≈ 6.5 g;
> GMAX-Vorhersagen im Bereich 6–14 g sind für γ ≈ −6.5° plausibel.

---

### Schritt 4: Zeit bis EI prüfen (V06 N63)

**Eingabe**: `V06N63E`

**Erwartet**:
| Register | Bedeutung | Einheit | Wert beim Szenario-Start |
|----------|-----------|---------|--------------------------|
| R1 | RTGO (Reichweite bis Splashdown) | NM | — |
| R2 | VIO (Geschwindigkeit bei EI) | m/s | ≈ 10 900 |
| R3 | TTE (Zeit bis EI) | MM:SS | ≈ 60:00 |

> **Hinweis zu TTE**: Das AGC zeigt R3 im MMBSS-Format (Minuten:Sekunden), nicht als
> Stunden:Minuten. 60 Minuten vor EI erscheint also `60 00`, nicht `01 00`.
> Das DSKY-Register hat keine Stundenstelle; Werte über 99:59 werden auf 99:59 geklemmt.

---

### Schritt 5: P62 auslösen — PRO-Taste nach N63-Anzeige

P62 startet **nicht** zeitgesteuert, sondern durch einen expliziten
**PROCEED** der Crew, nachdem P61 die N63-Anzeige (RTGO / VIO / TTE) angezeigt hat.

**Wann**: Direkt nach Schritt 4, wenn N63 auf dem DSKY blinkt.

**Eingabe**: `PRO` (= V33E)

**Ablauf**:

1. P61 zeigt N63 blinkend an (`RTGO`, `VIO`, `TTE`).
2. Crew drückt **PRO** → `proceed_flag` wird gesetzt.
3. `reentry_tick` erkennt: P61 aktiv + `proceed_flag` → PROG wechselt auf **62**.
4. P62 fällt im selben Tick sofort auf **63** durch (kein IMU-Manöver in der Sim).
5. PROG zeigt **63** — der AGC wartet nun auf .05G.

> **Hinweis**: Im echten AGC hätte P62 Zeit für ein CM/SM-Trennungs-Manöver
> (~10 min vor EI, ca. t = −600 s).  In der Simulation wird dieser Schritt
> überbrückt; P62 → P63 erfolgt sofort.

**Erwartet**:

| Anzeige | Wert    |
|---------|---------|
| PROG    | 63      |

Die Simulation läuft weiter — kein weiterer Eingriff nötig bis .05G (Schritt 6).

---

### Schritt 6: P64 beobachten — automatischer Eintritt bei .05G

~90 Sekunden nach dem simulierten EI-Zeitpunkt (t = 0) schaltet PROG
**automatisch auf 64**.

Rollsteuerung beobachten:

**Eingabe**: `V16N74E`
= VERB → 1 → 6 → NOUN → 7 → 4 → ENTER

| Register | Bedeutung | Einheit |
|----------|-----------|---------|
| R1 | ROLLC (Roll-Kommando) | 0.01° |
| R2 | VI (Inertialgeschwindigkeit) | m/s |
| R3 | D (Drag-Beschleunigung) | 0.01 g |

---

### Schritt 7: Roll-Kommandos im Monitor (V16 N69)

**Eingabe**: `V16N69E`

| Register | Bedeutung | Einheit |
|----------|-----------|---------|
| R1 | ROLLC (Roll-Kommando) | 0.01° |
| R2 | DL (Drag-Level) | 0.01 m/s² |

---

### Schritt 8: P65 — Up-Control Phase (automatisch)

P65 startet **automatisch** aus P64, sobald die geschätzte Restreichweite
zum Splashdown-Punkt unter **25 NM** fällt (GOTOUPSY-Sprung in `REENTRY_CONTROL.agc`).

PROG zeigt **65**. Anzeige N69 weiterhin nutzbar:

**Eingabe**: `V16N69E`

| Register | Bedeutung | Einheit |
|----------|-----------|---------|
| R1 | ROLLC (Roll-Kommando) | 0.01° |
| R2 | DL (Drag-Level, Q7) | 0.01 m/s² |
| R3 | VL (Zielgeschwindigkeit) | m/s |

> **Hinweis**: Die Rollsteuerung bleibt in P65 aktiv. HUNTEST überwacht
> weiterhin D gegenüber Q7. Fällt D unter Q7 (≈ 6 ft/s²), wechselt P65
> automatisch in P66.

---

### Schritt 9: P66 — Ballistische Trimmphase (automatisch)

P66 startet **automatisch** aus P65, wenn D unter Q7 fällt (KEP-Sprung in
`REENTRY_CONTROL.agc`). Dies tritt typischerweise auf, wenn das CM kurz
die dichte Atmosphäre verlässt (Skip-out) oder vor dem erneuten
Eintauchpunkt.

PROG zeigt **66**. Gimbal-Winkel-Anzeige (N22):

**Eingabe**: `V06N22E`

| Register | Bedeutung | Einheit |
|----------|-----------|---------|
| R1 | OGA (Outer Gimbal Angle) | 0.01° |
| R2 | IGA (Inner Gimbal Angle) | 0.01° |
| R3 | MGA (Middle Gimbal Angle) | 0.01° |

> **Hinweis**: In P66 werden **keine Roll-Kommandos** mehr erzeugt —
> das CM trimmt sich nur zur Relativgeschwindigkeit. Der letzte Roll-Befehl
> aus P65 (ROLLHOLD) wird eingefroren. Sobald D wieder über Q7 + 0.5 ft/s²
> (≈ 6.5 ft/s²) steigt, wechselt P66 automatisch in P67 (PREFINAL).

---

### Schritt 10: P67 — Terminal Phase (automatisch)

P67 startet **automatisch** aus P66 (oder direkt aus P65 bei Skip-out).
PROG zeigt **67**.

**Eingabe**: `V16N67E`

| Register | Bedeutung | Einheit |
|----------|-----------|---------|
| R1 | RTOGO (Restabstand Splashdown) | km |
| R2 | LAT(ZIE) (Ziel-Breite) | 0.01° |
| R3 | LON(ZIE) (Ziel-Länge) | 0.01° |

---

### Schritt 11: Guidance-Fehler-Monitor (V16 N66)

**Eingabe**: `V16N66E`

Monitor-Modus (1× pro Sekunde aktualisiert):

| Register | Bedeutung | Einheit |
|----------|-----------|---------|
| R1 | ROLLC (Roll-Kommando) | 0.01° |
| R2 | XRNGERR (Quer-Reichweitenfehler) | NM |
| R3 | DNRNGERR (Längs-Reichweitenfehler) | NM |

---

## Quellen

- `Comanche055/P61-P67.agc`, P61/P62/P63/P64/P65/P66/P67-Programme
- `Comanche055/REENTRY_CONTROL.agc`, SCALEPOP/INITROLL/GOTOUPSY/UPCONTRL/KEP/KEP2/PREFINAL/PREDICT3/P67.1
- `Comanche055/SERVICER207.agc`, AVGEXIT/READACCS-Zyklus (alle 2 s)
- `Comanche055/PINBALL_NOUN_TABLES.agc`, N60, N61, N63, N66, N67, N69, N74
- `Comanche055/ASSEMBLY_AND_OPERATION_INFORMATION.agc`, N61
- *The Apollo Guidance Computer*, Kap. 4 „Command Module entry" (Book S. 341–358 · PDF S. 357–374)
