# UC-3: Entry (P61 → P64 → P67)

**Szenario**: `docs/scenarios/entry_pre.scenario` (EI 122 km, Apollo-11-Referenz)  
**Integrationstest**: `agc-sim/tests/uc3_entry.rs`  
**Programme**: P61 (Entry Preparation), P64 (Constant-Drag Phase), P67 (Terminal Phase)

---

## Überblick

Der Wiedereintritts-Ablauf führt die Kapsel von der Entry-Interface-Höhe
(122 km) durch die Atmosphäre bis zum Fallschirm-Einsatz:

- **P61** bereitet den Eintritt vor: Splashdown-Zielkoordinaten und
  Eintrittsparameter werden gesetzt.
- **P64** steuert die Constant-Drag-Phase: Roll-Kommandos halten die
  gewünschte Abbremsung.
- **P67** verwaltet die Terminalphase: Zielrestabstand, aktuelle Position.

---

## Szenario laden

```sh
cargo run -p agc-sim -- --scenario docs/scenarios/entry_pre.scenario
```

Entry Interface Apollo 11:
- Höhe: 122 km
- Geschwindigkeit: ≈10 900 m/s (inertial)
- Eintrittwinkel: γ = −6.5°

---

## Checkliste

### Schritt 1: P61 starten (Entry Preparation)

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

### Schritt 3: Splashdown-Koordinaten prüfen (V06 N61)

**Eingabe**: `V06N61E`
= VERB → 0 → 6 → NOUN → 6 → 1 → ENTER

**Erwartet**:
| Register | Noun | Wert | Einheit |
|----------|------|------|---------|
| R1 | N61 | LAT(SPL) | 0.01° |
| R2 | N61 | LNG(SPL) | 0.01° |
| R3 | N61 | HEADSUP | ±1 |

---

### Schritt 4: Eintrittsparameter prüfen (V06 N60)

**Eingabe**: `V06N60E`
= VERB → 0 → 6 → NOUN → 6 → 0 → ENTER

**Erwartet**:
| Register | Noun | Bedeutung | Einheit |
|----------|------|-----------|---------|
| R1 | N60 | GMAX (max. Belastung) | 0.01 m/s² |
| R2 | N60 | VPRED (Geschwindigkeit bei Splashdown) | m/s |
| R3 | N60 | GAMMAEI (Eintrittwinkel) | 0.01° |

---

### Schritt 5: P64 starten (Constant-Drag Phase)

**Eingabe**: `V37E64E`
= VERB → 3 → 7 → ENTER → 6 → 4 → ENTER

**Erwartet**:
| Anzeige | Wert |
|---------|------|
| PROG | 64 |

Roll-Kommandos steuern Lift: ROLLC in N69 R1, DL in N69 R2.

---

### Schritt 6: Roll-Kommandos prüfen (V06 N69)

**Eingabe**: `V06N69E`
= VERB → 0 → 6 → NOUN → 6 → 9 → ENTER

| Register | Bedeutung | Einheit |
|----------|-----------|---------|
| R1 | ROLLC (Roll-Kommando) | 0.01° |
| R2 | DL (Drag-Level) | 0.01 m/s² |

---

### Schritt 7: P67 starten (Terminal Phase)

**Eingabe**: `V37E67E`
= VERB → 3 → 7 → ENTER → 6 → 7 → ENTER

**Erwartet**:
| Anzeige | Wert |
|---------|------|
| PROG | 67 |

---

### Schritt 8: Position prüfen (V06 N67)

**Eingabe**: `V06N67E`
= VERB → 0 → 6 → NOUN → 6 → 7 → ENTER

| Register | Noun | Bedeutung | Einheit |
|----------|------|-----------|---------|
| R1 | N67 | RTOGO (Restabstand zum Splashdown) | km |
| R2 | N67 | LAT(ZIE) (Ziel-Breite) | 0.01° |
| R3 | N67 | LON(ZIE) (Ziel-Länge) | 0.01° |

---

### Schritt 9: Guidance-Fehler-Monitor (V16 N66)

**Eingabe**: `V16N66E`
= VERB → 1 → 6 → NOUN → 6 → 6 → ENTER

Monitor-Modus: R1/R2/R3 werden 1× pro Sekunde aus P64/P67-Guidance-Daten
aktualisiert (Guidance-Abweichungen, Lateral-Fehler).

---

## Quellen

- `Comanche055/P61-P67.agc`, P61/P64/P67-Programme
- `Comanche055/PINBALL_NOUN_TABLES.agc`, N60, N61, N66, N67, N69
- `Comanche055/ASSEMBLY_AND_OPERATION_INFORMATION.agc`, N61
- *The Apollo Guidance Computer*, Kap. 4 „Command Module entry" (PDF S. 357–374)
