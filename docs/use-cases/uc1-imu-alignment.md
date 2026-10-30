# UC-1: IMU-Alignment (P51 → P52)

**Szenario**: `docs/scenarios/leo_circ.scenario` (LEO 400 km Kreisbahn)  
**Integrationstest**: `agc-sim/tests/uc1_imu_alignment.rs`  
**Programme**: P51 (IMU Coarse Alignment), P52 (IMU Fine Alignment)

---

## Überblick

Der IMU-Alignment-Ablauf richtet das Trägheitsnavigationssystem (IMU) des
Command Modules aus. P51 führt eine Grobausrichtung (Coarse Align) durch,
P52 eine Feinausrichtung (Fine Align) durch Gyro-Torquing.

Dieser Use Case benötigt keine aktiven Guidance-Programme davor und ist
daher der einfachste eigenständige Demo-Ablauf.

---

## Szenario laden

```sh
cargo run -p agc-sim -- --scenario docs/scenarios/leo_circ.scenario
```

---

## Checkliste

### Schritt 1: P51 starten (IMU Coarse Alignment)

**Eingabe**: `V37E51E`  
= VERB → 3 → 7 → ENTER → 5 → 1 → ENTER

**Erwartet**:
| Anzeige | Wert |
|---------|------|
| PROG | 51 |

---

### Schritt 2: CoarseAlign-Prompt auslösen

**Eingabe**: `V41E`  
= VERB → 4 → 1 → ENTER

**Erwartet**:
- VERB/NOUN-Anzeige blinkt (Flash aktiv)
- NOUN zeigt `22` (THETAD — Ziel-Gimbal-Winkel)

---

### Schritt 3: Ziel-Gimbal-Winkel eingeben (N22, Einheit 0.01°)

**Eingabe**: Drei Werte (OGA, MGA, IGA) direkt als Zifferneingabe

Beispiel: OGA = 10.00°, MGA = 20.00°, IGA = 30.00°
(als Centideg: 1000, 2000, 3000)

Tasteneingabe:
```
1 0 0 0 E   ← OGA = 1000 centideg
2 0 0 0 E   ← MGA = 2000 centideg
3 0 0 0 E   ← IGA = 3000 centideg
```

> **Hinweis**: Kein vorangestelltes **N** nötig. Während das V/N-Feld blinkt und
> V25 N22 angezeigt wird, werden Ziffern direkt als Datenwert akkumuliert —
> genau wie beim echten DSKY-Dateneingabe-Automaten (ENTPAS/STOREDAT).
> Nach dem dritten ENTER erlischt der Flash automatisch.

**Erwartet**:
- Flash erlischt nach dem dritten ENTER (IGA)
- `alignment_cmd = CoarseAlign` intern gesetzt

---

### Schritt 4: CDU-Winkel prüfen

**Eingabe**: `V06E20E`  
= VERB → 0 → 6 → ENTER → 2 → 0 → ENTER

**Erwartet**:
| Register | Wert | Bedeutung |
|----------|------|-----------|
| R1 | ≠ Blank | OGA (Outer Gimbal Angle) |
| R2 | ≠ Blank | MGA (Middle Gimbal Angle) |
| R3 | ≠ Blank | IGA (Inner Gimbal Angle) |

---

### Schritt 5: P52 starten (IMU Fine Alignment)

**Eingabe**: `V37E52E`  
= VERB → 3 → 7 → ENTER → 5 → 2 → ENTER

**Erwartet**:
| Anzeige | Wert |
|---------|------|
| PROG | 52 |

---

### Schritt 6: FineAlign-Prompt auslösen

**Eingabe**: `V42E`  
= VERB → 4 → 2 → ENTER

**Erwartet**:
- VERB/NOUN-Anzeige blinkt
- NOUN zeigt `93` (DELTA GYRO ANGLES)

---

### Schritt 7: Gyro-Korrekturwinkel bestätigen und FineAlign auslösen

**PRO** drücken (`P` oder `V33E`):

**Erwartet**:
- Flash erlischt
- `alignment_cmd = FineAlign` intern gesetzt
- P52 schließt alignment_cmd-Pfad ab

---

## Quellen

- `Comanche055/EXTENDED_VERBS.agc`, VBCOARK (V41), IMUFINEK (V42)
- `Comanche055/PINBALL_NOUN_TABLES.agc`, N22 (THETAD), N93 (DELTA GYRO ANGLES)
- *The Apollo Guidance Computer*, Kap. 8 „IMU Alignment"
