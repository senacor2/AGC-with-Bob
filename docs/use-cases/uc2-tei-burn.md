# UC-2: TEI-Vorbereitung (P30 → P40)

**Szenario**: `docs/scenarios/tei_pre.scenario` (Mondorbit 110 km, vor TEI-Burn)  
**Integrationstest**: `agc-sim/tests/uc2_tei_burn.rs`  
**Programme**: P30 (External ΔV Targeting), P40 (SPS-Triebwerk-Burn)

---

## Überblick

Der Trans-Earth-Injection-Burn (TEI) befeuert das SPS-Triebwerk, um das
Apollo-Raumschiff aus dem Mondorbit auf eine Erdrückkehrbahn zu bringen.

- **P30** berechnet den Burn-Plan (Δv, Apogee/Perigee nach Manöver).
- **P40** führt den Burn aus (Zündsequenz, Brenndauer, Δv-Monitor).

---

## Szenario laden

```sh
cargo run -p agc-sim -- --scenario docs/scenarios/tei_pre.scenario
```

Der Simulator erkennt automatisch den Mondorbit (Bahnradius < 3 000 km)
und wählt den Mond als Zentralkörper für den Physik-Integrator.

---

## Checkliste

> **Alarm 1400 — Ursache (behoben)**
>
> Der Alarm `P1400` (`0x0300`) trat auf, weil `perigee_apogee` eine
> **hyperbolische Post-Burn-Bahn** fälschlicherweise als Fehler behandelte.
> Ein TEI-Burn von +905 m/s along-track übersteigt die Mondfluchtgeschwindigkeit
> (~2 306 m/s), die resultierende Bahn ist hyperbolisch (kein Mondapogee).
> Der echte AGC behandelt diesen Fall mit `INFINAPO` (Apogee = +∞, kein Alarm).
> Die Rust-Implementierung folgt nun diesem Verhalten: bei hyperbolischer Bahn
> wird `h_apo = f64::MAX` gesetzt; die TUI zeigt den Maximalwert an.
>
> **Dieser Fehler ist behoben** und tritt nicht mehr auf.

### Schritt 0: Δv-Manövervektor eingeben (V25 N81)

Vor P30 muss die Crew den gewünschten Δv-Impuls im lokalen Vertikalsystem
(LV) eingeben: `[radial, along-track, normal]` in m/s.

Für den Apollo-11-TEI-Burn: ≈ 905 m/s along-track (R2), radial und normal ≈ 0.

**Eingabe**: `V25N81E` dann die drei Werte einzeln bestätigen
= VERB → 2 → 5 → NOUN → 8 → 1 → ENTER

Danach erscheint das Blinken (VERB/NOUN-Flash). Die drei Werte werden
nacheinander als Ziffernfolge + ENTER übernommen:

| Register | Wert (Beispiel Apollo 11 TEI) | Einheit |
|----------|-------------------------------|---------|
| R1 | `0` (radial) | m/s |
| R2 | `905` (along-track) | m/s |
| R3 | `0` (normal) | m/s |

---

### Schritt 1: P30 starten (External ΔV Targeting)

**Eingabe**: `V37E30E`
= VERB → 3 → 7 → ENTER → 3 → 0 → ENTER

**Erwartet**:
| Anzeige | Wert |
|---------|------|
| PROG | 30 |

P30 liest den in Schritt 0 eingegebenen Δv-Vektor und berechnet sofort
Apogee/Perigee der Post-Burn-Bahn sowie den in ECI rotierten Δv-Vektor.

---

### Schritt 2: Burn-Plan prüfen (N44: HAPO/HPER/TFF)

**Eingabe**: `V06N44E`
= VERB → 0 → 6 → NOUN → 4 → 4 → ENTER

**Erwartet**:
| Register | Noun | Wert (Beispiel) | Einheit |
|----------|------|-----------------|---------|
| R1 | N44 | HAPO ≈ 400 000 (Erdrückkehrbahn-Apogee) | km |
| R2 | N44 | HPER ≈ −5 300 (unter Erdradius = Eintritt) | km |
| R3 | N44 | TFF = 0 (noch nicht berechnet) | s |

Alle Register nicht Blank → P30 hat Burn-Plan berechnet.

---

### Schritt 3: Δv-Komponenten prüfen (N81: DVx/DVy/DVz)

> **Voraussetzung**: Schritt 0 muss vor P30-Start durchgeführt worden sein.
> Ohne Δv-Eingabe zeigen alle Register `0`.

**Eingabe**: `V06N81E`
= VERB → 0 → 6 → NOUN → 8 → 1 → ENTER

N81 zeigt den von P30 in ECI rotierten Δv-Vektor (DELVSIN).

**Erwartet** (nach Schritt 0 mit `[0, 905, 0]` m/s LV):
| Register | Bedeutung | Wert (Beispiel) | Einheit |
|----------|-----------|-----------------|---------|
| R1 | DVx (ECI x-Komponente) | ≠ 0 | m/s |
| R2 | DVy (ECI y-Komponente) | ≠ 0 | m/s |
| R3 | DVz (ECI z-Komponente) | ≈ 0 | m/s |

---

### Schritt 4: P40 starten (SPS-Burn)

**Eingabe**: `V37E40E`  
= VERB → 3 → 7 → ENTER → 4 → 0 → ENTER

**Erwartet**:
| Anzeige | Wert |
|---------|------|
| PROG | 40 |

---

### Schritt 5: Burn-Monitor einschalten (V16 N40)

**Eingabe**: `V16N40E`
= VERB → 1 → 6 → NOUN → 4 → 0 → ENTER

**Erwartet**:
- Monitor-Modus aktiv (R1/R2/R3 werden 1× pro Sekunde aktualisiert)

| Register | Noun | Wert | Einheit |
|----------|------|------|---------|
| R1 | N40 | TTOGO (Zeit bis Cutoff) | s |
| R2 | N40 | VG (Geschwindigkeitsrest) | m/s |
| R3 | N40 | DVTOTAL (akkumuliertes Δv) | m/s |

---

## Hinweise

- Der TEI-Burn dauert typisch ~2 min 30 s (Apollo 11: 151 s).
- DVTOTAL wächst während des Burns bis zum Ziel-Δv.
- Nach Burn-Ende: P40-Phase wechselt auf 0 (done).

---

## Quellen

- `Comanche055/P30,P37.agc`, P30-Routine (External ΔV Targeting)
- `Comanche055/P40-P47.agc`, P40-Routine (SPS-Burn)
- `Comanche055/PINBALL_NOUN_TABLES.agc`, N40, N44, N81
- *The Apollo Guidance Computer*, Kap. 5 „External ΔV Targeting"
- Vallado, *Fundamentals of Astrodynamics*, §5.4
