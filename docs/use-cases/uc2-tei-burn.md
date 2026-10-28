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

### Schritt 1: P30 starten (External ΔV Targeting)

**Eingabe**: `V37E30E`  
= VERB → 3 → 7 → ENTER → 3 → 0 → ENTER

**Erwartet**:
| Anzeige | Wert |
|---------|------|
| PROG | 30 |

P30 berechnet sofort Burn-Parameter aus dem aktuellen Bahnzustand.

---

### Schritt 2: Burn-Plan prüfen (N44: HAPO/HPER/TFF)

**Eingabe**: `V06E44E`  
= VERB → 0 → 6 → ENTER → NOUN → 4 → 4 → ENTER

**Erwartet**:
| Register | Noun | Wert | Einheit |
|----------|------|------|---------|
| R1 | N44 | HAPO (Apogee-Höhe nach Manöver) | km |
| R2 | N44 | HPER (Perigee-Höhe nach Manöver) | km |
| R3 | N44 | TFF (Zeit bis Atm.-Eintritt) | s |

Alle Register nicht Blank → P30 hat Burn-Plan berechnet.

---

### Schritt 3: Δv-Komponenten prüfen (N81: DVx/DVy/DVz)

**Eingabe**: `V06E81E`  
= VERB → 0 → 6 → ENTER → NOUN → 8 → 1 → ENTER

**Erwartet**:
| Register | Bedeutung | Einheit |
|----------|-----------|---------|
| R1 | DVx (ECI x-Komponente) | m/s |
| R2 | DVy (ECI y-Komponente) | m/s |
| R3 | DVz (ECI z-Komponente) | m/s |

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

**Eingabe**: `V16E40E`  
= VERB → 1 → 6 → ENTER → NOUN → 4 → 0 → ENTER

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
