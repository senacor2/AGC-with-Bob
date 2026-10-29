# UC-2: TEI-Vorbereitung (P30 → P40)

**Szenario**: `docs/scenarios/tei_pre.scenario` (Mondorbit 110 km, **15 min vor TIG-TEI**)  
**Integrationstest**: `agc-sim/tests/uc2_tei_burn.rs`  
**Programme**: P30 (External ΔV Targeting), P40 (SPS-Triebwerk-Burn)

---

## Überblick

Der Trans-Earth-Injection-Burn (TEI) befeuert das SPS-Triebwerk, um das
Apollo-Raumschiff aus dem Mondorbit auf eine Erdrückkehrbahn zu bringen.

- **P30** berechnet den Burn-Plan (Δv, Apogee/Perigee nach Manöver).  
  Wenn ein TIG gesetzt ist, propagiert P30 den aktuellen SV per Kepler-Schritt
  bis zum TIG und rechnet dort — das Manöver wird also für den richtigen Punkt
  auf der Bahn berechnet.
- **P40** führt den Burn aus. Startet automatisch zum TIG (kein V33 PRO nötig),
  sobald die Sim-MET den TIG erreicht.

Das Szenario startet **15 Minuten vor TIG** (MET 486 522 s = TIG − 900 s),
damit die Crew Zeit hat, alle Eingaben vorzunehmen.

---

## Szenario laden

```sh
cargo run -p agc-sim -- --scenario docs/scenarios/tei_pre.scenario
```

Der Simulator erkennt automatisch den Mondorbit (Bahnradius < 3 000 km)
und wählt den Mond als Zentralkörper für den Physik-Integrator.

| Szenario-Parameter | Wert |
|--------------------|------|
| Startzeit (MET)    | 486 522 s (= TIG − 15 min) |
| TIG-TEI (MET)      | 487 422 s (= 135:23:42) |
| Bahnhöhe           | ~110 km Kreisbahn |
| Fahrzeugmasse      | 28 802 kg (CSM vor TEI) |
| Ziel-Δv            | ≈ 905 m/s along-track |

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

---

### Schritt 0: TIG von Houston annehmen und per V25 N33 eingeben

MCC-Houston übermittelt den Zündzeitpunkt (TIG) per Funk an die Crew.

> **Apollo-11-Referenz (Mission Report, Kap. 4.1.3)**  
> TIG-TEI = GET 135:23:42.4  →  R1 = **135**, R2 = **23**, R3 = **42**

**Eingabe**: `V25N33E` → Flash erscheint, dann drei Werte einzeln bestätigen

```
V25 N33 ENTER
+ 1 3 5 ENTER    ← R1 = 135 (Stunden)
+ 2 3 ENTER      ← R2 = 23  (Minuten)
+ 4 2 ENTER      ← R3 = 42  (Sekunden)
```

Nach der Eingabe:
- Flash erlischt
- `ctx.guidance.tig_s = 487 422 s`
- P30 wird den SV später bis zu diesem Zeitpunkt propagieren
- P40 zündet **automatisch** wenn die Sim-MET den TIG erreicht

**TIG prüfen** (optional): `V06N33E`

| Register | Wert | Bedeutung |
|----------|------|-----------|
| R1 | 135 | Stunden |
| R2 | 23 | Minuten |
| R3 | 42 | Sekunden |

**Countdown bis TIG beobachten** (optional): `V16N35E`

| Register | Wert | Bedeutung |
|----------|------|-----------|
| R1 | MM:SS | Verbleibende Zeit bis TIG |
| R2 | — | Blank |
| R3 | — | Blank |

---

### Schritt 1: Δv-Manövervektor eingeben (V25 N81)

Vor P30 muss die Crew den gewünschten Δv-Impuls im lokalen Vertikalsystem
(LV) eingeben: `[radial, along-track, normal]` in m/s.

Für den Apollo-11-TEI-Burn: ≈ 905 m/s along-track (R2), radial und normal ≈ 0.

**Eingabe**: `V25N81E` dann die drei Werte einzeln bestätigen:

```
V25 N81 ENTER
+ 0 ENTER        ← R1 = 0   (radial)
+ 9 0 5 ENTER    ← R2 = 905 (along-track)
+ 0 ENTER        ← R3 = 0   (normal)
```

| Register | Wert (Apollo 11 TEI) | Einheit |
|----------|----------------------|---------|
| R1 | `0` | m/s |
| R2 | `905` | m/s |
| R3 | `0` | m/s |

---

### Schritt 2: P30 starten (External ΔV Targeting)

**Eingabe**: `V37E30E`

**Erwartet**:
| Anzeige | Wert |
|---------|------|
| PROG | 30 |

P30 liest den TIG aus `guidance.tig_s`, propagiert den aktuellen SV per
Kepler-Schritt bis dorthin und berechnet dann den Burn-Plan (DELVSIN,
HAPO, HPER) für den richtigen Bahnpunkt.

---

### Schritt 3: Burn-Plan prüfen (N44: HAPO/HPER/TFF)

**Eingabe**: `V06N44E`

**Erwartet**:
| Register | Noun | Wert (Beispiel) | Einheit |
|----------|------|-----------------|---------|
| R1 | N44 | HAPO ≈ 400 000 (Erdrückkehrbahn-Apogee) | km |
| R2 | N44 | HPER ≈ −5 300 (unter Erdradius = Eintritt) | km |
| R3 | N44 | TFF = 0 (noch nicht berechnet) | MM:SS |

Alle Register nicht Blank → P30 hat Burn-Plan berechnet.

---

### Schritt 4: Δv-Komponenten prüfen (N81: DVx/DVy/DVz)

> **Voraussetzung**: Schritt 1 muss vor P30-Start durchgeführt worden sein.
> Ohne Δv-Eingabe zeigen alle Register `0`.

**Eingabe**: `V06N81E`

N81 zeigt den von P30 in ECI rotierten Δv-Vektor (DELVSIN), berechnet
**am TIG-SV** (nach Kepler-Propagation).

**Erwartet** (nach Schritt 1 mit `[0, 905, 0]` m/s LV):
| Register | Bedeutung | Wert (Beispiel) | Einheit |
|----------|-----------|-----------------|---------|
| R1 | DVx (ECI x-Komponente) | ≠ 0 | m/s |
| R2 | DVy (ECI y-Komponente) | ≠ 0 | m/s |
| R3 | DVz (ECI z-Komponente) | ≈ 0 | m/s |

---

### Schritt 5: P40 starten (SPS-Burn-Programm)

**Eingabe**: `V37E40E`

**Erwartet**:
| Anzeige | Wert |
|---------|------|
| PROG | 40 |

P40 liest den DELVSIN-Betrag aus dem P30-Ergebnis und die Fahrzeugmasse aus
`guidance.vehicle_mass_kg`. Damit berechnet es in Phase 2 die Brenndauer
(Tsiolkovsky-Gleichung) und legt `ctx.guidance.p40` an — erst dann hat
`N40` auflösbare Werte.

> **Voraussetzung**: P30 muss bereits gelaufen sein (Schritt 2).
> Ohne P30-Ergebnis bleibt `ctx.guidance.p40 = None` und N40 zeigt Blank.

---

### Schritt 6: Burn-Monitor einschalten (V16 N40)

**Eingabe**: `V16N40E`

**Erwartet**:
- Monitor-Modus aktiv — Werte erscheinen **sofort** und werden danach 1× pro Sekunde aktualisiert

| Register | Noun | Inhalt | Wert vor Zündung | Einheit |
|----------|------|--------|------------------|---------|
| R1 | N40 | TTOGO — Restbrennzeit bis Cutoff | berechnete Gesamtbrennzeit (≈ MM:SS) | MM:SS |
| R2 | N40 | VGDISP — Ziel-Δv (aus P30 DELVSIN-Betrag) | ≈ 905 | m/s |
| R3 | N40 | DVTOTAL — akkumuliertes Δv seit Ignition | 0 | m/s |

> **Hinweis**: R1/R2/R3 bleiben **Blank**, wenn P40 nicht gestartet wurde
> (Schritt 5 fehlt) oder P30 nicht vor P40 ausgeführt wurde.

---

### Schritt 7: Burn zum TIG — automatisch

P40 startet **automatisch** wenn die Sim-MET den TIG erreicht.
Die Crew muss **kein V33 PRO** drücken.

P40 ist bereits gestartet (Schritt 5) und durchläuft dann automatisch seine Phasen:

| Phase | Bedeutung |
|-------|-----------|
| 2 | Pre-Ignition: Brennzeit berechnen, Triebwerk zünden |
| 3 | Burn: Zeit akkumulieren, DVTOTAL wächst |
| 4 | Cutoff: Triebwerk aus, Phase → 0 (done) |

DVTOTAL in R3 wächst während des Burns bis auf ≈ 905 m/s.
Nach Burn-Ende: PROG zeigt weiter 40, TTOGO = 00:00.

---

## Hinweise

- Der TEI-Burn dauert typisch ~2 min 30 s (Apollo 11: 151 s bei 905 m/s / 28 802 kg).
- HAPO nach TEI = `2147483647 km` (i32-Max): korrekt — die Bahn ist
  hyperbolisch (Fluchtbahn), entspricht AGC `INFINAPO`.
- Der Simulator akkumuliert DVTOTAL noch nicht aus der IMU-Integration —
  das Feld wächst erst, wenn `p40_step()` mit `dt > 0` aufgerufen wird
  (Sub-Task 7: Guidance-Job-Verdrahtung).

---

## Fehlerbehebung

| Problem | Ursache | Lösung |
|---------|---------|--------|
| R1/R2/R3 Blank nach V16N40E | P40 noch nicht gestartet (`ctx.guidance.p40 = None`) | Schritt 5: `V37E40E` vor `V16N40E` ausführen |
| R1/R2/R3 Blank nach V16N40E (P40 bereits aktiv) | P30 wurde nicht vor P40 gestartet | Schritt 2: `V37E30E` zuerst ausführen |
| N33 zeigt Blank | TIG nicht eingegeben | Schritt 0: V25N33E ausführen |
| P40 zündet nicht automatisch | TIG liegt in der Vergangenheit oder P30 fehlt | TIG neu eingeben, P30 starten |
| Alarm 0440 | `delta_v_target ≤ 0` oder `mass_kg ≤ 0` | Szenario `mass_kg` prüfen |
| Alarm 0300 | Ungültiger SV beim P30-Start | Szenario `pos_*`-Felder prüfen |
| HAPO = 2147483647 km | TEI → hyperbolische Bahn (INFINAPO) | Kein Fehler — erwartet |

---

## Quellen

- `Comanche055/P30,P37.agc`, P30-Routine (External ΔV Targeting)
- `Comanche055/P40-P47.agc`, P40-Routine (SPS-Burn)
- `Comanche055/PINBALL_NOUN_TABLES.agc`, N33, N35, N40, N44, N81
- *Apollo 11 Mission Report* (MSC-00171), Kap. 4.1.3 (TEI-TIG, Burn-Parameter)
- *The Apollo Guidance Computer*, Kap. 5 „External ΔV Targeting"
- Vallado, *Fundamentals of Astrodynamics*, §5.4
