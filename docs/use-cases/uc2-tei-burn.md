# UC-2: TEI-Vorbereitung (P30 → P40)

**Szenario**: `docs/scenarios/tei_pre.scenario` (Mondorbit 110 km, **15 min vor TIG-TEI**)  
**Integrationstest**: `agc-sim/tests/uc2_tei_burn.rs`  
**Programme**: P30 (External ΔV Targeting), P40 (SPS-Triebwerk-Burn)

---

## Überblick

Der Trans-Earth-Injection-Burn (TEI) befeuert das SPS-Triebwerk, um das
Apollo-Raumschiff aus dem Mondorbit auf eine Erdrückkehrbahn zu bringen.

- **P30** berechnet den Burn-Plan (Δv, Apogee/Perigee nach Manöver).
- **P40** führt den Burn aus (Zündsequenz, Brenndauer, Δv-Monitor).

Das Szenario startet **15 Minuten vor TIG** (MET 486 522 s = TIG − 900 s),
damit die Crew ausreichend Zeit hat, alle Eingaben vorzunehmen und die
Burn-Parameter zu prüfen, bevor das Triebwerk zündet.

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

### Schritt 0: TIG kennen — Houston gibt den Zündzeitpunkt durch

Im echten Apollo-Flug übermittelt MCC-Houston den vorausberechneten
Zündzeitpunkt (TIG) per Funk, bevor die Crew mit der Burn-Vorbereitung beginnt.

> **Apollo-11-Referenz (Mission Report, Kap. 4.1.3)**  
> TIG-TEI = GET 135:23:42.4 = MET **487 422 s**

Das Szenario startet bei MET 486 522 s, d. h. **15 Minuten vor TIG**.
Die Crew hat also bis MET 487 422 s Zeit, die Schritte 1–4 abzuarbeiten
und P40 zum TIG mit V33 (PRO) auszulösen.

Kein DSKY-Input in diesem Schritt — nur Situationsbewusstsein.

---

### Schritt 1: Δv-Manövervektor eingeben (V25 N81)

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

### Schritt 2: P30 starten (External ΔV Targeting)

**Eingabe**: `V37E30E`
= VERB → 3 → 7 → ENTER → 3 → 0 → ENTER

**Erwartet**:
| Anzeige | Wert |
|---------|------|
| PROG | 30 |

P30 liest den in Schritt 1 eingegebenen Δv-Vektor und berechnet sofort
Apogee/Perigee der Post-Burn-Bahn sowie den in ECI rotierten Δv-Vektor.

---

### Schritt 3: Burn-Plan prüfen (N44: HAPO/HPER/TFF)

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

### Schritt 4: Δv-Komponenten prüfen (N81: DVx/DVy/DVz)

> **Voraussetzung**: Schritt 1 muss vor P30-Start durchgeführt worden sein.
> Ohne Δv-Eingabe zeigen alle Register `0`.

**Eingabe**: `V06N81E`
= VERB → 0 → 6 → NOUN → 8 → 1 → ENTER

N81 zeigt den von P30 in ECI rotierten Δv-Vektor (DELVSIN).

**Erwartet** (nach Schritt 1 mit `[0, 905, 0]` m/s LV):
| Register | Bedeutung | Wert (Beispiel) | Einheit |
|----------|-----------|-----------------|---------|
| R1 | DVx (ECI x-Komponente) | ≠ 0 | m/s |
| R2 | DVy (ECI y-Komponente) | ≠ 0 | m/s |
| R3 | DVz (ECI z-Komponente) | ≈ 0 | m/s |

---

### Schritt 5: P40 starten (SPS-Burn)

> **Voraussetzung**: Schritt 2 (P30) muss zuvor ausgeführt worden sein.
> P40 liest den von P30 berechneten `DELVSIN`-Betrag als Ziel-Δv und die
> Fahrzeugmasse aus dem Szenario (`mass_kg = 28802`) für die Tsiolkovsky-Gleichung.
> Ohne vorheriges P30 bleibt `ctx.guidance.p40 == None` → N40 zeigt Blank.

**Eingabe**: `V37E40E`
= VERB → 3 → 7 → ENTER → 4 → 0 → ENTER

**Erwartet**:
| Anzeige | Wert |
|---------|------|
| PROG | 40 |

---

### Schritt 6: Burn-Monitor einschalten (V16 N40)

**Eingabe**: `V16N40E`
= VERB → 1 → 6 → NOUN → 4 → 0 → ENTER

**Erwartet**:
- Monitor-Modus aktiv (R1/R2/R3 werden 1× pro Sekunde aktualisiert)

| Register | Noun | Inhalt | Einheit |
|----------|------|--------|---------|
| R1 | N40 | TTOGO — Restbrennzeit bis Cutoff | MM:SS |
| R2 | N40 | VGDISP — Ziel-Δv (aus P30 DELVSIN-Betrag) | m/s |
| R3 | N40 | DVTOTAL — akkumuliertes Δv seit Ignition | m/s |

Vor Zündung: TTOGO zeigt die berechnete Gesamtbrennzeit, DVTOTAL = 0.

---

### Schritt 7: Burn zum TIG auslösen (V33 PRO)

Wenn die MET den TIG erreicht (MET 487 422 s), drückt die Crew **PRO** (V33).

P40 durchläuft dann automatisch seine Phasen:

| Phase | Bedeutung |
|-------|-----------|
| 2 | Pre-Ignition: Brennzeit berechnen, Triebwerk zünden |
| 3 | Burn: Zeit akkumulieren, DVTOTAL wächst |
| 4 | Cutoff: Triebwerk aus, Phase → 0 (done) |

DVTOTAL in R3 wächst während des Burns bis auf ≈ 905 m/s.
Nach Burn-Ende: PROG zeigt weiter 40, TTOGO = 00:00.

---

## Hinweise

- Der TEI-Burn dauert typisch ~2 min 30 s (Apollo 11: 151 s bei 905 m/s / 28 802 kg CSM).
- HAPO nach TEI = `2147483647 km` (i32-Max): das ist korrekt — die Bahn ist
  hyperbolisch (Fluchtbahn), entspricht AGC `INFINAPO`. Kein Fehler.
- Der Simulator akkumuliert DVTOTAL noch nicht aus der IMU-Integration —
  das Feld wächst erst, wenn `p40_step()` mit `dt > 0` aufgerufen wird
  (Sub-Task 7: Guidance-Job-Verdrahtung).

---

## Fehlerbehebung

| Problem | Ursache | Lösung |
|---------|---------|--------|
| R1/R2/R3 bleiben Blank nach V16N40E | P30 wurde nicht vor P40 gestartet | Schritt 2 zuerst ausführen |
| R1/R2/R3 bleiben Blank nach korrekter Reihenfolge | Bug in `program_p40()` (behoben) | Simulator neu bauen |
| Alarm 0440 | `delta_v_target ≤ 0` oder `mass_kg ≤ 0` | Szenario prüfen: `mass_kg` muss gesetzt sein |
| Alarm 0300 | Ungültiger Zustandsvektor beim P30-Start | Szenario prüfen: `pos_*`-Felder |
| HAPO = 2147483647 km | TEI-Burn → hyperbolische Bahn (INFINAPO) | Kein Fehler — erwartet nach TEI |

---

## Quellen

- `Comanche055/P30,P37.agc`, P30-Routine (External ΔV Targeting)
- `Comanche055/P40-P47.agc`, P40-Routine (SPS-Burn)
- `Comanche055/PINBALL_NOUN_TABLES.agc`, N40, N44, N81
- *Apollo 11 Mission Report* (MSC-00171), Kap. 4.1.3 (TEI-TIG, Burn-Parameter)
- *The Apollo Guidance Computer*, Kap. 5 „External ΔV Targeting"
- Vallado, *Fundamentals of Astrodynamics*, §5.4
