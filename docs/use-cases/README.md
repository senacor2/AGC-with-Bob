# Use-Case-Checklisten — Format-Konventionen

Dieses Verzeichnis enthält Crew-Checklisten für jeden AGC-Demo-Use-Case.
Jede Checkliste ist:

- **Interaktiv ausführbar** — Schritt-für-Schritt-DSKY-Tasteneingaben
- **Maschinell verifiziert** — korrespondierende Integrationstests in `agc-sim/tests/`
- **Vorbefüllbar** — Szenario-Datei setzt den Startzustand

---

## Verfügbare Use Cases

| # | Titel | Szenario | Testdatei |
|---|-------|----------|-----------|
| UC-1 | [IMU-Alignment (P51 → P52)](uc1-imu-alignment.md) | `leo_circ.scenario` | `uc1_imu_alignment.rs` |
| UC-2 | [TEI-Vorbereitung (P30 → P40)](uc2-tei-burn.md) | `tei_pre.scenario` | `uc2_tei_burn.rs` |
| UC-3 | [Entry (P61 → P64 → P67)](uc3-entry.md) | `entry_pre.scenario` | `uc3_entry.rs` |

---

## Format-Konventionen

### Tasten-Notation

DSKY-Tasten werden als Kurzzeichen dargestellt:

| Zeichen | Taste | Bedeutung |
|---------|-------|-----------|
| `V` | VERB | Verb-Eingabe |
| `N` | NOUN | Noun-Eingabe |
| `E` | ENTER | Bestätigen |
| `P` | PRO | Proceed |
| `R` | RSET | Reset |
| `C` | CLR | Clear |
| `K` | KEY REL | Key Release |
| `+` | PLUS | Vorzeichen + |
| `-` | MINUS | Vorzeichen − |
| `0`–`9` | Ziffern | Zahleneingabe |

**Beispiel**: `V37E61E` = VERB → 3 → 7 → ENTER → 6 → 1 → ENTER

### Programm/Verb/Noun-Referenz

- Programme: `P30`, `P40`, `P51`, `P52`, `P61`, `P64`, `P67`
- Verben: `V06` (einmalige Anzeige), `V16` (Monitor-Modus), `V25` (Laden), `V33` (Proceed), `V37` (Programm wählen), `V41` (Coarse Align), `V42` (Fine Align)
- Nouns: `N20` (CDU-Winkel), `N22` (THETAD), `N40` (TTOGO/VG/DVTOTAL), `N44` (HAPO/HPER/TFF), `N60` (GMAX/VPRED/GAMMAEI), `N61` (LAT/LON SPL), `N66` (Guidance-Fehler), `N67` (RTOGO/LAT/LON), `N93` (Delta Gyro)

### Tabellen-Format für erwartete Display-Werte

Jeder Schritt, der Display-Ausgaben prüft, enthält eine Tabelle:

| Register | Noun | Wert | Einheit |
|----------|------|------|---------|
| R1 | N44 | HAPO | km |
| R2 | N44 | HPER | km |
| R3 | N44 | TFF | s |

### Schrittstruktur

```markdown
### Schritt N: [Beschreibung]

**Eingabe**: `VxxEyyE` (Tastenfolge)  
**Erwartet**: PROG=XX, R1/R2/R3 Werte / nicht Blank  
**Prüft**: [Zustand der Guidance-Variablen]  
```

---

## Szenario-Dateien

Alle Szenario-Dateien liegen in `docs/scenarios/`:

| Datei | Beschreibung | Höhe | Körper |
|-------|-------------|------|--------|
| `leo_circ.scenario` | Erdorbit Kreisbahn 400 km | 400 km | Erde |
| `tei_pre.scenario` | Mondorbit vor TEI-Burn 110 km | 110 km | Mond |
| `entry_pre.scenario` | Wiedereintritt EI Apollo-11 | 122 km | Erde |
