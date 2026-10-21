# DSKY Verb- und Noun-Referenz — Comanche 055 (Apollo 11 CM)

> Quelldateien: `input/virtualagc/Comanche055/`  
> Primäre Implementierung: `PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, `EXTENDED_VERBS.agc`, `PINBALL_NOUN_TABLES.agc`  
> Systemdokumentation: `ASSEMBLY_AND_OPERATION_INFORMATION.agc` (Seiten 5–22)

---

## Architektur-Überblick

Die Kommunikationssprache des DSKY besteht aus **Verb-Noun-Paaren**. Verben beschreiben die Aktion,
Nouns bezeichnen die Daten. Das Pinball-Programm empfängt alle Tasteneingaben als 5-Bit-Codes über
die Interrupt-Kanäle 15 (Navigation Keyboard) und 16 (Uplink).

```
KEYRUPT1/2 (Interrupt)  ──┐
                           ├──▶  CHARIN  ──▶  VERB/NOUN-Eingabe
UPRUPT (Uplink)         ──┘         │
                                    ▼
Internes Programm  ──▶  NVSUB  ──▶  ENTPAS0  ──▶  VERBTAB (V00–V39)
                                               └──▶  LST2FAN (V40–V99)
```

**Ausgabe**: Der 11-Register-Puffer `DSPTAB` wird von `DSPOUT` (im T4RUPT-Interrupt) über
Channel 10 an das DSKY-Relais-Display gesendet.

### Tastencodes (5-Bit, Channel 15/16)

| Taste | Code (binär) | Taste | Code (binär) |
|-------|-------------|-------|-------------|
| 0 | 10000 | KEY RELEASE | 11001 |
| 1–9 | 00001–01001 | + | 11010 |
| VERB | 10001 | − | 11011 |
| ERROR RESET | 10010 | ENTER | 11100 |
| | | CLEAR | 11110 |
| | | NOUN | 11111 |

### NVSUB — Interner Einstiegspunkt

Interne Programme rufen `NVSUB` mit dem V/N-Code in Register `A` auf:
- Bits 0–6 = Noun-Code
- Bits 7–13 = Verb-Code

Varianten: `NVSUB` (kehrt zurück), `NVSBWAIT` (wartet auf Operator-Antwort),
`NVSUBUSY`/`PRENVBSY` (prüft ob DSKY frei ist).

---

## Teil 1 — Regular Verbs (V01–V39)

Implementiert via `VERBTAB` in `PINBALL_GAME__BUTTONS_AND_LIGHTS.agc` (Seiten 325–326).

### Display-Verben (V01–V07)

| Verb | AGC-Routine | Anzeige | Argument / Noun | Beschreibung |
|------|------------|---------|-----------------|--------------|
| **V01** | `DSPA` | R1 | Beliebiger Noun | Zeigt Komponente 1 des Noun **oktal** in R1 |
| **V02** | `DSPB` | R1 | Noun ≥ 2 Komp. | Zeigt Komponente 2 des Noun **oktal** in R1 |
| **V03** | `DSPC` | R1 | Noun ≥ 3 Komp. | Zeigt Komponente 3 des Noun **oktal** in R1 |
| **V04** | `DSPAB` | R1, R2 | Noun ≥ 2 Komp. | Zeigt Komp. 1 und 2 des Noun **oktal** in R1, R2 |
| **V05** | `DSPABC` | R1, R2, R3 | Noun ≥ 3 Komp. | Zeigt alle 3 Komp. des Noun **oktal** in R1–R3 |
| **V06** | `DECDSP` | R1 oder R1–R3 | Noun mit SF-Routine ≠ 0 | **Dezimal-Anzeige** gemäß Noun-Skalierungsfaktor |
| **V07** | `DSPDPDEC` | R1, R2 | Noun mit DP-SF | **Doppelt-genaue Dezimal-Anzeige** (R1=Hi, R2=Lo) — nur Tests |

> **Hinweis:** `COMPTEST` alarmiert, wenn die Komponentenzahl des Verbs die des Nouns übersteigt.
> `DECTEST` alarmiert, wenn ein Oktal-Verb auf einen „Decimal Only"-Noun angewendet wird.

### Monitor-Verben (V11–V17)

Monitor-Verben aktualisieren die Anzeige einmal pro Sekunde (via `MONSAVE`/`MONSAVE1`).
Alle landen in derselben `MONITOR`-Routine — der Verb-Code unterscheidet intern.

| Verb | Anzeige | Entspricht | Beschreibung |
|------|---------|-----------|--------------|
| **V11** | R1 | V01 | Monitor Komp. 1 oktal |
| **V12** | R1 | V02 | Monitor Komp. 2 oktal |
| **V13** | R1 | V03 | Monitor Komp. 3 oktal |
| **V14** | R1, R2 | V04 | Monitor Komp. 1,2 oktal |
| **V15** | R1–R3 | V05 | Monitor Komp. 1,2,3 oktal |
| **V16** | R1–R3 | V06 | Monitor dezimal |
| **V17** | R1, R2 | V07 | Monitor DP-Dezimal (nur Tests) |

### Load-Verben (V21–V25)

Load-Verben fordern den Astronauten zur Dateneingabe auf. Nach `ENTER` werden die Werte
in die Noun-Register geschrieben. Dezimale Eingaben erfordern genau **5 Ziffern** vor jedem
`ENTER`. Nouns mit `NO LOAD`-Bit (Bit 5 im Komponentencode = 1) lehnen V24/V25 ab.

| Verb | AGC-Routine | Eingabe | Argument | Beschreibung |
|------|------------|---------|----------|--------------|
| **V21** | `ALOAD` | R1 (XREG) | Noun 1–3 Komp. | Lädt Komp. 1 in Noun-Register |
| **V22** | `BLOAD` | R2 (YREG) | Noun ≥ 2 Komp. | Lädt Komp. 2 in Noun-Register |
| **V23** | `CLOAD` | R3 (ZREG) | Noun ≥ 3 Komp. | Lädt Komp. 3 in Noun-Register |
| **V24** | `ABLOAD` | R1, R2 | Noun ≥ 2 Komp., kein NO LOAD | Lädt Komp. 1 und 2 |
| **V25** | `ABCLOAD` | R1, R2, R3 | Noun 3 Komp., kein NO LOAD | Lädt alle 3 Komponenten |

### Sonder-Verben (V27–V39)

| Verb | AGC-Routine | Argument | Beschreibung |
|------|------------|----------|--------------|
| **V27** | `DSPFMEM` | Noun 01/02/03 + R3 als FCADR | **Fixed Memory Display** — zeigt Festspeicherinhalt an der angegebenen Adresse |
| **V30** | `VBRQEXEC` | Noun 26 (Prio/Delay, Adres, BBCON) | **Request Executive** — startet einen Executive-Job mit Noun-26-Parametern |
| **V31** | `VBRQWAIT` | Noun 26 | **Request Waitlist** — fügt einen Waitlist-Task mit Noun-26-Parametern ein |
| **V32** | `VBRESEQ` | — | **Recycle** — wiederholt das aktuelle Programm |
| **V33** | `VBPROC` | — | **Proceed without data** — weckt wartende Programme ohne Dateneingabe |
| **V34** | `VBTERM` | — | **Terminate** — bricht aktuelle Last- oder Testanforderung ab |
| **V35** | `VBTSTLTS` | — | **Test Lights** — schaltet alle DSKY-Anzeigeelemente ein |
| **V36** | `SLAP1` | — | **Fresh Start** — kompletter Neustart (äquivalent zu Power-On) |
| **V37** | `MMCHANG` | R1: 2-stellige Programmnummer (dezimal) | **Change Major Mode** — startet Hauptprogramm P00–P99 |

---

## Teil 2 — Extended Verbs (V40–V99)

Implementiert via `LST2FAN` in `EXTENDED_VERBS.agc` (Seiten 236–267).
Werden als separater Executive-Job mit Priorität 30000 ausgeführt.

### IMU / Optik-Verben (V40–V43)

| Verb | AGC-Routine | V-N-Interaktion | Beschreibung |
|------|------------|-----------------|--------------|
| **V40** | `VBZERO` | **Nur Noun 20** (ICDU-Winkel) | **Zero CDU** — führt `IMUZERO` aus, setzt IMU-CDU-Winkel auf 0 |
| **V41** | `VBCOARK` | Noun **20**: V25N22 → ICORK2; Noun **91**: V24N92 | **Coarse Align** — IMU (N20) oder Optics (N91) |
| **V42** | `IMUFINEK` | Noun **93**: Flashing V25N93 | **Fine Align IMU** — Kreiselbefehlswinkel laden, `IMUFINE` + `IMUPULSE` ausführen |
| **V43** | `IMUATTCK` | Noun **22**: Flashing V25N22 | **Load IMU Attitude Error Meters** — überträgt geladene Winkel in AK-Register |

### Flag-Setzer (V44–V47)

| Verb | AGC-Routine | Flag | Beschreibung |
|------|------------|------|--------------|
| **V44** | `SETSURF` | `SURFFLAG` ↑ | **Set Surface Flag** |
| **V45** | `RESTSRF` | `SURFFLAG` ↓ | **Reset Surface Flag** |
| **V46** | `STABLISH` | — | **Establish G&N Autopilot Control** (RCS / Entry / Saturn) |
| **V47** | `LMTOCMSV` | — | **Move LM State Vector into CM State Vector** |

### Autopilot / Manöver (V48–V49)

| Verb | AGC-Routine | V-N-Sequenz | Beschreibung |
|------|------------|-------------|--------------|
| **V48** | `DAPDISP` | V04N46 → V06N47 → V06N48 | **Load DAP Data (R03)** — Konfiguration, CSM/LEM-Gewichte, Pitch/Yaw-Trim |
| **V49** | `CREWMANU` | Flashing V06N22 | **Crew-definiertes Manöver (R62)** — ICDU-Zielwinkel laden, R60CSM ausführen |

### „Please…"-Verben (V50–V53, V59, V97, V99)

Diese Verben zeigen eine blinkende Aufforderung und warten auf `ENTER` (= ausgeführt)
oder V33 (= nicht gewünscht).

| Verb | AGC-Routine | Beschreibung |
|------|------------|--------------|
| **V50** | `GOLOADLV` | **Please Perform** — allgemeine Handlungsaufforderung mit Noun 25 (Checkliste) |
| **V51** | `GOLOADLV` | **Please Mark** — Aufforderung zur Sternmarkierung |
| **V52** | `V52` | **Mark on Offset Landing Site** — setzt Offset-Landezielnummer für P22 |
| **V53** | `GOLOADLV` | **Please Perform Alternate LOS Mark** |
| **V59** | `GOLOADLV` | **Please Calibrate** |
| **V97** | `GOLOADLV` | **Please Perform Engine-Fail Procedure (R41)** |
| **V99** | `GOLOADLV` | **Please Enable Engine** |

### Tracking / Navigation (V54–V57, V86)

| Verb | AGC-Routine | Beschreibung |
|------|------------|--------------|
| **V54** | `GOTOR23` | **Request R23** — Rendezvous-Backup-Sichtmarkierung starten |
| **V55** | `ALINTIME` | **Increment AGC Time (dezimal)** — via V25N24 Delta-Zeit zur AGC-Uhr addieren |
| **V56** | `TRACKTRM` | **Terminate Tracking** — beendet P20+P25, löscht Rendezvous-/Track-Flags |
| **V57** | `GOTOR21` | **Request R21** — Rendezvous-Sichtmarkierungsroutine starten |
| **V86** | `V86PERF` | **Backup Mark Reject** — setzt `MRKBUF1` auf −1 (Markierung verwerfen) |

### Attitude-Modi (V58, V60–V63)

| Verb | AGC-Routine | Flag-Operation | Beschreibung |
|------|------------|---------------|--------------|
| **V58** | `ENATMA` | `STIKFLAG` ↓ | **Reset Stick Flag** — reaktiviert automatisches Tracking nach RHC-Unterbrechung |
| **V60** | `V60` | — | Kopiert aktuelle CDU-Winkel → `CPHIX` (N17) |
| **V61** | `V61` | `NEEDLFLG` ↓ | **Mode I** — DAP-Lagefehler-Anzeige (Phasenebene) |
| **V62** | `V62` | `NEEDLFLG` ↑, `N22ORN17` ↑ | **Mode II** — Gesamte Lagefehler bzgl. N22 (THETAD) |
| **V63** | `V63` | `NEEDLFLG` ↑, `N22ORN17` ↓ | **Mode III** — Gesamte Lagefehler bzgl. N17 (CPHIX) |

### Anzeige-Anforderungen (V64, V67, V82–V85, V89–V90)

| Verb | AGC-Routine | Anzeige | Beschreibung |
|------|------------|---------|--------------|
| **V64** | `VB64` | N51 (S-Band Pitch/Yaw) | **S-Band Antenna Routine** — berechnet und zeigt Antennenwinkel |
| **V67** | `V67` | W-Matrix | **W-Matrix Display** — Navigationsunsicherheitsmatrix |
| **V82** | `V82PERF` | V16N44 (Hapo, Hper, TFF) | **Orbit Parameters Display (R30)** — bei Avg-G aus: vorher V04N06 für Fahrzeugwahl |
| **V83** | `V83PERF` | N53 (Range, RRate, PHI) | **Rendezvous Parameter Display (R31)** |
| **V85** | `V85PERF` | N54 (Range, RRate, THETA) | **Rendezvous Parameter Display Nr. 2 (R34)** |
| **V89** | `V89PERF` | V04N06 → V06N18 | **Rendezvous Final Attitude (R63)** — Gimbal-Winkel für CSM→LM-Ausrichtung |
| **V90** | `V90PERF` | N90 (Y, Ydot, PSI) | **Out-of-Plane Display (R36)** |

### Test / Diagnose (V65, V68–V69, V91–V92)

| Verb | AGC-Routine | Beschreibung |
|------|------------|--------------|
| **V65** | `CKOPTVB` | **Optical Verification** — Prelaunch-Ausrichtungsverifizierung (nur MM02) |
| **V68** | `STROKON` | **CSM Stroke Test ON** — nur während TVC aktiv |
| **V69** | `VERB69` | **Cause Restart** — endloser Selbstaufruf → erzwingt GOJAM (Hardware-Neustart) |
| **V91** | `GOSHOSUM` | **Display Bank Sums** — Checksummen aller Speicherbänke (Hybrid-Test) |
| **V92** | `SYSTEST` | **IMU Performance Test (P07)** — lädt Testnummer 1–17 via V21N01 |

### Update-Verben (V70–V74)

Implementiert in `UPDATE_PROGRAM.agc`. Eingabesequenz: `Vxx E [Daten E] …`

| Verb | AGC-Routine | Eingabe-Sequenz | Beschreibung |
|------|------------|----------------|--------------|
| **V70** | `V70UPDAT` | `V70 E XXXXX E XXXXX E` | **Update Liftoff Time** — DP-Oktal-Zeitinkrement (2 × 5 oktal-Stellen) |
| **V71** | `V71UPDAT` | `V71 E II E AAAA E [Daten × (II−2)]` | **Universal Update Block Address** — II = Komponenten, AAAA = Startadresse |
| **V72** | `V72UPDAT` | `V72 E [(Adresse E Wert E) × n] V33` | **Universal Update Single Address** — Adress/Wert-Paare, V33 zum Beenden |
| **V73** | `V73UPDAT` | `V73 E XXXXX E XXXXX E` | **Update AGC Time (oktal)** — DP-Oktal-Zeitinkrement |
| **V74** | `DNEDUMP` | — | **Initialize Erasable Dump** — ersetzt Downlink-Liste durch Erasable Memory |

### Flag/Vektor-Verben (V75–V81, V87–V88, V93–V96)

| Verb | AGC-Routine | Flag | Beschreibung |
|------|------------|------|--------------|
| **V75** | `LFTFLGON` | `GRRBKFLG` ↑ | **Backup Liftoff** — setzt Guidance-Reference-Release-Backup-Flag |
| **V76** | `SETPRFLG` | `PRFTRKAT` ↑ | **Set Preferred Attitude Flag** |
| **V77** | `RESETPRF` | `PRFTRKAT` ↓ | **Reset Preferred Attitude Flag** |
| **V78** | `CHAZFOGC` | — | **Update Prelaunch Azimuth** (nur MM02) |
| **V80** | `LEMVEC` | `VEHUPFLG` ↓ | **Update LEM State Vector** |
| **V81** | `CSMVEC` | `VEHUPFLG` ↑ | **Update CSM State Vector** |
| **V87** | `SETVHFLG` | `VHFRFLAG` ↑ | **Set VHF Range Flag** — R22 akzeptiert Range-Daten |
| **V88** | `RESETVHF` | `VHFRFLAG` ↓ | **Reset VHF Range Flag** |
| **V93** | `WMATRXNG` | `RENDWFLG`↓, `ORBWFLAG`↓ | **Enable W-Matrix Init** — löscht W-Matrix-Flags |
| **V94** | `VERB94` | — | **Cislunar Attitude Maneuver (P23)** — via ENEMA-Restart |
| **V96** | `VERB96` | `QUITFLAG`↑, `V96ONFLG`↑ | **Terminate Integration → P00** |

---

## Teil 3 — Noun-Tabellen (N00–N99)

Implementiert in `PINBALL_NOUN_TABLES.agc` (Seiten 268–284).

### Skala-Typen (Kurzreferenz)

| Typ | Einheit | Format | Präzision |
|-----|---------|--------|-----------|
| A | Oktal | XXXXX | SP |
| B | Bruchwert | .XXXXX | SP, Bit1 = 2⁻¹⁴ |
| C | Ganzzahl | XXXXX. | SP, Bit1 = 1 Einheit |
| D | CDU-Grad | XXX.XX° | SP, Bit1 = 360/2¹⁵ ° |
| E | Elevationsgrad | XX.XXX° | SP, Bit1 = 90/2¹⁴ ° |
| G | DP-Grad (90°) | XX.XXX° | DP, Bit1 Low = 360/2²⁸ ° |
| H | DP-Grad (360°) | XXX.XX° | DP, Bit1 Low = 360/2²⁸ ° |
| J | Y-Optik-Grad | XX.XXX° | SP, Bias +19.775° bei Anzeige |
| K | Zeit H:M:S | 00XXX. / 000XX. / 0XX.XX | DP, Bit1 Low = 10⁻² s |
| L | Zeit M:S | XXBXX | DP, Bit1 Low = 10⁻² s, NUR ANZEIGE |
| P | Geschwindigkeit2 | XXXXX. ft/s | DP, Bit1 Hi = 2⁻⁷ m/cs |
| Q | Position4 | XXXX.X NM | DP, Bit1 Low = 2 m |
| S | Geschwindigkeit3 | XXXX.X ft/s | DP, Bit1 Hi = 2⁻⁷ m/cs |
| KK | Gewicht2 | XXXXX. lbs | SP, Bruchteil von 2¹⁶ kg |
| LL | Position6 | XXXX.X NM | DP |
| MM | Drag-Beschleunigung | XXX.XX G | DP, Bit1 Low = 25×2⁻²⁸ G |
| PP | 2 Ganzzahlen | +XXBYY | DP, NUR ANZEIGE |
| UU | Geschwindigkeit/2VS | XXXXX. ft/s | DP, Bruchteil von 2VS |

### Normale Nouns (N00–N39)

| Noun | Register | Typ | Skala | Beschreibung |
|------|----------|-----|-------|--------------|
| **N00** | — | — | — | Nicht in Verwendung |
| **N01** | Adresse angeben | 3K | B (.XXXXX) | Maschinenaddresse (Bruchwert) |
| **N02** | Adresse angeben | 3K | C (XXXXX.) | Maschinenaddresse (Ganzzahl) |
| **N03** | Adresse angeben | 3K | D (XXX.XX°) | Maschinenaddresse (Grad) |
| **N05** | `DSPTEM1` | 1K | H (XXX.XX°) | Winkelfehler/-differenz |
| **N06** | `OPTION1` | 2K | A (Oktal) | Option-Code |
| **N07** | `XREG` | 3K | A (Oktal) | ECADR + Bitmaske + 1=Set/0=Reset |
| **N08** | `ALMCADR` | 3K | A (Oktal) | Alarm-Daten |
| **N09** | `FAILREG` | 3K | A (Oktal) | Alarm-Codes |
| **N10** | Kanal angeben | 1K | A (Oktal) | I/O-Kanal angeben |
| **N11** | `TCSI` | 3K | K | TIG CSI (HH:MM:SS.SS) |
| **N12** | `OPTIONX` | 2K | A (Oktal) | Option-Code (nur Extended Verbs) |
| **N13** | `TCDH` | 3K | K | TIG CDH (HH:MM:SS.SS) |
| **N15** | Inkrement-ADR | 1K | A (Oktal) | Maschinenaddresse inkrementieren |
| **N16** | `DSPTEMX` | 3K | K | Zeit des Ereignisses (nur Ext. Verbs) |
| **N17** | `CPHIX` | 3K | D (XXX.XX°) | Astronauten-Gesamtlage (Roll/Pitch/Yaw) |
| **N18** | `THETAD` | 3K | D (XXX.XX°) | Auto-Manöver-Kugelwinkel |
| **N19** | `THETAD` | 3K | D (XXX.XX°) | Bypass Attitude Trim-Manöver |
| **N20** | `CDUX` | 3K | D (XXX.XX°) | ICDU-Winkel (Roll, Pitch, Yaw) |
| **N21** | `PIPAX` | 3K | C (XXXXX.) | PIPA-Impulse (3 Achsen) |
| **N22** | `THETAD` | 3K | D (XXX.XX°) | Neue ICDU-Winkel |
| **N24** | `DSPTEM2+1` | 3K | K | Delta-Zeit für AGC-Uhr (HH:MM:SS.SS) |
| **N25** | `DSPTEM1` | 3K | C (XXXXX.) | Checkliste (nur mit V50) |
| **N26** | `DSPTEM1` | 3K | A (Oktal) | Priorität/Delay, Adresse, BBCON (V30/V31) |
| **N27** | `SMODE` | 1K | C (XXXXX.) | Self-Test Ein/Aus-Schalter |
| **N29** | `DSPTEM1` | 1K | D (XXX.XX°, DEC) | XSM Launch-Azimuth |
| **N30** | `DSPTEM1` | 3K | C (XXXXX.) | Ziel-Codes |
| **N31** | `DSPTEM1` | 3K | K | Zeit Landestelle (HH:MM:SS.SS) |
| **N32** | `-TPER` | 3K | K | Zeit bis Perigäum (HH:MM:SS.SS) |
| **N33** | `TIG` | 3K | K | Zündzeit TIG (HH:MM:SS.SS) |
| **N34** | `DSPTEM1` | 3K | K | Zeit des Ereignisses (HH:MM:SS.SS) |
| **N35** | `TTOGO` | 3K | K | Zeit bis Ereignis (HH:MM:SS.SS) |
| **N36** | `TIME2` | 3K | K | AGC-Uhrzeit (HH:MM:SS.SS) |
| **N37** | `TTPI` | 3K | K | TIG TPI (HH:MM:SS.SS) |
| **N38** | `TET` | 3K | K | Zeit des Zustandsvektors (HH:MM:SS.SS) |
| **N39** | `T3TOT4` | 3K | K | Delta-Zeit für Bahnübergang (HH:MM:SS.SS) |

### Mixed Nouns (N40–N99)

Mixed Nouns haben pro Komponente eigene Skalierungsfaktoren (via `IDADDTAB`/`RUTMXTAB`).
`NO LOAD` = nur Anzeige/Monitor, kein Laden möglich.
`DEC ONLY` = nur Dezimalbetrieb.

| Noun | Komp. | Register | Skala | Einschränk. | Beschreibung |
|------|-------|----------|-------|-------------|--------------|
| **N40** | 3 | TTOGO / VGDISP / DVTOTAL | L / S / S | NO LOAD, DEC | Zeit bis Zündung (MM:SS) / VG / Delta-V akkum. |
| **N41** | 2 | DSPTEM1 / DSPTEM1+1 | D / E | — | Ziel-Azimuth (XXX.XX°) / Elevation (XX.XXX°) |
| **N42** | 3 | HAPO / HPER / VGDISP | Q / Q / S | DEC | Apogäum / Perigäum (XXXX.X NM) / Delta-V req. |
| **N43** | 3 | LAT / LONG / ALT | H / H / Q | DEC | Breite / Länge (XXX.XX°) / Höhe (XXXX.X NM) |
| **N44** | 3 | HAPOX / HPERX / TFF | Q / Q / L | NO LOAD, DEC | Apogäum / Perigäum (XXXX.X NM) / TFF (MM:SS) |
| **N45** | 3 | VHFCNT / TTOGO / +MGA | PP / L / H | NO LOAD, DEC | Marks (VHF-Optik) / TFI nächste Zündung / MGA |
| **N46** | 2 | DAPDATR1 / DAPDATR2 | A / A | — | Autopilot-Konfiguration (Oktal) |
| **N47** | 2 | CSMMASS / LEMMASS | KK / KK | DEC | CSM-Gewicht / LEM-Gewicht (XXXXX. lbs) |
| **N48** | 2 | PACTOFF / YACTOFF | FF / FF | DEC | Pitch-Trim / Yaw-Trim (XXX.XX°) |
| **N49** | 3 | N49DISP / +2 / +4 | Q / S / C | DEC | Delta-R (XXXX.X NM) / Delta-V / VHF-Code |
| **N50** | 3 | RSP-RREC / HPERX / TFF | LL / Q / L | NO LOAD, DEC | Splash-Error / Perigäum / TFF |
| **N51** | 2 | RHOSB / GAMMASB | H / H | DEC | S-Band Antennenwinkel Pitch / Yaw (XXX.XX°) |
| **N52** | 1 | ACTCENT | H | — | Zentriwinkel aktives Fahrzeug (XXX.XX°) |
| **N53** | 3 | RANGE / RRATE / RTHETA | JJ / S / H | DEC | Range (XXX.XX NM) / Range Rate / PHI (XXX.XX°) |
| **N54** | 3 | RANGE / RRATE / RTHETA | JJ / S / H | DEC | Range / Range Rate / THETA (XXX.XX°) |
| **N55** | 3 | NN1 / ELEV / CENTANG | C / H / H | DEC | Perigäum-Code / Elevationswinkel / Zentriwinkel |
| **N56** | 2 | RTEGAM2D / RTEDVD | H / P | DEC | Rückkehr-Winkel (XXX.XX°) / Delta-V (XXXXX. ft/s) |
| **N57** | 1 | DELTAR | Q | DEC | Delta-R (XXXX.X NM) |
| **N58** | 3 | POSTTPI / DELVTPI / DELVTPF | Q / S / S | DEC | Perigäum-Höhe (Post-TPI) / Delta-V TPI / Delta-V TPF |
| **N59** | 3 | DVLOS / +2 / +4 | S / S / S | DEC | Delta-Geschwindigkeit LOS (3 Komp., XXXX.X ft/s) |
| **N60** | 3 | GMAX / VPRED / GAMMAEI | T / P / H | DEC | G-max (XXX.XX G) / Vpred (XXXXX. ft/s) / Gamma EI |
| **N61** | 3 | LAT(SPL) / LNG(SPL) / HEADSUP | H / H / C | DEC | Impact-Breite / -Länge (XXX.XX°) / Heads Up/Down |
| **N62** | 3 | VMAGI / HDOT / ALTI | P / P / Q | DEC | VI (XXXXX. ft/s) / HDOT / Höhe (XXXX.X NM) |
| **N63** | 3 | RTGO / VIO / TTE | LL / P / L | NO LOAD, DEC | Distanz bis Splash / Vpred / TFE — NUR ANZEIGE |
| **N64** | 3 | D / VMAGI / RTGON64 | MM / P / LL | DEC | Drag-Beschl. (XXX.XX G) / VI / Range bis Splash |
| **N65** | 3 | SAMPTIME (×3) | K / K / K | DEC | Gesamplete AGC-Zeit HH:MM:SS (aus Interrupt) |
| **N66** | 3 | ROLLC / XRNGERR / DNRNGERR | H / VV / LL | DEC | Bank-Winkel Beta / Cross-Range-Error / Down-Range-Error |
| **N67** | 3 | RTGON67 / LAT / LONG | LL / H / H | DEC | Range zum Ziel / Breite / Länge |
| **N68** | 3 | ROLLC / VMAGI / RDOT | H / P / UU | DEC | Beta (XXX.XX°) / VI / RDOT (XXXXX. ft/s) |
| **N69** | 3 | ROLLC / Q7 / VL | H / MM / UU | DEC | Beta / DL (XXX.XX G) / VL (XXXXX. ft/s) |
| **N70** | 3 | STARCODE / LANDMARK / HORIZON | A / A / A | — | Stern-Code / Landmark / Horizont (alle Oktal) |
| **N71** | 3 | STARCODE / LANDMARK / HORIZON | A / A / A | — | wie N70 |
| **N72** | 3 | THETZERO / DELHITE / OPTION2 | H / Q / C | DEC | Delta-Winkel / Delta-Höhe / Such-Option |
| **N73** | 3 | P21ALT / P21VEL / P21GAM | Q / P / H | — | Höhe (XXXX.X NM) / Geschwindigkeit / Flugpfadwinkel |
| **N74** | 3 | ROLLC / VMAGI / D | H / P / MM | — | Beta / VI / Drag-Beschl. |
| **N75** | 3 | DIFFALT / T1TOT2 / T2TOT3 | Q / L / L | NO LOAD, DEC | Delta-Höhe CDH / Delta-Zeit CDH-CSI / Delta-Zeit TPI-CDH |
| **N80** | 3 | TTOGO / VGDISP / DVTOTAL | L / P / P | NO LOAD, DEC | wie N40, aber Velocity2-Skala |
| **N81** | 3 | DELVLVC / +2 / +4 | S / S / S | DEC | Delta-V (LV-Koord., 3 Komp., XXXX.X ft/s) |
| **N82** | 3 | DELVLVC / +2 / +4 | S / S / S | DEC | Delta-V (LV) — anderer Kontext als N81 |
| **N83** | 3 | DELVIMU / +2 / +4 | S / S / S | DEC | Delta-V (Körperkoord.) |
| **N84** | 3 | DELVOV / +2 / +4 | S / S / S | DEC | Delta-V (anderes Fahrzeug) |
| **N85** | 3 | VGBODY / +2 / +4 | S / S / S | DEC | VG (Körperkoord.) |
| **N86** | 3 | DELVLVC / +2 / +4 | P / P / P | DEC | Delta-V (LV, Velocity2-Skala XXXXX. ft/s) |
| **N87** | 2 | MRKBUF1+3 / MRKBUF1+5 | D / J | — | Markdaten: Shaft (XXX.XX°) / Trunion (XX.XXX°) |
| **N88** | 3 | STARSAV3 / +2 / +4 | ZZ / ZZ / ZZ | DEC | Halber Einheitsvektor Sonne/Planet (.XXXXX je Komp.) |
| **N89** | 3 | LANDLAT / LANDLONG / LANDALT | G / G / JJ | DEC | Landmark-Breite / -Länge (XX.XXX°) / Höhe (XXX.XX NM) |
| **N90** | 3 | RANGE / RRATE / RTHETA | JJ / S / H | DEC | Y (XXX.XX NM) / Ydot (XXXX.X ft/s) / PSI (XXX.XX°) |
| **N91** | 2 | CDUS / CDUT | D / J | — | OCDU-Winkel: Shaft / Trunion |
| **N92** | 2 | SAC / PAC | D / J | — | Neue Optik-Winkel: Shaft / Trunion |
| **N93** | 3 | OGC / +2 / +4 | G / G / G | — | Delta-Kreiselwinkel (XX.XXX° je Achse) |
| **N94** | 2 | MRKBUF1+3 / MRKBUF1+5 | D / J | — | Neue Optik-Winkel: Shaft / Trunion (wie N92, anderer Kontext) |
| **N95** | 3 | PRAXIS / +1 / +2 | D / D / D | — | Preferred Attitude ICDU-Winkel (XXX.XX° je Achse) |
| **N96** | 3 | CPHIX / +1 / +2 | D / D / D | — | +X-Axis Attitude ICDU-Winkel (XXX.XX° je Achse) |
| **N97** | 3 | DSPTEM1 / +1 / +2 | C / C / C | — | System-Test-Eingaben (XXXXX. je Komp.) |
| **N98** | 3 | DSPTEM2 / +1 / +2 | C / B / C | — | System-Test-Ergebnisse (XXXXX. / .XXXXX / XXXXX.) |
| **N99** | 3 | WWPOS / WWVEL / WWOPT | XX / YY / C | DEC | RMS Position (XXXXX. ft) / Geschwindigkeit (XXXX.X ft/s) / Option |

---

## Teil 4 — Häufige Verb-Noun-Kombinationen

| V-N | Verwendung |
|-----|-----------|
| V04 N06 | Optionscode anzeigen und auf Eingabe in R2 warten |
| V06 N18 | Manöver-Kugelwinkel dezimal anzeigen |
| V06 N22 | Neue ICDU-Zielwinkel dezimal anzeigen (R62-Manöver) |
| V06 N47 | CSM/LEM-Gewichte anzeigen (DAP-Load) |
| V06 N48 | Pitch/Yaw-Trim anzeigen (DAP-Load) |
| V16 N44 | Apogäum/Perigäum/TFF monitoren (R30) |
| V21 N01 | System-Testnummer laden (V92) |
| V24 N92 | Neue Optics-CDU-Winkel laden (V41) |
| V25 N22 | Neue ICDU-Winkel laden (V41, V43) |
| V25 N24 | Delta-Zeit für AGC-Uhr laden (V55) |
| V25 N93 | Delta-Kreiselwinkel laden (V42) |
| V37 Nxx | Hauptprogramm wechseln (P00–P99) |

---

## Teil 5 — Implementierungshinweise

### DSPTAB-Ausgabepuffer (11 Register)

```
DSPTAB[10]  Relayword 1011  →  MD1 (25) / MD2 (24)   Major Mode
DSPTAB[ 9]  Relayword 1010  →  VD1 (23) / VD2 (22)   Verb
DSPTAB[ 8]  Relayword 1001  →  ND1 (21) / ND2 (20)   Noun
DSPTAB[ 7]  Relayword 1000  →            / R1D1 (16)
DSPTAB[ 6]  Relayword 0111  →  +R1       / R1D2 (15) / R1D3 (14)
DSPTAB[ 5]  Relayword 0110  →  -R1       / R1D4 (13) / R1D5 (12)
DSPTAB[ 4]  Relayword 0101  →  +R2       / R2D1 (11) / R2D2 (10)
DSPTAB[ 3]  Relayword 0100  →  -R2       / R2D3  (7) / R2D4  (6)
DSPTAB[ 2]  Relayword 0011  →            / R2D5  (5) / R3D1  (4)
DSPTAB[ 1]  Relayword 0010  →  +R3       / R3D2  (3) / R3D3  (2)
DSPTAB[ 0]  Relayword 0001  →  -R3       / R3D4  (1) / R3D5  (0)
```

Ausgabeformat je Wort: `AAAABCCCCCDDDDD`
- A (4 Bit): Relayword — wählt das Zeichenpaar
- B (1 Bit): Sonderrelais (Vorzeichen-LED)
- C (5 Bit): Relay-Code linkes Zeichen
- D (5 Bit): Relay-Code rechtes Zeichen

### Channel-Bits für DSKY-Lichter

| Funktion | Kanal | Bit |
|----------|-------|-----|
| KEY RELEASE Licht | 11 | Bit 5 |
| VERB/NOUN FLASH | 11 | Bit 6 |
| OPERATOR ERROR Licht | 11 | Bit 7 |

### Skalierungsfaktor-Routinen (SF-Routinen, Codes 0–13)

| Code | Routine | Beschreibung |
|------|---------|--------------|
| 00 | — | Nur Oktal |
| 01 | FRACT | Bruchzahl (SP) |
| 02 | CDU DEG | CDU-Grad (XXX.XX°) |
| 03 | ARITH | Arithmetisch (SP) |
| 04 | ARITHDP1 | DP Arithmetisch 1 (×2¹⁴ Ausgabe) |
| 05 | ARITHDP2 | DP Arithmetisch 2 (gerade Ausgabe) |
| 06 | YOPTDEG | Y-Optik-Grad (XX.XXX°) |
| 07 | ARITHDP3 | DP Arithmetisch 3 (SL 7 Ausgabe) |
| 08 | HMS | Stunden/Minuten/Sekunden — Alarm bei Oktal |
| 09 | M/S | Minuten/Sekunden (nur Anzeige) — Alarm bei Oktal |
| 10 | ARITHDP4 | DP Arithmetisch 4 (SL 3 Ausgabe) |
| 11 | ARITH1 | Arithmetisch 1 (×2¹⁴ Ausgabe) |
| 12 | 2INTOUT | 2 Ganzzahlen — Alarm bei Oktal |
| 13 | DPFRAC | DP Bruchzahl |

---

*Generiert aus: `input/virtualagc/Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`,*
*`EXTENDED_VERBS.agc`, `PINBALL_NOUN_TABLES.agc`, `ASSEMBLY_AND_OPERATION_INFORMATION.agc`*
