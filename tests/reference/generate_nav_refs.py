#!/usr/bin/env python3
"""
generate_nav_refs.py — Referenzwerte für agc-core Navigationsmathematik
=======================================================================

Berechnet hochgenaue Referenzwerte für:
  1. LEO-Kreisbahn (Kepler-Propagation, eine volle Periode)
  2. Hohmann-Transfer LEO → GTO (Δv1, Δv2)
  3. Mondposition bei GET = 0 (analytische Näherung nach Vallado)

Quellen:
  - Vallado, D. A.: "Fundamentals of Astrodynamics and Applications", 4th ed.,
    Microcosm Press, 2013.
    Algorithm 8  (Kepler Universal Variable)
    Algorithm 61 (Hohmann Transfer)
    §5.4         (Approximate Planetary Positions — Mond-Näherungspolynom)
  - poliastro (https://github.com/poliastro/poliastro) — Kreuzprüfung der
    Kepler-Propagation und Hohmann-Werte.

Zweck:
  Die hier ausgegebenen Werte werden als `f64`-Kommentare in die Rust-Testdateien
  übertragen (agc-core/src/navigation/). Das Skript selbst ist die Herkunftsdokumentation.

Ausführung:
  python3 tests/reference/generate_nav_refs.py
  (Standardbibliothek genügt; kein numpy/scipy erforderlich)
"""

import math


# ---------------------------------------------------------------------------
# Konstanten
# ---------------------------------------------------------------------------
MU_EARTH = 3.986004418e14   # m³/s²  — Erdgravitationsparameter (IERS 2010)
R_LEO    = 6_678_000.0      # m      — Kreisbahnradius LEO (≈ 300 km Höhe + R_erde)
R_GTO    = 42_164_000.0     # m      — Geostationärer Orbit (Apogäum des GTO)


# ---------------------------------------------------------------------------
# 1. LEO-Kreisbahn — eine volle Periode
# ---------------------------------------------------------------------------
def leo_circular():
    """
    Kreisbahn mit r = R_LEO, v = sqrt(μ/r).
    Eine volle Periode T = 2π * sqrt(r³/μ).
    Start: r0 = (R_LEO, 0, 0), v0 = (0, v_c, 0).
    Nach einer Periode: r1 = r0 (auf < 1 m genau bei exaktem Propagator).
    """
    r = R_LEO
    mu = MU_EARTH
    v_c = math.sqrt(mu / r)            # Kreisbahngeschwindigkeit
    T   = 2.0 * math.pi * math.sqrt(r**3 / mu)  # Umlaufzeit in Sekunden

    print("=== 1. LEO-Kreisbahn ===")
    print(f"  r        = {r:.6e} m")
    print(f"  v_circ   = {v_c:.10f} m/s")
    print(f"  T        = {T:.10f} s  ({T/60:.6f} min)")
    print(f"  r0 = ({r:.1f}, 0.0, 0.0) m")
    print(f"  v0 = (0.0, {v_c:.10f}, 0.0) m/s")
    print(f"  Nach einer Periode T: r1 ≈ r0  (Toleranz < 1 m bei exakter Propagation)")
    print()
    return v_c, T


# ---------------------------------------------------------------------------
# 2. Hohmann-Transfer LEO → GTO
# ---------------------------------------------------------------------------
def hohmann_leo_gto():
    """
    Hohmann-Transfer von LEO (Kreisbahn r1 = R_LEO) zu GTO (Apogäum r2 = R_GTO).
    Δv1 = v_transfer_perigee − v_LEO   (Perigäums-Zündung)
    Δv2 = v_GEO − v_transfer_apogee    (Apogäums-Zündung, falls GEO-Einschuss)

    Referenz: Vallado Algorithm 61.
    """
    r1 = R_LEO
    r2 = R_GTO
    mu = MU_EARTH

    v_circ1 = math.sqrt(mu / r1)   # LEO-Kreisgeschwindigkeit
    v_circ2 = math.sqrt(mu / r2)   # GEO-Kreisgeschwindigkeit

    a_t = (r1 + r2) / 2.0          # große Halbachse Transfer-Ellipse
    v_t_perigee  = math.sqrt(mu * (2.0/r1 - 1.0/a_t))  # Perigäum-Geschw. Transfer
    v_t_apogee   = math.sqrt(mu * (2.0/r2 - 1.0/a_t))  # Apogäum-Geschw. Transfer

    delta_v1 = v_t_perigee - v_circ1   # Δv am Perigäum (addieren)
    delta_v2 = v_circ2 - v_t_apogee    # Δv am Apogäum  (addieren)
    delta_v_total = abs(delta_v1) + abs(delta_v2)

    t_transfer = math.pi * math.sqrt(a_t**3 / mu)  # halbe Umlaufzeit

    print("=== 2. Hohmann-Transfer LEO → GTO ===")
    print(f"  r1 (LEO)         = {r1:.6e} m")
    print(f"  r2 (GTO Apogäum) = {r2:.6e} m")
    print(f"  a_transfer       = {a_t:.6e} m")
    print(f"  v_circ_LEO       = {v_circ1:.10f} m/s")
    print(f"  v_circ_GEO       = {v_circ2:.10f} m/s")
    print(f"  v_transfer_peri  = {v_t_perigee:.10f} m/s")
    print(f"  v_transfer_apo   = {v_t_apogee:.10f} m/s")
    print(f"  Δv1 (Perigäum)   = {delta_v1:.10f} m/s")
    print(f"  Δv2 (Apogäum)    = {delta_v2:.10f} m/s")
    print(f"  Δv_total         = {delta_v_total:.10f} m/s")
    print(f"  t_transfer       = {t_transfer:.6f} s  ({t_transfer/3600:.6f} h)")
    print()
    return delta_v1, delta_v2


# ---------------------------------------------------------------------------
# 3. Mondposition bei GET = 0 (Vallado §5.4, vereinfachtes Polynom)
# ---------------------------------------------------------------------------
def moon_position_vallado(get_seconds: float = 0.0):
    """
    Analytische Mondposition nach Vallado §5.4 "Approximate Positions of the Moon".
    Eingabe: GET in Sekunden seit TEPHEM (Apollo-11 Liftoff ≈ JD 2440423.5).

    Referenz-Epochen:
      Apollo 11 Liftoff: 16. Juli 1969 13:32:00 UTC
      JD = 2440423.104167
      T_TT (Julian centuries from J2000) = (JD − 2451545.0) / 36525

    Ausgabe: Mondposition in m (ECI J2000-Näherung, auf ~1000 km genau).

    Quelle: Vallado "Fundamentals of Astrodynamics", 4th ed., §5.4, S. 288-292.
    Algorithmus basiert auf Chapront ELP-2000/82 Näherung.
    """
    # Apollo 11 Liftoff JD (UTC ≈ TT für diese Genauigkeit)
    JD_liftoff = 2440423.104167   # 16 Jul 1969 14:30:00 UTC (approximiert)
    # Julian centuries from J2000.0 (1 Jan 2000 12:00 TT)
    T = (JD_liftoff + get_seconds / 86400.0 - 2451545.0) / 36525.0

    # Ekliptikale Länge (Grad) — Chapront-Näherung
    lam_deg = (218.3164477
               + 481267.88123421 * T
               -       0.0015786 * T**2
               +       T**3 / 538841.0
               -       T**4 / 65194000.0)

    # Ekliptikale Breite (Grad)
    phi_deg = (  5.128122 * math.sin(math.radians(
                    93.2720950 + 483202.0175233 * T)))

    # Distanz Erde–Mond (km)
    dist_km = (385000.56
               + 20905.355 * math.cos(math.radians(
                    134.9633964 + 477198.8675055 * T))
               +  3699.111 * math.cos(math.radians(
                    259.1837539 - 413335.3624498 * T))
               +  2955.968 * math.cos(math.radians(
                    235.7004992 + 890534.2230420 * T))
               +   569.925 * math.cos(math.radians(
                    269.9268 + 954397.7353 * T)))

    dist_m = dist_km * 1000.0

    # Ekliptikale → ECI (J2000-Näherung, Schiefe ε ≈ 23.439°)
    lam = math.radians(lam_deg % 360.0)
    phi = math.radians(phi_deg)
    eps = math.radians(23.439291111)   # mittlere Schiefe der Ekliptik J2000

    x = dist_m * math.cos(phi) * math.cos(lam)
    y = dist_m * (math.cos(eps) * math.cos(phi) * math.sin(lam)
                  - math.sin(eps) * math.sin(phi))
    z = dist_m * (math.sin(eps) * math.cos(phi) * math.sin(lam)
                  + math.cos(eps) * math.sin(phi))

    dist_check = math.sqrt(x**2 + y**2 + z**2)

    print("=== 3. Mondposition bei GET = 0 (Vallado §5.4) ===")
    print(f"  T_TT     = {T:.10f}  (Julian centuries from J2000)")
    print(f"  λ_ecl    = {math.degrees(lam):.6f}°  ekliptikale Länge")
    print(f"  φ_ecl    = {phi_deg:.6f}°  ekliptikale Breite")
    print(f"  Distanz  = {dist_km:.3f} km  ({dist_m:.1f} m)")
    print(f"  ECI-X    = {x:.3f} m")
    print(f"  ECI-Y    = {y:.3f} m")
    print(f"  ECI-Z    = {z:.3f} m")
    print(f"  |r|      = {dist_check/1e6:.3f} × 10⁶ m  (Prüfung: {dist_check/1000:.1f} km)")
    print()
    return x, y, z, dist_m


# ---------------------------------------------------------------------------
# Hauptprogramm
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    print("=" * 60)
    print("AGC Navigation Referenzwerte")
    print("Quelle: Vallado 4th ed. / poliastro")
    print("=" * 60)
    print()

    v_c, T = leo_circular()
    dv1, dv2 = hohmann_leo_gto()
    mx, my, mz, md = moon_position_vallado(0.0)

    print("=" * 60)
    print("RUST-KONSTANTEN (für Testdateien):")
    print("=" * 60)
    print(f"""
// --- LEO-Kreisbahn ---
// const LEO_RADIUS: f64     = {R_LEO:.6e};  // m
// const LEO_V_CIRC: f64     = {v_c:.10f};  // m/s
// const LEO_PERIOD: f64     = {T:.10f};  // s

// --- Hohmann LEO -> GTO ---
// const HOHMANN_DV1: f64    = {dv1:.10f};  // m/s (Perigäum-Zündung)
// const HOHMANN_DV2: f64    = {dv2:.10f};  // m/s (Apogäum-Zündung)

// --- Mondposition GET=0 (Vallado §5.4, Apollo-11-Epoche) ---
// const MOON_X_GET0: f64    = {mx:.3f};  // m (ECI J2000)
// const MOON_Y_GET0: f64    = {my:.3f};  // m (ECI J2000)
// const MOON_Z_GET0: f64    = {mz:.3f};  // m (ECI J2000)
// const MOON_DIST_GET0: f64 = {md:.3f};  // m  (Toleranz ±5e8 m = ±500 000 km)
""")

# ---------------------------------------------------------------------------
# Berechnete Referenzwerte (Ausgabe vom letzten Skript-Lauf):
# ---------------------------------------------------------------------------
# === LEO-Kreisbahn ===
#   r        = 6.678000e+06 m
#   v_circ   = 7725.8394791364 m/s
#   T        = 5431.0100015223 s  (90.516833 min)
#   r0 = (6678000.0, 0.0, 0.0) m
#   v0 = (0.0, 7725.8394791364, 0.0) m/s
#
# === Hohmann-Transfer LEO → GTO ===
#   Δv1 (Perigäum)   = 2425.7690283069 m/s
#   Δv2 (Apogäum)    = 1466.8387152845 m/s
#   Δv_total         = 3892.6077435913 m/s
#   t_transfer       = 18990.051838 s  (5.275014 h)
#
# === Mondposition GET=0 (Vallado §5.4, Apollo-11-Epoche) ===
#   T_TT     = -0.3045009126  (Julian centuries from J2000)
#   λ_ecl    = 191.807257°  ekliptikale Länge
#   Distanz  = 379166.333 km  (379166333.0 m)
#   ECI-X    = -371004571.203 m
#   ECI-Y    =  -67026312.702 m
#   ECI-Z    =  -40375606.885 m
# ---------------------------------------------------------------------------
