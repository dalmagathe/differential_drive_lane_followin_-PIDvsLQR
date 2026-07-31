# Platform constraints

## Convention

Everything is expressed on the **wheel side**. The manufacturer's rated speed (90 RPM) is given with the encoder/gearbox included.

## Geometry

| Quantity | Value | Source |
|---|---|---|
| Wheel Ø | 6.6 cm | Duckiebot datasheet |
| Track width L | 10.2 cm | Duckiebot datasheet |
| L × W × H | 130 × 86 × 123 mm | Duckiebot datasheet |
| Mass | 1 kg | structure + camera |
| Camera ground footprint | 10 × 5 cm | **assumption: not measured** |

## Motor

| Quantity | Value | Source |
|---|---|---|
| Motor ref. | DG01D-E | Duckiebot datasheet |
| Speed at 4.5 V (encoder included) | 90 RPM | motor datasheet |
| Gear ratio | 1:48 | motor datasheet |
| HAT supply | 5 V | Duckiebot |
| Battery current budget | 4 A total, 2.5 A per port | Duckiebot |
| Encoder resolution | 137 ticks/rev | motor datasheet |

## Estimated motor parameters

**Ke (back-EMF constant).** At no load: torque ≈ 0 so current ≈ 0, and the R·i
drop is negligible compared with Ke·ω. The steady-state equation
V = R·i + Ke·ω reduces to V ≈ Ke·ω.

    ω = 90 RPM = 90 × 2π / 60 = 9.42 rad/s
    Ke = 4.5 / 9.42 = 0.478 V·s/rad

Check: at 5 V, ω_noload = 5 / 0.478 ≈ 10.5 rad/s ≈ 100 RPM. Consistent with 90 RPM at 4.5 V.

**Kt (torque constant).** Equal to Ke in SI units:

    Kt = 0.48 N·m/A

Ke is estimated at no load, so it is slightly biased (internal friction is not zero).

## Set for system_model.md

| Quantity | Status |
|---|---|
| Resistance R | **set by order of magnitude: R ≈ 5 Ω** (see history below) |
| Inductance L | neglected: τ_e = L/R ≪ τ_m. First-order electrical model |
| Wheel inertia J | **set: J ≈ 2.5×10⁻⁴ kg·m²** (dominated by reflected rotor ×N², see below) |
| Yaw inertia I_z | **computed: I_z ≈ 4.1×10⁻³ kg·m²** (point-mass decomposition, see below) |

### Resistance R: history and decision

No source publishes R, the no-load current, or the stall current for the DG01D-E.

Decision: we set **R ≈ 5 Ω by order of magnitude** (typical range 3–10 Ω for a small geared brushed DC motor of this class).

Rationale for not searching further: R affects the static gain of the model (τ = (Kt/R)·(V − Ke·ω)). We will **treat the uncertainty as a robustness-test axis**: gains frozen (PID and LQR tuned at R = 5 Ω), then the simulation is re-run with R_test ∈ {0.5×, 1×, 1.5×, 2×} without retuning the gains, which yields a comparison of the degradation between the two controllers.

Distinction from L: neglecting L is a **structural** assumption (the electrical pole is outside the useful bandwidth). Getting R wrong is a **value** error that directly affects the system gain: hence the different treatment of the two.

### Wheel inertia J: decomposition and decision

J is the inertia seen on the **wheel side** (document convention), the sum of three contributions:

1. **Physical wheel** (disc, r = 3.3 cm, estimated mass ~30 g, not weighed):

       J_wheel = ½·m·r² ≈ ½·0.030·0.033² ≈ 1.6×10⁻⁵ kg·m²

2. **Rotor + gearbox, reflected to the wheel side**:

       J_rotor,wheel = J_rotor·N² ≈ 2.3×10⁻⁴ kg·m²

3. **Gears**: neglected, absorbed into the rotor uncertainty.

4. **Total**:

       J = J_wheel + J_rotor,wheel ≈ 2.5×10⁻⁴ kg·m²

**Reliability**: J is ~90% dominated by the rotor, making it **the least reliable parameter in the model**; a ×10 error on J_rotor becomes a ×10 error on J.

### Yaw inertia I_z: point-mass decomposition

I_z = inertia of the whole chassis about the **vertical** axis through the midpoint of the wheel axle.

Formulas used:
- Parallel-axis theorem (Huygens): **I_z = I_own + m·d²**, where d = horizontal distance from the block's center of mass to the wheel axle.
- Own inertia of a rectangular block (horizontal plane a×b):  **I_own = (m/12)·(a² + b²)**.

Decomposition (masses and positions measured/estimated on the real robot):

| Block | m (g) | d to axle (mm) | I_z (kg·m²) | Share |
|---|---|---|---|---|
| Battery | 189 | 100 | 2.1×10⁻³ | 51% |
| Camera + arm | 230 | 50 | 1.2×10⁻³ | 29% |
| Raspberry | 50 | ~100 | 0.5×10⁻³ | 12% |
| Chassis | ~80 | ~0 (centered) | 0.3×10⁻³ | 7% |
| **Total** | | | **4.1×10⁻³** | |