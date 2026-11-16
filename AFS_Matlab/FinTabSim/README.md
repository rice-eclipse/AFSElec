# FinTabSim: 6DOF flight of a fin-tab controlled rocket

A Simulink model (`AFS_FinTabSim.slx`) of a rocket whose roll and pitch/yaw are
steered by servo-driven tabs on its four fins.

## Quick start

```matlab
cd AFS_Matlab/FinTabSim
run_AFS_FinTabSim        % active vs. passive flight, summary table + plots
```

`build_AFS_FinTabSim` regenerates the `.slx` from scratch. Run it after you
change the block structure. Parameter changes only need `afs_params.m`.

## Model structure

| Block | What it does | Code |
|---|---|---|
| Roll Program | Roll-angle command vs. time (`P.ctrl.rollProg`) | lookup table |
| Sensors | Gyro rates + bias + noise, attitude quaternion, noisy dynamic pressure | blocks |
| Flight Computer | 100 Hz ZOH + control law: roll PID on gyro-integrated roll angle, pitch/yaw PID holding `P.ctrl.targetDir`, gain-scheduled by 1/q̄, mixed onto 4 tabs with `pinv(B0)` | `afs_controller.m` |
| Fin Tab Servos | First-order servo with slew and travel limits (vector of 4) | blocks |
| Wind Model | Steady wind + filtered gusts | blocks |
| Rocket Plant | Thrust curve, variable mass/CG/inertia, ISA atmosphere, axial drag(Mach), body lift, per-fin aero, including the local flow from body rotation (gives roll/pitch damping), fin cant, and tab force. 6DOF quaternion EOM with a launch-rail constraint | `afs_aero.m`, `afs_eom.m`, `afs_mass_props.m`, `afs_atmosphere.m` |
| Apogee Stop | Ends the run at apogee | blocks |

Frames: inertial ENU at the pad; body x out the nose; fin *i* at roll angle
`P.geo.finAngle(i)`. A positive tab pushes its fin along `x_b × e_r`. With all four tabs
positive, the rocket rolls in the +x direction.

## Vehicle: driven by the OpenRocket file

`afs_params.m` reads [AFS_rocket.ork](AFS_rocket.ork) through `afs_import_ork.m`. To
update the vehicle:

1. Edit the design in OpenRocket.
2. Run its simulation and save the file.
3. Copy the `.ork` over this one, or call `afs_params('path/to/file.ork')`.

The following come from the file:

- **From the design:** body diameter and length, nose shape and length, fin
  planform and position.
- **From the simulation conditions:** rail length and angle, wind speed,
  direction and turbulence.
- **From OpenRocket's stored flight data:** thrust curve, mass, CG and pitch
  inertia vs. time, and axial drag coefficient vs. Mach (power-on and power-off).

`afs_params.m` computes the normal-force slopes and CPs with Barrowman's equations
from the same geometry. For the current design they give CP = 0.887 m and CNα = 14.5/rad;
OpenRocket gives 0.886 m and about 14.6/rad.

**Hand-set parameters** (OpenRocket does not model them) are at the bottom of
`afs_params.m`:

- tab effectiveness `P.aero.tauTab`
- servo speed and limits
- sensor noise
- controller gains and roll program
- build misalignments (fin cant, thrust angle)
- roll inertia: OpenRocket stores it to only 3 decimals (it reads 0.002/0.001), so it is set from a parts estimate (~1.5e-3 kg m²)

**Validation.** With the tabs locked, the Simulink model reaches **416 m at 8.60 s**.
OpenRocket's own simulation reaches **415 m at 8.60 s**, with max Mach 0.34 in both.

**Active control results:**

- The roll program (0 → 90° → 0°) is tracked within a few tenths of a second.
- Tilt from vertical stays under 2° until q̄ drops below 200 Pa, about 7 s.
- At apogee the tilt is 8° with the tabs active and 21° with them locked.

## Assumptions and limitations

- The controller is given the true attitude quaternion; there is no attitude estimator.
- The aero model is linear (CNα, small angles) with a capped Prandtl–Glauert
  correction. Tab effectiveness is a fixed fraction of fin CNα.
- There is no jet damping, no Ixx/Iyy rate terms, no fin flutter and no servo backlash.
