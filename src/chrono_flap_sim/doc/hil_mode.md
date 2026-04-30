# HIL Mode Design Document

## Overview

HIL (Hardware-in-the-Loop) mode makes `chrono_flap_node` a **load-torque evaluator** only.
The real motor, encoder, and `velocity_pid_node` form the primary control loop; Chrono provides
a hydrodynamic disturbance torque based on measured shaft state. This torque is mixed with the
PID torque by `hil_torque_mixer_node` before commanding the real motor.

---

## Causality

```
θ_meas, ω_meas  ←  encoder  ←  /joint_states
         │
         ▼
chrono_flap_node (mode=hil)
  τ_hydro = f(θ_meas, ω_meas, t)   ← stub today; HydroChrono later
         │
         ▼  ~/load_torque
hil_torque_mixer_node
  τ_total = clamp(τ_pid + τ_hydro, ±hard_clip_nm)
         │
         ▼  /motor_effort_controller/commands
ODrive (real motor)
         │
         ▼
encoder  →  /joint_states  →  (loop)
```

Key invariants:
1. **Chrono does NOT integrate joint dynamics** in HIL mode. The real shaft is the integrator.
2. **No shadow PID** is applied in HIL mode (`use_shadow_pid` forced false internally).
3. Chrono publishes `/sim_joint_states` from the measured state so the RViz overlay tracks
   reality without any simulation drift.

---

## Load Torque Stub

The current stub `compute_load_torque(θ, ω, t)` computes:

```
τ_hydro = hil_constant_load_nm
        − hil_virtual_stiffness  · θ
        − hil_virtual_damping    · ω
        − hil_quadratic_drag     · ω · |ω|
        + hil_wave_amp_nm · sin(hil_wave_omega_rad_s · t)
```

Result is clamped to `[−hil_torque_clip_nm, +hil_torque_clip_nm]` before publishing.

### HydroChrono Integration Point

Only the body of `compute_load_torque(...)` needs to change for full hydrodynamic simulation.
Everything else (topics, watchdog, engage gate, safety clamp) remains unchanged.

### Future Active PTO (axis1)

When axis1 becomes active, the only change is the mixer's `output_topic` (or the load torque
goes directly to axis1's effort topic and the mixer is bypassed for axis1). No logic changes in
`chrono_flap_node`.

---

## Safety Architecture

### Watchdog (`hil_feedback_timeout_s`, default 0.1 s)

If `/joint_states` is not received within `hil_feedback_timeout_s` seconds, the published
`τ_hydro` is forced to zero and a throttled WARN is logged.

### Engage Gate (`~/engage_hil` service)

The `std_srvs/srv/SetBool` service at `~/engage_hil` controls whether load torque is active:
- `data: true`  → engage (ramp in over `hil_ramp_time_s`)
- `data: false` → disengage (ramp down to zero over `hil_ramp_time_s`)

Default: `hil_engaged_default=false` for safety on first launch.

### Ramp-In (`hil_ramp_time_s`, default 1.0 s)

When engaging, `τ_hydro` ramps linearly from 0 to its computed value over `hil_ramp_time_s`
seconds. This prevents step disturbances at engage time.

### Mixer Hard Clamp (`hard_clip_nm`, default 0.5 N·m)

`hil_torque_mixer_node` applies a final hard clamp on `τ_total = τ_pid + τ_hydro` before
publishing to `/motor_effort_controller/commands`. This is independent of `hil_torque_clip_nm`.

### Shutdown Zero

Both `chrono_flap_node` (on destructor) and `hil_torque_mixer_node` (on destructor) publish a
final zero command on their respective output topics.

---

## Parameters Reference

| Parameter | Default | Mutable | Description |
|---|---|---|---|
| `mode` | `"parallel"` | No | Operating mode: `"sil"`, `"parallel"`, or `"hil"`. |
| `hil_constant_load_nm` | `0.1` | Yes | Stub constant load torque bias (N·m). |
| `hil_virtual_stiffness` | `0.0` | Yes | Virtual spring stiffness (N·m/rad), ≥ 0. |
| `hil_virtual_damping` | `0.0` | Yes | Virtual damping (N·m·s/rad), ≥ 0. |
| `hil_quadratic_drag` | `0.0` | Yes | Quadratic drag coefficient, ≥ 0. |
| `hil_wave_amp_nm` | `0.0` | Yes | Wave excitation amplitude (N·m), ≥ 0. |
| `hil_wave_omega_rad_s` | `0.0` | Yes | Wave excitation frequency (rad/s), ≥ 0. |
| `hil_torque_clip_nm` | `0.3` | Yes | Hard clamp on τ_hydro (N·m), > 0. |
| `hil_feedback_timeout_s` | `0.1` | Yes | Watchdog timeout on /joint_states (s), > 0. |
| `hil_ramp_time_s` | `1.0` | Yes | Ramp-in time when engaging (s), ≥ 0. |
| `hil_engaged_default` | `false` | No | Whether to engage at startup. |
| `hil_load_topic` | `"~/load_torque"` | No | Topic name for τ_hydro output. |

---

## Commissioning Procedure

1. **First run**: start with `hil_constant_load_nm:=0.05`, `hil_torque_clip_nm:=0.1`, and
   motor at low current limit. Verify sign: positive load torque should resist positive ω.
   If sign is reversed, negate `hil_constant_load_nm`.

2. **Verify watchdog**: disconnect the CAN bus. Within `hil_feedback_timeout_s` seconds,
   `/chrono_flap_node/load_torque` should drop to 0.0. Reconnect and verify it recovers.

3. **Verify engage gate**: with hardware running, call the engage service. Verify that
   `/chrono_flap_node/load_torque` ramps from 0 to the expected value over `hil_ramp_time_s`.

4. **Verify mixer clamp**: set `hil_constant_load_nm` to a value above `hard_clip_nm` in
   `hil_torque_mixer_node`. Verify that `/motor_effort_controller/commands` is clamped.

5. **Increase load**: once satisfied with sign convention and safety, gradually increase
   `hil_constant_load_nm` and enable wave excitation (`hil_wave_amp_nm`, `hil_wave_omega_rad_s`).

---

## Topic / Service Summary

| Name | Type | Description |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | Hardware feedback (subscribed in HIL) |
| `~/load_torque` | `std_msgs/Float64` | τ_hydro output (default `/chrono_flap_node/load_torque`) |
| `/sim_joint_states` | `sensor_msgs/JointState` | Digital twin for RViz overlay (from measured state) |
| `~/engage_hil` | `std_srvs/SetBool` service | Engage / disengage load torque |
| `~/sim_position` | `std_msgs/Float64` | Measured θ (passes through in HIL for plotting) |
| `~/sim_velocity` | `std_msgs/Float64` | Measured ω (passes through in HIL for plotting) |
| `~/sim_acceleration` | `std_msgs/Float64` | Δω/Δt (derivative of measured ω in HIL) |
