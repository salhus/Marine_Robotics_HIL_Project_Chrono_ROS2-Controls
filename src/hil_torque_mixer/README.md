# hil_torque_mixer

A minimal ROS 2 C++ package that safely combines the PID controller torque command and the
HIL load torque from `chrono_flap_node`, applying a hard safety clamp before commanding the
real motor.

## Role in HIL

In HIL mode the control loop is:

```
/joint_states (encoder) → velocity_pid_node → /velocity_pid_node/torque_command ─┐
                                                                                   ▼
                    chrono_flap_node (mode=hil) → ~/load_torque ──────────────► hil_torque_mixer_node
                                                                                   │
                                              /motor_effort_controller/commands ◄──┘
                                                          │
                                                      ODrive (real motor)
```

`hil_torque_mixer_node` is the **only** node in the HIL safety path. It is intentionally
minimal so that its behaviour is easy to audit and verify.

## Safety Properties

| Property | Detail |
|---|---|
| **Independent watchdogs** | PID input and load input each have separate timeout checks. |
| **Default-safe failure** | If either input goes stale, its contribution is zeroed. |
| **Hard output clamp** | `τ_total` is always clamped to `±hard_clip_nm` before publishing. |
| **Shutdown zero** | On node shutdown, publishes a final `[0.0]` command. |
| **Runtime toggle** | `enable_load` parameter (and `~/enable_load` SetBool service) can disable the load contribution without stopping the node. |

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `pid_torque_topic` | `Float64MultiArray` | Subscribe | PID torque command (first element used) |
| `load_torque_topic` | `Float64` | Subscribe | HIL load torque from `chrono_flap_node` |
| `output_topic` | `Float64MultiArray` | Publish | Combined command to motor effort controller |
| `~/enable_load` | `std_srvs/SetBool` | Service | Runtime enable/disable of load contribution |

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `pid_torque_topic` | `/velocity_pid_node/torque_command` | Immutable |
| `load_torque_topic` | `/chrono_flap_node/load_torque` | Immutable |
| `output_topic` | `/motor_effort_controller/commands` | Immutable |
| `output_rate_hz` | `200.0` | Mixer publish rate (Hz). Immutable. |
| `hard_clip_nm` | `0.5` | Final hard clamp on τ_total (N·m). Must be > 0. |
| `pid_timeout_s` | `0.2` | Watchdog on PID input. Must be > 0. |
| `load_timeout_s` | `0.2` | Watchdog on load input. Must be > 0. |
| `enable_load` | `true` | Runtime toggle for load contribution. |

## Quick start

```bash
ros2 launch hil_odrive_ros2_control hil_mode.launch.py
```

Or manually:

```bash
ros2 run hil_torque_mixer hil_torque_mixer_node --ros-args \
  -p hard_clip_nm:=0.3 \
  -p output_rate_hz:=200.0
```

To disable the load torque contribution at runtime:

```bash
ros2 service call /hil_torque_mixer_node/enable_load std_srvs/srv/SetBool "{data: false}"
```

## Future: active PTO migration (axis1)

When axis1 becomes an actively controlled Power Take-Off joint, the only change required in this
node is the `output_topic` parameter (point it at axis1's effort topic). Alternatively, the load
torque can be published directly to axis1's effort topic and the mixer is bypassed for axis1
entirely. No logic changes are needed in `hil_torque_mixer_node`.
