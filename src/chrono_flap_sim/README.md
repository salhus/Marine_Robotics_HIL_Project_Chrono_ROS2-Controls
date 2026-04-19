# chrono_flap_sim

ROS 2 C++ package that runs a **Project Chrono** multibody simulation of a motor driving a rigid
flap about a revolute joint. The node subscribes to the same effort/torque topic that
`velocity_pid_node` publishes to, steps the Chrono simulation at a configurable rate, and
publishes simulated kinematics so you can compare **desired**, **motor**, and **simulated**
kinematics side-by-side in PlotJuggler.

An optional real-time 3D visualization window shows the flap body rotating about the revolute
joint (requires a Chrono build with VSG or Irrlicht support; see [Building](#building) below).

## Published topics

All topics are namespaced under `~/` (i.e. `/<node_name>/`):

| Topic | Type | Description |
|---|---|---|
| `~/sim_position` | `std_msgs/Float64` | Simulated joint angle (rad) |
| `~/sim_velocity` | `std_msgs/Float64` | Simulated joint angular velocity (rad/s) |
| `~/sim_acceleration` | `std_msgs/Float64` | Simulated joint angular acceleration (rad/s²) |

## Subscribed topics

| Topic | Type | Description |
|---|---|---|
| `/motor_effort_controller/commands` | `std_msgs/Float64MultiArray` | Effort/torque command (first element used) |

## Parameters

| Parameter | Type | Default | Reconfigurable | Description |
|---|---|---|---|---|
| `rate_hz` | double | `100.0` | ✗ (restart required) | Simulation step rate (Hz) |
| `effort_topic` | string | `/motor_effort_controller/commands` | ✗ (restart required) | Topic to subscribe for torque |
| `flap_length_m` | double | `0.3` | ✓ | Flap length (m) — updates inertia and CoM in-place; resets flap to home |
| `flap_mass_kg` | double | `0.05` | ✓ | Flap mass (kg) — updates inertia in-place |
| `joint_damping` | double | `0.001` | ✓ | Revolute joint viscous damping coefficient (N·m·s/rad) |

> **Note:** `amplitude_rad_s` and `omega_rad_s` (trajectory parameters) belong exclusively to
> `velocity_pid_node`, which outputs torque to `/motor_effort_controller/commands`.  The
> `chrono_flap_node` consumes that torque directly and does not need its own trajectory params.

## Prerequisites

* [Project Chrono](https://projectchrono.org) — core library (`ChronoEngine`).
  Optional: VSG (`Chrono_vsg`) or Irrlicht (`Chrono_irrlicht`) module for 3D visualization.
* ROS 2 Jazzy (or compatible).

## Building

```bash
# Headless (core only — always works):
colcon build --packages-select chrono_flap_sim

# With VSG visualization (if Chrono was built with VSG support):
colcon build --packages-select chrono_flap_sim \
  --cmake-args -DChronoConfig_DIR=/path/to/chrono/lib/cmake/Chrono

# CMakeLists.txt automatically detects Chrono_vsg / Chrono_irrlicht targets and
# adds the appropriate compile definition (CHRONO_VSG or CHRONO_IRRLICHT).
```

## Usage

The node is launched automatically via `motor_control.launch.py`.  To run it standalone:

```bash
ros2 run chrono_flap_sim chrono_flap_node --ros-args \
  -p rate_hz:=100.0 \
  -p flap_length_m:=0.3 \
  -p flap_mass_kg:=0.05 \
  -p joint_damping:=0.001
```

Physical model parameters (`flap_length_m`, `flap_mass_kg`, `joint_damping`) can be tuned at
runtime via `rqt_reconfigure` without restarting the node.

## PlotJuggler comparison

After launching the full stack, subscribe to the following topics in PlotJuggler:

* `velocity_pid_node/desired_velocity` — desired kinematics from the PID node
* `/joint_states` — measured kinematics from the motor hardware
* `chrono_flap_node/sim_velocity` — simulated kinematics from this node
