# Vendored Dependencies

## ros_odrive

**Upstream repository:** https://github.com/odriverobotics/ros_odrive  
**Upstream commit SHA:** `6386bf7871ec495c74f9a6d37de1fd6fa3bfdece`  
**License:** MIT (upstream repository: https://github.com/odriverobotics/ros_odrive)

### Directories copied

| Vendored path | Upstream path |
|---|---|
| `src/odrive_base/` | `odrive_base/` |
| `src/odrive_ros2_control/` | `odrive_ros2_control/` |

The following upstream packages were intentionally **not** vendored:
- `odrive_node/`
- `odrive_botwheel_explorer/`

### Why these packages

- **`odrive_ros2_control`** — the ros2_control hardware plugin that exposes ODrive axes as `hardware_interface::SystemInterface`.
- **`odrive_base`** — header-only + source library (no separate `package.xml`) used directly by `odrive_ros2_control` via relative CMake paths (`../odrive_base/include` and `../odrive_base/src/`).

### Build integration

Both packages live directly under `src/` in this repository as sibling colcon packages.  
Colcon will automatically discover `odrive_ros2_control` (it contains a `package.xml`).  
`odrive_base` is not a standalone ROS package; its sources are compiled directly by
`odrive_ros2_control/CMakeLists.txt` using the relative paths `../odrive_base/include` and `../odrive_base/src/`.

### How to update from upstream

1. Identify the new upstream commit SHA on https://github.com/odriverobotics/ros_odrive.
2. Replace the contents of `src/odrive_base/` and
   `src/odrive_ros2_control/` with the new upstream versions.
3. Update the **Upstream commit SHA** field in this file.
4. Review any CMakeLists.txt path changes if the upstream directory layout changed.
