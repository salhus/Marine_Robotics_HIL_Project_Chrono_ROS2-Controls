# VSG Visualization Setup

How to get the Chrono VSG (Vulkan Scene Graph) 3D visualizer working with
`chrono_flap_sim` from a fresh machine.

> **TL;DR** — VSG needs to be discoverable at three independent stages
> (build, runtime linker, runtime asset loader). Each stage uses a
> different env var. Miss any one and you get a different failure at a
> different time. Setting all three correctly makes it Just Work.

---

## The 3 discovery stages

| Stage | Tool that looks | Env var | What it finds | Failure if missing |
|---|---|---|---|---|
| **Build** | CMake | `CMAKE_PREFIX_PATH` | VSG C++ headers + `libChrono_vsg.so` | Compile error: `vsg/all.h: No such file or directory` |
| **Run (linker)** | `ld.so` | `LD_LIBRARY_PATH` | `libvsg.so`, `libvsgXchange.so` | Startup: `libvsg.so.X: cannot open shared object file` |
| **Run (assets)** | VSG's loader | `VSG_FILE_PATH` | Fonts, shaders, models (e.g. `vsg/fonts/OpenSans-Bold.vsgb`) | Window opens then segfaults; log shows `Failed to read font` |

---

## Prerequisites

### 1. Vulkan SDK

```bash
sudo apt install libvulkan-dev vulkan-tools
vulkaninfo --summary   # should print a GPU + driver, not an error
```

### 2. Build & install VSG to a known prefix

```bash
git clone https://github.com/vsg-dev/VulkanSceneGraph.git
cd VulkanSceneGraph
cmake -S . -B build -DCMAKE_INSTALL_PREFIX=$HOME/Packages/vsg
cmake --build build -j$(nproc)
cmake --install build
```

`$HOME/Packages/vsg` is the convention used in this guide. You can put it
anywhere — just be consistent in the env-var exports below.

### 3. Build Chrono with VSG support

When configuring Chrono, point it at the VSG install and enable the VSG
module:

```bash
cmake -S . -B build \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DCMAKE_PREFIX_PATH=$HOME/Packages/vsg \
  -DENABLE_MODULE_VSG=ON
cmake --build build -j$(nproc)
sudo cmake --install build
```

`sudo make install` is what places Chrono's VSG data assets (the font that
trips up the visualizer at runtime) under `/usr/local/share/chrono/data/`.

> **Not using `sudo make install`?**
> If you're building Chrono locally without installing system-wide (e.g.
> for [SEA-Stack](https://github.com/Project-SEA-Stack/SEA-Stack) interop,
> which requires `-DCH_USE_SIMD=OFF`), see
> [`local-chrono-build.md`](local-chrono-build.md) for the env-var setup
> that points at the build tree instead of `/usr/local`. The VSG-side
> setup in this doc still applies — the local-build doc covers what
> changes on the Chrono side (`Chrono_DIR`, `LD_LIBRARY_PATH`,
> `VSG_FILE_PATH`).

### 4. Sanity check before continuing

```bash
ls $HOME/Packages/vsg/include/vsg/all.h                       # VSG headers
ls $HOME/Packages/vsg/lib/libvsg*.so*                         # VSG libs
ls /usr/local/include/chrono_vsg/ChVisualSystemVSG.h          # Chrono VSG module
ls /usr/local/lib/libChrono_vsg.so                            # Chrono VSG lib
ls /usr/local/share/chrono/data/vsg/fonts/OpenSans-Bold.vsgb  # Chrono VSG assets
```

All five paths must exist. If any are missing, fix that first — no env-var
juggling will substitute for a missing file.

---

## The env vars that make it work

Add to `~/.bashrc`:

```bash
# Vulkan Scene Graph (for chrono_flap_sim VSG visualization)
# 1. Build-time: where CMake finds VSG headers + libraries
export CMAKE_PREFIX_PATH=$HOME/Packages/vsg:$CMAKE_PREFIX_PATH
# 2. Run-time linker: where ld.so finds libvsg.so / libvsgXchange.so
export LD_LIBRARY_PATH=$HOME/Packages/vsg/lib:$LD_LIBRARY_PATH
# 3. Run-time assets: where VSG resolves relative paths like
#    "vsg/fonts/OpenSans-Bold.vsgb" (loader is VSG's, not Chrono's, so
#    CHRONO_DATA_PATH does NOT help here).
export VSG_FILE_PATH="/usr/local/share/chrono/data:$HOME/Packages/vsg/share/vsgExamples"
```

Reload: `source ~/.bashrc` (or open a new terminal).

Verify:
```bash
echo "$CMAKE_PREFIX_PATH" | tr ':' '\n' | grep vsg     # should print at least one line
echo "$LD_LIBRARY_PATH"   | tr ':' '\n' | grep vsg
echo "$VSG_FILE_PATH"     | tr ':' '\n'
```

---

## Build

```bash
cd ~/<your-workspace>/Marine_Robotics_HIL_Project_Chrono_ROS2-Controls

# Important: wipe stale CMake cache so the new CMAKE_PREFIX_PATH is honored.
# CMake caches find_library results — without this, env-var changes are ignored.
rm -rf build/chrono_flap_sim install/chrono_flap_sim

colcon build --packages-select chrono_flap_sim
```

Success signal in the configure output:

```
-- chrono_flap_sim: Found Chrono_vsg at /usr/local/lib/libChrono_vsg.so
-- chrono_flap_sim: Found VSG headers at /home/<you>/Packages/vsg/include
-- chrono_flap_sim: Found libvsg at /home/<you>/Packages/vsg/lib/libvsg.so...
-- chrono_flap_sim: Compiling with CHRONO_FLAP_USE_VSG
```

If you see `chrono_flap_sim: Chrono_vsg found but vsg/all.h not found in
any of: ...` then `CMAKE_PREFIX_PATH` isn't reaching CMake — check that
you exported it in the same shell you're running `colcon` from.

---

## Run

VSG is also gated by a runtime parameter (off by default for headless CI
safety):

```bash
source install/setup.bash
ros2 launch chrono_flap_sim sil_mode.launch.py enable_visualization:=true
```

Other entry points that accept the same flag:

```bash
ros2 launch hil_odrive_ros2_control parallel_mode.launch.py enable_visualization:=true
ros2 launch hil_odrive_ros2_control hil_mode.launch.py      enable_visualization:=true
ros2 run    chrono_flap_sim chrono_flap_node --ros-args -p enable_visualization:=true
```

Success looks like:

- Log line: `VSG visualization initialized.`
- **No** `Failed to read font` line.
- An 800×600 window titled *"Chrono Flap Simulation (Inverted Pendulum)"*
  opens and stays up.

---

## Why each piece is required — the gotchas

These are the traps we hit during bring-up. Knowing them up front will
save you the same investigation.

### Gotcha #1 — `CMAKE_PREFIX_PATH` env var ≠ CMake variable

CMake's `find_package()` reads both the variable and the env var, but
custom CMake code (like a `foreach` walking prefixes) only sees what was
set via `-D` or `set()`. `colcon` passes workspace prefixes via the env
var, so naïve detection code silently fails. The CMakeLists in this repo
explicitly merges `$ENV{CMAKE_PREFIX_PATH}` to handle this.

### Gotcha #2 — "Library found, headers missing" is its own failure mode

If CMake links `libChrono_vsg.so` but never finds `vsg/all.h`, the build
will fail at the very first `#include <vsg/all.h>` line. The CMakeLists
here delays committing to a VSG build (defining `CHRONO_FLAP_USE_VSG` and
adding the lib) until BOTH library AND headers are confirmed. Without
that guard, missing headers turn a recoverable "go headless" into a hard
compile error.

### Gotcha #3 — `CHRONO_DATA_PATH` won't fix the font crash

Even though `OpenSans-Bold.vsgb` lives in Chrono's data tree, the
**loader** is VSG's, not Chrono's. The font is loaded via
`vsg::read_cast<vsg::Font>("vsg/fonts/OpenSans-Bold.vsgb", options)`,
which uses VSG's path resolution → `VSG_FILE_PATH`. Setting
`CHRONO_DATA_PATH` does nothing for this. Always set `VSG_FILE_PATH`.

### Gotcha #4 — Font failure logs as warning, then segfaults

`ChVisualSystemVSG::Initialize()` logs `Failed to read font` as
non-fatal, then null-derefs on the first render frame when the HUD tries
to draw text with a null font handle. Always exit code `-11` (SIGSEGV)
one frame after init. Treat any `Failed to read font` line as critical.

### Gotcha #5 — CMake caches `find_*` results

Once `find_library(CHRONO_VSG_LIB ...)` caches a path in
`build/<pkg>/CMakeCache.txt`, changing `CMAKE_PREFIX_PATH` won't trigger
a re-lookup. Always `rm -rf build/<pkg> install/<pkg>` (or pass
`--cmake-clean-cache`) when you change any discovery-related env var.

### Gotcha #6 — Visual shape axis order ≠ URDF box axis order

`ChVisualShapeBox(x, y, z)` and URDF `<box size="x y z"/>` are both
positional, so the order of arguments must match in both files or the
flap will look rotated wrong in VSG vs. RViz. The dynamics are unaffected
(they come from `SetInertiaXX` and the joint axis), only the visual mesh
orientation differs. Keep `chrono_flap_node.cpp` and `motor.urdf.xacro`
in sync if you change flap dimensions.

---

## Troubleshooting matrix

| Symptom | Likely cause | Fix |
|---|---|---|
| `fatal error: vsg/all.h: No such file or directory` | `CMAKE_PREFIX_PATH` missing VSG, or stale CMake cache | Export env var; `rm -rf build/chrono_flap_sim`; rebuild |
| `error while loading shared libraries: libvsg.so.X: cannot open shared object file` | `LD_LIBRARY_PATH` missing VSG | Add `$HOME/Packages/vsg/lib` to `LD_LIBRARY_PATH` |
| `Failed to read font : vsg/fonts/OpenSans-Bold.vsgb` + process exits with code `-11` | `VSG_FILE_PATH` doesn't include Chrono's data dir | Add `/usr/local/share/chrono/data` to `VSG_FILE_PATH` |
| Build succeeds, log says `Visualization disabled by parameter`, no window | `enable_visualization` is false (the default) | Re-launch with `enable_visualization:=true` |
| Build succeeds, log says `VSG initialization failed (...)`, no window | No Vulkan driver, no display, or Wayland incompatibility | Run `vulkaninfo --summary`; check `$DISPLAY`; try `XDG_SESSION_TYPE=x11` |
| Window opens but flap orientation looks rotated vs. RViz | `ChVisualShapeBox` axis order doesn't match URDF `<box size>` order | Reconcile axes between `chrono_flap_node.cpp` and `motor.urdf.xacro` |
| `colcon build` ignores newly exported `CMAKE_PREFIX_PATH` | Stale `CMakeCache.txt` | `rm -rf build/chrono_flap_sim install/chrono_flap_sim` |

---

## Useful diagnostic commands

```bash
# Where is the font, really?
sudo find / -name "OpenSans-Bold.vsgb" 2>/dev/null

# What libs does the built executable actually link against / require?
ldd install/chrono_flap_sim/lib/chrono_flap_sim/chrono_flap_node | grep -E 'vsg|not found'

# Does Vulkan work at all?
vulkaninfo --summary

# Get a real backtrace if it still segfaults after enabling VSG
ros2 run --prefix 'gdb -batch -ex run -ex "bt 40"' chrono_flap_sim chrono_flap_node \
  --ros-args -p enable_visualization:=true -p sil_mode:=true
```

---

## Related files

- `src/chrono_flap_sim/CMakeLists.txt` — VSG detection logic (build stage)
- `src/chrono_flap_sim/src/chrono_flap_node.cpp` — visualization init
  (look for `#if defined(CHRONO_FLAP_USE_VSG)` and `init_visualization()`)
- `src/hil_odrive_ros2_control/description/urdf/motor.urdf.xacro` — URDF
  box that must agree with the Chrono `ChVisualShapeBox` axes
