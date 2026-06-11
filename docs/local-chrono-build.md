# Building Against a Local Chrono Tree (No `make install`)

How to point `chrono_flap_sim` at a Chrono build that lives in your home
directory instead of `/usr/local`. This is the configuration you need
if you're also using [SEA-Stack](https://github.com/Project-SEA-Stack/SEA-Stack)
on the same machine — SEA-Stack requires `CH_USE_SIMD=OFF`, which
differs from the default `/usr/local` Chrono build.

> **TL;DR** — `find_package(Chrono)` and the runtime linker each need
> to be told where the build-tree Chrono lives. Four env vars in
> `~/.bashrc` solve it. No `sudo make install` required.

---

## When you need this guide

You need the local-Chrono setup if **any** of these is true:

- You're also running [SEA-Stack](https://github.com/Project-SEA-Stack/SEA-Stack) on the same machine (which requires `-DCH_USE_SIMD=OFF`)
- You can't or don't want to `sudo make install` a system-wide Chrono
- You're iterating on Chrono itself and want changes picked up without re-installing
- Your `/usr/local` Chrono is from a different version/configuration than what you need here

If none of these apply and you have a working `/usr/local` Chrono install,
the standard `~/.bashrc` block in [`VSG_SETUP.md`](VSG_SETUP.md) is what
you want — not this guide.

---

## The 4 discovery stages

| Stage | Tool that looks | Env var | What it finds | Failure if missing |
|---|---|---|---|---|
| **Build (config)** | CMake | `Chrono_DIR` | `ChronoConfig.cmake` in the build tree | `Could not find a package configuration file provided by "Chrono"` |
| **Build (linker)** | CMake | `CMAKE_PREFIX_PATH` | Backup search path; colcon also propagates this | Inconsistent — usually surfaces as a missing dependency further into the configure |
| **Run (linker)** | `ld.so` | `LD_LIBRARY_PATH` | `libChrono_core.so`, `libChrono_vsg.so` | Startup: `error while loading shared libraries: libChrono_core.so: cannot open shared object file` |
| **Run (VSG assets)** | VSG's loader | `VSG_FILE_PATH` | Fonts/shaders in `<chrono-build>/data/vsg/...` | Window opens then segfaults with `Failed to read font` (see [VSG_SETUP.md Gotcha #4](VSG_SETUP.md)) |

---

## Prerequisites

### 1. Build Chrono locally

```bash
git clone https://github.com/projectchrono/chrono.git ~/project-chrono
cd ~/project-chrono
git checkout 10.0.0   # or whichever tag you need

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCH_USE_SIMD=OFF \
  -DCMAKE_PREFIX_PATH=$HOME/Packages/vsg \
  -DENABLE_MODULE_VSG=ON
cmake --build build -j$(nproc)
```

**Do NOT run `cmake --install build`** — the whole point of this guide is
to avoid that step. The build tree itself is what we'll point at.

### 2. Confirm the artifacts exist

```bash
ls $HOME/project-chrono/build/cmake/ChronoConfig.cmake            # CMake config
ls $HOME/project-chrono/build/lib/libChrono_core.so               # core lib
ls $HOME/project-chrono/build/lib/libChrono_vsg.so                # VSG lib (optional)
ls $HOME/project-chrono/build/data/vsg/fonts/OpenSans-Bold.vsgb   # VSG fonts (optional)
```

All four paths must exist. If any are missing, fix that first — no
env-var juggling will substitute for a missing file. For VSG support
specifically, [`VSG_SETUP.md`](VSG_SETUP.md) covers the VSG SDK install
and the `-DENABLE_MODULE_VSG=ON` Chrono build flag.

---

## The env vars that make it work

Add to `~/.bashrc` (order matters — these must come AFTER any system
`/opt/ros/...` sourcing, so they win on `CMAKE_PREFIX_PATH` and
`LD_LIBRARY_PATH`):

```bash
# Project Chrono (local build tree — never `make install`'d)
# Build-time: where find_package(Chrono) finds ChronoConfig.cmake
export Chrono_DIR=$HOME/project-chrono/build/cmake
export CMAKE_PREFIX_PATH=$HOME/project-chrono/build/cmake:$CMAKE_PREFIX_PATH

# Runtime: where ld.so finds libChrono_core.so / libChrono_vsg.so
# (build-tree libs aren't in /etc/ld.so.conf.d/, so ldconfig won't find them)
export LD_LIBRARY_PATH=$HOME/project-chrono/build/lib:$LD_LIBRARY_PATH

# Runtime: VSG assets live under the build tree, not /usr/local/share
# (this REPLACES /usr/local/share/chrono/data from the standard VSG_SETUP)
export VSG_FILE_PATH="$HOME/project-chrono/build/data:$HOME/Packages/vsg/share/vsgExamples"
```

Reload: `source ~/.bashrc` (or open a new terminal).

### Verify

```bash
echo "Chrono_DIR         = $Chrono_DIR"
echo "ChronoConfig.cmake = $(test -f $Chrono_DIR/ChronoConfig.cmake && echo OK || echo MISSING)"
echo "CMAKE_PREFIX_PATH chrono entries: $(echo $CMAKE_PREFIX_PATH | tr ':' '\n' | grep -c project-chrono)"
echo "LD_LIBRARY_PATH  chrono entries:  $(echo $LD_LIBRARY_PATH  | tr ':' '\n' | grep -c project-chrono)"
echo "VSG_FILE_PATH font findable?      $(test -f $HOME/project-chrono/build/data/vsg/fonts/OpenSans-Bold.vsgb && echo OK || echo NO)"
```

All five lines should show `OK` / a number ≥ 1.

---

## Build

```bash
cd ~/<your-workspace>/Marine_Robotics_HIL_Project_Chrono_ROS2-Controls

# Wipe stale CMake cache so the new Chrono_DIR is honored. CMake caches
# find_package results — without this, env-var changes are silently ignored.
rm -rf build/chrono_flap_sim install/chrono_flap_sim

colcon build --packages-select chrono_flap_sim
```

Success signal in the configure output:

```
-- chrono_flap_sim: Using Chrono::Chrono_vsg imported target
-- chrono_flap_sim: Found VSG headers at /home/<you>/Packages/vsg/include
-- chrono_flap_sim: Found libvsg at /home/<you>/Packages/vsg/lib/libvsg.so
-- chrono_flap_sim: Compiling with CHRONO_FLAP_USE_VSG
```

Note: the path after `Found Chrono_vsg at` should be
`$HOME/project-chrono/build/lib/...`, **not** `/usr/local/lib/...`. If
you see the latter, the system install is winning over your env vars —
check the order in `~/.bashrc`.

You may see a harmless warning during configure:

```
-- Chrono libraries not found for the debug configuration: yaml-cpp;ChronoModels_robot;Chrono_core
```

This just means Chrono was built only in Release configuration. The
project also builds Release, so this is fine — ignore it.

---

## Why each piece is required — the gotchas

These are the traps you'll hit if any one env var is wrong. Knowing them
up front will save you the same investigation we did.

### Gotcha #1 — `Chrono_DIR` vs `CMAKE_PREFIX_PATH` are not interchangeable

`find_package(Chrono REQUIRED)` uses CMake's CONFIG mode lookup. It
prefers `Chrono_DIR` (case-sensitive, exact match — package-specific
hint) over `CMAKE_PREFIX_PATH` (general search path). Setting only
`CMAKE_PREFIX_PATH` works, but `Chrono_DIR` is the more direct contract
and removes ambiguity if multiple Chrono installs are on the same
machine.

### Gotcha #2 — `ldconfig -p` doesn't list your build-tree libs

`ldconfig` only sees libraries in paths listed in `/etc/ld.so.conf` and
`/etc/ld.so.conf.d/*.conf`. Your `~/project-chrono/build/lib/` is in
neither. This is normal and not a bug:

```bash
$ ldconfig -p | grep libChrono_core
# (no output — this is expected for build-tree libs)
```

`LD_LIBRARY_PATH` is the right tool for non-installed libs; it bypasses
`ldconfig` entirely at runtime.

### Gotcha #3 — `VSG_FILE_PATH` differs from the `/usr/local` case

Standard [`VSG_SETUP.md`](VSG_SETUP.md) sets:

```bash
export VSG_FILE_PATH="/usr/local/share/chrono/data:..."   # assumes Chrono is installed
```

For the local-build case, the equivalent path is
`$HOME/project-chrono/build/data` instead. Having both in
`VSG_FILE_PATH` is harmless (VSG skips paths that don't exist) — but
the build-tree path **must** be present if you never installed Chrono.

### Gotcha #4 — colcon doesn't auto-propagate `Chrono_DIR`

`colcon` reads `CMAKE_PREFIX_PATH` from the environment when configuring
each package, but it does **not** read individual `<Pkg>_DIR` vars from
the env and pass them through. If you ever see `find_package(Chrono)`
failing inside a colcon build but passing in a manual `cmake -S ... -B
build` outside it, this is why. The fix: make sure your Chrono build
directory is on `CMAKE_PREFIX_PATH`, not only `Chrono_DIR`.

### Gotcha #5 — VSG link-interface dependencies

Chrono 10's `Chrono::Chrono_vsg` target lists `vsg::vsg`,
`vsgXchange::vsgXchange`, and `vsgImGui::vsgImGui` in its link
interface. All three must be defined as imported CMake targets when
`find_package(Chrono)` materializes its export — even if you don't link
against them. The CMakeLists in this repo already calls
`find_package(vsg|vsgXchange|vsgImGui QUIET)` for you, but if you build
Chrono with a partial VSG SDK install (e.g. `vsg` only, no
`vsgXchange`), you'll get a CMake error like:

```
The link interface of target "Chrono::Chrono_vsg" contains:
    vsgXchange::vsgXchange
  but the target was not found.
```

Fix: install the full VSG SDK per [`VSG_SETUP.md`](VSG_SETUP.md).

---

## Troubleshooting matrix

| Symptom | Likely cause | Fix |
|---|---|---|
| `Could not find a package configuration file provided by "Chrono"` | `Chrono_DIR` not exported, or points to a non-build path | Re-check `Chrono_DIR=$HOME/project-chrono/build/cmake`; confirm `ChronoConfig.cmake` exists there |
| `fatal error: chrono/physics/ChSystemNSC.h: No such file or directory` | CMakeLists is on an older version that uses `${CHRONO_INCLUDE_DIRS}` (Chrono 8/9 style) | Update `src/chrono_flap_sim/CMakeLists.txt` per the Chrono 10 CMake modernization PR (uses `Chrono::Chrono_core` imported target) |
| `error while loading shared libraries: libChrono_core.so: cannot open shared object file` | `LD_LIBRARY_PATH` missing build-tree lib dir | Add `$HOME/project-chrono/build/lib` to `LD_LIBRARY_PATH` |
| Configure finds `/usr/local/lib/libChrono_vsg.so` instead of your build-tree one | A previous `sudo make install` left a stale system Chrono | Either uninstall it (`sudo rm -rf /usr/local/lib/libChrono*` etc.) or put build-tree paths FIRST in `CMAKE_PREFIX_PATH` |
| `Failed to read font : vsg/fonts/OpenSans-Bold.vsgb` + exit code `-11` | `VSG_FILE_PATH` doesn't include `$HOME/project-chrono/build/data` | Update `VSG_FILE_PATH` as shown above |
| `colcon build` ignores newly exported env vars | Stale `CMakeCache.txt` | `rm -rf build/chrono_flap_sim install/chrono_flap_sim` and rebuild |

---

## Useful diagnostic commands

```bash
# Which Chrono is the executable actually linked against?
ldd install/chrono_flap_sim/lib/chrono_flap_sim/chrono_flap_node | grep -i chrono

# Which Chrono is CMake about to find? (run from your workspace root)
cmake --find-package -DNAME=Chrono -DCOMPILER_ID=GNU -DLANGUAGE=CXX -DMODE=EXIST

# What's in CMake's cache for chrono_flap_sim?
grep -i chrono build/chrono_flap_sim/CMakeCache.txt | head -20

# Confirm Release-only Chrono is fine for our use
cmake -L build/chrono_flap_sim 2>&1 | grep -i CMAKE_BUILD_TYPE
```

---

## Related files

- [`VSG_SETUP.md`](VSG_SETUP.md) — VSG SDK setup (covers VSG itself; this doc covers Chrono around it)
- [`../README.md`](../README.md) — quick-start instructions (assume `/usr/local` Chrono by default)
- [`../src/chrono_flap_sim/CMakeLists.txt`](../src/chrono_flap_sim/CMakeLists.txt) — `find_package(Chrono)` invocation and VSG detection logic
