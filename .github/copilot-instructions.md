# Copilot Cloud Agent Instructions for `LeapMotion3DScanner`

## Repository at a glance
- C++17 + CMake project for Leap Motion–based 3D scanning.
- Main build is in `/home/runner/work/LeapMotion3DScanner/LeapMotion3DScanner/CMakeLists.txt`.
- CI currently validates the lightweight math tests only (`tests/`), not the full dependency-heavy app build.
- Key directories:
  - `common/`: shared scanner math logic.
  - `tests/`: dependency-free unit tests (`scanner_math_tests`).
  - `final code/`: full pipeline binaries (`fypCode`, `mainGUI`).
  - `codes/`, `Calibration/`, `Disparity/`, `distortion/`, `Cloud Visualizer/`: standalone tools and experiments.

## Preferred agent workflow
1. **Scope first**: only touch files needed for the requested task.
2. **Read before edit**: inspect related CMake target definitions to avoid breaking optional dependency gating.
3. **Validate fast** with repository-standard commands:
   ```bash
   cmake -S tests -B build-tests -DCMAKE_BUILD_TYPE=Release
   cmake --build build-tests --parallel
   ctest --test-dir build-tests --output-on-failure
   ```
4. If task requires full app targets, configure top-level build and respect missing-optional-dependency behavior.

## Build/dependency behavior you should preserve
- OpenCV is required by the top-level build.
- `opencv_ximgproc` is optional and detected via `if(TARGET opencv_ximgproc)`.
  - Do **not** add a second `find_package(OpenCV ...)` call for ximgproc.
- PCL and LeapC are optional at configure time; affected targets are skipped when absent.
- Leap SDK root hints are resolved from:
  - `LEAPSDK_ROOT` (CMake cache var),
  - `ULTRALEAP_SDK_ROOT` (cache var),
  - corresponding environment variables,
  - then platform defaults.

## CI and local validation expectations
- CI workflow (`.github/workflows/ci.yml`) runs:
  - configure/build/test for `tests/` on Ubuntu + macOS.
- For most code changes, at minimum run the `tests/` commands above.
- If changing top-level CMake or dependency resolution logic, also run:
  ```bash
  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
  ```
  and report whether missing dependencies are expected in the environment.

## Errors encountered in this repository and workarounds
- **Encountered**: top-level configure failed with:
  - `Could not find a package configuration file provided by "OpenCV"`.
- **Why**: runner environment did not have OpenCV CMake package discoverable for main build.
- **Workarounds**:
  1. Install OpenCV development package for your platform, or
  2. Set `OpenCV_DIR` to directory containing `OpenCVConfig.cmake`, or
  3. Add install prefix to `CMAKE_PREFIX_PATH`.
- While full dependencies are unavailable, continue using `tests/` target for regression validation.

## Commit hygiene for agents
- Avoid committing generated `build/` or `build-tests/` artifacts unless explicitly requested.
- Keep changes minimal and task-focused; do not refactor unrelated legacy code.
