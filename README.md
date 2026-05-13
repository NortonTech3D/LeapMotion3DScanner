# LeapScanner
This project is about developing a handheld 3D scanner using Leap Motion® controller. The idea of this project is to develop a cheap 3D scanner to scan simple objects into a 3D mesh model. After scanning, the object could be used in a virtual environment like a game or simulation system. This simple scanning method may avoid the time-consuming and complicated modeling methods of designing objects typically followed in the conventional game development and simulation workflow. The motivation for developing the handheld scanner using Leap Motion® device is to reduce time and cost for modeling of small and complex 3D objects.

# Diagrammatic Representation of the Overall System
![prjct](https://cloud.githubusercontent.com/assets/10797726/17104468/73b6384e-529c-11e6-8f8c-c0395500df7e.PNG)

# Data Flow
![capture](https://cloud.githubusercontent.com/assets/10797726/17104708/7e729fd8-529d-11e6-98ef-81607098cf50.PNG)

After getting this mesh model we can use it in games and other simulation environments.

# Required Hardware & Software Components

| Component | Version |
|-----------|---------|
| [OpenCV](https://opencv.org/) | 4.x (core, calib3d, highgui, imgproc, imgcodecs, features2d) |
| [opencv_contrib](https://github.com/opencv/opencv_contrib) | matching OpenCV version (for `ximgproc` disparity filter) |
| [PCL – Point Cloud Library](https://pointclouds.org/) | 1.12+ |
| [VTK](https://vtk.org/) | 9.x (required by PCL) |
| [Ultraleap SDK (LeapC)](https://developer.leapmotion.com/sdk-leap-motion-controller-2) | 5.x |
| Leap Motion® Controller | — |
| CMake | 3.16+ |
| C++ compiler | C++17 or later |

> **CUDA** is optional and **not supported on Apple Silicon**. All CUDA-dependent paths in the code are disabled by default.

---

# Building

## macOS – Apple Silicon (M1 / M2 / M3)

> Homebrew on Apple Silicon installs to `/opt/homebrew`. The CMake build system detects this automatically.

```bash
# 1. Install build dependencies via Homebrew
brew install cmake opencv pcl vtk flann eigen

# 2. Download and install the Ultraleap SDK for macOS (arm64)
#    https://developer.leapmotion.com/sdk-leap-motion-controller-2
#    Default install path: /Library/Application Support/Ultraleap/LeapSDK/

# 3. Configure and build
git clone https://github.com/NortonTech3D/LeapMotion3DScanner.git
cd LeapMotion3DScanner
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

If the Ultraleap SDK is installed in a non-default location, point CMake to it:

```bash
cmake -B build \
      -DCMAKE_BUILD_TYPE=Release \
      -DLEAPSDK_ROOT=/path/to/LeapSDK
cmake --build build --parallel
```

## macOS – Intel

The build procedure is identical to Apple Silicon, but Homebrew installs to `/usr/local` instead of `/opt/homebrew`. The CMake build system handles this automatically based on the detected processor.

## Linux (Ubuntu / Debian)

```bash
# 1. Install build dependencies
sudo apt update
sudo apt install cmake build-essential \
    libopencv-dev libpcl-dev libvtk9-dev

# 2. Install the Ultraleap SDK
#    Download the .deb package from https://developer.leapmotion.com/sdk-leap-motion-controller-2
#    or set LEAPSDK_ROOT to your extracted SDK directory.

# 3. Configure and build
cmake -B build -DCMAKE_BUILD_TYPE=Release \
      -DLEAPSDK_ROOT=/path/to/LeapSDK
cmake --build build --parallel
```

## Windows

```powershell
# 1. Install dependencies (example using vcpkg)
vcpkg install opencv[contrib]:x64-windows pcl:x64-windows vtk:x64-windows

# 2. Install the Ultraleap SDK for Windows from
#    https://developer.leapmotion.com/sdk-leap-motion-controller-2
#    Default: C:\Program Files\Ultraleap\LeapSDK\

# 3. Configure and build (Visual Studio 2022)
cmake -B build -G "Visual Studio 17 2022" -A x64 ^
      -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build --config Release
```

## Tests

A lightweight, dependency-free unit test suite is provided for shared scanner
math logic used by the distortion and depth projection pipeline.

```bash
cmake -S tests -B build-tests
cmake --build build-tests --parallel
ctest --test-dir build-tests --output-on-failure
```

---

# Project Structure

```
LeapMotion3DScanner/
├── CMakeLists.txt              ← Top-level cross-platform build
├── Calibration/
│   ├── Leap Calibration.cpp    ← Leap camera calibration (LeapC)
│   ├── Stereo  Calibration.cpp ← Stereo camera calibration (OpenCV)
│   └── hp Cam calibration.cpp  ← Webcam calibration (OpenCV)
├── Cloud Visualizer/
│   ├── CMakeLists.txt          ← Stand-alone PCL viewer build
│   └── cloud_viewer.cpp        ← PCD point-cloud viewer
├── Disparity/
│   └── disparity.cpp           ← SGBM disparity computation
├── codes/                      ← Prototype / utility programs
│   ├── Depth.cpp
│   ├── Filter.cpp              ← WLS disparity filter (requires ximgproc)
│   ├── Leap loop images.cpp    ← Continuous Leap image capture (LeapC)
│   ├── Leap manual images.cpp  ← Manual Leap image capture (LeapC)
│   ├── LeapDistortionMap.cpp   ← Leap distortion map (LeapC)
│   ├── PCL.cpp                 ← Point-cloud writer + viewer (PCL)
│   ├── calib.cpp
│   ├── disparity.cpp
│   ├── dist_nocalibr.cpp
│   ├── distortion Map.cpp      ← Image un-distortion (LeapC)
│   ├── resize.cpp
│   ├── triangulation.cpp       ← Greedy-projection triangulation (PCL)
│   └── undistort.cpp
├── distortion/
│   └── dist_nocalibr.cpp       ← Standalone distortion correction
└── final code/
    ├── fypCode.cpp             ← Full pipeline (LeapC + OpenCV + PCL)
    └── mainGUI.cpp             ← GUI front-end (LeapC + OpenCV)
```
