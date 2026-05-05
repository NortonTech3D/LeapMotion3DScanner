cmake_minimum_required(VERSION 3.16)
project(LeapMotion3DScanner CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Platform / architecture detection
# ---------------------------------------------------------------------------
if(APPLE)
    # Detect Apple Silicon (arm64) vs Intel (x86_64)
    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64" OR
       CMAKE_OSX_ARCHITECTURES STREQUAL "arm64")
        message(STATUS "Targeting Apple Silicon (arm64)")
        # Homebrew on Apple Silicon installs to /opt/homebrew
        list(PREPEND CMAKE_PREFIX_PATH
            "/opt/homebrew"
            "/opt/homebrew/opt/opencv"
            "/opt/homebrew/opt/pcl"
            "/opt/homebrew/opt/vtk"
            "/opt/homebrew/opt/flann"
            "/opt/homebrew/opt/eigen"
        )
        # CUDA is not available on Apple Silicon
        set(WITH_CUDA OFF CACHE BOOL "Enable CUDA (not supported on Apple Silicon)" FORCE)
    else()
        message(STATUS "Targeting Intel Mac (x86_64)")
        # Homebrew on Intel Mac installs to /usr/local
        list(PREPEND CMAKE_PREFIX_PATH
            "/usr/local"
            "/usr/local/opt/opencv"
            "/usr/local/opt/pcl"
            "/usr/local/opt/vtk"
        )
        option(WITH_CUDA "Enable CUDA support" OFF)
    endif()
elseif(WIN32)
    option(WITH_CUDA "Enable CUDA support" OFF)
else()
    # Linux
    option(WITH_CUDA "Enable CUDA support" OFF)
endif()

# ---------------------------------------------------------------------------
# Find OpenCV
# ---------------------------------------------------------------------------
find_package(OpenCV REQUIRED COMPONENTS
    core calib3d highgui imgproc imgcodecs features2d
)
# ximgproc lives in opencv_contrib; find it separately so the build does not
# fail when it is absent (Filter.cpp and fypCode.cpp need it).
find_package(OpenCV QUIET COMPONENTS ximgproc)
if(OpenCV_ximgproc_FOUND OR TARGET opencv_ximgproc)
    set(HAVE_XIMGPROC TRUE)
    message(STATUS "OpenCV ximgproc found — disparity-filter targets will be built")
else()
    set(HAVE_XIMGPROC FALSE)
    message(WARNING "OpenCV ximgproc NOT found — Filter and fypCode targets will be skipped")
endif()

# ---------------------------------------------------------------------------
# Find PCL
# ---------------------------------------------------------------------------
find_package(PCL QUIET COMPONENTS common io visualization kdtree features surface)
if(PCL_FOUND)
    message(STATUS "PCL ${PCL_VERSION} found — point-cloud targets will be built")
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
else()
    message(WARNING "PCL NOT found — PCL-dependent targets will be skipped")
endif()

# ---------------------------------------------------------------------------
# Find LeapC SDK
# ---------------------------------------------------------------------------
if(APPLE)
    # Ultraleap installs the macOS SDK here by default
    set(_LEAP_HINT_INCLUDE
        "/Library/Application Support/Ultraleap/LeapSDK/include"
        "$ENV{LEAPSDK_ROOT}/include"
        "$ENV{ULTRALEAP_SDK_ROOT}/include"
    )
    set(_LEAP_HINT_LIB
        "/Library/Application Support/Ultraleap/LeapSDK/lib"
        "$ENV{LEAPSDK_ROOT}/lib"
        "$ENV{ULTRALEAP_SDK_ROOT}/lib"
    )
elseif(WIN32)
    set(_LEAP_HINT_INCLUDE
        "C:/Program Files/Ultraleap/LeapSDK/include"
        "$ENV{LEAPSDK_ROOT}/include"
        "$ENV{ULTRALEAP_SDK_ROOT}/include"
    )
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(_LEAP_HINT_LIB
            "C:/Program Files/Ultraleap/LeapSDK/lib/x64"
            "$ENV{LEAPSDK_ROOT}/lib/x64"
            "$ENV{ULTRALEAP_SDK_ROOT}/lib/x64"
        )
    else()
        set(_LEAP_HINT_LIB
            "C:/Program Files/Ultraleap/LeapSDK/lib/x86"
            "$ENV{LEAPSDK_ROOT}/lib/x86"
            "$ENV{ULTRALEAP_SDK_ROOT}/lib/x86"
        )
    endif()
else()
    # Linux
    set(_LEAP_HINT_INCLUDE
        "/usr/local/include"
        "$ENV{LEAPSDK_ROOT}/include"
        "$ENV{ULTRALEAP_SDK_ROOT}/include"
    )
    set(_LEAP_HINT_LIB
        "/usr/local/lib"
        "$ENV{LEAPSDK_ROOT}/lib"
        "$ENV{ULTRALEAP_SDK_ROOT}/lib"
    )
endif()

find_path(LEAPC_INCLUDE_DIR LeapC.h HINTS ${_LEAP_HINT_INCLUDE})
find_library(LEAPC_LIBRARY NAMES LeapC HINTS ${_LEAP_HINT_LIB})

if(LEAPC_INCLUDE_DIR AND LEAPC_LIBRARY)
    message(STATUS "LeapC SDK found: ${LEAPC_LIBRARY}")
    set(HAVE_LEAPC TRUE)
else()
    message(WARNING "LeapC SDK NOT found — Leap-dependent targets will be skipped. "
                    "Set LEAPSDK_ROOT to your Ultraleap SDK directory.")
    set(HAVE_LEAPC FALSE)
endif()

# ---------------------------------------------------------------------------
# Helper macros
# ---------------------------------------------------------------------------
# Add a simple OpenCV-only executable
macro(add_opencv_target NAME SOURCE)
    add_executable(${NAME} "${SOURCE}")
    target_include_directories(${NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${NAME} PRIVATE ${OpenCV_LIBS})
endmacro()

# Add an executable that also needs LeapC
macro(add_leapc_target NAME SOURCE)
    if(HAVE_LEAPC)
        add_executable(${NAME} "${SOURCE}")
        target_include_directories(${NAME} PRIVATE
            ${OpenCV_INCLUDE_DIRS}
            ${LEAPC_INCLUDE_DIR}
        )
        target_link_libraries(${NAME} PRIVATE
            ${OpenCV_LIBS}
            ${LEAPC_LIBRARY}
        )
        if(UNIX AND NOT APPLE)
            target_link_libraries(${NAME} PRIVATE pthread)
        endif()
    endif()
endmacro()

# Add an executable that needs PCL
macro(add_pcl_target NAME SOURCE)
    if(PCL_FOUND)
        add_executable(${NAME} "${SOURCE}")
        target_include_directories(${NAME} PRIVATE
            ${OpenCV_INCLUDE_DIRS}
            ${PCL_INCLUDE_DIRS}
        )
        target_link_libraries(${NAME} PRIVATE
            ${OpenCV_LIBS}
            ${PCL_LIBRARIES}
        )
    endif()
endmacro()

# ---------------------------------------------------------------------------
# codes/ targets
# ---------------------------------------------------------------------------
add_opencv_target(Depth           "codes/Depth.cpp")
add_opencv_target(calib           "codes/calib.cpp")
add_opencv_target(dist_nocalibr   "codes/dist_nocalibr.cpp")
add_opencv_target(disparity       "codes/disparity.cpp")
add_opencv_target(resize          "codes/resize.cpp")
add_opencv_target(undistort       "codes/undistort.cpp")

add_leapc_target(LeapLoopImages   "codes/Leap loop images.cpp")
add_leapc_target(LeapManualImages "codes/Leap manual images.cpp")
add_leapc_target(LeapDistortionMap "codes/LeapDistortionMap.cpp")
add_leapc_target(distortionMap    "codes/distortion Map.cpp")

add_pcl_target(PCLViewer          "codes/PCL.cpp")
add_pcl_target(triangulation      "codes/triangulation.cpp")

if(HAVE_XIMGPROC)
    add_opencv_target(Filter "codes/Filter.cpp")
    target_link_libraries(Filter PRIVATE opencv_ximgproc)
endif()

# ---------------------------------------------------------------------------
# Calibration/ targets
# ---------------------------------------------------------------------------
add_opencv_target(StereoCalibration   "Calibration/Stereo  Calibration.cpp")
add_opencv_target(hpCamCalibration    "Calibration/hp Cam calibration.cpp")
add_leapc_target(LeapCalibration      "Calibration/Leap Calibration.cpp")

# ---------------------------------------------------------------------------
# Disparity/ target
# ---------------------------------------------------------------------------
add_opencv_target(DisparityMap "Disparity/disparity.cpp")

# ---------------------------------------------------------------------------
# distortion/ target
# ---------------------------------------------------------------------------
add_opencv_target(distortion_nocalibr "distortion/dist_nocalibr.cpp")

# ---------------------------------------------------------------------------
# Cloud Visualizer/ target
# ---------------------------------------------------------------------------
add_pcl_target(CloudViewer "Cloud Visualizer/cloud_viewer.cpp")

# ---------------------------------------------------------------------------
# final code/ targets (need OpenCV + LeapC, optionally PCL)
# ---------------------------------------------------------------------------
if(HAVE_LEAPC AND HAVE_XIMGPROC)
    if(PCL_FOUND)
        add_executable(fypCode "final code/fypCode.cpp")
        target_include_directories(fypCode PRIVATE
            ${OpenCV_INCLUDE_DIRS}
            ${PCL_INCLUDE_DIRS}
            ${LEAPC_INCLUDE_DIR}
        )
        target_link_libraries(fypCode PRIVATE
            ${OpenCV_LIBS}
            opencv_ximgproc
            ${PCL_LIBRARIES}
            ${LEAPC_LIBRARY}
        )
        if(UNIX AND NOT APPLE)
            target_link_libraries(fypCode PRIVATE pthread)
        endif()
    endif()

    add_executable(mainGUI "final code/mainGUI.cpp")
    target_include_directories(mainGUI PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        ${LEAPC_INCLUDE_DIR}
    )
    target_link_libraries(mainGUI PRIVATE
        ${OpenCV_LIBS}
        ${LEAPC_LIBRARY}
    )
    if(UNIX AND NOT APPLE)
        target_link_libraries(mainGUI PRIVATE pthread)
    endif()
endif()
