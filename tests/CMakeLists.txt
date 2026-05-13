cmake_minimum_required(VERSION 3.16)
project(LeapMotion3DScannerTests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

add_executable(scanner_math_tests
    ../common/scanner_math.cpp
    test_scanner_math.cpp
)

target_include_directories(scanner_math_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

add_test(NAME scanner_math_tests COMMAND scanner_math_tests)
