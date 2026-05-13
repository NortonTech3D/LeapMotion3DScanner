#include "../common/scanner_math.h"

#include <cmath>
#include <iostream>
#include <string>

namespace {

bool nearly_equal(float a, float b, float eps = 1e-6f) {
    return std::fabs(a - b) <= eps;
}

int run_test(const std::string& name, bool passed) {
    if (!passed) {
        std::cerr << "FAILED: " << name << std::endl;
        return 1;
    }
    return 0;
}

}  // namespace

int main() {
    int failures = 0;

    {
        const auto coord = scanner::calibration_coord(0, 0, 320, 120);
        failures += run_test("calibration origin x", nearly_equal(coord.x, 0.0f));
        failures += run_test("calibration origin y", nearly_equal(coord.y, 0.0f));
    }

    {
        const auto coord = scanner::calibration_coord(319, 119, 320, 120);
        failures += run_test("calibration max x", nearly_equal(coord.x, 1.99375f));
        failures += run_test("calibration max y", nearly_equal(coord.y, 0.99166667f, 1e-5f));
    }

    {
        const auto left = scanner::calibration_coord(10, 10, 320, 120);
        const auto right = scanner::calibration_coord(11, 10, 320, 120);
        failures += run_test("calibration x monotonic", right.x > left.x);
        failures += run_test("calibration y stable", nearly_equal(right.y, left.y));
    }

    {
        failures += run_test("bounds inside", scanner::is_within_bounds(5, 10, 20, 30));
        failures += run_test("bounds left border", scanner::is_within_bounds(0, 0, 20, 30));
        failures += run_test("bounds right exclusive", !scanner::is_within_bounds(20, 10, 20, 30));
        failures += run_test("bounds bottom exclusive", !scanner::is_within_bounds(10, 30, 20, 30));
        failures += run_test("bounds negative", !scanner::is_within_bounds(-1, 0, 20, 30));
    }

    {
        const float dc1 = 0.66999066565804488f;
        const float dc2 = 0.70185345602029203f;
        const float fx = 181.39592173744651f;
        const float fy = 181.39592173744651f;
        const float px = 317.38099136734206f;
        const float py = 138.35989671763309f;

        const float depth = 2.5f;
        const int u = 100;
        const int v = 150;
        const float expected_z = 1.0f / (depth * dc1 + dc2);
        const float expected_x = expected_z * (static_cast<float>(u) - px) / fx;
        const float expected_y = expected_z * (static_cast<float>(v) - py) / fy;

        const auto p = scanner::depth_to_point(depth, u, v, dc1, dc2, fx, fy, px, py);
        failures += run_test("depth z", nearly_equal(p.z, expected_z));
        failures += run_test("depth x", nearly_equal(p.x, expected_x));
        failures += run_test("depth y", nearly_equal(p.y, expected_y));
    }

    {
        const auto f = scanner::frame_filename("left_", 0, ".jpg");
        const auto g = scanner::frame_filename("right_", 41, ".tiff");
        failures += run_test("filename first", f == "left_1.jpg");
        failures += run_test("filename indexed", g == "right_42.tiff");
    }

    if (failures == 0) {
        std::cout << "All scanner_math tests passed" << std::endl;
    }
    return failures == 0 ? 0 : 1;
}
