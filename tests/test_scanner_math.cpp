#include "../common/scanner_math.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

bool nearly_equal(float a, float b, float eps = 1e-6f) {
    return std::fabs(a - b) <= eps;
}

class TestReporter {
public:
    void check(int code, const std::string& name, bool passed) {
        ++total_;
        if (passed) {
            std::cout << "PASS [T" << code << "] " << name << std::endl;
            return;
        }
        ++failures_;
        failed_codes_.push_back(code);
        std::cerr << "FAIL [E" << code << "] " << name << std::endl;
    }

    int finish() const {
        std::cout << "SUMMARY: " << (total_ - failures_) << "/" << total_ << " checks passed" << std::endl;
        if (failures_ == 0) {
            std::cout << "RESULT: PASS" << std::endl;
            return 0;
        }

        std::cerr << "RESULT: FAIL (" << failures_ << " checks failed)" << std::endl;
        std::cerr << "ERROR_CODES:";
        for (const int code : failed_codes_) {
            std::cerr << " E" << code;
        }
        std::cerr << std::endl;
        return std::min(failures_, EXIT_FAILURE);
    }

private:
    int total_ = 0;
    int failures_ = 0;
    std::vector<int> failed_codes_;
};

}  // namespace

int main() {
    TestReporter tests;

    {
        const auto coord = scanner::calibration_coord(0, 0, 320, 120);
        tests.check(1001, "calibration origin x", nearly_equal(coord.x, 0.0f));
        tests.check(1002, "calibration origin y", nearly_equal(coord.y, 0.0f));
    }

    {
        const auto coord = scanner::calibration_coord(319, 119, 320, 120);
        tests.check(1003, "calibration max x", nearly_equal(coord.x, 1.99375f));
        tests.check(1004, "calibration max y", nearly_equal(coord.y, 0.99166667f, 1e-5f));
    }

    {
        const auto coord_zero_width = scanner::calibration_coord(5, 8, 0, 120);
        const auto coord_zero_height = scanner::calibration_coord(5, 8, 320, 0);
        tests.check(1005, "calibration zero width x", nearly_equal(coord_zero_width.x, 0.0f));
        tests.check(1006, "calibration zero width y", nearly_equal(coord_zero_width.y, 0.0f));
        tests.check(1007, "calibration zero height x", nearly_equal(coord_zero_height.x, 0.0f));
        tests.check(1008, "calibration zero height y", nearly_equal(coord_zero_height.y, 0.0f));
    }

    {
        const auto midpoint = scanner::calibration_coord(160, 60, 320, 120);
        tests.check(1009, "calibration midpoint x", nearly_equal(midpoint.x, 1.0f));
        tests.check(1010, "calibration midpoint y", nearly_equal(midpoint.y, 0.5f));
    }

    {
        const auto left = scanner::calibration_coord(10, 10, 320, 120);
        const auto right = scanner::calibration_coord(11, 10, 320, 120);
        tests.check(1011, "calibration x monotonic", right.x > left.x);
        tests.check(1012, "calibration y stable", nearly_equal(right.y, left.y));
    }

    {
        tests.check(2001, "bounds inside", scanner::is_within_bounds(5, 10, 20, 30));
        tests.check(2002, "bounds left border", scanner::is_within_bounds(0, 0, 20, 30));
        tests.check(2003, "bounds right exclusive", !scanner::is_within_bounds(20, 10, 20, 30));
        tests.check(2004, "bounds bottom exclusive", !scanner::is_within_bounds(10, 30, 20, 30));
        tests.check(2005, "bounds negative", !scanner::is_within_bounds(-1, 0, 20, 30));
        tests.check(2006, "bounds zero width", !scanner::is_within_bounds(0, 0, 0, 30));
        tests.check(2007, "bounds zero height", !scanner::is_within_bounds(0, 0, 20, 0));
        tests.check(2008, "bounds negative dimensions", !scanner::is_within_bounds(0, 0, -20, -30));
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
        tests.check(3001, "depth z", nearly_equal(p.z, expected_z));
        tests.check(3002, "depth x", nearly_equal(p.x, expected_x));
        tests.check(3003, "depth y", nearly_equal(p.y, expected_y));
    }

    {
        const auto f = scanner::frame_filename("left_", 0, ".jpg");
        const auto g = scanner::frame_filename("right_", 41, ".tiff");
        const auto h = scanner::frame_filename("frame_", -1, ".bin");
        const auto i = scanner::frame_filename("frame_", 1000000, ".dat");
        tests.check(4001, "filename first", f == "left_1.jpg");
        tests.check(4002, "filename indexed", g == "right_42.tiff");
        tests.check(4003, "filename negative index", h == "frame_0.bin");
        tests.check(4004, "filename large index", i == "frame_1000001.dat");
    }

    {
        const auto p = scanner::depth_to_point(-1.0475570660838274f, 10, 20, 0.66999066565804488f,
                                               0.70185345602029203f, 181.39592173744651f,
                                               181.39592173744651f, 317.38099136734206f,
                                               138.35989671763309f);
        tests.check(3004, "depth denominator guard x", nearly_equal(p.x, 0.0f));
        tests.check(3005, "depth denominator guard y", nearly_equal(p.y, 0.0f));
        tests.check(3006, "depth denominator guard z", nearly_equal(p.z, 0.0f));
    }

    {
        const auto p_zero_fx = scanner::depth_to_point(2.5f, 10, 20, 0.6f, 0.7f, 0.0f, 181.3f, 317.3f, 138.3f);
        tests.check(3007, "depth zero fx guard x", nearly_equal(p_zero_fx.x, 0.0f));
        tests.check(3008, "depth zero fx guard y", nearly_equal(p_zero_fx.y, 0.0f));
        tests.check(3009, "depth zero fx guard z", nearly_equal(p_zero_fx.z, 0.0f));

        const auto p_zero_fy = scanner::depth_to_point(2.5f, 10, 20, 0.6f, 0.7f, 181.3f, 0.0f, 317.3f, 138.3f);
        tests.check(3010, "depth zero fy guard x", nearly_equal(p_zero_fy.x, 0.0f));
        tests.check(3011, "depth zero fy guard y", nearly_equal(p_zero_fy.y, 0.0f));
        tests.check(3012, "depth zero fy guard z", nearly_equal(p_zero_fy.z, 0.0f));
    }

    {
        const auto p_non_finite = scanner::depth_to_point(std::numeric_limits<float>::quiet_NaN(),
                                                          10,
                                                          20,
                                                          0.66999066565804488f,
                                                          0.70185345602029203f,
                                                          181.39592173744651f,
                                                          181.39592173744651f,
                                                          317.38099136734206f,
                                                          138.35989671763309f);
        tests.check(3013, "depth non-finite depth guard x", nearly_equal(p_non_finite.x, 0.0f));
        tests.check(3014, "depth non-finite depth guard y", nearly_equal(p_non_finite.y, 0.0f));
        tests.check(3015, "depth non-finite depth guard z", nearly_equal(p_non_finite.z, 0.0f));
    }

    {
        const auto j = scanner::frame_filename("frame_", -2, ".bin");
        const auto k = scanner::frame_filename("frame_", std::numeric_limits<int>::max(), ".dat");
        tests.check(4005, "filename clamp below zero", j == "frame_0.bin");
        tests.check(4006, "filename int max safe increment", k == "frame_2147483648.dat");
    }

    {
        const auto l = scanner::frame_filename("", 0, "");
        tests.check(4007, "filename empty prefix extension", l == "1");
    }

    {
        const auto p_non_finite_dc1 = scanner::depth_to_point(2.5f,
                                                              10,
                                                              20,
                                                              std::numeric_limits<float>::infinity(),
                                                              0.7f,
                                                              181.3f,
                                                              181.3f,
                                                              317.3f,
                                                              138.3f);
        tests.check(3016, "depth non-finite dc1 guard", nearly_equal(p_non_finite_dc1.z, 0.0f));

        const auto p_non_finite_dc2 = scanner::depth_to_point(2.5f,
                                                              10,
                                                              20,
                                                              0.6f,
                                                              -std::numeric_limits<float>::infinity(),
                                                              181.3f,
                                                              181.3f,
                                                              317.3f,
                                                              138.3f);
        tests.check(3017, "depth non-finite dc2 guard", nearly_equal(p_non_finite_dc2.z, 0.0f));

        const auto p_non_finite_px = scanner::depth_to_point(2.5f,
                                                             10,
                                                             20,
                                                             0.6f,
                                                             0.7f,
                                                             181.3f,
                                                             181.3f,
                                                             std::numeric_limits<float>::quiet_NaN(),
                                                             138.3f);
        tests.check(3018, "depth non-finite px guard", nearly_equal(p_non_finite_px.z, 0.0f));
    }

    {
        const auto p_near_zero_focal = scanner::depth_to_point(2.5f, 10, 20, 0.6f, 0.7f, 1e-13f, 181.3f, 317.3f, 138.3f);
        tests.check(3019, "depth near-zero fx guard", nearly_equal(p_near_zero_focal.z, 0.0f));

        const auto p_non_zero_focal = scanner::depth_to_point(2.5f, 10, 20, 0.6f, 0.7f, 1e-11f, 181.3f, 317.3f, 138.3f);
        tests.check(3020, "depth small non-zero fx accepted", std::isfinite(p_non_zero_focal.z) && !nearly_equal(p_non_zero_focal.z, 0.0f));
    }

    {
        const auto p_near_zero_denominator = scanner::depth_to_point(1.0f, 10, 20, 1.0f, -1.0f + 5e-13f, 181.3f, 181.3f, 317.3f, 138.3f);
        tests.check(3021, "depth near-zero denominator guard", nearly_equal(p_near_zero_denominator.z, 0.0f));

        const auto p_small_non_zero_denominator = scanner::depth_to_point(1.0f, 10, 20, 1.0f, -1.0f + 2e-12f, 181.3f, 181.3f, 317.3f, 138.3f);
        tests.check(3022,
                    "depth small non-zero denominator accepted",
                    std::isfinite(p_small_non_zero_denominator.z) && !nearly_equal(p_small_non_zero_denominator.z, 0.0f));
    }

    return tests.finish();
}
