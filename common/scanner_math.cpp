#include "scanner_math.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace scanner {

namespace {
constexpr float kDenominatorEpsilon = 1e-6f;
constexpr float kFocalLengthEpsilon = 1e-6f;
}

CalibrationCoord calibration_coord(unsigned int x,
                                   unsigned int y,
                                   unsigned int map_width,
                                   unsigned int map_height) {
    CalibrationCoord coord{};
    if (map_width == 0 || map_height == 0) {
        return coord;
    }
    coord.x = (static_cast<float>(x) / static_cast<float>(map_width)) * 2.0f;
    coord.y = (static_cast<float>(y) / static_cast<float>(map_height));
    return coord;
}

bool is_within_bounds(int x, int y, int cols, int rows) {
    return x >= 0 && x < cols && y >= 0 && y < rows;
}

Point3f depth_to_point(float depth_value,
                       int u,
                       int v,
                       float dc1,
                       float dc2,
                       float fx,
                       float fy,
                       float px,
                       float py) {
    if (!std::isfinite(depth_value) || !std::isfinite(dc1) || !std::isfinite(dc2) ||
        !std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(px) || !std::isfinite(py)) {
        return {0.0f, 0.0f, 0.0f};
    }
    if (std::fabs(fx) <= kFocalLengthEpsilon || std::fabs(fy) <= kFocalLengthEpsilon) {
        return {0.0f, 0.0f, 0.0f};
    }
    const float denominator = depth_value * dc1 + dc2;
    if (std::fabs(denominator) <= kDenominatorEpsilon) {
        return {0.0f, 0.0f, 0.0f};
    }
    const float z = 1.0f / denominator;
    return {
        z * (static_cast<float>(u) - px) / fx,
        z * (static_cast<float>(v) - py) / fy,
        z,
    };
}

std::string frame_filename(const std::string& prefix, int frame_index, const std::string& extension) {
    const long long safe_index = std::max(0LL, static_cast<long long>(frame_index) + 1LL);
    std::ostringstream out;
    out << prefix << safe_index << extension;
    return out.str();
}

}  // namespace scanner
