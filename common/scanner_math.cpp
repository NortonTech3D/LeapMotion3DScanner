#include "scanner_math.h"

#include <cmath>
#include <sstream>

namespace scanner {

CalibrationCoord calibration_coord(unsigned int x,
                                   unsigned int y,
                                   unsigned int map_width,
                                   unsigned int map_height) {
    CalibrationCoord coord{};
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
    const float denominator = depth_value * dc1 + dc2;
    if (std::fabs(denominator) <= 1e-12f) {
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
    std::ostringstream out;
    out << prefix << (frame_index + 1) << extension;
    return out.str();
}

}  // namespace scanner
