#pragma once

#include <string>

namespace scanner {

struct CalibrationCoord {
    float x;
    float y;
};

struct Point3f {
    float x;
    float y;
    float z;
};

CalibrationCoord calibration_coord(unsigned int x,
                                   unsigned int y,
                                   unsigned int map_width,
                                   unsigned int map_height);

bool is_within_bounds(int x, int y, int cols, int rows);

Point3f depth_to_point(float depth_value,
                       int u,
                       int v,
                       float dc1,
                       float dc2,
                       float fx,
                       float fy,
                       float px,
                       float py);

std::string frame_filename(const std::string& prefix, int frame_index, const std::string& extension);

}  // namespace scanner
