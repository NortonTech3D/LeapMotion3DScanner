#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <atomic>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <mutex>
#include <thread>
#include <vector>

#include "LeapC.h"

using namespace cv;
using namespace cv::ximgproc;
using namespace std;

namespace {

static const uint32_t CAMERA_WIDTH = 640;
static const uint32_t CAMERA_HEIGHT = 240;
static const uint64_t IMAGE_BUFFER_SIZE = CAMERA_WIDTH * CAMERA_HEIGHT * 2;

struct StereoCalibration {
    Mat CM1, CM2, D1, D2, R, T;
    Mat R1, R2, P1, P2, Q;
    Mat map1x, map1y, map2x, map2y;
    bool ready = false;
};

struct DisparityResult {
    Mat disparity16;
    Mat disparity32;
    Mat confidence;
    Mat disparityVis;
};

LEAP_CONNECTION g_connection{};
vector<uint8_t> g_image_buffer(IMAGE_BUFFER_SIZE);
atomic<bool> g_running(false);
mutex g_frame_mutex;
Mat g_latest_left_raw;
Mat g_latest_right_raw;
LEAP_DISTORTION_MATRIX g_latest_left_distortion{};
LEAP_DISTORTION_MATRIX g_latest_right_distortion{};
bool g_has_frame = false;
bool g_saved_preview = false;

bool file_exists(const string& path) {
    return std::filesystem::exists(path);
}

bool load_stereo_calibration(const string& path, StereoCalibration& calib, const Size& image_size) {
    FileStorage fs(path, FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "Failed to open calibration file: " << path << endl;
        return false;
    }

    fs["CM1"] >> calib.CM1;
    fs["CM2"] >> calib.CM2;
    fs["D1"] >> calib.D1;
    fs["D2"] >> calib.D2;
    fs["R"] >> calib.R;
    fs["T"] >> calib.T;

    if (calib.CM1.empty() || calib.CM2.empty() || calib.D1.empty() || calib.D2.empty() ||
        calib.R.empty() || calib.T.empty()) {
        cerr << "Calibration file is missing required matrices (CM1/CM2/D1/D2/R/T)." << endl;
        return false;
    }

    stereoRectify(calib.CM1, calib.D1,
                  calib.CM2, calib.D2,
                  image_size,
                  calib.R, calib.T,
                  calib.R1, calib.R2, calib.P1, calib.P2, calib.Q,
                  CALIB_ZERO_DISPARITY,
                  0.0,
                  image_size);

    initUndistortRectifyMap(calib.CM1, calib.D1, calib.R1, calib.P1, image_size, CV_32FC1, calib.map1x, calib.map1y);
    initUndistortRectifyMap(calib.CM2, calib.D2, calib.R2, calib.P2, image_size, CV_32FC1, calib.map2x, calib.map2y);

    calib.ready = true;
    return true;
}

static inline float bilerp(float q11, float q21, float q12, float q22, float tx, float ty) {
    return q11 * (1.0f - tx) * (1.0f - ty) +
           q21 * tx * (1.0f - ty) +
           q12 * (1.0f - tx) * ty +
           q22 * tx * ty;
}

void build_distortion_remap(const LEAP_DISTORTION_MATRIX& distortion,
                           int width,
                           int height,
                           Mat& map_x,
                           Mat& map_y) {
    map_x.create(height, width, CV_32FC1);
    map_y.create(height, width, CV_32FC1);

    for (int y = 0; y < height; ++y) {
        const float normY = (height > 1) ? (static_cast<float>(y) / static_cast<float>(height - 1)) : 0.0f;
        const float calibrationY = 62.0f * (1.0f - normY);
        const int y1 = std::max(0, std::min(63, static_cast<int>(calibrationY)));
        const int y2 = std::max(0, std::min(63, y1 + 1));
        const float ty = calibrationY - std::floor(calibrationY);

        for (int x = 0; x < width; ++x) {
            const float normX = (width > 1) ? (static_cast<float>(x) / static_cast<float>(width - 1)) : 0.0f;
            const float calibrationX = 63.0f * normX;
            const int x1 = std::max(0, std::min(63, static_cast<int>(calibrationX)));
            const int x2 = std::max(0, std::min(63, x1 + 1));
            const float tx = calibrationX - std::floor(calibrationX);

            const float dX = bilerp(distortion.matrix[y1][x1].x,
                                    distortion.matrix[y1][x2].x,
                                    distortion.matrix[y2][x1].x,
                                    distortion.matrix[y2][x2].x,
                                    tx, ty);
            const float dY = bilerp(distortion.matrix[y1][x1].y,
                                    distortion.matrix[y1][x2].y,
                                    distortion.matrix[y2][x1].y,
                                    distortion.matrix[y2][x2].y,
                                    tx, ty);

            if (dX >= 0.0f && dX <= 1.0f && dY >= 0.0f && dY <= 1.0f) {
                map_x.at<float>(y, x) = dX * static_cast<float>(width - 1);
                map_y.at<float>(y, x) = dY * static_cast<float>(height - 1);
            } else {
                map_x.at<float>(y, x) = -1.0f;
                map_y.at<float>(y, x) = -1.0f;
            }
        }
    }
}

Mat undistort_leap_frame(const Mat& raw,
                         const LEAP_DISTORTION_MATRIX& distortion,
                         Mat& map_x,
                         Mat& map_y,
                         bool& map_initialized) {
    if (raw.empty()) {
        return Mat();
    }

    if (!map_initialized || map_x.size() != raw.size() || map_y.size() != raw.size()) {
        build_distortion_remap(distortion, raw.cols, raw.rows, map_x, map_y);
        map_initialized = true;
    }

    Mat corrected;
    remap(raw, corrected, map_x, map_y, INTER_LINEAR, BORDER_CONSTANT, Scalar(0));
    return corrected;
}

DisparityResult compute_disparity_quality(const Mat& left_rect, const Mat& right_rect) {
    DisparityResult out;

    const int num_disparities = 128;
    const int block_size = 5;

    Ptr<StereoSGBM> left_matcher = StereoSGBM::create(0, num_disparities, block_size);
    left_matcher->setPreFilterCap(63);
    left_matcher->setUniquenessRatio(10);
    left_matcher->setSpeckleWindowSize(100);
    left_matcher->setSpeckleRange(2);
    left_matcher->setDisp12MaxDiff(1);
    left_matcher->setP1(8 * block_size * block_size);
    left_matcher->setP2(32 * block_size * block_size);
    left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);

    Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);
    Ptr<DisparityWLSFilter> wls_filter = createDisparityWLSFilter(left_matcher);
    wls_filter->setLambda(12000.0);
    wls_filter->setSigmaColor(1.3);

    Mat right_disp;
    left_matcher->compute(left_rect, right_rect, out.disparity16);
    right_matcher->compute(right_rect, left_rect, right_disp);

    Mat filtered16;
    wls_filter->filter(out.disparity16, left_rect, filtered16, right_disp);
    out.confidence = wls_filter->getConfidenceMap().clone();

    out.disparity16 = filtered16;
    out.disparity16.convertTo(out.disparity32, CV_32F, 1.0 / 16.0);
    getDisparityVis(out.disparity16, out.disparityVis, 1.0);

    return out;
}

void write_disparity_outputs(const DisparityResult& disparity) {
    imwrite("streamingData/filtered_disparity_vis.png", disparity.disparityVis);

    FileStorage fs("streamingData/disparity_metric.yml", FileStorage::WRITE);
    fs << "disparity32" << disparity.disparity32;
    fs << "confidence" << disparity.confidence;
    fs.release();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr build_cloud_from_depth(const Mat& xyz,
                                                           const Mat& disparity32,
                                                           const Mat& confidence) {
    auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->reserve(static_cast<size_t>(xyz.rows * xyz.cols));

    const float confidence_threshold = 64.0f;

    for (int y = 0; y < xyz.rows; ++y) {
        const Vec3f* xyz_row = xyz.ptr<Vec3f>(y);
        const float* disp_row = disparity32.ptr<float>(y);
        const float* conf_row = confidence.ptr<float>(y);
        for (int x = 0; x < xyz.cols; ++x) {
            const float d = disp_row[x];
            const float conf = conf_row[x];
            if (!(d > 0.0f) || conf < confidence_threshold) {
                continue;
            }

            const Vec3f p = xyz_row[x];
            if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2])) {
                continue;
            }
            if (p[2] <= 0.0f || p[2] > 1500.0f) {
                continue;
            }

            cloud->push_back(pcl::PointXYZ(p[0], p[1], p[2]));
        }
    }

    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = false;
    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr filter_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input) {
    if (!input || input->empty()) {
        return input;
    }

    auto sor_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input);
    sor.setMeanK(20);
    sor.setStddevMulThresh(1.0);
    sor.filter(*sor_cloud);

    auto radius_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(sor_cloud);
    ror.setRadiusSearch(8.0);
    ror.setMinNeighborsInRadius(4);
    ror.filter(*radius_cloud);

    auto voxel_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(radius_cloud);
    voxel.setLeafSize(1.5f, 1.5f, 1.5f);
    voxel.filter(*voxel_cloud);

    return voxel_cloud;
}

void process_pipeline(const Mat& left_raw,
                      const Mat& right_raw,
                      const LEAP_DISTORTION_MATRIX& left_distortion,
                      const LEAP_DISTORTION_MATRIX& right_distortion) {
    if (left_raw.empty() || right_raw.empty()) {
        cerr << "No frame captured for processing." << endl;
        return;
    }

    static Mat left_map_x, left_map_y, right_map_x, right_map_y;
    static bool left_map_ready = false;
    static bool right_map_ready = false;

    Mat left_corrected = undistort_leap_frame(left_raw, left_distortion, left_map_x, left_map_y, left_map_ready);
    Mat right_corrected = undistort_leap_frame(right_raw, right_distortion, right_map_x, right_map_y, right_map_ready);

    if (left_corrected.empty() || right_corrected.empty()) {
        cerr << "Distortion correction failed." << endl;
        return;
    }

    imwrite("streamingData/left_corrected.png", left_corrected);
    imwrite("streamingData/right_corrected.png", right_corrected);

    StereoCalibration calib;
    if (!load_stereo_calibration("Leapcalib.yml", calib, left_corrected.size())) {
        cerr << "Skipping disparity/depth reconstruction due to missing calibration." << endl;
        return;
    }

    Mat left_rect, right_rect;
    remap(left_corrected, left_rect, calib.map1x, calib.map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar(0));
    remap(right_corrected, right_rect, calib.map2x, calib.map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar(0));

    imwrite("streamingData/left_rectified.png", left_rect);
    imwrite("streamingData/right_rectified.png", right_rect);

    DisparityResult disparity = compute_disparity_quality(left_rect, right_rect);
    write_disparity_outputs(disparity);

    Mat xyz;
    reprojectImageTo3D(disparity.disparity32, xyz, calib.Q, true, CV_32F);

    auto cloud_raw = build_cloud_from_depth(xyz, disparity.disparity32, disparity.confidence);
    auto cloud_filtered = filter_point_cloud(cloud_raw);

    if (!cloud_raw || cloud_raw->empty()) {
        cerr << "Generated point cloud is empty." << endl;
        return;
    }

    pcl::io::savePCDFileBinary("streamingData/cloud_raw.pcd", *cloud_raw);
    if (cloud_filtered && !cloud_filtered->empty()) {
        pcl::io::savePCDFileBinary("streamingData/cloud_filtered.pcd", *cloud_filtered);
    }

    cout << "Pipeline complete: corrected, rectified, disparity, and cloud outputs are in streamingData/." << endl;
}

void handle_image_event(const LEAP_IMAGE_EVENT* image_event) {
    const uint32_t width = image_event->image[0].properties.width;
    const uint32_t height = image_event->image[0].properties.height;

    const uint8_t* left_pixels = g_image_buffer.data() + image_event->image[0].offset;
    const uint8_t* right_pixels = g_image_buffer.data() + image_event->image[1].offset;

    Mat left(height, width, CV_8UC1, const_cast<uint8_t*>(left_pixels));
    Mat right(height, width, CV_8UC1, const_cast<uint8_t*>(right_pixels));

    Mat left_copy = left.clone();
    Mat right_copy = right.clone();

    {
        lock_guard<mutex> lock(g_frame_mutex);
        g_latest_left_raw = left_copy;
        g_latest_right_raw = right_copy;
        g_latest_left_distortion = *image_event->image[0].distortion_matrix;
        g_latest_right_distortion = *image_event->image[1].distortion_matrix;
        g_has_frame = true;
    }

    Mat preview(left.rows, left.cols * 2, CV_8UC1);
    left.copyTo(preview(Rect(0, 0, left.cols, left.rows)));
    right.copyTo(preview(Rect(left.cols, 0, right.cols, right.rows)));
    imshow("Video Streaming", preview);
    waitKey(1);

    if (!g_saved_preview) {
        imwrite("streamingData/left_raw_preview.png", left_copy);
        imwrite("streamingData/right_raw_preview.png", right_copy);
        g_saved_preview = true;
    }
}

void poll_thread() {
    while (g_running) {
        LEAP_CONNECTION_MESSAGE msg{};
        eLeapRS result = LeapPollConnection(g_connection, 100, &msg);
        if (result != eLeapRS_Success) {
            continue;
        }

        switch (msg.type) {
        case eLeapEventType_Connection:
            cout << "Connected" << endl;
            break;
        case eLeapEventType_ConnectionLost:
            cout << "Disconnected" << endl;
            g_running = false;
            break;
        case eLeapEventType_Device:
            cout << "Device found" << endl;
            break;
        case eLeapEventType_Tracking: {
            uint64_t frame_id = msg.tracking_event->info.frame_id;
            LeapRequestImages(g_connection, frame_id, eLeapImageType_Default,
                              g_image_buffer.data(), IMAGE_BUFFER_SIZE);
            break;
        }
        case eLeapEventType_Image:
            handle_image_event(msg.image_event);
            break;
        default:
            break;
        }
    }
}

}  // namespace

int main() {
    std::filesystem::create_directories("streamingData");

    eLeapRS res = LeapCreateConnection(nullptr, &g_connection);
    if (res != eLeapRS_Success) {
        cerr << "LeapCreateConnection failed: " << res << endl;
        return 1;
    }

    res = LeapOpenConnection(g_connection);
    if (res != eLeapRS_Success) {
        cerr << "LeapOpenConnection failed: " << res << endl;
        LeapDestroyConnection(g_connection);
        return 1;
    }

    LeapSetPolicyFlags(g_connection, eLeapPolicyFlag_Images, 0);

    g_running = true;
    thread poller(poll_thread);

    cout << "Capturing frames. Press Enter to stop and run reconstruction..." << endl;
    cin.get();

    g_running = false;
    poller.join();

    Mat left_raw, right_raw;
    LEAP_DISTORTION_MATRIX left_distortion{}, right_distortion{};
    {
        lock_guard<mutex> lock(g_frame_mutex);
        if (g_has_frame) {
            left_raw = g_latest_left_raw.clone();
            right_raw = g_latest_right_raw.clone();
            left_distortion = g_latest_left_distortion;
            right_distortion = g_latest_right_distortion;
        }
    }

    process_pipeline(left_raw, right_raw, left_distortion, right_distortion);

    LeapCloseConnection(g_connection);
    LeapDestroyConnection(g_connection);

    return 0;
}
