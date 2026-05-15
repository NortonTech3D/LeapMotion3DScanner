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
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <atomic>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <deque>
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
static const size_t MAX_CAPTURED_FRAMES = 20;
static const uint64_t FRAME_SAMPLE_STRIDE = 2;
static const char* QUALITY_MODE_PREVIEW = "preview";
static const char* QUALITY_MODE_HIGH = "high";

enum class ReconstructionMode {
    FastPreview,
    HighQuality,
};

struct ReconstructionConfig {
    int numDisparities;
    int blockSize;
    int uniquenessRatio;
    int speckleWindowSize;
    int speckleRange;
    int disp12MaxDiff;
    double wlsLambda;
    double wlsSigma;
    float voxelLeafSize;
    float confidencePercentile;
    int maxIcpIterations;
    double maxIcpFitness;
};

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
    Rect roi;
    float adaptiveConfidenceThreshold = 64.0f;
};

struct CapturedFrame {
    Mat leftRaw;
    Mat rightRaw;
    LEAP_DISTORTION_MATRIX leftDistortion{};
    LEAP_DISTORTION_MATRIX rightDistortion{};
    uint64_t frameId = 0;
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
uint64_t g_image_event_counter = 0;
deque<CapturedFrame> g_captured_frames;

ReconstructionMode get_reconstruction_mode() {
    const char* quality_env = std::getenv("SCANNER_QUALITY");
    if (!quality_env) {
        return ReconstructionMode::HighQuality;
    }

    string value = quality_env;
    transform(value.begin(), value.end(), value.begin(),
              [](char c) { return static_cast<char>(std::tolower(static_cast<unsigned char>(c))); });

    if (value == "preview" || value == "fast") {
        return ReconstructionMode::FastPreview;
    }

    return ReconstructionMode::HighQuality;
}

ReconstructionConfig build_reconstruction_config(ReconstructionMode mode) {
    if (mode == ReconstructionMode::FastPreview) {
        return ReconstructionConfig{
            96,
            5,
            12,
            120,
            3,
            1,
            10000.0,
            1.4,
            1.5f,
            0.50f,
            35,
            3.0,
        };
    }

    return ReconstructionConfig{
        160,
        3,
        8,
        80,
        2,
        1,
        14000.0,
        1.2,
        0.75f,
        0.35f,
        75,
        2.0,
    };
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

Mat apply_clahe_normalization(const Mat& input) {
    Mat normalized;
    Ptr<CLAHE> clahe = createCLAHE(2.0, Size(8, 8));
    clahe->apply(input, normalized);
    return normalized;
}

float compute_adaptive_confidence_threshold(const Mat& confidence,
                                            const Mat& disparity32,
                                            const Rect& roi,
                                            float percentile) {
    vector<float> confidence_values;
    confidence_values.reserve(static_cast<size_t>(roi.area() / 2));

    for (int y = roi.y; y < roi.y + roi.height; ++y) {
        const float* conf_row = confidence.ptr<float>(y);
        const float* disp_row = disparity32.ptr<float>(y);
        for (int x = roi.x; x < roi.x + roi.width; ++x) {
            if (disp_row[x] > 0.0f) {
                confidence_values.push_back(conf_row[x]);
            }
        }
    }

    if (confidence_values.empty()) {
        return 64.0f;
    }
    if (confidence_values.size() == 1) {
        return std::clamp(confidence_values.front(), 32.0f, 224.0f);
    }

    const float normalized_percentile = std::clamp(percentile, 0.0f, 1.0f);
    const float raw_index = normalized_percentile * static_cast<float>(confidence_values.size() - 1);
    const size_t percentile_index = std::min(confidence_values.size() - 1, static_cast<size_t>(raw_index));
    std::nth_element(confidence_values.begin(), confidence_values.begin() + static_cast<std::ptrdiff_t>(percentile_index), confidence_values.end());
    const float percentile_value = confidence_values[percentile_index];
    return std::clamp(percentile_value, 32.0f, 224.0f);
}

DisparityResult compute_disparity_quality(const Mat& left_rect,
                                          const Mat& right_rect,
                                          const ReconstructionConfig& config) {
    DisparityResult out;

    Mat left_norm = apply_clahe_normalization(left_rect);
    Mat right_norm = apply_clahe_normalization(right_rect);

    Ptr<StereoSGBM> left_matcher = StereoSGBM::create(0, config.numDisparities, config.blockSize);
    left_matcher->setPreFilterCap(63);
    left_matcher->setUniquenessRatio(config.uniquenessRatio);
    left_matcher->setSpeckleWindowSize(config.speckleWindowSize);
    left_matcher->setSpeckleRange(config.speckleRange);
    left_matcher->setDisp12MaxDiff(config.disp12MaxDiff);
    left_matcher->setP1(8 * config.blockSize * config.blockSize);
    left_matcher->setP2(32 * config.blockSize * config.blockSize);
    left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);

    Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);
    Ptr<DisparityWLSFilter> wls_filter = createDisparityWLSFilter(left_matcher);
    wls_filter->setLambda(config.wlsLambda);
    wls_filter->setSigmaColor(config.wlsSigma);

    Mat right_disp;
    left_matcher->compute(left_norm, right_norm, out.disparity16);
    right_matcher->compute(right_norm, left_norm, right_disp);

    Mat filtered16;
    wls_filter->filter(out.disparity16, left_norm, filtered16, right_disp);
    out.confidence = wls_filter->getConfidenceMap().clone();

    out.disparity16 = filtered16;
    out.disparity16.convertTo(out.disparity32, CV_32F, 1.0 / 16.0);
    getDisparityVis(out.disparity16, out.disparityVis, 1.0);

    out.roi = wls_filter->getROI();
    if (out.roi.width <= 0 || out.roi.height <= 0) {
        out.roi = Rect(0, 0, out.disparity32.cols, out.disparity32.rows);
    }
    out.adaptiveConfidenceThreshold = compute_adaptive_confidence_threshold(out.confidence, out.disparity32, out.roi, config.confidencePercentile);

    return out;
}

void write_disparity_outputs(const DisparityResult& disparity) {
    imwrite("streamingData/filtered_disparity_vis.png", disparity.disparityVis);

    FileStorage fs("streamingData/disparity_metric.yml", FileStorage::WRITE);
    fs << "disparity32" << disparity.disparity32;
    fs << "confidence" << disparity.confidence;
    fs << "wls_roi_x" << disparity.roi.x;
    fs << "wls_roi_y" << disparity.roi.y;
    fs << "wls_roi_w" << disparity.roi.width;
    fs << "wls_roi_h" << disparity.roi.height;
    fs << "adaptive_confidence_threshold" << disparity.adaptiveConfidenceThreshold;
    fs.release();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr build_cloud_from_depth(const Mat& xyz,
                                                           const Mat& disparity32,
                                                           const Mat& confidence,
                                                           const Rect& roi,
                                                           float confidence_threshold) {
    auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->reserve(static_cast<size_t>(roi.area()));

    for (int y = roi.y; y < roi.y + roi.height; ++y) {
        const Vec3f* xyz_row = xyz.ptr<Vec3f>(y);
        const float* disp_row = disparity32.ptr<float>(y);
        const float* conf_row = confidence.ptr<float>(y);
        for (int x = roi.x; x < roi.x + roi.width; ++x) {
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

pcl::PointCloud<pcl::PointXYZ>::Ptr filter_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                                                        const ReconstructionConfig& config) {
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
    voxel.setLeafSize(config.voxelLeafSize, config.voxelLeafSize, config.voxelLeafSize);
    voxel.filter(*voxel_cloud);

    return voxel_cloud;
}

bool register_with_icp(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
                       const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
                       const ReconstructionConfig& config,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr& aligned,
                       Eigen::Matrix4f& transform,
                       double& fitness_score) {
    if (!source || source->empty() || !target || target->empty()) {
        return false;
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations(config.maxIcpIterations);
    icp.setMaxCorrespondenceDistance(25.0);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-5);
    icp.setInputSource(source);
    icp.setInputTarget(target);

    pcl::PointCloud<pcl::PointXYZ> aligned_stack;
    icp.align(aligned_stack);

    if (!icp.hasConverged()) {
        return false;
    }

    fitness_score = icp.getFitnessScore();
    if (fitness_score > config.maxIcpFitness) {
        return false;
    }

    aligned = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(aligned_stack);
    transform = icp.getFinalTransformation();
    return true;
}

void process_pipeline(const vector<CapturedFrame>& frames) {
    if (frames.empty()) {
        cerr << "No frame captured for processing." << endl;
        return;
    }

    const ReconstructionMode mode = get_reconstruction_mode();
    const ReconstructionConfig config = build_reconstruction_config(mode);

    static Mat left_map_x, left_map_y, right_map_x, right_map_y;
    static bool left_map_ready = false;
    static bool right_map_ready = false;

    StereoCalibration calib;
    if (!load_stereo_calibration("Leapcalib.yml", calib, frames.front().leftRaw.size())) {
        cerr << "Skipping disparity/depth reconstruction due to missing calibration." << endl;
        return;
    }

    auto fused_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud;
    int accepted_frames = 0;
    int skipped_frames = 0;
    DisparityResult last_disparity;

    for (size_t i = 0; i < frames.size(); ++i) {
        const CapturedFrame& frame = frames[i];
        Mat left_corrected = undistort_leap_frame(frame.leftRaw, frame.leftDistortion, left_map_x, left_map_y, left_map_ready);
        Mat right_corrected = undistort_leap_frame(frame.rightRaw, frame.rightDistortion, right_map_x, right_map_y, right_map_ready);

        if (left_corrected.empty() || right_corrected.empty()) {
            skipped_frames++;
            continue;
        }

        Mat left_rect, right_rect;
        remap(left_corrected, left_rect, calib.map1x, calib.map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar(0));
        remap(right_corrected, right_rect, calib.map2x, calib.map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar(0));

        if (i == 0) {
            imwrite("streamingData/left_corrected.png", left_corrected);
            imwrite("streamingData/right_corrected.png", right_corrected);
            imwrite("streamingData/left_rectified.png", left_rect);
            imwrite("streamingData/right_rectified.png", right_rect);
        }

        DisparityResult disparity = compute_disparity_quality(left_rect, right_rect, config);
        Mat xyz;
        reprojectImageTo3D(disparity.disparity32, xyz, calib.Q, true, CV_32F);

        auto cloud_raw = build_cloud_from_depth(xyz,
                                                disparity.disparity32,
                                                disparity.confidence,
                                                disparity.roi,
                                                disparity.adaptiveConfidenceThreshold);
        auto frame_cloud = filter_point_cloud(cloud_raw, config);

        if (!frame_cloud || frame_cloud->empty()) {
            skipped_frames++;
            continue;
        }

        if (!reference_cloud || reference_cloud->empty()) {
            *fused_cloud += *frame_cloud;
            reference_cloud = frame_cloud;
            accepted_frames++;
            last_disparity = disparity;
            continue;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud;
        Eigen::Matrix4f icp_transform = Eigen::Matrix4f::Identity();
        double fitness_score = std::numeric_limits<double>::infinity();

        if (!register_with_icp(frame_cloud, reference_cloud, config, aligned_cloud, icp_transform, fitness_score)) {
            skipped_frames++;
            continue;
        }

        *fused_cloud += *aligned_cloud;
        reference_cloud = aligned_cloud;
        accepted_frames++;
        last_disparity = disparity;
    }

    fused_cloud->width = static_cast<uint32_t>(fused_cloud->size());
    fused_cloud->height = 1;
    fused_cloud->is_dense = false;

    if (!fused_cloud || fused_cloud->empty()) {
        cerr << "Generated fused point cloud is empty." << endl;
        return;
    }

    auto cloud_filtered = filter_point_cloud(fused_cloud, config);
    write_disparity_outputs(last_disparity);

    FileStorage summary("streamingData/reconstruction_summary.yml", FileStorage::WRITE);
    summary << "quality_mode" << (mode == ReconstructionMode::HighQuality ? QUALITY_MODE_HIGH : QUALITY_MODE_PREVIEW);
    summary << "captured_frames" << static_cast<int>(frames.size());
    summary << "accepted_frames" << accepted_frames;
    summary << "skipped_frames" << skipped_frames;
    summary << "voxel_leaf_size" << config.voxelLeafSize;
    summary.release();

    pcl::io::savePCDFileBinary("streamingData/cloud_raw.pcd", *fused_cloud);
    if (cloud_filtered && !cloud_filtered->empty()) {
        pcl::io::savePCDFileBinary("streamingData/cloud_filtered.pcd", *cloud_filtered);
    }

    cout << "Pipeline complete: fused reconstruction generated from " << accepted_frames
         << " frame(s), skipped " << skipped_frames << ". Outputs are in streamingData/." << endl;
}

void handle_image_event(const LEAP_IMAGE_EVENT* image_event) {
    if (!image_event) {
        return;
    }

    const uint32_t left_width = image_event->image[0].properties.width;
    const uint32_t left_height = image_event->image[0].properties.height;
    const uint32_t right_width = image_event->image[1].properties.width;
    const uint32_t right_height = image_event->image[1].properties.height;
    if (left_width == 0 || left_height == 0 || right_width == 0 || right_height == 0) {
        return;
    }
    if (left_width != right_width || left_height != right_height) {
        return;
    }
    if (!image_event->image[0].distortion_matrix || !image_event->image[1].distortion_matrix) {
        return;
    }

    const size_t frame_bytes = static_cast<size_t>(left_width) * static_cast<size_t>(left_height);
    const size_t left_offset = static_cast<size_t>(image_event->image[0].offset);
    const size_t right_offset = static_cast<size_t>(image_event->image[1].offset);
    if (left_offset > g_image_buffer.size() || right_offset > g_image_buffer.size()) {
        return;
    }
    if (frame_bytes > g_image_buffer.size() - left_offset || frame_bytes > g_image_buffer.size() - right_offset) {
        return;
    }

    const uint8_t* left_pixels = g_image_buffer.data() + image_event->image[0].offset;
    const uint8_t* right_pixels = g_image_buffer.data() + image_event->image[1].offset;

    Mat left(static_cast<int>(left_height), static_cast<int>(left_width), CV_8UC1, const_cast<uint8_t*>(left_pixels));
    Mat right(static_cast<int>(right_height), static_cast<int>(right_width), CV_8UC1, const_cast<uint8_t*>(right_pixels));

    Mat left_copy = left.clone();
    Mat right_copy = right.clone();

    {
        lock_guard<mutex> lock(g_frame_mutex);
        g_latest_left_raw = left_copy;
        g_latest_right_raw = right_copy;
        g_latest_left_distortion = *image_event->image[0].distortion_matrix;
        g_latest_right_distortion = *image_event->image[1].distortion_matrix;
        g_has_frame = true;

        g_image_event_counter++;
        if ((g_image_event_counter % FRAME_SAMPLE_STRIDE) == 0) {
            CapturedFrame captured;
            captured.leftRaw = left_copy;
            captured.rightRaw = right_copy;
            captured.leftDistortion = *image_event->image[0].distortion_matrix;
            captured.rightDistortion = *image_event->image[1].distortion_matrix;
            captured.frameId = g_image_event_counter;
            g_captured_frames.push_back(std::move(captured));
            while (g_captured_frames.size() > MAX_CAPTURED_FRAMES) {
                g_captured_frames.pop_front();
            }
        }
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

    vector<CapturedFrame> frames;
    {
        lock_guard<mutex> lock(g_frame_mutex);
        frames.assign(g_captured_frames.begin(), g_captured_frames.end());
        if (frames.empty() && g_has_frame) {
            CapturedFrame fallback;
            fallback.leftRaw = g_latest_left_raw.clone();
            fallback.rightRaw = g_latest_right_raw.clone();
            fallback.leftDistortion = g_latest_left_distortion;
            fallback.rightDistortion = g_latest_right_distortion;
            fallback.frameId = g_image_event_counter;
            frames.push_back(std::move(fallback));
        }
    }

    process_pipeline(frames);

    LeapCloseConnection(g_connection);
    LeapDestroyConnection(g_connection);

    return 0;
}
