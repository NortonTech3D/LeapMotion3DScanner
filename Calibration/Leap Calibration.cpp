#include <vector>
#include <cstdlib>
#include <iostream>
#include <atomic>
#include <algorithm>
#include <limits>
#include <numeric>
#include <thread>
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "LeapC.h"

using namespace cv;
using namespace std;

// ---------------------------------------------------------------------------
// Calibration parameters
// ---------------------------------------------------------------------------
static const int NUM_BOARDS = 20;
static const int BOARD_W = 9;
static const int BOARD_H = 6;
static const float BOARD_SQUARE_SIZE_MM = 10.0f;
static const Size BOARD_SZ(BOARD_W, BOARD_H);
static const int BOARD_N = BOARD_W * BOARD_H;
static const int MIN_BOARDS_AFTER_REJECTION = 12;
static const double MAX_ACCEPTABLE_STEREO_RMS = 1.5;
static const double MAX_ACCEPTABLE_MEAN_REPROJECTION_RMSE = 1.25;
static const double MIN_CENTROID_SPREAD_X = 0.20;
static const double MIN_CENTROID_SPREAD_Y = 0.15;
static const double MIN_OUTLIER_REJECTION_THRESHOLD = 1.0;
static const double MEDIAN_OUTLIER_REJECTION_SCALE = 1.6;

struct CalibrationQuality {
    double stereoRms = std::numeric_limits<double>::infinity();
    double meanReprojection = std::numeric_limits<double>::infinity();
    double medianReprojection = std::numeric_limits<double>::infinity();
    double maxReprojection = std::numeric_limits<double>::infinity();
    double centroidSpreadX = 0.0;
    double centroidSpreadY = 0.0;
    int usedSamples = 0;
    int rejectedSamples = 0;
    bool accepted = false;
};

// ---------------------------------------------------------------------------
// Persistent calibration data.
// ---------------------------------------------------------------------------
static vector<vector<Point3f>> object_points;
static vector<vector<Point2f>> imagePoints1, imagePoints2;
static int g_success = 0;

// Build the template object-point grid once at startup
static vector<Point3f> buildObjTemplate() {
    vector<Point3f> obj;
    obj.reserve(BOARD_N);
    for (int j = 0; j < BOARD_N; j++) {
        obj.push_back(Point3f(static_cast<float>(j % BOARD_W) * BOARD_SQUARE_SIZE_MM,
                              static_cast<float>(j / BOARD_W) * BOARD_SQUARE_SIZE_MM,
                              0.0f));
    }
    return obj;
}
static const vector<Point3f> OBJ_TEMPLATE = buildObjTemplate();

// ---------------------------------------------------------------------------
// Image buffer
// ---------------------------------------------------------------------------
static const uint32_t CAMERA_WIDTH = 640;
static const uint32_t CAMERA_HEIGHT = 240;
static const uint64_t IMAGE_BUFFER_SIZE = CAMERA_WIDTH * CAMERA_HEIGHT * 2;
static vector<uint8_t> image_buffer(IMAGE_BUFFER_SIZE);

static LEAP_CONNECTION g_connection;
static atomic<bool> g_running(false);
static atomic<bool> g_calibrated(false);

static double reprojection_rmse(const vector<Point3f>& obj,
                                const vector<Point2f>& img,
                                const Mat& rvec,
                                const Mat& tvec,
                                const Mat& camera_matrix,
                                const Mat& distortion) {
    vector<Point2f> projected;
    projectPoints(obj, rvec, tvec, camera_matrix, distortion, projected);

    if (projected.size() != img.size() || projected.empty()) {
        return std::numeric_limits<double>::infinity();
    }

    double sum_sq = 0.0;
    for (size_t i = 0; i < projected.size(); ++i) {
        const Point2f delta = projected[i] - img[i];
        sum_sq += static_cast<double>(delta.dot(delta));
    }

    return std::sqrt(sum_sq / static_cast<double>(projected.size()));
}

static bool compute_view_errors(const vector<vector<Point3f>>& obj_points,
                                const vector<vector<Point2f>>& img_points_1,
                                const vector<vector<Point2f>>& img_points_2,
                                const Mat& CM1,
                                const Mat& D1,
                                const Mat& CM2,
                                const Mat& D2,
                                vector<double>& view_errors) {
    view_errors.clear();
    const size_t views = obj_points.size();
    if (views == 0 || img_points_1.size() != views || img_points_2.size() != views) {
        return false;
    }

    view_errors.reserve(views);

    for (size_t i = 0; i < views; ++i) {
        Mat rvec1, tvec1, rvec2, tvec2;
        bool ok1 = solvePnP(obj_points[i], img_points_1[i], CM1, D1, rvec1, tvec1, false, SOLVEPNP_ITERATIVE);
        bool ok2 = solvePnP(obj_points[i], img_points_2[i], CM2, D2, rvec2, tvec2, false, SOLVEPNP_ITERATIVE);
        if (!ok1 || !ok2) {
            view_errors.push_back(std::numeric_limits<double>::infinity());
            continue;
        }

        const double rmse1 = reprojection_rmse(obj_points[i], img_points_1[i], rvec1, tvec1, CM1, D1);
        const double rmse2 = reprojection_rmse(obj_points[i], img_points_2[i], rvec2, tvec2, CM2, D2);
        view_errors.push_back((rmse1 + rmse2) * 0.5);
    }

    return true;
}

static void measure_pose_coverage(const vector<vector<Point2f>>& points,
                                  const Size& image_size,
                                  double& spread_x,
                                  double& spread_y) {
    spread_x = 0.0;
    spread_y = 0.0;
    if (points.empty() || image_size.width <= 0 || image_size.height <= 0) {
        return;
    }

    float min_x = 1.0f;
    float max_x = 0.0f;
    float min_y = 1.0f;
    float max_y = 0.0f;

    for (const auto& corners : points) {
        if (corners.empty()) {
            continue;
        }
        Point2f centroid(0.0f, 0.0f);
        for (const auto& c : corners) {
            centroid += c;
        }
        centroid *= (1.0f / static_cast<float>(corners.size()));

        const float nx = centroid.x / static_cast<float>(image_size.width);
        const float ny = centroid.y / static_cast<float>(image_size.height);
        min_x = std::min(min_x, nx);
        max_x = std::max(max_x, nx);
        min_y = std::min(min_y, ny);
        max_y = std::max(max_y, ny);
    }

    spread_x = static_cast<double>(max_x - min_x);
    spread_y = static_cast<double>(max_y - min_y);
}

static bool quality_gate_and_save(const vector<vector<Point3f>>& obj_points,
                                  const vector<vector<Point2f>>& img_points_1,
                                  const vector<vector<Point2f>>& img_points_2,
                                  Size image_size,
                                  const string& output_path,
                                  CalibrationQuality& quality) {
    Mat CM1 = Mat::eye(3, 3, CV_64FC1);
    Mat CM2 = Mat::eye(3, 3, CV_64FC1);
    Mat D1, D2, R, T, E, F;

    quality.stereoRms = stereoCalibrate(obj_points,
                                        img_points_1,
                                        img_points_2,
                                        CM1,
                                        D1,
                                        CM2,
                                        D2,
                                        image_size,
                                        R,
                                        T,
                                        E,
                                        F,
                                        CALIB_SAME_FOCAL_LENGTH | CALIB_ZERO_TANGENT_DIST,
                                        TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));

    vector<double> view_errors;
    if (!compute_view_errors(obj_points, img_points_1, img_points_2, CM1, D1, CM2, D2, view_errors)) {
        return false;
    }

    vector<double> finite_errors;
    finite_errors.reserve(view_errors.size());
    for (double err : view_errors) {
        if (std::isfinite(err)) {
            finite_errors.push_back(err);
        }
    }

    if (finite_errors.empty()) {
        return false;
    }

    sort(finite_errors.begin(), finite_errors.end());
    quality.medianReprojection = finite_errors[finite_errors.size() / 2];
    quality.meanReprojection = accumulate(finite_errors.begin(), finite_errors.end(), 0.0) /
                               static_cast<double>(finite_errors.size());
    quality.maxReprojection = finite_errors.back();

    measure_pose_coverage(img_points_1, image_size, quality.centroidSpreadX, quality.centroidSpreadY);

    const bool rms_ok = std::isfinite(quality.stereoRms) && quality.stereoRms <= MAX_ACCEPTABLE_STEREO_RMS;
    const bool reproj_ok = quality.meanReprojection <= MAX_ACCEPTABLE_MEAN_REPROJECTION_RMSE;
    const bool coverage_ok = quality.centroidSpreadX >= MIN_CENTROID_SPREAD_X &&
                             quality.centroidSpreadY >= MIN_CENTROID_SPREAD_Y;

    quality.accepted = rms_ok && reproj_ok && coverage_ok;

    if (!quality.accepted) {
        cerr << "Calibration quality gate failed: "
             << "stereoRms=" << quality.stereoRms
             << ", meanReprojection=" << quality.meanReprojection
             << ", centroidSpreadX=" << quality.centroidSpreadX
             << ", centroidSpreadY=" << quality.centroidSpreadY << endl;
        return false;
    }

    Mat R1, R2, P1, P2, Q;
    stereoRectify(CM1, D1, CM2, D2, image_size, R, T, R1, R2, P1, P2, Q);

    FileStorage fs(output_path, FileStorage::WRITE);
    fs << "CM1" << CM1;
    fs << "CM2" << CM2;
    fs << "D1" << D1;
    fs << "D2" << D2;
    fs << "R" << R;
    fs << "T" << T;
    fs << "E" << E;
    fs << "F" << F;
    fs << "R1" << R1;
    fs << "R2" << R2;
    fs << "P1" << P1;
    fs << "P2" << P2;
    fs << "Q" << Q;
    fs << "board_square_size_mm" << BOARD_SQUARE_SIZE_MM;
    fs << "quality" << "{";
    fs << "stereo_rms" << quality.stereoRms;
    fs << "mean_reprojection_rmse" << quality.meanReprojection;
    fs << "median_reprojection_rmse" << quality.medianReprojection;
    fs << "max_reprojection_rmse" << quality.maxReprojection;
    fs << "centroid_spread_x" << quality.centroidSpreadX;
    fs << "centroid_spread_y" << quality.centroidSpreadY;
    fs << "used_samples" << quality.usedSamples;
    fs << "rejected_samples" << quality.rejectedSamples;
    fs << "accepted" << static_cast<int>(quality.accepted);
    fs << "}";
    fs.release();

    return true;
}

// ---------------------------------------------------------------------------
// Run stereo calibration once enough boards have been collected
// ---------------------------------------------------------------------------
static void runCalibration(Size imageSize) {
    cout << "Starting Calibration" << endl;

    vector<vector<Point3f>> filtered_obj;
    vector<vector<Point2f>> filtered_left;
    vector<vector<Point2f>> filtered_right;

    Mat bootstrap_CM1 = Mat::eye(3, 3, CV_64FC1);
    Mat bootstrap_CM2 = Mat::eye(3, 3, CV_64FC1);
    Mat bootstrap_D1, bootstrap_D2, bootstrap_R, bootstrap_T, bootstrap_E, bootstrap_F;

    stereoCalibrate(object_points,
                    imagePoints1,
                    imagePoints2,
                    bootstrap_CM1,
                    bootstrap_D1,
                    bootstrap_CM2,
                    bootstrap_D2,
                    imageSize,
                    bootstrap_R,
                    bootstrap_T,
                    bootstrap_E,
                    bootstrap_F,
                    CALIB_SAME_FOCAL_LENGTH | CALIB_ZERO_TANGENT_DIST,
                    TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));

    vector<double> view_errors;
    compute_view_errors(object_points,
                        imagePoints1,
                        imagePoints2,
                        bootstrap_CM1,
                        bootstrap_D1,
                        bootstrap_CM2,
                        bootstrap_D2,
                        view_errors);

    vector<double> sorted_errors;
    for (double err : view_errors) {
        if (std::isfinite(err)) {
            sorted_errors.push_back(err);
        }
    }

    double rejection_threshold = 1.5;
    if (!sorted_errors.empty()) {
        sort(sorted_errors.begin(), sorted_errors.end());
        const double median_err = sorted_errors[sorted_errors.size() / 2];
        rejection_threshold = std::max(MIN_OUTLIER_REJECTION_THRESHOLD,
                                       median_err * MEDIAN_OUTLIER_REJECTION_SCALE);
    }

    for (size_t i = 0; i < object_points.size(); ++i) {
        if (i < view_errors.size() && std::isfinite(view_errors[i]) && view_errors[i] <= rejection_threshold) {
            filtered_obj.push_back(object_points[i]);
            filtered_left.push_back(imagePoints1[i]);
            filtered_right.push_back(imagePoints2[i]);
        }
    }

    if (static_cast<int>(filtered_obj.size()) < MIN_BOARDS_AFTER_REJECTION) {
        cerr << "Rejected too many views (" << (object_points.size() - filtered_obj.size())
             << "), falling back to all samples." << endl;
        filtered_obj = object_points;
        filtered_left = imagePoints1;
        filtered_right = imagePoints2;
    }

    CalibrationQuality quality;
    quality.usedSamples = static_cast<int>(filtered_obj.size());
    quality.rejectedSamples = static_cast<int>(object_points.size() - filtered_obj.size());

    if (!quality_gate_and_save(filtered_obj,
                               filtered_left,
                               filtered_right,
                               imageSize,
                               "Leapcalib.yml",
                               quality)) {
        cerr << "Calibration rejected due to poor quality. Collect more diverse board poses and retry." << endl;
        g_calibrated = false;
        g_running = false;
        return;
    }

    cout << "Calibration accepted. stereoRms=" << quality.stereoRms
         << ", meanRMSE=" << quality.meanReprojection
         << ", used=" << quality.usedSamples
         << ", rejected=" << quality.rejectedSamples << endl;

    g_calibrated = true;
    g_running = false;
}

// ---------------------------------------------------------------------------
// Process each image event: detect chessboard corners and accumulate samples
// ---------------------------------------------------------------------------
static void handleImageEvent(const LEAP_IMAGE_EVENT* image_event) {
    if (g_calibrated || g_success >= NUM_BOARDS) {
        return;
    }

    uint32_t width = image_event->image[0].properties.width;
    uint32_t height = image_event->image[0].properties.height;

    const uint8_t* left_pixels = image_buffer.data() + image_event->image[0].offset;
    const uint8_t* right_pixels = image_buffer.data() + image_event->image[1].offset;

    Mat left = Mat(height, width, CV_8UC1, const_cast<uint8_t*>(left_pixels)).clone();
    Mat right = Mat(height, width, CV_8UC1, const_cast<uint8_t*>(right_pixels)).clone();

    vector<Point2f> corners1, corners2;
    bool found1 = findChessboardCorners(left, BOARD_SZ, corners1,
        CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);
    bool found2 = findChessboardCorners(right, BOARD_SZ, corners2,
        CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);

    if (found1) {
        cornerSubPix(left, corners1, Size(11, 11), Size(-1, -1),
            TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.1));
        drawChessboardCorners(left, BOARD_SZ, corners1, found1);
    }
    if (found2) {
        cornerSubPix(right, corners2, Size(11, 11), Size(-1, -1),
            TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.1));
        drawChessboardCorners(right, BOARD_SZ, corners2, found2);
    }

    imshow("leftMat", left);
    imshow("rightMat", right);
    waitKey(1);

    if (found1 && found2) {
        imagePoints1.push_back(corners1);
        imagePoints2.push_back(corners2);
        object_points.push_back(OBJ_TEMPLATE);
        g_success++;
        cout << "Corners stored: " << g_success << "/" << NUM_BOARDS << endl;

        if (g_success >= NUM_BOARDS) {
            runCalibration(Size(width, height));
        }
    }
}

// ---------------------------------------------------------------------------
// Polling thread
// ---------------------------------------------------------------------------
static void pollThread() {
    while (g_running) {
        LEAP_CONNECTION_MESSAGE msg;
        eLeapRS result = LeapPollConnection(g_connection, 100, &msg);
        if (result != eLeapRS_Success)
            continue;

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
                              image_buffer.data(), IMAGE_BUFFER_SIZE);
            break;
        }
        case eLeapEventType_Image:
            handleImageEvent(msg.image_event);
            break;
        default:
            break;
        }
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
int main() {
    if (LeapCreateConnection(nullptr, &g_connection) != eLeapRS_Success) {
        cerr << "Failed to create Leap connection" << endl;
        return 1;
    }
    if (LeapOpenConnection(g_connection) != eLeapRS_Success) {
        cerr << "Failed to open Leap connection" << endl;
        LeapDestroyConnection(g_connection);
        return 1;
    }

    LeapSetPolicyFlags(g_connection, eLeapPolicyFlag_Images, 0);

    g_running = true;
    thread poller(pollThread);

    cin.get();

    g_running = false;
    poller.join();

    LeapCloseConnection(g_connection);
    LeapDestroyConnection(g_connection);
    return g_calibrated ? 0 : 1;
}
