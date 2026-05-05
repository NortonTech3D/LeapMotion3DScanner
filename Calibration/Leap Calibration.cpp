#include <vector>
#include <cstdlib>
#include <iostream>
#include <atomic>
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
static const int BOARD_W    = 9;
static const int BOARD_H    = 6;
static const Size BOARD_SZ(BOARD_W, BOARD_H);
static const int  BOARD_N  = BOARD_W * BOARD_H;

// ---------------------------------------------------------------------------
// Persistent calibration data.
// These MUST live at file scope so they accumulate across many image-event
// callbacks. If they were local to the callback they would be re-initialised
// to empty on every frame, making calibration impossible (no boards would
// ever accumulate to the required NUM_BOARDS threshold).
// ---------------------------------------------------------------------------
static vector<vector<Point3f>> object_points;
static vector<vector<Point2f>> imagePoints1, imagePoints2;
static int g_success = 0;

// Build the template object-point grid once at startup
static vector<Point3f> buildObjTemplate()
{
    vector<Point3f> obj;
    for (int j = 0; j < BOARD_N; j++)
        obj.push_back(Point3f((float)(j / BOARD_W), (float)(j % BOARD_W), 0.0f));
    return obj;
}
static const vector<Point3f> OBJ_TEMPLATE = buildObjTemplate();

// ---------------------------------------------------------------------------
// Image buffer
// ---------------------------------------------------------------------------
static const uint32_t CAMERA_WIDTH  = 640;
static const uint32_t CAMERA_HEIGHT = 240;
static const uint64_t IMAGE_BUFFER_SIZE = CAMERA_WIDTH * CAMERA_HEIGHT * 2;
static vector<uint8_t> image_buffer(IMAGE_BUFFER_SIZE);

static LEAP_CONNECTION g_connection;
static atomic<bool> g_running(false);
static atomic<bool> g_calibrated(false);

// ---------------------------------------------------------------------------
// Run stereo calibration once enough boards have been collected
// ---------------------------------------------------------------------------
static void runCalibration(Size imageSize)
{
    cout << "Starting Calibration" << endl;
    Mat CM1 = Mat(3, 3, CV_64FC1);
    Mat CM2 = Mat(3, 3, CV_64FC1);
    Mat D1, D2, R, T, E, F;

    stereoCalibrate(object_points, imagePoints1, imagePoints2,
        CM1, D1, CM2, D2, imageSize, R, T, E, F,
        TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5),
        CALIB_SAME_FOCAL_LENGTH | CALIB_ZERO_TANGENT_DIST);

    FileStorage fs1("Leapcalib.yml", FileStorage::WRITE);
    fs1 << "CM1" << CM1;
    fs1 << "CM2" << CM2;
    fs1 << "D1"  << D1;
    fs1 << "D2"  << D2;
    fs1 << "R"   << R;
    fs1 << "T"   << T;
    fs1 << "E"   << E;
    fs1 << "F"   << F;
    fs1.release();

    cout << "Done Calibration" << endl;
    g_calibrated = true;
    g_running    = false;  // signal the polling thread to exit
}

// ---------------------------------------------------------------------------
// Process each image event: detect chessboard corners and accumulate samples
// ---------------------------------------------------------------------------
static void handleImageEvent(const LEAP_IMAGE_EVENT* image_event)
{
    if (g_calibrated || g_success >= NUM_BOARDS)
        return;

    uint32_t width  = image_event->image[0].properties.width;
    uint32_t height = image_event->image[0].properties.height;

    const uint8_t* left_pixels  = image_buffer.data() + image_event->image[0].offset;
    const uint8_t* right_pixels = image_buffer.data() + image_event->image[1].offset;

    // Clone so we can annotate the images without corrupting the shared buffer
    Mat left  = Mat(height, width, CV_8UC1, const_cast<uint8_t*>(left_pixels)).clone();
    Mat right = Mat(height, width, CV_8UC1, const_cast<uint8_t*>(right_pixels)).clone();

    vector<Point2f> corners1, corners2;
    bool found1 = findChessboardCorners(left,  BOARD_SZ, corners1,
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

    imshow("leftMat",  left);
    imshow("rightMat", right);
    waitKey(1);

    if (found1 && found2) {
        imagePoints1.push_back(corners1);
        imagePoints2.push_back(corners2);
        object_points.push_back(OBJ_TEMPLATE);
        g_success++;
        cout << "Corners stored: " << g_success << "/" << NUM_BOARDS << endl;

        if (g_success >= NUM_BOARDS)
            runCalibration(Size(width, height));
    }
}

// ---------------------------------------------------------------------------
// Polling thread
// ---------------------------------------------------------------------------
static void pollThread()
{
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
int main()
{
    if (LeapCreateConnection(nullptr, &g_connection) != eLeapRS_Success) {
        cerr << "Failed to create Leap connection" << endl;
        return 1;
    }
    if (LeapOpenConnection(g_connection) != eLeapRS_Success) {
        cerr << "Failed to open Leap connection" << endl;
        LeapDestroyConnection(g_connection);
        return 1;
    }

    // Enable both image streaming and background-frame delivery
    LeapSetPolicyFlags(g_connection, eLeapPolicyFlag_Images, 0);

    g_running = true;
    thread poller(pollThread);

    // Wait for the user to press Enter, or until calibration completes and
    // sets g_running = false, causing the polling thread to exit on its own.
    cin.get();

    g_running = false;
    poller.join();

    LeapCloseConnection(g_connection);
    LeapDestroyConnection(g_connection);
    return 0;
}
