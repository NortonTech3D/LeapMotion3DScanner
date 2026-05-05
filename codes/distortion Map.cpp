#include <vector>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <atomic>
#include <thread>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "LeapC.h"

using namespace cv;
using namespace std;

// Pre-allocated image buffer: 640x240 per camera, 2 cameras.
static const uint32_t CAMERA_WIDTH  = 640;
static const uint32_t CAMERA_HEIGHT = 240;
static const uint64_t IMAGE_BUFFER_SIZE = CAMERA_WIDTH * CAMERA_HEIGHT * 2;
static vector<uint8_t> image_buffer(IMAGE_BUFFER_SIZE);

static LEAP_CONNECTION g_connection;
static atomic<bool> g_running(false);

// Apply the Leap distortion calibration map to produce a corrected output image.
// The distortion map is a 64x64 grid of (x,y) pairs in normalised [0..1] image
// coordinates. Bilinear interpolation maps each destination pixel back to the
// raw sensor pixel it should sample.
static void applyDistortionMap(const LEAP_IMAGE_EVENT* image_event)
{
    const int destinationWidth  = 320;
    const int destinationHeight = 120;

    // Variables declared before the inner loops as in the original code
    float calibrationX, calibrationY;
    float weightX, weightY;
    float dX, dX1, dX2, dX3, dX4;
    float dY, dY1, dY2, dY3, dY4;
    int x1, x2, y1, y2;
    int denormalizedX, denormalizedY;

    // Iterate over both cameras.
    // Renamed outer loop variable from 'i' to 'cam' to prevent shadowing the
    // inner loop variable 'i' — the original code had a bug where the inner
    // loop's 'for (i = ...)' clobbered the outer 'for (int i = 0; i < 2; i++)'
    // so the second camera was never processed.
    for (int cam = 0; cam < 2; ++cam)
    {
        uint32_t width  = image_event->image[cam].properties.width;
        uint32_t height = image_event->image[cam].properties.height;

        const uint8_t* raw = image_buffer.data() + image_event->image[cam].offset;

        // Per-image 64x64 distortion calibration matrix from the Gemini SDK.
        // Each element is a LEAP_VECTOR with .x and .y in the range [0..1]
        // giving the normalised raw-sensor position for that grid point.
        const LEAP_DISTORTION_MATRIX* distortion = image_event->image[cam].distortion_matrix;

        // Output buffer for this camera's corrected pixels
        unsigned char destination[destinationWidth][destinationHeight];

        for (int px = 0; px < destinationWidth; px++)
        {
            for (int py = 0; py < destinationHeight; py++)
            {
                // Map destination pixel to a fractional position in the 64x64 grid
                calibrationX = 63.0f * px / (float)destinationWidth;
                // Y origin in the calibration map is at the bottom
                calibrationY = 62.0f * (1.0f - (float)py / (float)destinationHeight);

                // Fractional weights for bilinear interpolation
                weightX = calibrationX - truncf(calibrationX);
                weightY = calibrationY - truncf(calibrationY);

                // Grid cell corners — clamped to [0..63] to avoid out-of-bounds access
                x1 = (int)calibrationX;
                y1 = (int)calibrationY;
                x2 = min(x1 + 1, 63);
                y2 = min(y1 + 1, 63);

                // Look up the four surrounding distortion matrix entries.
                // matrix[row][col] → matrix[y][x]
                dX1 = distortion->matrix[y1][x1].x;
                dX2 = distortion->matrix[y1][x2].x;
                dX3 = distortion->matrix[y2][x1].x;
                dX4 = distortion->matrix[y2][x2].x;
                dY1 = distortion->matrix[y1][x1].y;
                dY2 = distortion->matrix[y1][x2].y;
                dY3 = distortion->matrix[y2][x1].y;
                dY4 = distortion->matrix[y2][x2].y;

                // Bilinear interpolation for the normalised X coordinate
                dX = dX1 * (1 - weightX) * (1 - weightY) +
                     dX2 * weightX       * (1 - weightY) +
                     dX3 * (1 - weightX) * weightY       +
                     dX4 * weightX       * weightY;

                // Bilinear interpolation for the normalised Y coordinate
                dY = dY1 * (1 - weightX) * (1 - weightY) +
                     dY2 * weightX       * (1 - weightY) +
                     dY3 * (1 - weightX) * weightY       +
                     dY4 * weightX       * weightY;

                if ((dX >= 0) && (dX <= 1) && (dY >= 0) && (dY <= 1)) {
                    denormalizedX = (int)(dX * width);
                    denormalizedY = (int)(dY * height);
                    destination[px][py] = raw[denormalizedX + denormalizedY * width];
                } else {
                    // Pixel maps outside the valid sensor area — output black.
                    // Original code used -1 which wraps silently to 255 for unsigned char.
                    destination[px][py] = 0;
                }
            }
        }

        // Display the corrected image for this camera
        Mat result(destinationHeight, destinationWidth, CV_8UC1);
        for (int px = 0; px < destinationWidth; px++)
            for (int py = 0; py < destinationHeight; py++)
                result.at<uchar>(py, px) = destination[px][py];

        if (cam == 0)
            imshow("Left corrected", result);
        else
            imshow("Right corrected", result);
    }
    waitKey(1);
}

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
            // Request raw images for this tracking frame
            uint64_t frame_id = msg.tracking_event->info.frame_id;
            LeapRequestImages(g_connection, frame_id, eLeapImageType_Default,
                              image_buffer.data(), IMAGE_BUFFER_SIZE);
            break;
        }
        case eLeapEventType_Image:
            applyDistortionMap(msg.image_event);
            break;
        default:
            break;
        }
    }
}

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

    // Enable raw image streaming
    LeapSetPolicyFlags(g_connection, eLeapPolicyFlag_Images, 0);

    g_running = true;
    thread poller(pollThread);

    cin.get();

    g_running = false;
    poller.join();

    LeapCloseConnection(g_connection);
    LeapDestroyConnection(g_connection);
    return 0;
}
