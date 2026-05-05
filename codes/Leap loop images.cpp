#include <vector>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <atomic>
#include <thread>
#include <chrono>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "LeapC.h"

using namespace cv;
using namespace std;

static int ct = 0;
static int counter = 0;

// Pre-allocated image buffer: 640x240 per camera, 2 cameras = 307,200 bytes.
// Resize if your device reports a different resolution.
static const uint32_t CAMERA_WIDTH  = 640;
static const uint32_t CAMERA_HEIGHT = 240;
static const uint64_t IMAGE_BUFFER_SIZE = CAMERA_WIDTH * CAMERA_HEIGHT * 2;
static vector<uint8_t> image_buffer(IMAGE_BUFFER_SIZE);

static LEAP_CONNECTION g_connection;
static atomic<bool> g_running(false);

static void handleImageEvent(const LEAP_IMAGE_EVENT* image_event)
{
    uint32_t width  = image_event->image[0].properties.width;
    uint32_t height = image_event->image[0].properties.height;

    const uint8_t* left_pixels  = image_buffer.data() + image_event->image[0].offset;
    const uint8_t* right_pixels = image_buffer.data() + image_event->image[1].offset;

    // Clone before any display or file I/O since the buffer is reused each frame
    Mat leftMat  = Mat(height, width, CV_8UC1, const_cast<uint8_t*>(left_pixels)).clone();
    Mat rightMat = Mat(height, width, CV_8UC1, const_cast<uint8_t*>(right_pixels)).clone();

    stringstream ss1, ss2;
    ss1 << "Left_"  << (ct + 1) << ".jpg";
    ss2 << "Right_" << (ct + 1) << ".jpg";

    imshow("leftMat",  leftMat);
    imshow("rightMat", rightMat);
    waitKey(1);

    imwrite(ss1.str(), leftMat);
    imwrite(ss2.str(), rightMat);

    cout << counter << endl;

    // Delay between captures to match original waitKey(1000) pacing
    this_thread::sleep_for(chrono::milliseconds(1000));

    counter++;
    ct++;
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
            // Request raw images for this tracking frame into the pre-allocated buffer
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
