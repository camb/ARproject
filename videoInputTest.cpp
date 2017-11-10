// compiled with `pkg-config --libs --cflags opencv`
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <iomanip>

using namespace std;
using namespace cv;


int main (int argc, char** argv) {
    VideoCapture webcam(0);
    if (!webcam.isOpened()){
        CV_Assert("webcam failed to open");
    }
    // Set windows resolution
    webcam.set(CAP_PROP_FRAME_HEIGHT, 720);
    webcam.set(CAP_PROP_FRAME_WIDTH, 1280);
    // Set codec to YUYV to avoid MJPG corrupt JPEG warnings
    webcam.set(CAP_PROP_FOURCC, CV_FOURCC('Y', 'U', 'Y', 'V'));


    // Continuously capture fames until keystroke
    while (true) {
        // Capture video frame
        Mat webcamFrame;
        webcam.read(webcamFrame);

        // Detect 8x8 chessboard pattern, 7x7 internal points
        Size pattern = Size (7, 7);
        vector<Point2f> corners;
        findChessboardCorners(webcamFrame, pattern, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
        + CALIB_CB_FAST_CHECK);

        Scalar color = Scalar(0, 0, 255);
        for (auto p : corners) {
            Rect rect(p.x - 2, p.y - 2, 5, 5);
            rectangle(webcamFrame, rect, color, 2, LINE_8, 0);
        }

        // Display Result
        imshow("Camera Output", webcamFrame);
        if (waitKey(30) >= 0) break;
    }

    return 0;
}