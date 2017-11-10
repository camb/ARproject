// compiled with `pkg-config --libs --cflags opencv`
#include <opencv2/opencv.hpp>
#include <iostream>

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
        Mat cameraFrame;
        webcam.read(cameraFrame);

        // Display Result
        imshow("Camera Output", cameraFrame);
        if (waitKey(40) >= 0) break;
    }

    return 0;
}