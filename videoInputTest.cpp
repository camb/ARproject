// compiled with `pkg-config --libs --cflags opencv`
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;


int main (int argc, char** argv) {
    VideoCapture webcam(0);
    if (!webcam.isOpened()) {
        CV_Assert("webcam failed to open");
    }


    webcam.set(CAP_PROP_FRAME_HEIGHT, 720);
    webcam.set(CAP_PROP_FRAME_WIDTH, 1280);

    while (true) {
        // Capture video frame
        Mat cameraFrame;
        webcam.read(cameraFrame);
        // Display Result
        imshow("Camera Output", cameraFrame);
        if (waitKey(30) >= 0) break;
    }
    return 0;
}