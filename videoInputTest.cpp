#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


int main (int argc, char** argv) {
	VideoCapture stream1(0);

	while (true) {
		// Capture video frame
		Mat cameraFrame;
		stream1.read(cameraFrame);
		// Display Result
		imshow("Camera Output", cameraFrame);
		if (waitKey(30) >= 0) break;
	}
    return 0;
}