#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

using namespace std;

std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

int main(int argc, char** argv)
{
	int capture_width = 1280 ;
    int capture_height = 720 ;
    int display_width = 1280 ;
    int display_height = 720 ;
    int framerate = 60 ;
    int flip_method = 0 ;

    std::string pipeline = gstreamer_pipeline(capture_width,
		capture_height,
		display_width,
		display_height,
		framerate,
		flip_method);
    std::cout << "Using pipeline: \n\t" << pipeline << "\n";
 

	cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
	if (!cap.open(0)) 
	{
 		cout << "Camera opening failed." << endl;	
		return (-1);
       	} else {
		cout << "Camera opening succeeded." << endl;
	}


	cv::namedWindow("CSI Camera", cv::WINDOW_AUTOSIZE);
	cv::Mat img;

	cout << "Hit ESC to exit" << endl;
	while(true)
	{
		if (!cap.read(img)) {
			cout << "Capture read error" << endl;
			break;
		}

		
		
	}

	return 0;
}
