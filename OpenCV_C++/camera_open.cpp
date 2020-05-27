#include "opencv2/opencv.hpp"
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
	cv::VideoCapture cap;
	if (!cap.open(0)) 
	{
 		cout << "Camera opening failed." << endl;	
		return 0;
       	} else {
		cout << "Camera opening succeeded." << endl;
	}
	for (;;)
	{
		cv::Mat frame;
		cap >> frame;
		if (frame.empty())
			break;
		
	}

	return 0;
}
