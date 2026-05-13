#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>

using namespace cv;
using namespace std;


int main(){

	Mat output;
	Mat input = imread("disparity_hand_Undistorted.jpg", IMREAD_ANYDEPTH | IMREAD_ANYCOLOR);
	output=input(Rect(10,20,320,200));

	waitKey(100);
	imshow("output",output);
	imwrite("disparity_hand_Undistorted.jpg",output);
	waitKey(1000);
	cin.get();
	return 0;

}