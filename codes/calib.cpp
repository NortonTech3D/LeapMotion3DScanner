#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

int main()
{
	Mat img = imread("left_167.JPG", IMREAD_UNCHANGED); 
	if (img.empty()) 
	{
		cout << "Error : Image cannot be loaded..!!" << endl;
		return -1;
	}

	namedWindow("MyWindow", WINDOW_AUTOSIZE); 
	imshow("MyWindow", img); 


	int numBoards = 1;
	int numCornersHor = 4;
	int numCornersVer = 4;

	int numSquares = numCornersHor * numCornersVer;
	Size board_sz = Size(numCornersHor, numCornersVer);
	
	vector<vector<Point3f>> object_points;
	vector<vector<Point2f>> image_points;

	vector<Point2f> corners;
	int successes = 0;

	Mat gray_image;
	
	vector<Point3f> obj;
	for (int j = 0; j<numSquares; j++)
		obj.push_back(Point3f(j / numCornersHor, j%numCornersHor, 0.0f));

	// cornerSubPix requires a single-channel (grayscale) image
	cvtColor(img, gray_image, COLOR_BGR2GRAY);

	bool found = findChessboardCorners(gray_image, board_sz, corners,
	                                   CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK);

	if (found)
	{
		cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.1));
		drawChessboardCorners(img, board_sz, corners, found);
	}
	imshow("win1", img);
		//imshow("win2", gray_image);
	waitKey(0); 
	destroyWindow("MyWindow"); 
	return 0;
}