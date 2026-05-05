#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <string.h>
#include <iostream>
using namespace cv;
using namespace std;


int main()
{
	Mat img1, img2, g1, g2;
	Mat disp, disp8;

	img1 = imread("dis9.jpg", IMREAD_GRAYSCALE);
	img2 = imread("dst9.jpg", IMREAD_GRAYSCALE);
	/*
			StereoBM sbm;
			sbm.state->SADWindowSize = 11;
			sbm.state->numberOfDisparities = 96;
			sbm.state->preFilterSize = 7;
			sbm.state->preFilterCap = 61;
			sbm.state->minDisparity = -39;
			sbm.state->textureThreshold = 1500;
			sbm.state->uniquenessRatio = 0;
			sbm.state->speckleWindowSize = 0;
			sbm.state->speckleRange = 8;
			sbm.state->disp12MaxDiff = 1;
			sbm(img1,img2, disp);
		*/
		Ptr<StereoSGBM> sgbm = StereoSGBM::create(-64, 128, 21);
		sgbm->setPreFilterCap(4);
		sgbm->setUniquenessRatio(1);
		sgbm->setSpeckleWindowSize(150);
		sgbm->setSpeckleRange(2);
		sgbm->setDisp12MaxDiff(10);
		sgbm->setMode(StereoSGBM::MODE_SGBM);
		sgbm->setP1(600);
		sgbm->setP2(2400);
		sgbm->compute(img1, img2, disp);
		
		normalize(disp, disp8, 0, 255, NORM_MINMAX, CV_8UC1);
		imshow("left", img1);
		imshow("right", img2);
		imshow("disp", disp8);
		imwrite("ddst.jpg", disp8);

		waitKey(100);
		cin.get();
	return(0);
}