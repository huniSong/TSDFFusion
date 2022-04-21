#include <iostream>
#include "KinectCapture.h"

using namespace std;

KinectCapture* cam0;
KinectCapture* cam1;
KinectCapture* cam2;

int main() {

	cam0 = new KinectCapture(0);
	cam0->InitDevice();
	cam0->StartCameres();
	cam0->InitCapture();

	//cam0 = new KinectCapture(0);
	//cam1 = new KinectCapture(1);
	//cam2 = new KinectCapture(2);

	//cam0->InitDevice();
	//cam1->InitDevice();
	//cam2->InitDevice();

	//cam1->StartCameres();//先打开从设备，再打开主设备
	//cam0->StartCameres();
	//cam2->StartCameres();

	//cam0->InitCapture();
	//cam1->InitCapture();
	//cam2->InitCapture();

	int i = 0;
	//double time = 0;
	while (i<20) {
		cam0->SaveFrame_png("E:/PLY/RGBD/sub1_"+ to_string(i) +"_");
		//cam1->SaveFrame_png("E:/PLY/RGBD/sub2_" + to_string(i) + "_");
		//cam2->SaveFrame_png("E:/PLY/RGBD/master_" + to_string(i) + "_");
		i++;
	}
	//cout << "平均耗时：" << time / (i * 3) << "ms" << endl;
	//cv::Mat cv_depth_image = cv::imread("E:/PLY/png/970276646_depth.png");
	//double min;
	//double max;
	//cv::minMaxIdx(cv_depth_image, &min, &max);
	//cv::Mat adjMap;
	//cv::convertScaleAbs(cv_depth_image, adjMap, 255 / max);
	//cv::imshow("depth", adjMap);
	//cv::waitKey(0);
	return 0;
 }
