#pragma once
#pragma comment(lib, "k4a.lib")
#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <k4a/k4a.hpp>
#include <k4abt.hpp>
#include <pcl/console/time.h>

struct Body
{
	float joints[96];
	char joint_confidence_level[32];

};

class KinectCapture
{
public:
	KinectCapture(int);
	~KinectCapture();

	bool bInitialized_ = false;
	bool bMasterDevice_ = false;
	int pSize_;
	std::vector<int16_t> pCloudXYZ_;
	std::vector<uint8_t> pColorRGB_;
	cv::Mat depth_img;
	cv::Mat color_img;

	bool InitDevice();
	bool StartCameres();
	bool InitCapture();
	bool UpdateFrame();
	bool UpdateFrame_img_in_depth_space();
	bool UpdateFrame_img();
	bool SaveFrame_png(std::string path);
	double SaveFrame_png_full(std::string path);
	bool SavePC_txt();

private:
	k4a::device device_;
	k4a::capture capture_;
	k4abt::tracker tracker_;
	k4a::calibration sensor_calibration_;
	k4a::transformation transformation;
	k4a_device_configuration_t config;
	int deviceno_ = 0;
	uint64_t timeStamp_;
	int color_image_width_pixels;
	int color_image_height_pixels;
	int depth_image_width_pixels;
	int depth_image_height_pixels;
};