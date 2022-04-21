#include <iostream>
#include <k4a/k4a.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/console/time.h>

using namespace std;

inline std::pair<int, int> GetColorDimensions(const k4a_color_resolution_t resolution) {
	//彩色相机不同设置下的分辨率
	switch (resolution)
	{
	case K4A_COLOR_RESOLUTION_720P:
		return { 1280, 720 };
	case K4A_COLOR_RESOLUTION_2160P:
		return { 3840, 2160 };
	case K4A_COLOR_RESOLUTION_1440P:
		return { 2560, 1440 };
	case K4A_COLOR_RESOLUTION_1080P:
		return { 1920, 1080 };
	case K4A_COLOR_RESOLUTION_3072P:
		return { 4096, 3072 };
	case K4A_COLOR_RESOLUTION_1536P:
		return { 2048, 1536 };

	default:
		throw std::logic_error("Invalid color dimensions value!");
	}
}

static cv::Mat color_to_opencv(const k4a::image& im) {
	cv::Mat cv_image_with_alpha(im.get_height_pixels(), im.get_width_pixels(), CV_8UC4, (void*)im.get_buffer());
	cv::Mat cv_image_no_alpha;
	cv::cvtColor(cv_image_with_alpha, cv_image_no_alpha, cv::COLOR_BGRA2RGB);
	return cv_image_no_alpha;
}

static cv::Mat depth_to_opencv(const k4a::image& im) {
	cv::Mat depth_image = cv::Mat(
		im.get_height_pixels(),
		im.get_width_pixels(),
		CV_16U,
		(void*)im.get_buffer(),
		static_cast<size_t>(im.get_stride_bytes()));
	return depth_image;
}


int main() {

	//启动kinect设备
	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	config.color_resolution = K4A_COLOR_RESOLUTION_720P;
	config.synchronized_images_only = true;

	cout << "Started opening K4A device..." << endl;
	k4a::device dev_sub = k4a::device::open(0);
	cout << "dev_sub:" << dev_sub.get_serialnum() << endl;
	dev_sub.start_cameras(&config);
	k4a::device dev_master = k4a::device::open(1);
	cout << "dev_master:" << dev_master.get_serialnum() << endl;
	dev_master.start_cameras(&config);
	cout << "Finished opening K4A device." << endl;

	//获取相机参数
	k4a::calibration calibration_master = dev_master.get_calibration(config.depth_mode, config.color_resolution);
	k4a::calibration calibration_sub = dev_sub.get_calibration(config.depth_mode, config.color_resolution);
	k4a_calibration_intrinsic_parameters_t* intrinsics_color = &calibration_master.color_camera_calibration.intrinsics.parameters;
	std::pair<int, int> color_dimensions = GetColorDimensions(config.color_resolution);

	//初始化捕获图像
	k4a::transformation transformation_master = k4a::transformation(calibration_master);
	k4a::transformation transformation_sub = k4a::transformation(calibration_sub);
	k4a::capture capture_master;
	k4a::capture capture_sub;
	k4a::image colorImage_master;
	cv::Mat colorFrame_master;
	k4a::image colorImage_sub;
	cv::Mat colorFrame_sub;
	k4a::image depthImage_master;
	k4a::image depthImage_in_color_master = NULL;
	cv::Mat depthFrame_master;
	k4a::image depthImage_sub;
	k4a::image depthImage_in_color_sub = NULL;
	cv::Mat depthFrame_sub;

	//初始化KinectFusion
	pcl::console::TicToc time0;
	pcl::console::TicToc time1;
	pcl::console::TicToc time2;
	pcl::console::TicToc time3;
	pcl::console::TicToc time4;

	//捕获图像
	time3.tic();
	dev_master.get_capture(&capture_master, std::chrono::milliseconds(0));
	dev_sub.get_capture(&capture_sub, std::chrono::milliseconds(0));
	colorImage_master = capture_master.get_color_image();
	colorFrame_master = color_to_opencv(colorImage_master);
	cv::flip(colorFrame_master, colorFrame_master, 1);
	cout << "master color frame finished " << colorFrame_master.size << endl;
	colorImage_sub = capture_sub.get_color_image();
	colorFrame_sub = color_to_opencv(colorImage_sub);
	cv::flip(colorFrame_sub, colorFrame_sub, 1);
	cout << "sub color frame finished " << colorFrame_sub.size << endl;
	depthImage_master = capture_master.get_depth_image();
	depthImage_in_color_master = transformation_master.depth_image_to_color_camera(depthImage_master);
	depthFrame_master = depth_to_opencv(depthImage_in_color_master);
	cv::flip(depthFrame_master, depthFrame_master, 1);
	//cv::imshow("master", depthFrame_master);
	cv::waitKey(0);
	cout << "master depth frame finished " << depthFrame_master.size << endl;
	depthImage_sub = capture_sub.get_depth_image();
	depthImage_in_color_sub = transformation_sub.depth_image_to_color_camera(depthImage_sub);
	depthFrame_sub = depth_to_opencv(depthImage_in_color_sub);
	cv::flip(depthFrame_sub, depthFrame_sub, 1);
	cout << "sub depth frame finished " << depthFrame_sub.size << endl;
	cout << "捕获及转换图像耗时：" << time3.toc() << "ms" << endl;

	dev_master.close();
	dev_sub.close();

	return 0;
}