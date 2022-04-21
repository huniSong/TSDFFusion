#include "KinectCapture.h"

using namespace std;

KinectCapture::KinectCapture(int deviceno)
{
	device_ = NULL;
	capture_ = NULL;
	tracker_ = NULL;
	transformation = NULL;
	deviceno_ = deviceno;
	pColorRGB_.reserve(3000000);
	pCloudXYZ_.reserve(3000000);
	timeStamp_ = 0;
	config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
}

KinectCapture::~KinectCapture()
{
	device_.close();
	capture_.reset();
	capture_ = NULL;
	tracker_.destroy();
	transformation.destroy();
}

bool KinectCapture::InitDevice()
{
	int32_t color_exposure_usec = 8000;  // somewhat reasonable default exposure time
	int32_t powerline_freq = 2;          // default to a 60 Hz powerline
	//Ϊ���豸�ľ�ȷ��ʱ�����ֶ�������ɫͼ����ع�ʱ�䣬�Զ�ģʽ��ʹ��ͬ�����豸�Ͽ��ʧȥͬ����
	//ʹ��k4a_device_set_color_control�������ع�ʱ������Ϊ�ֶ�ģʽ��
	//ͬʱ��k4a_device_set_color_control�����������ÿ���Ƶ��Ϊ�ֶ�ģʽ
	device_ = k4a::device::open(deviceno_);
	device_.set_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE,
		K4A_COLOR_CONTROL_MODE_MANUAL,
		color_exposure_usec);
	device_.set_color_control(K4A_COLOR_CONTROL_POWERLINE_FREQUENCY,
		K4A_COLOR_CONTROL_MODE_MANUAL,
		powerline_freq);
	//���������豸�����ò���
	config.camera_fps = K4A_FRAMES_PER_SECOND_15;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	config.color_resolution = K4A_COLOR_RESOLUTION_720P;
	config.synchronized_images_only = true;//Ϊ��ͬʱ��ȡdepth��colorͼ������������ͼ����ͬ����
	config.subordinate_delay_off_master_usec = 0;
	if (device_.is_sync_out_connected()) {
		config.wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
		bMasterDevice_ = true;
	}	
	else {
		config.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
		bMasterDevice_ = false;
	}
	return true;
}

bool KinectCapture::InitCapture() 
{
	//��ȡ������
	device_.get_capture(&capture_, std::chrono::milliseconds{ K4A_WAIT_INFINITE });
	//��ȡ���������������Ͳ�ɫ�������ȷУ׼�ڲ�calibration
	sensor_calibration_ = device_.get_calibration(config.depth_mode, config.color_resolution);
	//�������������tracker
	tracker_ = k4abt::tracker::create(sensor_calibration_);
	//�����ڲΣ�����ת�������ִ����������ͼ�����ͼ�Ŀռ�ת������������ת����ɫ�����transformation
	transformation = k4a::transformation(sensor_calibration_);
	//��ɫ������
	color_image_width_pixels = sensor_calibration_.color_camera_calibration.resolution_width;
	color_image_height_pixels = sensor_calibration_.color_camera_calibration.resolution_height;
	//���������
	depth_image_width_pixels = sensor_calibration_.depth_camera_calibration.resolution_width;
	depth_image_height_pixels = sensor_calibration_.depth_camera_calibration.resolution_height;
	bInitialized_ = true;
	cout << "------------------" << endl;
	cout << "��ʼ����ɣ���ʼ����" << endl;
	cout << "------------------" << endl << endl;
	return true;
}

bool KinectCapture::StartCameres()
{
	//�����
	device_.start_cameras(&config);
	return true;
}

bool KinectCapture::UpdateFrame()
{
	if (!bInitialized_)
	{
		cout << "δ��ʼ��" << endl;
		return false;
	}
	if (!device_.get_capture(&capture_, std::chrono::milliseconds(0)))
	{
		cout << "δ����ͼ��" << endl;
		return false;
	}
	std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
	std::chrono::nanoseconds nanonow = now.time_since_epoch();
	std::chrono::milliseconds millinow = std::chrono::duration_cast<chrono::milliseconds>(nanonow);
	if (!tracker_.enqueue_capture(capture_))//��k4a������������ӵ�������������У����첽������������ٽ����
	{
		cout << "���岶��ʧ��" << endl;
		return false;
	}
	k4abt::frame body_frame = tracker_.pop_result();
	uint32_t num_bodies = body_frame.get_num_bodies();
	if (num_bodies == 0) {
		cout << "δʶ������" << endl;
		return false;
	}
	k4a::image colorImage = capture_.get_color_image();//��ȡ�豸��ɫͼ->��ɫ���
	std::chrono::nanoseconds get_colorImage_time = colorImage.get_system_timestamp();//��ɫͼʱ��� ���ʱ���
	timeStamp_ = get_colorImage_time.count() / 1000000; //ʱ���
	k4a::image depthImage = capture_.get_depth_image();//��ȡ�豸���ͼ->������

	k4a::image body_index_map = body_frame.get_body_index_map();//��������ͼ

	//Ԥ�������ͼתΪ��ɫ����ռ��Ļ�����
	k4a::image depth_image_in_color_space = NULL;
	depth_image_in_color_space = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
		color_image_width_pixels,
		color_image_height_pixels,
		color_image_width_pixels * (int)sizeof(uint16_t));
	//Ԥ������������ͼ������
	k4a::image body_index_map_in_color_space = NULL;
	body_index_map_in_color_space = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM8,
		color_image_width_pixels,
		color_image_height_pixels,
		color_image_width_pixels * (int)sizeof(uint8_t));
	//Ԥ������ƻ�����
	k4a::image point_cloud_image = NULL;
	point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
		color_image_width_pixels,
		color_image_height_pixels,
		color_image_width_pixels * 3 * (int)sizeof(int16_t));
	//���������ͼ�����ͼ���������ռ�תΪ��ɫ����ռ�k4a_transformation_depth_image_to_color_camera_custom
	transformation.depth_image_to_color_camera_custom(
		depthImage,
		body_index_map,
		&depth_image_in_color_space,
		&body_index_map_in_color_space,
		K4A_TRANSFORMATION_INTERPOLATION_TYPE_NEAREST,
		K4ABT_BODY_INDEX_MAP_BACKGROUND);
	//ָ����������ͼbuffer��ָ��
	uint8_t* bodyIndxBuffer;
	bodyIndxBuffer = (uint8_t*)body_index_map_in_color_space.get_buffer();
	//ָ���ɫͼbuffer��ָ��
	uint8_t* colorTextureBuffer;
	colorTextureBuffer = (uint8_t*)colorImage.get_buffer();
	//���ͼת����
	transformation.depth_image_to_point_cloud(
		depth_image_in_color_space,
		K4A_CALIBRATION_TYPE_COLOR,
		&point_cloud_image);
	//����ָ�����ͼbuffer��ָ�� get_buffer()
	int16_t* point_cloud_image_data = (int16_t*)(void*)point_cloud_image.get_buffer();

	std::chrono::system_clock::time_point end0 = std::chrono::system_clock::now();
	std::chrono::nanoseconds nanoend0 = end0.time_since_epoch();
	std::chrono::milliseconds milliend0 = std::chrono::duration_cast<chrono::milliseconds>(nanoend0);

	pCloudXYZ_.clear();
	pColorRGB_.clear();
	pSize_ = 0;
	//ѭ��ÿ������ ��������
	for (int i = 0; i < color_image_width_pixels * color_image_height_pixels; ++i) {
		//�����z�᲻Ϊ0��rgbaֵ����Ϊ0�������õ����ڸ�λ�õ�ֵ
		if (bodyIndxBuffer[i] == 255) {
			continue;//�������������򲻼���
		}
		//rgb
		pColorRGB_.push_back(colorTextureBuffer[4 * i + 2]);
		pColorRGB_.push_back(colorTextureBuffer[4 * i + 1]);
		pColorRGB_.push_back(colorTextureBuffer[4 * i + 0]);
		//xyz
		pCloudXYZ_.push_back(point_cloud_image_data[3 * i + 0]);
		pCloudXYZ_.push_back(point_cloud_image_data[3 * i + 1]);
		pCloudXYZ_.push_back(point_cloud_image_data[3 * i + 2]);
		pSize_++;//����
	}

	std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
	std::chrono::nanoseconds nanoend = end.time_since_epoch();
	std::chrono::milliseconds milliend = std::chrono::duration_cast<chrono::milliseconds>(nanoend);
	//cout << "�豸" << deviceno_ << "��ȡ����֡�ʣ�" << std::to_string(1000 / (milliend.count() - millinow.count())) << " fps" << "��ǰ֡������" << pSize_ << endl;
	cout << "�豸" << deviceno_ << "��ȡ����֡��ʱ��" << std::to_string(milliend.count() - millinow.count()) << " ms" << "��ǰ֡������" << pSize_ << endl;
	cout << "�豸" << deviceno_ << "����ͼ���ʱ��" << std::to_string(milliend0.count() - millinow.count()) << " ms" << endl;
	cout << "�豸" << deviceno_ << "ת�����ƺ�ʱ��" << std::to_string(milliend.count() - milliend0.count()) << " ms" << endl << endl;

	return true;
}

bool KinectCapture::UpdateFrame_img_in_depth_space()
{
	pcl::console::TicToc time;
	time.tic();
	if (!bInitialized_)
	{
		cout << "δ��ʼ��" << endl;
		return false;
	}
	if (!device_.get_capture(&capture_, std::chrono::milliseconds(0)))
	{
		cout << "δ����ͼ��" << endl;
		return false;
	}
	std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
	std::chrono::nanoseconds nanonow = now.time_since_epoch();
	std::chrono::milliseconds millinow = std::chrono::duration_cast<chrono::milliseconds>(nanonow);
	if (!tracker_.enqueue_capture(capture_))//��k4a������������ӵ�������������У����첽������������ٽ����
	{
		cout << "���岶��ʧ��" << endl;
		return false;
	}
	k4abt::frame body_frame = tracker_.pop_result();
	uint32_t num_bodies = body_frame.get_num_bodies();
	if (num_bodies == 0) {
		cout << "δʶ������" << endl;
		return false;
	}
	k4a::image colorImage = capture_.get_color_image();//��ȡ�豸��ɫͼ->��ɫ���
	std::chrono::nanoseconds get_colorImage_time = colorImage.get_system_timestamp();//��ɫͼʱ��� ���ʱ���
	timeStamp_ = get_colorImage_time.count() / 1000000; //ʱ���
	k4a::image depthImage = capture_.get_depth_image();//��ȡ�豸���ͼ->������

	k4a::image body_index_map = body_frame.get_body_index_map();//��������ͼ

	//Ԥ�����ɫͼתΪ�������ռ��Ļ�����
	k4a::image color_image_in_depth_space = NULL;
	color_image_in_depth_space = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
		depth_image_width_pixels,
		depth_image_height_pixels,
		depth_image_width_pixels * (int)sizeof(uint8_t));
	//����ɫͼתΪ�������ռ�
	color_image_in_depth_space = transformation.color_image_to_depth_camera(
		depthImage,
		colorImage
	);
	//ָ����������ͼbuffer��ָ��
	uint8_t* bodyIndxBuffer;
	bodyIndxBuffer = (uint8_t*)body_index_map.get_buffer();
	//��ɫͼ����ΪCV��ʽ
	cv::Mat cv_image_with_alpha(
		color_image_in_depth_space.get_height_pixels(),
		color_image_in_depth_space.get_width_pixels(),
		CV_8UC4,
		(void*)color_image_in_depth_space.get_buffer());
	cv::Mat cv_color_image;
	cv::cvtColor(cv_image_with_alpha, cv_color_image, cv::COLOR_BGRA2BGR);
	//���ͼ����ΪCV��ʽ
	cv::Mat cv_depth_image(
		depthImage.get_height_pixels(),
		depthImage.get_width_pixels(),
		CV_16U,
		(void*)depthImage.get_buffer(),
		static_cast<size_t>(depthImage.get_stride_bytes()));

	//����ÿ����������
	for (int i = 0; i < depth_image_width_pixels * depth_image_height_pixels; ++i) {
		//�����z�᲻Ϊ0��rgbaֵ����Ϊ0�������õ����ڸ�λ�õ�ֵ
		if (bodyIndxBuffer[i] == 255) {
			//��������������ȥ��
			int x = i % depth_image_width_pixels;
			int y = i / depth_image_width_pixels;
			cv_color_image.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
			cv_depth_image.at<ushort>(y, x) = 0;
		}
		else {
			//��������������
			continue;
		}
	}
	cv_color_image.copyTo(color_img);
	cv_depth_image.copyTo(depth_img);
	cv_color_image.release();
	cv_depth_image.release();
	cout << "��֡��ʱ��" << time.toc() << "ms" << endl;
	return true;
}

bool KinectCapture::UpdateFrame_img()
{
	pcl::console::TicToc time;
	time.tic();
	if (!bInitialized_)
	{
		cout << "δ��ʼ��" << endl;
		return false;
	}
	if (!device_.get_capture(&capture_, std::chrono::milliseconds(0)))
	{
		//cout << "����ʧ��" << endl;
		return false;
	}

	if (!tracker_.enqueue_capture(capture_))//��k4a������������ӵ�������������У����첽������������ٽ����
	{
		cout << "���岶��ʧ��" << endl;
		return false;
	}
	k4abt::frame body_frame = tracker_.pop_result();
	uint32_t num_bodies = body_frame.get_num_bodies();
	if (num_bodies == 0) {
		cout << "No bodies are detected!" << endl;
		return false;
	}
	k4a::image colorImage = capture_.get_color_image();//��ȡ�豸��ɫͼ->��ɫ���
	std::chrono::nanoseconds get_colorImage_time = colorImage.get_system_timestamp();//��ɫͼʱ��� ���ʱ���
	timeStamp_ = get_colorImage_time.count() / 1000000; //ʱ���
	k4a::image depthImage = capture_.get_depth_image();//��ȡ�豸���ͼ->������

	k4a::image body_index_map = body_frame.get_body_index_map();//��������ͼ

	//Ԥ�������ͼתΪ��ɫ����ռ��Ļ�����
	k4a::image depth_image_in_color_space = NULL;
	depth_image_in_color_space = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
		color_image_width_pixels,
		color_image_height_pixels,
		color_image_width_pixels * (int)sizeof(uint16_t));
	//Ԥ������������ͼ������
	k4a::image body_index_map_in_color_space = NULL;
	body_index_map_in_color_space = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM8,
		color_image_width_pixels,
		color_image_height_pixels,
		color_image_width_pixels * (int)sizeof(uint8_t));
	//���������ͼ�����ͼ���������ռ�תΪ��ɫ����ռ�k4a_transformation_depth_image_to_color_camera_custom
	transformation.depth_image_to_color_camera_custom(
		depthImage,
		body_index_map,
		&depth_image_in_color_space,
		&body_index_map_in_color_space,
		K4A_TRANSFORMATION_INTERPOLATION_TYPE_NEAREST,
		K4ABT_BODY_INDEX_MAP_BACKGROUND);
	//ָ����������ͼbuffer��ָ��
	uint8_t* bodyIndxBuffer;
	bodyIndxBuffer = (uint8_t*)body_index_map_in_color_space.get_buffer();
	//��ɫͼ����ΪCV��ʽ
	cv::Mat cv_image_with_alpha(
		colorImage.get_height_pixels(),
		colorImage.get_width_pixels(),
		CV_8UC4,
		(void*)colorImage.get_buffer());
	cv::Mat cv_color_image;
	cv::cvtColor(cv_image_with_alpha, cv_color_image, cv::COLOR_BGRA2BGR);
	//���ͼ����ΪCV��ʽ
	cv::Mat cv_depth_image(
		depth_image_in_color_space.get_height_pixels(),
		depth_image_in_color_space.get_width_pixels(),
		CV_16U,
		(void*)depth_image_in_color_space.get_buffer(),
		static_cast<size_t>(depth_image_in_color_space.get_stride_bytes()));

	//����ÿ����������
	for (int i = 0; i < color_image_width_pixels * color_image_height_pixels; ++i) {
		//�����z�᲻Ϊ0��rgbaֵ����Ϊ0�������õ����ڸ�λ�õ�ֵ
		if (bodyIndxBuffer[i] == 255) {
			//��������������ȥ��
			int x = i % color_image_width_pixels;
			int y = i / color_image_width_pixels;
			cv_color_image.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
			cv_depth_image.at<ushort>(y, x) = 0;
		}
		else {
			//��������������
			continue;
		}
	}
	cv_color_image.copyTo(color_img);
	cv_depth_image.copyTo(depth_img);
	cv_color_image.release();
	cv_depth_image.release();
	cout << "��֡��ʱ��" << time.toc() << "ms" << endl;
	return true;
}

bool KinectCapture::SaveFrame_png(string path)
{
	pcl::console::TicToc time;
	time.tic();
	if (!bInitialized_)
	{
		cout << "δ��ʼ��" << endl;
		return false;
	}
	if (!device_.get_capture(&capture_, std::chrono::milliseconds(0)))
	{
		//cout << "����ʧ��" << endl;
		return false;
	}

	if (!tracker_.enqueue_capture(capture_))//��k4a������������ӵ�������������У����첽������������ٽ����
	{
		cout << "���岶��ʧ��" << endl;
		return false;
	}
	k4abt::frame body_frame = tracker_.pop_result();
	uint32_t num_bodies = body_frame.get_num_bodies();
	if (num_bodies == 0) {
		cout << "No bodies are detected!" << endl;
		return false;
	}
	k4a::image colorImage = capture_.get_color_image();//��ȡ�豸��ɫͼ->��ɫ���
	std::chrono::nanoseconds get_colorImage_time = colorImage.get_system_timestamp();//��ɫͼʱ��� ���ʱ���
	timeStamp_ = get_colorImage_time.count() / 1000000; //ʱ���
	k4a::image depthImage = capture_.get_depth_image();//��ȡ�豸���ͼ->������

	k4a::image body_index_map = body_frame.get_body_index_map();//��������ͼ

	//Ԥ�������ͼתΪ��ɫ����ռ��Ļ�����
	k4a::image depth_image_in_color_space = NULL;
	depth_image_in_color_space = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
		color_image_width_pixels,
		color_image_height_pixels,
		color_image_width_pixels * (int)sizeof(uint16_t));
	//Ԥ������������ͼ������
	k4a::image body_index_map_in_color_space = NULL;
	body_index_map_in_color_space = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM8,
		color_image_width_pixels,
		color_image_height_pixels,
		color_image_width_pixels * (int)sizeof(uint8_t));
	//���������ͼ�����ͼ���������ռ�תΪ��ɫ����ռ�k4a_transformation_depth_image_to_color_camera_custom
	transformation.depth_image_to_color_camera_custom(
		depthImage,
		body_index_map,
		&depth_image_in_color_space,
		&body_index_map_in_color_space,
		K4A_TRANSFORMATION_INTERPOLATION_TYPE_NEAREST,
		K4ABT_BODY_INDEX_MAP_BACKGROUND);
	//ָ����������ͼbuffer��ָ��
	uint8_t* bodyIndxBuffer;
	bodyIndxBuffer = (uint8_t*)body_index_map_in_color_space.get_buffer();
	//��ɫͼ����ΪCV��ʽ
	cv::Mat cv_image_with_alpha(
		colorImage.get_height_pixels(), 
		colorImage.get_width_pixels(), 
		CV_8UC4, 
		(void*)colorImage.get_buffer());
	cv::Mat cv_color_image;
	cv::cvtColor(cv_image_with_alpha, cv_color_image, cv::COLOR_BGRA2BGR);
	//���ͼ����ΪCV��ʽ
	cv::Mat cv_depth_image(
		depth_image_in_color_space.get_height_pixels(),
		depth_image_in_color_space.get_width_pixels(),
		CV_16U,
		(void*)depth_image_in_color_space.get_buffer(),
		static_cast<size_t>(depth_image_in_color_space.get_stride_bytes()));

	//����ÿ����������
	for (int i = 0; i < color_image_width_pixels * color_image_height_pixels; ++i) {
		//�����z�᲻Ϊ0��rgbaֵ����Ϊ0�������õ����ڸ�λ�õ�ֵ
		if (bodyIndxBuffer[i] == 255) {
			//��������������ȥ��
			int x = i % color_image_width_pixels;
			int y = i / color_image_width_pixels;
			cv_color_image.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
			cv_depth_image.at<ushort>(y, x) = 0;
		}
		else {
			//��������������
			continue;
		}
	}
	string filename_depth = path + to_string(timeStamp_) + "_depth" + ".png";
	string filename_color = path + to_string(timeStamp_) + "_color" + ".png";
	cv::imwrite(filename_color, cv_color_image);
	cv::imwrite(filename_depth, cv_depth_image);
	cout << "��֡��ʱ��" << time.toc() << "ms" << endl;
	return true;
}

bool KinectCapture::SavePC_txt() {
	ofstream outFile;
	outFile.open("E:/PLY/" + to_string(timeStamp_) + ".txt");
	if (!outFile.is_open()) {
		std::cout << "fail to open file to write\n";
		return false;
	}

	for (int i = 0; i < pSize_; i++) {
		for (int j = 0; j < 3; j++)
		{
			outFile << (uint16_t)pColorRGB_[3 * i + j] << " ";
		}
		for (int r = 0; r < 3; r++)
		{
			outFile << (int16_t)pCloudXYZ_[3 * i + r] << " ";
		}
		outFile << "\n";
	}
	outFile << "\n";
	outFile.close();
	return true;
}

double KinectCapture::SaveFrame_png_full(string path)
{
	pcl::console::TicToc time;
	time.tic();
	if (!bInitialized_)
	{
		cout << "δ��ʼ��" << endl;
		return false;
	}
	if (!device_.get_capture(&capture_, std::chrono::milliseconds(0)))
	{
		//cout << "����ʧ��" << endl;
		return false;
	}

	k4a::image colorImage = capture_.get_color_image();//��ȡ�豸��ɫͼ->��ɫ���
	std::chrono::nanoseconds get_colorImage_time = colorImage.get_system_timestamp();//��ɫͼʱ��� ���ʱ���
	timeStamp_ = get_colorImage_time.count() / 1000000; //ʱ���
	k4a::image depthImage = capture_.get_depth_image();//��ȡ�豸���ͼ->������

	//Ԥ�������ͼתΪ��ɫ����ռ��Ļ�����
	k4a::image depth_image_in_color_space = NULL;
	depth_image_in_color_space = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
		color_image_width_pixels,
		color_image_height_pixels,
		color_image_width_pixels * (int)sizeof(uint16_t)); 
	//���������ͼ�����ͼ���������ռ�תΪ��ɫ����ռ�k4a_transformation_depth_image_to_color_camera
	depth_image_in_color_space = transformation.depth_image_to_color_camera(depthImage);
	double time_d = time.toc();
	cout << "��֡��ʱ��" << time_d << "ms" << endl;

	//��ɫͼ����ΪCV��ʽ
	cv::Mat cv_image_with_alpha(
		colorImage.get_height_pixels(),
		colorImage.get_width_pixels(),
		CV_8UC4,
		(void*)colorImage.get_buffer());
	cv::Mat cv_color_image;
	cv::cvtColor(cv_image_with_alpha, cv_color_image, cv::COLOR_BGRA2BGR);
	//���ͼ����ΪCV��ʽ
	cv::Mat cv_depth_image(
		depth_image_in_color_space.get_height_pixels(),
		depth_image_in_color_space.get_width_pixels(),
		CV_16U,
		(void*)depth_image_in_color_space.get_buffer(),
		static_cast<size_t>(depth_image_in_color_space.get_stride_bytes()));

	string filename_depth = path + to_string(timeStamp_) + "_depth" + ".png";
	string filename_color = path + to_string(timeStamp_) + "_color" + ".png";
	cv::imwrite(filename_color, cv_color_image);
	cv::imwrite(filename_depth, cv_depth_image);
	return time_d;
}