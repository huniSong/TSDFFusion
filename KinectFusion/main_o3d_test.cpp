#include <iostream>
#include <string>
#include <fstream>

#include "Open3D/Open3D.h"
#include "Open3D/Integration/ScalableTSDFVolume.h"
#include "Open3D/Geometry/Image.h"
#include "Open3D/Geometry/RGBDImage.h"
#include "Open3D/IO/ClassIO/ImageIO.h"
#include "Open3D/IO/ClassIO/PointCloudIO.h"
#include "Open3D/IO/Sensor/AzureKinect/AzureKinectSensor.h"
#include "Open3D/Camera/PinholeCameraIntrinsic.h"
#include "Open3D/Visualization/Visualizer/Visualizer.h"
#include "Open3D/Utility/Timer.h"

#include "ServerSocket.h"

using namespace open3d;
using namespace std;
using namespace Eigen;

ServerSocket* server;

Matrix4d loadtxt2Matrix(string filename) {
    ifstream ifile(filename);
    Matrix4d matrix = Matrix4d::Random(4, 4);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            ifile >> matrix(i, j);
        }
    }
    ifile.close();
    return matrix.inverse();
}

int main() {
    
    using namespace integration;
    using namespace geometry;
    using namespace camera;
    using namespace utility;

    //auto config_filename = "E:/PLY/config.json";
    //io::AzureKinectSensorConfig sensor_config;
    //if (!io::ReadIJsonConvertibleFromJSON(config_filename, sensor_config))
    //    LogInfo("Invalid sensor config");
    //io::AzureKinectSensor sensor(sensor_config);
    //if (!sensor.Connect(0))
    //    utility::LogWarning("Failed to connect to sensor, abort.");

    //// Start viewing
    //bool flag_exit = false;
    //bool is_geometry_added = false;
    //visualization::VisualizerWithKeyCallback vis;
    //vis.RegisterKeyCallback(GLFW_KEY_ESCAPE,
    //    [&](visualization::Visualizer* vis) {
    //        flag_exit = true;
    //        return false;
    //    });

    //vis.CreateVisualizerWindow("Open3D Azure Kinect Recorder", 1920, 540);
    //do {
    //    auto im_rgbd = sensor.CaptureFrame(true);
    //    if (im_rgbd == nullptr) {
    //        utility::LogInfo("Invalid capture, skipping this frame");
    //        continue;
    //    }

    //    if (!is_geometry_added) {
    //        vis.AddGeometry(im_rgbd);
    //        is_geometry_added = true;
    //    }

    //    // Update visualizer
    //    vis.UpdateGeometry();
    //    vis.PollEvents();
    //    vis.UpdateRender();

    //} while (!flag_exit);

    server = new ServerSocket();
    server->createSocket();
    server->bindSocket("10.128.255.165", 8000);
    server->listenSocket();
    server->acceptSocket();

    PinholeCameraIntrinsic* Intrinsic0 = new PinholeCameraIntrinsic(1280, 720, 606.79, 606.68, 636.65234, 368.05954);
    PinholeCameraIntrinsic* Intrinsic1 = new PinholeCameraIntrinsic(1280, 720, 609.18, 608.90, 638.95776, 367.28314);
    PinholeCameraIntrinsic* Intrinsic2 = new PinholeCameraIntrinsic(1280, 720, 608.05, 607.86, 640.42676, 367.47488);

    auto volume = ScalableTSDFVolume(0.015, 0.04, TSDFVolumeColorType::RGB8);
    // ����TSDF���voxel_length(���ر߳�����sdf_trunc���ضϾ��룩��colorType(��ɫ��Ҫ��

    Matrix4d extrinsic0 = loadtxt2Matrix("E:/PLY/2022.2.21/master_pose.txt");
    Matrix4d extrinsic1 = loadtxt2Matrix("E:/PLY/2022.2.21/sub1_pose.txt");
    Matrix4d extrinsic2 = loadtxt2Matrix("E:/PLY/2022.2.21/sub2_pose.txt");

    Timer timer_total;
    timer_total.Start();
    double timeStamp = timer_total.GetSystemTimeInMilliseconds();

    for (int i = 0; i < 20; ) {
        Timer timer = Timer();
        timer.Start();

        Image image_color0, image_depth0;
        Image image_color1, image_depth1;
        Image image_color2, image_depth2;

        string filename_color0 = "E:/PLY/2022.2.21/master_" + to_string(i) + "_color.png";
        string filename_depth0 = "E:/PLY/2022.2.21/master_" + to_string(i) + "_depth.png";
        string filename_color1 = "E:/PLY/2022.2.21/sub1_" + to_string(i) + "_color.png";
        string filename_depth1 = "E:/PLY/2022.2.21/sub1_" + to_string(i) + "_depth.png";
        string filename_color2 = "E:/PLY/2022.2.21/sub2_" + to_string(i) + "_color.png";
        string filename_depth2 = "E:/PLY/2022.2.21/sub2_" + to_string(i) + "_depth.png";
        
        io::ReadImageFromPNG(filename_color0, image_color0);
        io::ReadImageFromPNG(filename_depth0, image_depth0);
        io::ReadImageFromPNG(filename_color1, image_color1);
        io::ReadImageFromPNG(filename_depth1, image_depth1);
        io::ReadImageFromPNG(filename_color2, image_color2);
        io::ReadImageFromPNG(filename_depth2, image_depth2);

        shared_ptr<RGBDImage> image_rgbd0 = RGBDImage::CreateFromColorAndDepth(image_color0, image_depth0, 1000, 3.0, false);
        shared_ptr<RGBDImage> image_rgbd1 = RGBDImage::CreateFromColorAndDepth(image_color1, image_depth1, 1000, 3.0, false);
        shared_ptr<RGBDImage> image_rgbd2 = RGBDImage::CreateFromColorAndDepth(image_color2, image_depth2, 1000, 3.0, false);
        
        volume.Integrate(*image_rgbd0, *Intrinsic0, extrinsic0);
        volume.Integrate(*image_rgbd1, *Intrinsic1, extrinsic1);
        volume.Integrate(*image_rgbd2, *Intrinsic2, extrinsic2);
        timer.Stop();
        cout << "integrate frame " << i << " in " << timer.GetDuration() << "ms" << endl;

        auto mesh = volume.ExtractTriangleMesh();//���ǻ�����ģ��
        mesh->ComputeVertexNormals();//���㷨����
        int t = mesh->triangles_.size() - 1;
        //visualization::DrawGeometries({ mesh }, "Mesh");//��ʾMesh

        if (server->bConnected)
        {  //����пͻ������ӣ�����Mesh֡
            Timer time0;
            time0.Start();
            int size = mesh->vertices_.size() * 9 + mesh->triangles_.size() * 6;
            //����Mesh��������ÿ������9B��ÿ��������6B
            char ready = '0';   //���տ�ʼλ
            cout << "�ȴ�֡����" << endl;
            server->SendBytes(&ready, 1);
            cout << "��ʼ����Mesh֡  " << "������" << mesh->vertices_.size() << endl;
            cout << "������" << mesh->triangles_.size() << endl;
            cout << "size: " << size << endl;
            char headbuffer[12];
            char temp[4];
            int pos = 0;
            int32_t * p = (int32_t *)temp;
            p[0] = (int32_t)size;  //��֡������
            headbuffer[pos++] = temp[0];
            headbuffer[pos++] = temp[1];
            headbuffer[pos++] = temp[2];
            headbuffer[pos++] = temp[3];
            p[0] = (int32_t)mesh->vertices_.size();  //����
            headbuffer[pos++] = temp[0];
            headbuffer[pos++] = temp[1];
            headbuffer[pos++] = temp[2];
            headbuffer[pos++] = temp[3];
            p[0] = (int32_t)mesh->triangles_.size();  //����
            headbuffer[pos++] = temp[0];
            headbuffer[pos++] = temp[1];
            headbuffer[pos++] = temp[2];
            headbuffer[pos++] = temp[3];

            //��Mesh����֡д��buffer
            vector<char> buffer(size);
            pos = 0;
            for (int i = 0; i < mesh->vertices_.size(); i++) { //����д��ÿ�����RGBXYZֵ
                buffer[pos++] = (char)(mesh->vertex_colors_[i][0] * 255);
                buffer[pos++] = (char)(mesh->vertex_colors_[i][1] * 255);
                buffer[pos++] = (char)(mesh->vertex_colors_[i][2] * 255);
                char temp[2];
                int16_t* p = (int16_t*)temp;
                p[0] = (int16_t)(mesh->vertices_[i][0] * 1000);
                buffer[pos++] = temp[0];
                buffer[pos++] = temp[1];
                p[0] = (int16_t)(mesh->vertices_[i][1] * 1000);
                buffer[pos++] = temp[0];
                buffer[pos++] = temp[1];
                p[0] = (int16_t)(mesh->vertices_[i][2] * 1000);
                buffer[pos++] = temp[0];
                buffer[pos++] = temp[1];
            }
            for (int i = 0; i < mesh->triangles_.size(); i++) { //����д��ÿ����
                char temp[2];
                uint16_t* p = (uint16_t*)temp;
                p[0] = (uint16_t)mesh->triangles_[i][0];
                buffer[pos++] = temp[0];
                buffer[pos++] = temp[1];
                p[0] = (uint16_t)mesh->triangles_[i][1];
                buffer[pos++] = temp[0];
                buffer[pos++] = temp[1];
                p[0] = (uint16_t)mesh->triangles_[i][2];
                buffer[pos++] = temp[0];
                buffer[pos++] = temp[1];
            }
            server->SendBytes(headbuffer, sizeof(headbuffer));//���͵�������������ֹ���ղ�ȫ
            server->SendBytes(buffer.data(), size);//���͵�������֡��rgb+xyz
            time0.Stop();
            cout << "���ͺ�ʱ��" << time0.GetDuration() << " ms" << endl;
            cout << "��֡������� " << endl;
            cout << "��֡�ܺ�ʱ�� " << timer_total.GetSystemTimeInMilliseconds() - timeStamp << "ms" << endl;
            cout << "--------------------------" << endl << endl << endl;
            timeStamp = timer_total.GetSystemTimeInMilliseconds();
        }

        volume.Reset();
        i++;
        i = i % 20;
    }
    return 0;
}