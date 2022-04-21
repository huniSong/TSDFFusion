//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/point_types.h>
//#include <pcl/common/impl/transforms.hpp> 
//#include <pcl/console/time.h>
//#include <pcl/io/obj_io.h>
//#include <boost/thread/thread.hpp>
//#include <Eigen/Dense>
//
//#include <string>
//#include <iostream>
//
//int main() {
//    std::string objPath = "E:/ShadowCreatorHoloRecon/bin/TestApi.obj"; //当前目录下的obj文件
//    //读取
//    pcl::TextureMesh mesh;
//    pcl::io::loadOBJFile(objPath, mesh);
//
//    //显示
//    boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("3D viewer A"));
//
//    viewer->addTextureMesh(mesh, "mesh");
//
//    while (!viewer->wasStopped())  // 在按下 "q" 键之前一直会显示窗口
//    {
//        viewer->spinOnce();
//    }
//	return 0;
//}
