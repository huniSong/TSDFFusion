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
//    std::string objPath = "E:/ShadowCreatorHoloRecon/bin/TestApi.obj"; //��ǰĿ¼�µ�obj�ļ�
//    //��ȡ
//    pcl::TextureMesh mesh;
//    pcl::io::loadOBJFile(objPath, mesh);
//
//    //��ʾ
//    boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("3D viewer A"));
//
//    viewer->addTextureMesh(mesh, "mesh");
//
//    while (!viewer->wasStopped())  // �ڰ��� "q" ��֮ǰһֱ����ʾ����
//    {
//        viewer->spinOnce();
//    }
//	return 0;
//}
