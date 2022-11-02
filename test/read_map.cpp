#include <iostream> //标准输入输出流
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
#include <pcl/io/ply_io.h> //PCL的ply格式文件的输入输出头文件
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件
#include <pcl/visualization/cloud_viewer.h>
#include "unistd.h"

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>); // 创建点云（指针）

    if (pcl::io::loadPCDFile<pcl::PointXYZI>("../map/map-old.pcd", *cloud) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n"); //文件不存在时，返回错误，终止程序。
        return (-1);
    }
    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from test_file.pcd with the following fields: "
              << std::endl;
//    for (auto & point : cloud->points) //显示所有的点
//        std::cout << "    " << point.x
//                  << " " << point.y
//                  << " " << point.z << std::endl;
    pcl::visualization::CloudViewer viewer("pcd viewer");
    viewer.showCloud(cloud);
    sleep(-1);
    return (0);
}
