#include <iostream>
#include <fstream>
#include "laser_odometry.h"
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>

using namespace std::chrono_literals;
class Publisher : public rclcpp::Node
{
public:
    Publisher()
            : Node("test_laser")
    {
        this->declare_parameter<double>("ndt/resolution", 1.0);
        this->declare_parameter<double>("ndt/step_size", 0.1);
        this->declare_parameter<double>("ndt/epsilon", 0.01);
        this->declare_parameter<double>("ndt/max_iterations", 30.0);
        this->declare_parameter<double>("ndt/frame_resolution", 2.0);
        this->declare_parameter<double>("odom/kf_distance", 3.0);
        this->declare_parameter<int>("odom/local_map_size", 10);
        this->declare_parameter<double>("ndt/local_map_resolution", 2.0);

        odometry = new laserOdometry(this);
        timer_ = this->create_wall_timer(
                500ms, std::bind(&Publisher::timer_callback, this));
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr getLocalMap(){return odometry->getLocalMap();}
    std::vector<std::string> getKittiData(const std::string &path)
    {
        std::vector<std::string> names;
        int num = getFileNum(path);

        for (int i = 0;i < num;i ++)
        {
            std::stringstream ss;
            ss << setw(6) << setfill('0') << i;
            names.emplace_back(path + ss.str() + ".bin");
        }
        return names;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr readKittiBinData(std::string &in_file)
    {
        std::fstream input(in_file.c_str(), ios::in | ios::binary);
        if (!input.good())
        {
            std::cerr<<"Could not read file: "<< in_file << std::endl;
            return nullptr;
        }
        input.seekg(0, ios::beg);
        pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>);
        int i;
        for (i=0; input.good() && !input.eof(); i++)
        {
            pcl::PointXYZI point;
            input.read((char*) &point.x, 3*sizeof(float));
            input.read((char*) &point.intensity, sizeof(float));
            points->push_back(point);
        }
        input.close();
        return points;
    }
    laserOdometry *odometry;
private:
    void timer_callback()
    {
    }
    rclcpp::TimerBase::SharedPtr timer_;
    int getFileNum(const std::string &path)
    {   //需要用到<dirent.h>头文件
        int fileNum = 0;
        DIR *pDir;
        struct dirent *ptr;
        if (!(pDir = opendir(path.c_str())))
            return fileNum;
        while ((ptr = readdir(pDir)) != nullptr)
        {
            if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
                fileNum++;
        }
        closedir(pDir);
        return fileNum;
    }

};

boost::mutex updateModelMutex;
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//    viewer->setCameraPosition(0, 0, 0, 0, 0, 0, 0, 0, -1);
    viewer->setBackgroundColor(1.0,1.0,1.0);
/*    viewer->addPointCloud<pcl::PointXYZI> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->initCameraParameters ();*/
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> view_h(cloud, 0, 255, 0);
    viewer->addPointCloud(cloud, view_h, "show");
    viewer->addPointCloud(cloud, view_h, "map");
    return (viewer);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Publisher>();

    std::string path = "/media/lyc/软件/1-10/05/velodyne/";
    std::vector<std::string> file_name = node->getKittiData(path);
    pcl::PointCloud<pcl::PointXYZI>::Ptr show(new pcl::PointCloud<pcl::PointXYZI>);
    Eigen::Matrix4f last_pose = Eigen::Matrix4f::Identity();
    auto viewer = simpleVis(show);

    for (auto & file : file_name)
    {
        if (!rclcpp::ok())
            break;
        auto points = node->readKittiBinData(file);
        // process
        Eigen::Matrix4f pose = node->odometry->addFrame(points);

        // visualization
        viewer->addLine(pcl::PointXYZ(last_pose(0,3), last_pose(1,3), last_pose(2,3)), pcl::PointXYZ(pose(0,3), pose(1,3), pose(2,3)), file);
        last_pose = pose;
        pcl::transformPointCloud(*points, *show, pose);

        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> view_h(show, "intensity");
        viewer->updatePointCloud(show, view_h, "show");

        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> view_h2(node->getLocalMap(), "intensity");
        viewer->updatePointCloud(node->getLocalMap(), view_h2, "map");
        viewer->spinOnce();
//        boost::this_thread::sleep (boost::posix_time::milliseconds (50));
        rclcpp::spin_some(node);
    }
    viewer->spin();
    rclcpp::shutdown();
    return 0;
}
