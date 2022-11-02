#include <iostream>
#include <fstream>
#include "global_localization.h"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include "localization/tools.h"

using namespace std::chrono_literals;
class Publisher : public rclcpp::Node
{
public:
    Publisher(): Node("test_laser"), global_localization_(new global_localization::GlobalLocalizationBBS(this))
    {
        this->declare_parameter<double>("bbs/max_range", 15.0);
        this->declare_parameter<double>("bbs/min_tx", -50.0);
        this->declare_parameter<double>("bbs/max_tx", 50.0);
        this->declare_parameter<double>("bbs/min_ty", -50.0);
        this->declare_parameter<double>("bbs/max_ty", 50.0);
        this->declare_parameter<double>("bbs/min_theta", -3.14);
        this->declare_parameter<double>("bbs/max_theta", 3.14);
        this->declare_parameter<double>("bbs/map_min_z", 0.4);
        this->declare_parameter<double>("bbs/map_max_z", 1.2);
        this->declare_parameter<int>("bbs/map_width", 512);
        this->declare_parameter<int>("bbs/map_height", 512);
        this->declare_parameter<double>("bbs/map_resolution", 0.5);
        this->declare_parameter<int>("bbs/max_points_pre_cell", 5);
        this->declare_parameter<int>("bbs/map_pyramid_level", 6);
        this->declare_parameter<double>("bbs/scan_min_z", 0.4);
        this->declare_parameter<double>("bbs/scan_max_z", 1.2);
        this->declare_parameter<double>("bbs/map_filter_resolution", 1);
        this->declare_parameter<double>("bbs/scan_filter_resolution", 1);
        this->declare_parameter<double>("global_map_width", 400.0);
        this->declare_parameter<double>("global_map_height", 400.0);

        pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>);
        if (pcl::io::loadPCDFile<pcl::PointXYZI>("../map/kitti.pcd", *map) == -1)
        {
            PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        }
        map = downSample(map, 1);
        global_localization_->set_global_map(map);

        std::string path = "/media/lyc/软件/1-10/05/velodyne/";
        std::vector<std::string> file_name = getKittiData(path);
        pcl::visualization::PCLVisualizer viewer_v("demo");
        viewer_v.setBackgroundColor(1.0, 1.0, 1.0);
        map = downSample(map, 10);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> view_h(map, 0, 255, 0);
        viewer_v.addPointCloud(map, view_h, "map");
        Eigen::Matrix4f predict = Eigen::Matrix4f::Identity();

        for (int i = 100;i < file_name.size() / 2; i += 10)
        {
            auto frame =  readKittiBinData(file_name.at(i));
            Eigen::Matrix4f T;

            global_localization_->globalLocalization(frame, predict, T);
            predict = T;
            std::cout<<"T "<<T<<std::endl;
            pcl::transformPointCloud(*frame, *frame, T);
            if (i == 100){
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> view_f(frame, 0, 0, 255);
                viewer_v.addPointCloud(frame, view_f, "frame");
            }else {
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> view_f(frame, 0, 0, 255);
                viewer_v.updatePointCloud(frame, view_f, "frame");
            }

            viewer_v.spin();
        }
    }

private:
    global_localization::GlobalLocalizationBBS* global_localization_;


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
            float intensity = 0;
            input.read((char*) &intensity, sizeof(float));
            points->push_back(point);
        }
        input.close();
        return points;
    }
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

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Publisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

