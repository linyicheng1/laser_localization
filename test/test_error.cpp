#include "error_estimater/error_estimater.h"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
class Publisher : public rclcpp::Node
{
public:
    Publisher(): Node("test_laser"), error_(new match_error::MatchingCostEvaluaterFlann())
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>);
        if (pcl::io::loadPCDFile<pcl::PointXYZI>("../map/kitti.pcd", *map) == -1)
        {
            PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        }
        map = downSample(map, 0.3);
        std::string path = "/media/lyc/软件/1-10/05/velodyne/";
        std::vector<std::string> file_name = getKittiData(path);

        for (int i = 200;i < 800;i += 40){

            auto frame =  readKittiBinData(file_name.at(i));
            frame = downSample(frame, 0.3);

            Eigen::Matrix4f T = Eigen::Matrix4f::Identity();

            error_->set_target(map, 0.3);
            auto s = std::chrono::system_clock::now();
            double error = error_->calc_matching_error(*frame, T, nullptr);
            auto e = std::chrono::system_clock::now();
            std::cout<<"cost : "<<std::chrono::duration_cast<std::chrono::milliseconds>(e-s).count()<<" ms"<<std::endl;
            std::cout<<" error: "<<error<<std::endl;
        }
    }

private:
    match_error::MatchingCostEvaluaterFlann * error_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr downSample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double resolution)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::ApproximateVoxelGrid<pcl::PointXYZI> voxelGrid;
        voxelGrid.setLeafSize(resolution, resolution, resolution);
        voxelGrid.setInputCloud(cloud);
        voxelGrid.filter(*filtered);
        return filtered;
    }
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

