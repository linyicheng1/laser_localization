#include <iostream>
#include <fstream>
#include "laser_odometry.h"
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/ndt.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;
class Publisher : public rclcpp::Node
{
public:
    Publisher()
            : Node("test_laser")
    {
        this->declare_parameter<std::string>("kitti_path", "/media/lyc/软件/1-10/05/velodyne/");

        this->get_parameter("kitti_path", path);
        file_name = getKittiData(path);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points", rclcpp::SensorDataQoS());
        timer_ = this->create_wall_timer(
                200ms, std::bind(&Publisher::timer_callback, this));
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
            input.read((char*) &point.intensity, sizeof(float));
            points->push_back(point);
        }
        input.close();
        return points;
    }

    std::string path;
    std::vector<std::string> file_name;
private:
    void timer_callback()
    {
        static size_t publish_frame_cnt = cnt;
        if (publish_frame_cnt < file_name.size()){
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "base_link";
            t.child_frame_id = "laser";
            geometry_msgs::msg::Transform transform;
            Eigen::Quaternionf q1(Eigen::Matrix3f::Identity());
            transform.translation.x = 0;
            transform.translation.y = 0;
            transform.translation.z = 0;
            transform.rotation.x = q1.x();
            transform.rotation.y = q1.y();
            transform.rotation.z = q1.z();
            transform.rotation.w = q1.w();
            t.transform = transform;
            // Send the transformation
            tf_broadcaster_->sendTransform(t);

            auto points = readKittiBinData(file_name.at(publish_frame_cnt));
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*points, msg);
            msg.header.frame_id = "laser";
            publisher_->publish(msg);
            if (play)
                publish_frame_cnt ++;
//            std::cout<<" publish_frame_cnt "<<publish_frame_cnt<<std::endl;
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    int cnt = 0;
    bool play = true;
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





