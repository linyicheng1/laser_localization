#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "pcl/io/pcd_io.h"
#include <pcl/registration/registration.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
            : Node("minimal_subscriber")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    "/points",10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        // 10Hz
        timer_ = this->create_wall_timer(
                100ms,std::bind(&MinimalSubscriber::timer_callback,this));
        // 100Hz
        timer2_ = this->create_wall_timer(
                10ms,std::bind(&MinimalSubscriber::timer2_callback,this));
    }

private:
    void timer_callback()
    {
        sensor_msgs::msg::PointCloud2 data;
        pcl::PointCloud<pcl::PointXYZI> points;
        pcl::io::loadPCDFile<pcl::PointXYZI>(
                "/home/lyc/Inspection_3d_ws/src/inspection_3d_robot/localization/map/frame.pcd",
                points);
        pcl::toROSMsg(points, data);
        data.header.stamp = this->get_clock()->now();
        data.header.frame_id = "os_sensor";
        publisher_->publish(data);
    }

    void timer2_callback()
    {
        geometry_msgs::msg::Point pos;
        // test
        static float i = 0;
        i += 0.0001;
        pos.x = sin(i) * 3;
        pos.y = (cos(i)-1) * 3;
        pos.z = M_PI*0.15;//sin(i) * 3;
//        pos.x = 1.2;//sin(i) * 3;
//        pos.y = 0;//(cos(i)-1) * 3;
//        pos.z = M_PI*0.15;//sin(i) * 3;
        publish_pos(pos);
    }

    void publish_pos(geometry_msgs::msg::Point pos)
    {
        {
            rclcpp::Time now = this->get_clock()->now();
            geometry_msgs::msg::TransformStamped t;

            t.header.stamp = now;
            t.header.frame_id = "odom";
            t.child_frame_id = "base_link";
            t.transform.translation.x = pos.x;
            t.transform.translation.y = pos.y;
            t.transform.translation.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, pos.z);
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();
            // Send the transformation
            tf_broadcaster_->sendTransform(t);
        }
        {
            rclcpp::Time now = this->get_clock()->now();
            geometry_msgs::msg::TransformStamped t;

            t.header.stamp = now;
            t.header.frame_id = "base_link";
            t.child_frame_id = "os_sensor";
            t.transform.translation.x = 0.5;
            t.transform.translation.y = 0;
            t.transform.translation.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, 0);
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();
            // Send the transformation
            tf_broadcaster_->sendTransform(t);
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer2_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}


