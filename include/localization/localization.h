#ifndef LOCALIZATION_LOCALIZATION_H
#define LOCALIZATION_LOCALIZATION_H
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "localization/UKF.h"
#include "error_estimater/error_estimater.h"
#include <pcl/registration/registration.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include "rclcpp/rclcpp.hpp"

namespace laser_localization
{
    class localization
    {
    public:
        localization(rclcpp::Node *node);
        void update_local_map(const Eigen::Matrix4f& pose);
        void update_pos(const geometry_msgs::msg::Twist::SharedPtr& cmd, float dt);
        void predict(const Eigen::Matrix4f& update);
        bool correct(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int times = 10);
        void init_localization(const Eigen::Matrix4f& laser_base);

        void set_inv_odom_base(Eigen::Matrix4f p){inv_odom_base_ = p;}
        geometry_msgs::msg::Transform get_pos(){return pos_;}
        Eigen::Matrix4f get_estimate_pos(){return estimate_.get_pos();}
        pcl::PointCloud<pcl::PointXYZI>::Ptr get_map(){return map_points_;}
        pcl::PointCloud<pcl::PointXYZI>::Ptr get_view_map(){return map_points_view_;}
        pcl::PointCloud<pcl::PointXYZI>::Ptr get_aligned(){return aligned_;}
    private:
        Eigen::Matrix4f ndt_match(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, Eigen::Matrix4f gauss, int times);
        pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr create_registration() ;
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_points_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_points_;
        Eigen::Matrix4f local_map_pose_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_points_view_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_;
        geometry_msgs::msg::Transform pos_;
        geometry_msgs::msg::Transform Zero;
        filter estimate_;
        pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_;
        Eigen::Matrix4f inv_odom_base_;
        rclcpp::Node *node_;

        Eigen::Vector3f last_map_pos;
        pcl::PointCloud<pcl::PointXYZI>::Ptr local_map;
    };
}// laser_localization

#endif //LOCALIZATION_LOCALIZATION_H
