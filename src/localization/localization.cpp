#include "localization/localization.h"
#include <math.h>
#include <iostream>
#include <utility>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/ndt.h>
#include "localization/tools.h"

namespace laser_localization
{
    localization::localization(rclcpp::Node *node):
    estimate_(filter(node)), node_(node)
    {
        Zero.rotation.w= 1;
        Zero.rotation.x= 0;
        Zero.rotation.y= 0;
        Zero.rotation.z= 0;
        Zero.translation.x = 0;
        Zero.translation.y = 0;
        Zero.translation.z = 0;
        pos_ = Zero;

        // init
        map_points_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        std::string global_map;
        node_->get_parameter("global_map", global_map);
        pcl::io::loadPCDFile<pcl::PointXYZI>(global_map, *map_points_);

        float global_resolution, global_view_resolution;
        node_->get_parameter("global_resolution", global_resolution);
        node_->get_parameter("global_view_resolution", global_view_resolution);

        map_points_ = downSample(map_points_, global_resolution);
        map_points_view_ = downSample(map_points_, global_view_resolution);

        ndt_ = create_registration();
        ndt_->setInputTarget(map_points_);

        aligned_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
        local_map_points_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    }

    void localization::update_local_map(const Eigen::Matrix4f& pose)
    {
        static bool init = false;
        double distance;
        node_->get_parameter("local_map_size", distance);
        Eigen::Vector2f delta = (pose.block<2, 1>(0, 3) - local_map_pose_.block<2, 1>(0, 3));
        if (delta.norm() < distance / 3 && init)
            return;
        local_map_pose_ = pose;
        local_map_points_->points.clear();
        local_map_points_->points.reserve(map_points_->points.size());
        for (const auto& pt:map_points_->points){
            if (std::abs(pt.x - pose(0, 3)) + std::abs(pt.y - pose(1, 3)) < distance){
                local_map_points_->points.emplace_back(pt);
            }
        }
        ndt_ = create_registration();
        ndt_->setInputTarget(map_points_);
        init = true;
    }

    // odom
    void localization::update_pos(const geometry_msgs::msg::Twist::SharedPtr& cmd, float dt)
    {
        static float yaw = 0;
        pos_.translation.x = (pos_.translation.x + cmd->linear.x * cos(yaw) * dt - cmd->linear.y * sin(yaw) * dt);
        pos_.translation.y = (pos_.translation.y + cmd->linear.x * sin(yaw) * dt + cmd->linear.y * cos(yaw) * dt);
        yaw += (float)cmd->angular.z * dt;
        Eigen::Quaternionf q(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
        pos_.rotation.x = q.x();
        pos_.rotation.y = q.y();
        pos_.rotation.z = q.z();
        pos_.rotation.w = q.w();
    }

    void localization::predict(const Eigen::Matrix4f& update)
    {
        estimate_.predict(update);
    }

    bool localization::correct(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int times)
    {
        std::cout<<"start correcting !!!"<<std::endl;
        //
        double resolution;
        node_->get_parameter("global_frame_resolution", resolution);
        cloud = downSample(cloud, resolution);
        if (cloud->points.size() < 400)
            return true;
        // 1. predict laser pos
        Eigen::Matrix4f predict_pos = estimate_.get_pos();
        // 2. laser match
        Eigen::Matrix4f laser_pos = ndt_match(std::move(cloud), predict_pos, times);
        // 3. base_link pos
        Eigen::Matrix4f base_pos = laser_pos;
        Eigen::Quaternionf q(base_pos.block<3,3>(0,0));
        // 4. update filter, get estimate pos
        if (ndt_->getFitnessScore() < 5){
            estimate_.update(base_pos.block<3, 1>(0, 3), q);
            // 5. calculate map -->odom pos
            auto filter_pos = estimate_.get_pos();
            Eigen::Matrix4f map_odom = (filter_pos * inv_odom_base_);
            Eigen::Quaternionf q1(map_odom.block<3,3>(0,0));
            pos_.translation.x = map_odom(0, 3);
            pos_.translation.y = map_odom(1, 3);
            pos_.translation.z = map_odom(2, 3);
            pos_.rotation.x = q1.x();
            pos_.rotation.y = q1.y();
            pos_.rotation.z = q1.z();
            pos_.rotation.w = q1.w();
            std::cout<<"end correcting !!!"<<std::endl;
            return true;
        }else {
            std::cout<<" match error "<<ndt_->getFitnessScore()<<std::endl;
            return false;
        }
    }

    /**
     * @brief
     * @param laser_base
     */
    void localization::init_localization(const Eigen::Matrix4f& laser_base)
    {
        Eigen::Matrix3f R = laser_base.block<3, 3>(0, 0);
        Eigen::Quaternionf q(R);
        estimate_.set_pos(laser_base.block<3, 1>(0 ,3), q);

        auto filter_pos = estimate_.get_pos();

        Eigen::Matrix4f map_odom = (filter_pos * inv_odom_base_);
        Eigen::Quaternionf q1(map_odom.block<3,3>(0,0));
        pos_.translation.x = map_odom(0, 3);
        pos_.translation.y = map_odom(1, 3);
        pos_.translation.z = map_odom(2, 3);
        pos_.rotation.x = q1.x();
        pos_.rotation.y = q1.y();
        pos_.rotation.z = q1.z();
        pos_.rotation.w = q1.w();
    }


    Eigen::Matrix4f localization::ndt_match(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, Eigen::Matrix4f gauss, int times)
    {
        ndt_->setMaximumIterations(times);
        ndt_->setInputSource(cloud);
        ndt_->align(*aligned_, gauss);
        if (ndt_->getFitnessScore() > 3){
            std::cout<<" aligned score "<<ndt_->getFitnessScore()<<" re matching "<<std::endl;
            ndt_->setMaximumIterations(times*2);
            ndt_->align(*aligned_, gauss);
        }
//        pcl::visualization::PCLVisualizer viewer("pcd viewer");
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> c1(aligned_, 0, 255, 0);
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> c2(map_points_, 255, 0, 0);
//        viewer.addPointCloud(map_points_, c2, "map");
//        viewer.addPointCloud(aligned_, c1, "aligned");
//        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "aligned");
//        viewer.spin();
        return ndt_->getFinalTransformation();
    }

    pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr localization::create_registration()
    {
        float resolution, step_size, epsilon;
        node_->get_parameter("localization/ndt_resolution", resolution);
        node_->get_parameter("localization/ndt_step_size", step_size);
        node_->get_parameter("localization/ndt_epsilon", epsilon);

        pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt(new pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
        ndt->setResolution(resolution);
        ndt->setStepSize(step_size);
        ndt->setTransformationEpsilon(epsilon);
        return ndt;
    }
}

