#include "laser_odometry.h"
#include <pcl/filters/approximate_voxel_grid.h>
#include <iostream>
#include <chrono>
#include <tf2_ros/transform_broadcaster.h>

laserOdometry::laserOdometry(rclcpp::Node *node):
align(new pcl::PointCloud<pcl::PointXYZI>()),
local_map_cloud(new pcl::PointCloud<pcl::PointXYZI>()),
node_(node)
{
    float ndt_resolution, ndt_step_size, ndt_epsilon, ndt_max_iterations;
    node_->get_parameter("ndt/resolution", ndt_resolution);
    node_->get_parameter("ndt/step_size", ndt_step_size);
    node_->get_parameter("ndt/epsilon", ndt_epsilon);
    node_->get_parameter("ndt/max_iterations", ndt_max_iterations);

    ndt.setResolution(ndt_resolution);
    ndt.setStepSize(ndt_step_size);
    ndt.setTransformationEpsilon(ndt_epsilon);
    ndt.setMaximumIterations(ndt_max_iterations);
}


Eigen::Matrix4f laserOdometry::addFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr& point, const Eigen::Matrix4f& predict, bool &accept)
{
    auto s = std::chrono::steady_clock::now();
    float frame_resolution = 1;
    node_->get_parameter("ndt/frame_resolution", frame_resolution);

    auto filter = downSample(point, frame_resolution);
    if (filter->points.size() < 400){
        odometry = odometry * predict;
        accept = true;
        return odometry;
    }
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*filter, *filter, indices);
    if (local_map.empty())
    {// first frame
        odometry = Eigen::Matrix4f::Identity();
        nextGuess = Eigen::Matrix4f::Identity();
        updateLocalMap(filter, Eigen::Matrix4f::Identity());
        return  odometry;
    }
    ndt.setInputSource(filter);

    Eigen::Matrix4f odom_predict = odometry * predict;
//    double dt = (nextGuess.block<3, 1>(0, 3) - odom_predict.block<3, 1>(0, 3)).norm();
//
//    auto angle0 = nextGuess.block<3, 3>(0, 0).eulerAngles(2, 1, 0);
//    auto angle1 = odom_predict.block<3, 3>(0, 0).eulerAngles(2, 1, 0);
//    double da = std::abs(angle0.z() - angle1.z());
//    if (da > M_PI)
//        da = da - 2*M_PI;
//    else if (da < -M_PI)
//        da = da += 2*M_PI;
//    std::cout<<" t0: "<<dt<<" a0: "<<angle0<<" t1: "<<1<<" a1: "<<angle1<<std::endl;
    auto e1 = std::chrono::steady_clock::now();
//    if (dt < 5){
        ndt.align(*align, odom_predict);
//    }else{
//        ndt.align(*align, nextGuess);
//        accept = false;
//    }
    std::cout<<"fitness score: "<<ndt.getFitnessScore()<<std::endl;
    accept = true;
    if (ndt.getFitnessScore() > 3){
        nextGuess.block<3,3>(0,0) = odom_predict.block<3,3>(0,0);
        ndt.align(*align, nextGuess);
        accept = false;
    }
    Eigen::Matrix4f delta = odometry.inverse() * ndt.getFinalTransformation();
    nextGuess = odometry * delta;
    odometry = ndt.getFinalTransformation();

    auto e2 = std::chrono::steady_clock::now();


    Eigen::Matrix4f distance = local_map_pose.back().inverse() * odometry;
    float kf_distance;
    node_->get_parameter("odom/kf_distance", kf_distance);

    if (fabs(distance(0, 3)) + fabs(distance(1, 3)) > kf_distance)
    {// new key frame
        updateLocalMap(filter, odometry);
        std::cout<<" add new key frame !";
    }
    auto e3 = std::chrono::steady_clock::now();

    //std::cout<<" total cost: "<<std::chrono::duration_cast<std::chrono::milliseconds>(e3 - s).count()<<" ms align cost: "<<std::chrono::duration_cast<std::chrono::milliseconds>(e2 - e1).count()<<" ms ";

//    std::cout<<" x: "<<odometry(0,3)<<" y: "<<odometry(1,3)<<" z: "<<odometry(2,3)<<" ";
//    std::cout<<"delta x: "<<delta(0,3)<<" y: "<<delta(1,3)<<" z: "<<delta(2,3)<<" ";
//    std::cout<<"next guess x: "<<nextGuess(0,3)<<" y: "<<nextGuess(1,3)<<" z: "<<nextGuess(2,3)<<" ";
//    std::cout<<std::endl;
    return odometry;
}

Eigen::Matrix4f laserOdometry::addFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr &point)
{
    auto s = std::chrono::steady_clock::now();
    float frame_resolution = 1;
    node_->get_parameter("ndt/frame_resolution", frame_resolution);

    auto filter = downSample(point, frame_resolution);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*filter, *filter, indices);
    if (local_map.empty())
    {// first frame
        odometry = Eigen::Matrix4f::Identity();
        nextGuess = Eigen::Matrix4f::Identity();
        updateLocalMap(filter, Eigen::Matrix4f::Identity());
        return  odometry;
    }
    ndt.setInputSource(filter);
    auto e1 = std::chrono::steady_clock::now();
    ndt.align(*align, nextGuess);
    auto e2 = std::chrono::steady_clock::now();


    Eigen::Matrix4f delta = odometry.inverse() * ndt.getFinalTransformation();
    odometry = ndt.getFinalTransformation();
    nextGuess = odometry * delta;

    Eigen::Matrix4f distance = local_map_pose.back().inverse() * odometry;
    float kf_distance;
    node_->get_parameter("odom/kf_distance", kf_distance);

    if (fabs(distance(0, 3)) + fabs(distance(1, 3)) > kf_distance)
    {// new key frame
        updateLocalMap(filter, odometry);
        std::cout<<" add new key frame !";
    }
    auto e3 = std::chrono::steady_clock::now();

    //std::cout<<" total cost: "<<std::chrono::duration_cast<std::chrono::milliseconds>(e3 - s).count()<<" ms align cost: "<<std::chrono::duration_cast<std::chrono::milliseconds>(e2 - e1).count()<<" ms ";

//    std::cout<<" x: "<<odometry(0,3)<<" y: "<<odometry(1,3)<<" z: "<<odometry(2,3)<<" ";
//    std::cout<<"delta x: "<<delta(0,3)<<" y: "<<delta(1,3)<<" z: "<<delta(2,3)<<" ";
//    std::cout<<"next guess x: "<<nextGuess(0,3)<<" y: "<<nextGuess(1,3)<<" z: "<<nextGuess(2,3)<<" ";
//    std::cout<<std::endl;
    return odometry;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr laserOdometry::downSample(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, float resolution)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::ApproximateVoxelGrid<pcl::PointXYZI> voxelGrid;
    voxelGrid.setLeafSize(resolution, resolution, resolution);
    voxelGrid.setInputCloud(cloud);
    voxelGrid.filter(*filtered);
    return filtered;
}

void laserOdometry::updateLocalMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& kf, const Eigen::Matrix4f& pose)
{
    size_t local_map_size;
    node_->get_parameter("odom/local_map_size", local_map_size);
    local_map.push_back(kf);
    local_map_pose.push_back(pose);
    while (local_map.size() > local_map_size)
    {
        local_map.pop_front();
        local_map_pose.pop_front();
    }
    local_map_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI> transformed;

    for (size_t i = 0;i < local_map.size(); ++i)
    {
        pcl::transformPointCloud(*local_map.at(i), transformed, local_map_pose.at(i));
        *local_map_cloud += transformed;
        //std::cout<<" add key frame "<<i<<std::endl;
    }
    if (local_map.size() < local_map_size / 2)
    {
        ndt.setInputTarget(local_map_cloud);
    }
    else
    {
        float local_map_resolution;
        node_->get_parameter("ndt/local_map_resolution", local_map_resolution);
        auto filter = downSample(local_map_cloud, local_map_resolution);
        ndt.setInputTarget(local_map_cloud);
    }
}
