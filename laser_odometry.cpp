#include "laser_odometry.h"
#include <pcl/filters/approximate_voxel_grid.h>
#include <iostream>
#include <chrono>

laserOdometry::laserOdometry():align(new pcl::PointCloud<pcl::PointXYZI>()),local_map_cloud(new pcl::PointCloud<pcl::PointXYZI>())
{
    ndt.setResolution(1.0);
    ndt.setStepSize(0.1);
    ndt.setTransformationEpsilon(0.01);
    ndt.setMaximumIterations(30);
}

Eigen::Matrix4f laserOdometry::addFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr &point)
{
    auto s = std::chrono::steady_clock::now();
    auto filter = downSample(point, 1);
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

    Eigen::Matrix4f distance = local_map_pose.front().inverse() * odometry;
    if (fabs(distance(0, 3)) + fabs(distance(1, 3)) > 2)
    {// new key frame
        updateLocalMap(filter, odometry);
        std::cout<<" add new key frame !";
    }
    auto e3 = std::chrono::steady_clock::now();

    std::cout<<" total cost: "<<std::chrono::duration_cast<std::chrono::milliseconds>(e3 - s).count()<<" ms align cost: "<<std::chrono::duration_cast<std::chrono::milliseconds>(e2 - e1).count()<<" ms ";

//    std::cout<<" x: "<<odometry(0,3)<<" y: "<<odometry(1,3)<<" z: "<<odometry(2,3)<<" ";
//    std::cout<<"delta x: "<<delta(0,3)<<" y: "<<delta(1,3)<<" z: "<<delta(2,3)<<" ";
//    std::cout<<"next guess x: "<<nextGuess(0,3)<<" y: "<<nextGuess(1,3)<<" z: "<<nextGuess(2,3)<<" ";
    std::cout<<std::endl;
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
    local_map.push_back(kf);
    local_map_pose.push_back(pose);
    while (local_map.size() > 20)
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
    if (local_map.size() < 10)
    {
        ndt.setInputTarget(local_map_cloud);
    }
    else
    {
        auto filter = downSample(local_map_cloud, 1);
        ndt.setInputTarget(local_map_cloud);
    }
}
