#ifndef LASER_ODOMETRY_LASER_ODOMETRY_H
#define LASER_ODOMETRY_LASER_ODOMETRY_H

#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Core>

class laserOdometry
{
public:
    laserOdometry();
    Eigen::Matrix4f addFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr& point);
    pcl::PointCloud<pcl::PointXYZI>::Ptr downSample(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, float resolution);
    void updateLocalMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& keyFrame, const Eigen::Matrix4f& pose);
    pcl::PointCloud<pcl::PointXYZI>::Ptr getLocalMap(){return local_map_cloud;}
private:

    std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> local_map;
    std::deque<Eigen::Matrix4f> local_map_pose;
    pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_cloud;
    Eigen::Matrix4f odometry;
    Eigen::Matrix4f nextGuess;
//    pcl::PointCloud<pcl::PointXYZI>::Ptr lastKf;
    pcl::PointCloud<pcl::PointXYZI>::Ptr align;
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
};

#endif //LASER_ODOMETRY_LASER_ODOMETRY_H
