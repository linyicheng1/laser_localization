#ifndef LASER_LOCALIZATION_TOOLS_H
#define LASER_LOCALIZATION_TOOLS_H
#include <functional>
#include <memory>
#include <boost/filesystem/operations.hpp>
#include "fstream"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include <pcl/filters/approximate_voxel_grid.h>

bool read_position(const std::string& file, Eigen::Matrix4f& base);

void write_position(const std::string& file,const Eigen::Matrix4f& position);

pcl::PointCloud<pcl::PointXYZI>::Ptr downSample(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, double resolution);

#endif //LASER_LOCALIZATION_TOOLS_H
