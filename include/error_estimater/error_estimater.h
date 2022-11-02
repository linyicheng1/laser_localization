#ifndef LOCALIZATION_ERROR_ESTIMATER_H
#define LOCALIZATION_ERROR_ESTIMATER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include "error_estimater/voxelset.h"

namespace match_error
{
    class MatchingCostEvaluaterFlann
    {
    public:
        MatchingCostEvaluaterFlann() {}
        ~MatchingCostEvaluaterFlann(){}

        void set_target(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, double max_correspondence_distance);

        double calc_matching_error(const pcl::PointCloud<pcl::PointXYZI>& cloud, const Eigen::Matrix4f& transformation, double* inlier_fraction);

    private:
        double max_correspondence_distance_sq;
        pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr tree;
        pcl::PointCloud<pcl::PointXYZI>::ConstPtr map;

        std::unique_ptr<VoxelSet> voxels;
    };
}

#endif //LOCALIZATION_ERROR_ESTIMATER_H
