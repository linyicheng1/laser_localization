#ifndef LASER_LOCALIZATION_VOXELSET_H
#define LASER_LOCALIZATION_VOXELSET_H
#include <unordered_set>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace match_error{
    class Vector3iHash {
    public:
        size_t operator()(const Eigen::Vector3i& x) const;
    };

    class VoxelSet {
    public:
        VoxelSet(double max_correspondence_distance);

        void set_cloud(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud);
        double matching_error(const pcl::PointCloud<pcl::PointXYZI>& cloud, double* inlier_fraction) const;

    private:
        Eigen::Vector3i voxel_coord(const Eigen::Vector4f& x) const;
        Eigen::Vector4f voxel_center(const Eigen::Vector3i& coord) const;

    private:
        double resolution;

        using Voxels = std::unordered_set<Eigen::Vector3i, Vector3iHash, std::equal_to<Eigen::Vector3i>, Eigen::aligned_allocator<Eigen::Vector3i>>;
        Voxels voxels;
    };
}

#endif //LASER_LOCALIZATION_VOXELSET_H
