#include "error_estimater/error_estimater.h"
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>


namespace match_error
{

    void MatchingCostEvaluaterFlann::set_target(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                                                double max_correspondence_distance)
    {
        map = cloud;
        max_correspondence_distance_sq = max_correspondence_distance * max_correspondence_distance;
        tree.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
        tree->setInputCloud(cloud);

//        max_correspondence_distance_sq = max_correspondence_distance * max_correspondence_distance;
//
//        voxels.reset(new VoxelSet(max_correspondence_distance));
//        voxels->set_cloud(cloud);
    }

    double MatchingCostEvaluaterFlann::calc_matching_error(const pcl::PointCloud<pcl::PointXYZI> &cloud,
                                                           const Eigen::Matrix4f &transformation,
                                                           double *inlier_fraction)
    {
        int num_inliers = 0;
        double matching_error = 0.0;

        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(cloud, *transformed, transformation.inverse());

//        pcl::visualization::PCLVisualizer viewer("pcd viewer");
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> c1(transformed, 0, 255, 0);
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> c2(map, 255, 0, 0);
//        viewer.addPointCloud(map, c2, "map");
//        viewer.addPointCloud(transformed, c1, "aligned");
//        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "aligned");
//        viewer.spin();

        std::vector<int> indices;
        std::vector<float> sq_dists;
        for (int i = 0; i < transformed->size(); i++)
        {
            tree->nearestKSearch(transformed->at(i), 1, indices, sq_dists);
            if (sq_dists[0] < max_correspondence_distance_sq)
            {
                num_inliers++;
                matching_error += sq_dists[0];
            }
        }

        if (inlier_fraction)
        {
            *inlier_fraction = static_cast<double>(num_inliers) / cloud.size();
        }
        return num_inliers ? matching_error / num_inliers : std::numeric_limits<double>::max();
//        int num_inliers = 0;
//        double matching_error = 0.0;
//
//        pcl::PointCloud<pcl::PointXYZI> transformed;
//        pcl::transformPointCloud(cloud, transformed, transformation);
//
//        return voxels->matching_error(transformed, inlier_fraction);
    }
}