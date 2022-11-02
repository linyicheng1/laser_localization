#ifndef LOCALIZATION_GLOBAL_LOCALIZATION_H
#define LOCALIZATION_GLOBAL_LOCALIZATION_H

#include "bbs_localization.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "rclcpp/rclcpp.hpp"
#include <pcl/registration/ndt.h>

struct GlobalLocalizationResult {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<GlobalLocalizationResult>;

    GlobalLocalizationResult(double error, double inlier_fraction, const Eigen::Isometry3f& pose) : error(error), inlier_fraction(inlier_fraction), pose(pose) {}

    double error;
    double inlier_fraction;
    Eigen::Isometry3f pose;
};

struct GlobalLocalizationResults {
    GlobalLocalizationResults(const std::vector<GlobalLocalizationResult::Ptr>& results) : results(results) {}

    GlobalLocalizationResults& sort(int max_num_candidates) {
        auto remove_loc = std::remove_if(results.begin(), results.end(), [](const auto& result) { return result == nullptr; });
        results.erase(remove_loc, results.end());

        std::cout << "valid solutions:" << results.size() << std::endl;

        // std::sort(results.begin(), results.end(), [](const auto& lhs, const auto& rhs) { return lhs->error < rhs->error; });
        std::sort(results.begin(), results.end(), [](const auto& lhs, const auto& rhs) { return lhs->inlier_fraction > rhs->inlier_fraction; });
        if (results.size() > max_num_candidates) {
            results.erase(results.begin() + max_num_candidates, results.end());
        }
        return *this;
    }
    std::vector<GlobalLocalizationResult::Ptr> results;
};

namespace global_localization {
    class BBSLocalization;

    class GlobalLocalizationBBS {
    public:
        GlobalLocalizationBBS(rclcpp::Node *node);
        ~GlobalLocalizationBBS() ;
        // add api

        void set_global_map(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud);

        bool globalLocalization(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, const std::string& predict_txt, Eigen::Matrix4f& result);
        bool globalLocalization(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, const Eigen::Matrix4f& predict_pose, Eigen::Matrix4f& result);
        bool globalLocalization(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, Eigen::Matrix4f& result);

        void set_match_map(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud);

        GlobalLocalizationResults query(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud);

    private:
        using Points2D = std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>;
        Points2D slice(const pcl::PointCloud<pcl::PointXYZI>& cloud, double min_z, double max_z) const;

        pcl::PointCloud<pcl::PointXYZI>::Ptr unslice(const Points2D& points);
        // raw map
        pcl::PointCloud<pcl::PointXYZI>::ConstPtr map_;
        pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
    protected:
        std::unique_ptr<BBSLocalization> bbs;
        rclcpp::Node *node_;

    };
}  // namespace global_localization


#endif //LOCALIZATION_GLOBAL_LOCALIZATION_H
