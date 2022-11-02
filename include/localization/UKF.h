#ifndef LOCALIZATION_UKF_H
#define LOCALIZATION_UKF_H
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "rclcpp/rclcpp.hpp"
namespace laser_localization
{
    class filter
    {
    public:
        filter(rclcpp::Node *node);
        Eigen::Vector3f predict(const Eigen::Matrix4f& update);
        Eigen::Vector3f update(const Eigen::Vector3f& p, const Eigen::Quaternionf& a);
        void set_pos(const Eigen::Vector3f& p, const Eigen::Quaternionf& a);
        Eigen::Matrix4f get_pos();
    private:
        float k1_;
        bool first_ = true;

        Eigen::Vector3f pos_;
        Eigen::Quaternionf angular_;
        rclcpp::Node *node_;
    };

    class UKF
    {
    public:
        UKF();

    };
}
#endif //LOCALIZATION_UKF_H
