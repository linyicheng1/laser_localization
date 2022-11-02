#include "localization/UKF.h"

#include <utility>

namespace laser_localization
{
    filter::filter(rclcpp::Node *node):
    node_(node)
    {
        node_->get_parameter("filter/k1", k1_);
        pos_.x() = 0;
        pos_.y() = 0;
        pos_.z() = 0;
    }

    Eigen::Vector3f filter::predict(const Eigen::Matrix4f& update)
    {
        Eigen::Matrix4f p = Eigen::Matrix4f::Identity();
        p.block<3,1>(0,3) = pos_;
        p.block<3,3>(0,0) = angular_.toRotationMatrix();
        p = p * update;
        pos_ = p.block<3,1>(0,3);
        angular_ = Eigen::Quaternionf(p.block<3,3>(0,0));
    }

    Eigen::Vector3f filter::update(const Eigen::Vector3f& p, const Eigen::Quaternionf& a)
    {
        pos_ = pos_*k1_ + p * (1 - k1_);
        angular_ = angular_.coeffs() * k1_  + a.coeffs() * (1 - k1_);
        angular_ = angular_.normalized();
    }

    void filter::set_pos(const Eigen::Vector3f& p, const Eigen::Quaternionf& a)
    {
        pos_ = p;
        angular_ = a;
    }

    Eigen::Matrix4f filter::get_pos()
    {
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T.block<3, 1>(0, 3) = pos_;
        T.block<3, 3>(0, 0) = angular_.toRotationMatrix();
        return T;
    }
}