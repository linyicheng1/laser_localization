#ifndef LOCALIZATION_OCCUPANCY_GRIDMAP_H
#define LOCALIZATION_OCCUPANCY_GRIDMAP_H

#include <vector>
#include <Eigen/Core>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/opencv.hpp>

namespace global_localization {

    class OccupancyGridMap {
    public:
        using Ptr = std::shared_ptr<OccupancyGridMap>;

        OccupancyGridMap(double resolution, const cv::Mat& values) {
            this->resolution = resolution;
            this->values = values;
        }

        OccupancyGridMap(double resolution, int width, int height) {
            this->resolution = resolution;
            values = cv::Mat1f(height, width, 0.0f);
        }

        double grid_resolution() const { return resolution; }
        int width() const { return values.cols; }
        int height() const { return values.rows; }
        const float* data() const { return reinterpret_cast<const float*>(values.data); }

        void insert_points(const std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>& points, int min_points_per_grid) {
            for (const auto& pt : points) {
                auto loc = grid_loc(pt);
                if (in_map(loc)) {
                    value(loc) += 1;
                }
            }

            values /= min_points_per_grid;
            for (auto& x : values) {
                x = std::min(x, 1.0f);
            }
        }

        double calc_score(const std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>& points) const {
            double sum_dists = 0.0;
            for (const auto& pt : points) {
                auto loc = grid_loc(pt);
                loc.x() = std::max<int>(0, std::min<int>(values.cols - 1, loc.x()));
                loc.y() = std::max<int>(0, std::min<int>(values.rows - 1, loc.y()));
                sum_dists += value(loc);
            }

            return sum_dists;
        }

        OccupancyGridMap::Ptr pyramid_up() const {
            cv::Mat1f small_map(values.rows / 2, values.cols / 2);
            for (int i = 0; i < values.rows / 2; i++) {
                for (int j = 0; j < values.cols / 2; j++) {
                    float x = values.at<float>(i * 2, j * 2);
                    x = std::max(x, values.at<float>(i * 2 + 1, j * 2));
                    x = std::max(x, values.at<float>(i * 2, j * 2 + 1));
                    x = std::max(x, values.at<float>(i * 2 + 1, j * 2 + 1));
                    small_map.at<float>(i, j) = x;
                }
            }

            return std::make_shared<OccupancyGridMap>(resolution * 2.0, small_map);
        }

        nav_msgs::msg::OccupancyGrid::ConstSharedPtr to_rosmsg() {
            nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg(new nav_msgs::msg::OccupancyGrid);
            // TODO
        }

    private:
        bool in_map(const Eigen::Vector2i& pix) const {
            bool left_bound = (pix.array() >= Eigen::Array2i::Zero()).all();
            bool right_bound = (pix.array() < Eigen::Array2i(values.cols, values.rows)).all();
            return left_bound && right_bound;
        }

        float value(const Eigen::Vector2i& loc) const { return values.at<float>(loc.y(), loc.x()); }
        float& value(const Eigen::Vector2i& loc) { return values.at<float>(loc.y(), loc.x()); }

        Eigen::Vector2i grid_loc(const Eigen::Vector2f& pt) const {
            Eigen::Vector2i loc = (pt / resolution).array().floor().cast<int>();
            Eigen::Vector2i offset = Eigen::Vector2i(values.cols / 2, values.rows / 2);
            return loc + offset;
        }

    private:
        double resolution;
        cv::Mat1f values;
    };

}  // namespace global_localization

#endif //LOCALIZATION_OCCUPANCY_GRIDMAP_H
