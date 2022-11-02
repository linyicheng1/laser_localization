#ifndef LOCALIZATION_BBS_LOCALIZATION_H
#define LOCALIZATION_BBS_LOCALIZATION_H

#include <memory>
#include <queue>
#include <vector>
#include <boost/optional.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace global_localization {

    class OccupancyGridMap;

    struct DiscreteTransformation {
    public:
        using Points = std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>;

        DiscreteTransformation();
        DiscreteTransformation(int level, int x, int y, int theta);
        ~DiscreteTransformation();

        bool operator<(const DiscreteTransformation& rhs) const;

        bool is_leaf() const;

        Eigen::Isometry2f transformation(double theta_resolution, const std::vector<std::shared_ptr<OccupancyGridMap>>& gridmap_pyramid);

        Points transform(const Points& points, double trans_resolution, double theta_resolution);

        double calc_score(const Points& points, double theta_resolution, const std::vector<std::shared_ptr<OccupancyGridMap>>& gridmap_pyramid);

        std::vector<DiscreteTransformation> branch();

    public:
        double score;
        int level;
        int x;
        int y;
        int theta;
    };

    struct BBSParams {
        BBSParams() {
            max_range = 15.0;
            min_tx = min_ty = -50.0;
            max_tx = max_ty = 50.0;
            min_theta = -M_PI;
            max_theta = M_PI;
        }

        double max_range;
        double min_tx;
        double max_tx;
        double min_ty;
        double max_ty;
        double min_theta;
        double max_theta;
    };

    class BBSLocalization {
    public:
        using Points = std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>;

        BBSLocalization(const BBSParams& params = BBSParams());
        ~BBSLocalization();

        void set_map(const Points& map_points, double resolution, int width, int height, int pyramid_levels, int max_points_per_cell);

        boost::optional<Eigen::Isometry2f> localize(const Points& scan_points, double min_score, double* best_score = nullptr);

        std::shared_ptr<const OccupancyGridMap> gridmap() const;

    private:
        std::priority_queue<DiscreteTransformation> create_init_transset(const Points& scan_points) const;

    private:
        BBSParams params;

        double theta_resolution;
        std::vector<std::shared_ptr<OccupancyGridMap>> gridmap_pyramid;
    };

}  // namespace global_localization

#endif //LOCALIZATION_BBS_LOCALIZATION_H
