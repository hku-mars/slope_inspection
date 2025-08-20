#pragma once

#include <Eigen/Eigen>
#include <vector>
#include <iostream>
#include "memory"

#define ORIGIN_AT_CENTER 1
#define ORIGIN_AT_CORNER 2

#define RAYCAST_MODE ORIGIN_AT_CENTER
namespace geometry_utils {
    namespace raycaster {
        template<typename T>
        int signum(T val) {
            return (T(0) < val) - (val < T(0));
        }

        class RayCaster {
        public:
            typedef std::shared_ptr<RayCaster> Ptr;

            RayCaster() = default;

            void setResolution(const double &resolution);

            RayCaster(const double &resolution);

            ~RayCaster() = default;

            void posToIndex(const double d, int &id) const;

            void indexToPos(const int &id, double &d) const;


            bool setInput(const Eigen::Vector3d &start, const Eigen::Vector3d &end);

            bool step(Eigen::Vector3d &ray_pt);

        private:
            double resolution_{-1};
            bool first_point{true};
            // progress variables
            double start_x_d_, start_y_d_, start_z_d_;
            double end_x_d_, end_y_d_, end_z_d_;
            double t_to_bound_x_, t_to_bound_y_, t_to_bound_z_;
            int expand_dir_x_, expand_dir_y_, expand_dir_z_;
            int end_x_i_, end_y_i_, end_z_i_;
            int start_x_i_, start_y_i_, start_z_i_;
            int cur_ray_pt_id_x_, cur_ray_pt_id_y_, cur_ray_pt_id_z_;
            double t_when_step_x_, t_when_step_y_, t_when_step_z_;
            int step_num_{0};
        };
    }
}


