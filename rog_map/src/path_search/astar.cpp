#include <path_search/astar.h>

namespace path_search {
    RET_CODE
    Astar::setup(const Vec3f &start_pt, const Vec3f &goal_pt, const int &flag, const double &searching_horizon) {
        md_.start_pt = start_pt;
        md_.goal_pt = goal_pt;
        md_.mission_rcv_WT = ros::Time::now().toSec();
        md_.searching_horizon = searching_horizon;
        md_.use_inf_map = flag & ON_INF_MAP;
        md_.use_prob_map = flag & ON_PROB_MAP;
        md_.unknown_as_occ = flag & UNKNOWN_AS_OCCUPIED;
        md_.unknown_as_free = flag & UNKNOWN_AS_FREE;
        if (md_.use_inf_map && md_.use_prob_map) {
            cout << RED << " -- [A*] " << RET_CODE_STR[INIT_ERROR] << " : cannot use both inf map and prob map."
                 << RESET << endl;
            return INIT_ERROR;
        }
        if (md_.unknown_as_occ && md_.unknown_as_free) {
            cout << RED << " -- [A*] " << RET_CODE_STR[INIT_ERROR]
                 << ": cannot use both unknown_as_occupied and unknown_as_free." << RESET << endl;
            return INIT_ERROR;
        }
        if (md_.use_prob_map) {
            md_.resolution = map_ptr_->getCfg().resolution;
        } else {
            md_.resolution = map_ptr_->getCfg().inflation_resolution;
        }
        if (searching_horizon > 0) {
            md_.local_map_center_d = start_pt;
        } else {
            md_.local_map_center_d = (start_pt + goal_pt) / 2;
        }

        posToGlobalIndex(md_.local_map_center_d, md_.local_map_center_id_g);
        md_.local_map_min_d = md_.local_map_center_d - md_.resolution * cfg_.map_size_i.cast<double>();
        md_.local_map_max_d = md_.local_map_center_d + md_.resolution * cfg_.map_size_i.cast<double>();;
        if (cfg_.visual_process) {

            VisualUtils::VisualizeBoundingBox(mkr_arr_pub_, md_.local_map_min_d, md_.local_map_max_d, "local_map",
                                              Color::Chartreuse());
        }

        return type_utils::SUCCESS;
    }

    double Astar::getHeu(GridNodePtr node1, GridNodePtr node2, int type) const {
        switch (type) {
            case DIAG: {
                double dx = std::abs(node1->id_g(0) - node2->id_g(0));
                double dy = std::abs(node1->id_g(1) - node2->id_g(1));
                double dz = std::abs(node1->id_g(2) - node2->id_g(2));

                double h = 0.0;
                int diag = std::min(std::min(dx, dy), dz);
                dx -= diag;
                dy -= diag;
                dz -= diag;

                if (dx == 0) {
                    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dy, dz) + 1.0 * std::abs(dy - dz);
                }
                if (dy == 0) {
                    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dx, dz) + 1.0 * std::abs(dx - dz);
                }
                if (dz == 0) {
                    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dx, dy) + 1.0 * std::abs(dx - dy);
                }
                return tie_breaker_ * h;
            }
            case MANH: {
                double dx = std::abs(node1->id_g(0) - node2->id_g(0));
                double dy = std::abs(node1->id_g(1) - node2->id_g(1));
                double dz = std::abs(node1->id_g(2) - node2->id_g(2));

                return tie_breaker_ * (dx + dy + dz);
            }
            case EUCL: {
                return tie_breaker_ * (node2->id_g - node1->id_g).norm();
            }
            default: {
                ROS_ERROR(" -- [A*] Wrong hue type");
                return 0;
            }
        }
    }


}