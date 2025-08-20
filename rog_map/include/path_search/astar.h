#pragma once

#include "ros/ros.h"
#include "Eigen/Dense"
#include "vector"
#include "rog_map/rog_map.h"
#include "queue"
#include "path_search/config.hpp"
#include "type_utils/common_type_name.h"
#include "visualization_utils/visualization_utils.h"


namespace path_search {

    using namespace rog_map;
    constexpr double inf = 1 >> 20;
    struct GridNode;
    typedef GridNode *GridNodePtr;

    struct GridNode {
        enum enum_state {
            OPENSET = 1,
            CLOSEDSET = 2,
            UNDEFINED = 3
        } state{UNDEFINED};

        int rounds{0};
        Eigen::Vector3i id_g;
        double total_score{inf}, distance_score{inf};
        double distance_to_goal{inf};
        GridNodePtr father_ptr{nullptr};
    };

    class NodeComparator {
    public:
        bool operator()(GridNodePtr node1, GridNodePtr node2) {
            return node1->total_score > node2->total_score;
        }
    };

    class FrontierComparator {
    public:
        bool operator()(GridNodePtr node1, GridNodePtr node2) {
            return node1->distance_to_goal > node2->distance_to_goal;
        }
    };

    const int ON_INF_MAP = (1 << 0);
    const int ON_PROB_MAP = (1 << 1);
    const int UNKNOWN_AS_OCCUPIED = (1 << 3);
    const int UNKNOWN_AS_FREE = (1 << 4);

    class Astar {
    private:
        ros::NodeHandle nh_;
        ros::Publisher mkr_arr_pub_;
        rog_map::ROGMap::Ptr map_ptr_;
        PathSearchConfig cfg_;
        const double tie_breaker_ = 1.0 + 1e-5;
        vec_Vec3i sorted_pts;
        vector<GridNodePtr> grid_node_buffer_;

        int rounds_{0};

        static const int DIAG = 0;
        static const int MANH = 1;
        static const int EUCL = 2;

        struct MissionData {
            Vec3f start_pt;
            Vec3f goal_pt;
            double searching_horizon;
            bool use_inf_map;
            bool use_prob_map;
            bool unknown_as_occ;
            bool unknown_as_free;
            double resolution;
            Vec3i local_map_center_id_g;
            Vec3f local_map_center_d;
            double mission_rcv_WT{0};
            Vec3f local_map_max_d, local_map_min_d;

            std::mutex mission_mtx;
        } md_;


        double getHeu(GridNodePtr node1, GridNodePtr node2, int type = DIAG) const;

        inline void posToGlobalIndex(const Vec3f &pos, Vec3i &id) const {
            // add resolution/2 for rounding
            id = (1.0 / md_.resolution * pos + pos.cwiseSign() * 0.5)
                    .cast<int>();
        }

        inline void globalIndexToPos(const Vec3i &id_g, Vec3f &pos) const {
            pos = id_g.cast<double>() * md_.resolution;
        }

        inline int getLocalIndexHash(const Vec3i &id_in) const {
            Vec3i id = id_in - md_.local_map_center_id_g + cfg_.map_size_i;
            return id(0) * cfg_.map_voxel_num(1) * cfg_.map_voxel_num(2) +
                   id(1) * cfg_.map_voxel_num(2) +
                   id(2);
        }

        bool insideLocalMap(const Vec3f &pos) const {
            Vec3i id_g;
            posToGlobalIndex(pos, id_g);
            return insideLocalMap(id_g);
        }

        bool insideLocalMap(const Vec3i &id_g) const {
            Vec3i delta = id_g - md_.local_map_center_id_g;
            if (fabs(delta.x()) > cfg_.map_size_i.x() ||
                fabs(delta.y()) > cfg_.map_size_i.y() ||
                fabs(delta.z()) > cfg_.map_size_i.z()) {
                return false;
            }
            return true;
        }


    public:

        Astar() = default;

        ~Astar() {};

        typedef shared_ptr<Astar> Ptr;

        RET_CODE setup(const Vec3f &start_pt, const Vec3f &goal_pt, const int &flag,
                       const double &searching_horizon = 9999);


        void initMap(const ros::NodeHandle &nh, ROGMap::Ptr rm) {// TODO
            nh_ = nh;
            cfg_ = PathSearchConfig(nh);
            cout << GREEN << " -- [RM] Init Astar-map." << RESET << endl;
            map_ptr_ = rm;
            mkr_arr_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/astar_debug/mkr_arr", 1000);
            int map_buffer_size = cfg_.map_voxel_num(0) * cfg_.map_voxel_num(1) * cfg_.map_voxel_num(2);
            grid_node_buffer_.resize(map_buffer_size);
            for (size_t i = 0; i < grid_node_buffer_.size(); i++) {
                grid_node_buffer_[i] = new GridNode;
                grid_node_buffer_[i]->rounds = 0;
            }
            cout << BLUE << "\tmap index size: " << cfg_.map_size_i.transpose() << RESET << endl;
            cout << BLUE << "\tmap vox_num: " << cfg_.map_voxel_num.transpose() << RESET << endl;
            int test_num = 10;
            for (int i = -test_num; i <= test_num; i++) {
                for (int j = -test_num; j <= test_num; j++) {
                    for (int k = -test_num; k <= test_num; k++) {
                        Vec3i delta(i, j, k);
                        sorted_pts.push_back(delta);
                    }
                }
            }
            sort(sorted_pts.begin(), sorted_pts.end(),
                 [](const Vec3i &pt1, const Vec3i &pt2) {
                     double dist1 = pt1.x() * pt1.x() + pt1.y() * pt1.y() + pt1.z() * pt1.z();
                     double dist2 = pt2.x() * pt2.x() + pt2.y() * pt2.y() + pt2.z() * pt2.z();
                     return dist1 < dist2;
                 });
        }

        RET_CODE MultiWaypointPathSearch(const Vec3f &start_pt,
                                         const vec_E<Vec3f> &goal_pts,
                                         const int flag,
                                         vector<vec_Vec3f> &out_segs,
                                         const double searching_horizon = -1) {
            if (goal_pts.empty()) {
                cout << RED << " -- [A*] " << RET_CODE_STR[INIT_ERROR] << " goal waypoints empty." << RESET << endl;
                return INIT_ERROR;
            }
            Vec3f temp_start_pt = start_pt;
            Vec3f temp_goal_pt = goal_pts.front();
            out_segs.clear();
            double temp_search_horizon = searching_horizon;
            for (int i = 0; i < goal_pts.size(); i++) {
                temp_goal_pt = goal_pts[i];
                vec_Vec3f temp_path;
                RET_CODE ret = PointToPointPathSearch(temp_start_pt, temp_goal_pt, flag, temp_search_horizon,
                                                      temp_path);
                if (ret == REACH_GOAL) {
                    if (temp_path.size() < 2) {
//                        return FAILED;
                        temp_path = {temp_start_pt, temp_goal_pt};
                    }
                    out_segs.push_back(temp_path);
                    temp_search_horizon -= geometry_utils::computePathLength(temp_path);
                    temp_start_pt = temp_goal_pt;
                }
                if (ret == REACH_HORIZON || temp_search_horizon < 0.1) {
                    if (temp_path.size() < 2) {
                        temp_path = {temp_start_pt, temp_goal_pt};
//                        return FAILED;
                    }
                    out_segs.push_back(temp_path);
                    return REACH_HORIZON;
                }
                if (ret != REACH_HORIZON && ret != REACH_GOAL) {
                    return ret;
                }
            }
            return REACH_GOAL;
        }

        RET_CODE PointToPointPathSearch(const Vec3f &start_pt, const Vec3f &end_pt,
                                        const int &flag,
                                        const double &searching_horizon,
                                        vec_Vec3f &out_path,
                                        const double &time_out = 0.1) {
            RET_CODE setup_ret = setup(start_pt, end_pt, flag, searching_horizon);
            if (setup_ret != type_utils::SUCCESS) {
                return setup_ret;
            }
            out_path.clear();
            ros::Time time_1 = ros::Time::now();
            ++rounds_;
            GridType start_pt_type, end_pt_type;

            /// 2) Switch both start and end point to local map

            Vec3f hit_pt;
            Vec3f local_start_pt, local_end_pt;
            bool start_pt_out_local_map = false;
            bool end_pt_out_local_map = false;

            local_start_pt = start_pt;
            local_end_pt = end_pt;

            if (!insideLocalMap(start_pt)) {
                if (geometry_utils::lineIntersectBox(start_pt, md_.local_map_center_d, md_.local_map_min_d,
                                                     md_.local_map_max_d, hit_pt)) {
//                print(fg(color::orange_red),
//                      " -- [A*] WARN, end point not inside local map, find a waypoint to the map edge\n");
                    Vec3f dir = (hit_pt - start_pt).normalized();
                    double dis = (hit_pt - start_pt).norm();
                    local_start_pt = start_pt + dir * (dis + md_.resolution * 2);
                    start_pt_out_local_map = true;
                    if (!findNearestNeighborNot(OCCUPIED, local_start_pt,
                                                local_start_pt)) {
                        if (cfg_.visual_process ||cfg_.debug_visualization_en) {
                            VisualUtils::VisualizePoint(mkr_arr_pub_, local_start_pt, Color::Orange(), "local_start_pt",
                                                        0.3,
                                                        1);
                        }
                        cout << RED <<
                             " -- [A*] " << RET_CODE_STR[INIT_ERROR]
                             << " : start point deeply occupied, cannot find feasible path.\n" << RESET << endl;
                        return INIT_ERROR;
                    }
                }
            }

            if (!insideLocalMap(end_pt)) {
                Vec3f seed_pt = start_pt_out_local_map ? md_.local_map_center_d : start_pt;
                if (geometry_utils::lineIntersectBox(end_pt, seed_pt, md_.local_map_min_d, md_.local_map_max_d,
                                                     hit_pt)) {
//                    print(fg(color::orange_red),
//                      " -- [A*] WARN, end point not inside local map, find a waypoint to the map edge\n");
                    Vec3f dir = (hit_pt - end_pt).normalized();
                    double dis = (hit_pt - end_pt).norm();
                    local_end_pt = end_pt + dir * (dis + md_.resolution * 2);
                    /// Then shift local goal to free space
                    end_pt_out_local_map = true;
                    if (!findNearestNeighborNot(OCCUPIED, local_end_pt, local_end_pt)) {
                        cout << RED <<
                             " -- [A*] " << RET_CODE_STR[INIT_ERROR]
                             << " : goal point deeply occupied, cannot find feasible path.\n" << RESET << endl;
                        if (cfg_.visual_process || cfg_.debug_visualization_en) {
                            VisualUtils::VisualizePoint(mkr_arr_pub_, local_end_pt, Color::Green(), "local_end_pt", 0.3,
                                                        1);
                        }
                        return INIT_ERROR;
                    }
                }
            }

            if (cfg_.visual_process) {
                VisualUtils::VisualizePoint(mkr_arr_pub_, local_start_pt, Color::Orange(), "local_start_pt",
                                            0.3,
                                            1);
                VisualUtils::VisualizePoint(mkr_arr_pub_, local_end_pt, Color::Green(), "local_end_pt", 0.3,
                                            1);
            }
            Vector3i start_idx, end_idx;
            posToGlobalIndex(local_start_pt, start_idx);
            posToGlobalIndex(local_end_pt, end_idx);
            if (cfg_.visual_process) {
                VisualUtils::VisualizePoint(mkr_arr_pub_, local_start_pt, Color::Orange(), "local_start_pt", 0.3,
                                            1);
                VisualUtils::VisualizePoint(mkr_arr_pub_, local_end_pt, Color::Green(), "local_end_pt", 0.3, 1);
            }
            if (!insideLocalMap(start_idx) || !insideLocalMap(end_idx)) {
                cout << RED << " -- [RM] Start or end point is out of local map, which should not happen." << RESET
                     << endl;
                return INIT_ERROR;
            }


            GridNodePtr startPtr = grid_node_buffer_[getLocalIndexHash(start_idx)];
            GridNodePtr endPtr = grid_node_buffer_[getLocalIndexHash(end_idx)];
            endPtr->id_g = end_idx;

            std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> open_set;
            std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, FrontierComparator> frontier_queue;


            GridNodePtr neighborPtr = NULL;
            GridNodePtr current = NULL;

            startPtr->id_g = start_idx;
            startPtr->rounds = rounds_;
            startPtr->distance_score = 0;
            startPtr->total_score = getHeu(startPtr, endPtr, cfg_.heu_type);
            startPtr->state = GridNode::OPENSET; //put start node in open set
            startPtr->father_ptr = NULL;
            open_set.push(startPtr); //put start in open set
            int num_iter = 0;
            vector<GridNodePtr> node_path;

            if (cfg_.visual_process) {
                VisualUtils::VisualizePoint(mkr_arr_pub_,
                                            start_pt,
                                            Color::Green(),
                                            "start_pt",
                                            0.3, 1);
                VisualUtils::VisualizePoint(mkr_arr_pub_,
                                            end_pt,
                                            Color::Blue(),
                                            "goal_pt",
                                            0.3, 1);
            }

            while (!open_set.empty()) {
                num_iter++;
                current = open_set.top();
                open_set.pop();
                if (cfg_.visual_process) {
                    Vec3f local_pt;
                    globalIndexToPos(current->id_g, local_pt);
                    VisualUtils::VisualizePoint(mkr_arr_pub_,
                                                local_pt,
                                                Color(Color::Pink(), 0.5),
                                                "astar_process",
                                                0.1);
                    ros::Duration(0.001).sleep();
                }
                if (current->id_g(0) == endPtr->id_g(0) &&
                    current->id_g(1) == endPtr->id_g(1) &&
                    current->id_g(2) == endPtr->id_g(2)) {
                    retrievePath(current, node_path);
                    if (start_pt_out_local_map) {
                        Vec3i start_idx_g;
                        posToGlobalIndex(start_pt, start_idx_g);
                        GridNodePtr temp_ptr(new GridNode);
                        temp_ptr->id_g = start_idx_g;
                        node_path.push_back(temp_ptr);
                    }
                    ConvertNodePathToPointPath(node_path, out_path);
                    return REACH_GOAL;
                }

                // Distance terminate condition
                if (searching_horizon > 0 && current->distance_score > searching_horizon / md_.resolution) {
                    GridNodePtr local_goal = current;
                    if (md_.unknown_as_occ && !frontier_queue.empty()) {
                        local_goal = frontier_queue.top()->father_ptr;
                        if (local_goal->distance_to_goal > current->distance_to_goal) {
                            local_goal = current;
                        }
                    }
                    retrievePath(local_goal, node_path);
                    if (start_pt_out_local_map) {
                        node_path.push_back(startPtr);
                    }
                    ConvertNodePathToPointPath(node_path, out_path);
                    return REACH_HORIZON;
                }


                current->state = GridNode::CLOSEDSET; //move current node from open set to closed set.

                for (int dx = -1; dx <= 1; dx++)
                    for (int dy = -1; dy <= 1; dy++)
                        for (int dz = -1; dz <= 1; dz++) {
                            if (dx == 0 && dy == 0 && dz == 0) {
                                continue;
                            }
                            if (!cfg_.allow_diag &&
                                (std::abs(dx) + std::abs(dy) + std::abs(dz) > 1)) {
                                continue;
                            }

                            Vec3i neighborIdx;
                            Vec3f neighborPos;
                            neighborIdx(0) = (current->id_g)(0) + dx;
                            neighborIdx(1) = (current->id_g)(1) + dy;
                            neighborIdx(2) = (current->id_g)(2) + dz;
                            globalIndexToPos(neighborIdx, neighborPos);
//                            if (!map_ptr_->insideLocalMap(neighborPos)) {
//                                continue;
//                            }
                            if (!insideLocalMap(neighborIdx)) {
                                continue;
                            }

                            GridType neighbor_type;
                            if (md_.use_inf_map) {
                                neighbor_type = map_ptr_->getInfGridType(neighborPos);
                            } else {
                                neighbor_type = map_ptr_->getGridType(neighborPos);
                            }

                            if (neighbor_type == OCCUPIED || neighbor_type == OUT_OF_MAP) {
                                continue;
                            }

                            if (md_.unknown_as_occ && neighbor_type == OUT_OF_MAP) {
                                continue;
                            }


                            neighborPtr = grid_node_buffer_[getLocalIndexHash(neighborIdx)];
                            if (neighborPtr == nullptr) {
                                cout << RED << " -- [RM] neighborPtr is null, which should not happen." << RESET
                                     << endl;
                                continue;
                            }
                            neighborPtr->id_g = neighborIdx;

                            bool flag_explored = neighborPtr->rounds == rounds_;

                            if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET) {
                                continue; //in closed set.
                            }

                            if (md_.unknown_as_occ && neighbor_type == UNKNOWN && neighborPtr) {
                                // the frontier is recorded but not expand.
                                neighborPtr->father_ptr = current;
                                Vec3f pos;
                                globalIndexToPos(neighborIdx, pos);
                                neighborPtr->distance_to_goal = getHeu(neighborPtr, endPtr, cfg_.heu_type);
                                frontier_queue.push(neighborPtr);
                                continue;
                            }

                            neighborPtr->rounds = rounds_;
                            double distance_score = sqrt(dx * dx + dy * dy + dz * dz);
                            distance_score = current->distance_score + distance_score;
                            Vec3f pos;
                            globalIndexToPos(neighborIdx, pos);
                            double heu_score = getHeu(neighborPtr, endPtr, cfg_.heu_type);

                            if (!flag_explored) {
                                //discover a new node
                                neighborPtr->state = GridNode::OPENSET;
                                neighborPtr->father_ptr = current;
                                neighborPtr->distance_score = distance_score;
                                neighborPtr->distance_to_goal = heu_score;
                                neighborPtr->total_score = distance_score + heu_score;
                                open_set.push(neighborPtr); //put neighbor in open set and record it.
                            } else if (distance_score < neighborPtr->distance_score) {
                                neighborPtr->father_ptr = current;
                                neighborPtr->distance_score = distance_score;
                                neighborPtr->distance_to_goal = heu_score;
                                neighborPtr->total_score = distance_score + heu_score;
                            }
                        }
                ros::Time time_2 = ros::Time::now();
                if (!cfg_.visual_process && (time_2 - time_1).toSec() > time_out) {
                    ROS_WARN("Failed in A star path searching !!! %lf seconds time limit exceeded.", time_out);
                    return TIME_OUT;
                }
            }
            ros::Time time_2 = ros::Time::now();
            if ((time_2 - time_1).toSec() > time_out) {
                ROS_WARN("Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).toSec(),
                         num_iter);
                return type_utils::NO_PATH;
            }

            if (md_.unknown_as_occ && !frontier_queue.empty()) {
                GridNodePtr local_goal;
                while (!frontier_queue.empty()) {
                    local_goal = frontier_queue.top();
                    frontier_queue.pop();
                    Vec3f pos;
                    globalIndexToPos(local_goal->id_g, pos);
                    if ((pos - start_pt).norm() < 1.0) {
                        continue;
                    }
                    break;
                }
                if (frontier_queue.empty()) {
                    cout << RED << " -- [A*] Frontier queue is empty, return." << RESET << endl;
                    return NO_PATH;
                }
                retrievePath(local_goal, node_path);
                if (start_pt_out_local_map) {
                    node_path.push_back(startPtr);
                }
                ConvertNodePathToPointPath(node_path, out_path);
                cout << BLUE << "Frontier queue: " << frontier_queue.size() << endl;
                return REACH_HORIZON;
            }

            cout << RED << " -- [A*] Point to point path cannot find path with iter num: " << num_iter << ", return."
                 << RESET << endl;
            return NO_PATH;
        }

        /// @ brief: The escape path only for path search from prob map to inf map. from non-occupied point to
        ///          inf map free (or known freee) point . Aim to find a path from current point to (known) free point
        /// @ param:
        RET_CODE EscapePathSearch(const Vec3f &start_pt, const int flag, vec_Vec3f &out_path) {
            md_.searching_horizon = 999;
            md_.use_inf_map = flag & ON_INF_MAP;
            md_.use_prob_map = flag & ON_PROB_MAP;
            md_.unknown_as_occ = flag & UNKNOWN_AS_OCCUPIED;
            md_.unknown_as_free = flag & UNKNOWN_AS_FREE;
            md_.local_map_center_d = start_pt;
            md_.resolution = map_ptr_->getCfg().resolution;
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

            ros::Time time_1 = ros::Time::now();
            ++rounds_;

            posToGlobalIndex(md_.local_map_center_d, md_.local_map_center_id_g);
            /// 2) Check start point

            if (!insideLocalMap(start_pt) ||
                !map_ptr_->insideLocalMap(start_pt)) {
                ROS_ERROR(" -- [A*] %s: escape start point is not inside local map.\n",
                          RET_CODE_STR[INIT_ERROR].c_str());
                return INIT_ERROR;
            }
            Vec3f local_start_pt = start_pt;
            GridType start_type = map_ptr_->getGridType(local_start_pt);
            if (!findNearestNeighborNot(OCCUPIED, start_pt, local_start_pt)) {
                cout << RED <<
                     " -- [A*] " << RET_CODE_STR[INIT_ERROR]
                     << " : escape start point deeply occupied, cannot find feasible path.\n" << RESET << endl;
                return INIT_ERROR;
            }
//            int shift_cnt = 0;
//            while (start_type == OCCUPIED) {
//                if (shift_cnt >= sorted_pts.size()) {
//
//                }
//                local_start_pt = start_pt + sorted_pts[shift_cnt].cast<double>() * resolution;
//                start_type = map_ptr_->getGridType(local_start_pt);
//            }
            Vec3i start_idx;
            posToGlobalIndex(local_start_pt, start_idx);

            GridNodePtr startPtr = grid_node_buffer_[getLocalIndexHash(start_idx)];
            std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> open_set;
            GridNodePtr neighborPtr = NULL;
            GridNodePtr current = NULL;

            startPtr->id_g = start_idx;
            startPtr->rounds = rounds_;
            startPtr->distance_score = 0;
            startPtr->total_score = 0;
            startPtr->state = GridNode::OPENSET; //put start node in open set
            startPtr->father_ptr = NULL;
            open_set.push(startPtr); //put start in open set
            int num_iter = 0;

            vector<GridNodePtr> node_path;

            while (!open_set.empty()) {
                num_iter++;
                current = open_set.top();
                open_set.pop();
                if (cfg_.visual_process) {
                    Vec3f local_pt;
                    globalIndexToPos(current->id_g, local_pt);
                    VisualUtils::VisualizePoint(mkr_arr_pub_,
                                                local_pt,
                                                Color::Pink(),
                                                "astar_process",
                                                0.1);
                    ros::Duration(0.01).sleep();
                }
                Vec3f cur_pos;
                globalIndexToPos(current->id_g, cur_pos);
                GridType cur_inf_type = map_ptr_->getInfGridType(cur_pos);
                if (md_.unknown_as_occ && cur_inf_type != OCCUPIED && cur_inf_type != UNKNOWN) {
                    retrievePath(current, node_path);
                    ConvertNodePathToPointPath(node_path, out_path);
                    ros::Time time_2 = ros::Time::now();
//                    printf("\033[34m Escape: A star iter:%d, time:%.3f ms\033[0m\n", num_iter, (time_2 - time_1).toSec() * 1000);
                    return REACH_HORIZON;
                }

                if (md_.unknown_as_free && cur_inf_type != OCCUPIED) {
                    retrievePath(current, node_path);
                    ConvertNodePathToPointPath(node_path, out_path);
                    ros::Time time_2 = ros::Time::now();
//                    printf("\033[34m Escape: A star iter:%d, time:%.3f ms\033[0m\n", num_iter, (time_2 - time_1).toSec() * 1000);
                    return REACH_HORIZON;
                }

                current->state = GridNode::CLOSEDSET; //move current node from open set to closed set.

                for (int dx = -1; dx <= 1; dx++)
                    for (int dy = -1; dy <= 1; dy++)
                        for (int dz = -1; dz <= 1; dz++) {
                            if (dx == 0 && dy == 0 && dz == 0) {
                                continue;
                            }
                            Vec3i neighborIdx;
                            Vec3f neighborPos;
                            neighborIdx(0) = (current->id_g)(0) + dx;
                            neighborIdx(1) = (current->id_g)(1) + dy;
                            neighborIdx(2) = (current->id_g)(2) + dz;
                            globalIndexToPos(neighborIdx, neighborPos);
                            if (!map_ptr_->insideLocalMap(neighborPos) ||
                                !insideLocalMap(neighborIdx)) {
                                continue;
                            }

                            GridType neighbor_type;
                            if (md_.use_inf_map) {
                                neighbor_type = map_ptr_->getInfGridType(neighborPos);
                            } else {
                                neighbor_type = map_ptr_->getGridType(neighborPos);
                            }

                            if (neighbor_type == OCCUPIED || neighbor_type == OUT_OF_MAP) {
                                continue;
                            }

                            if (md_.unknown_as_occ && neighbor_type == UNKNOWN) {
                                continue;
                            }

                            neighborPtr = grid_node_buffer_[getLocalIndexHash(neighborIdx)];
                            if (neighborPtr == nullptr) {
                                cout << RED << " -- [RM] neighborPtr is null, which should not happen" << RESET << endl;
                                continue;
                            }
                            neighborPtr->id_g = neighborIdx;

                            bool flag_explored = neighborPtr->rounds == rounds_;

                            if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET) {
                                continue; //in closed set.
                            }

                            neighborPtr->rounds = rounds_;
                            double distance_score = sqrt(dx * dx + dy * dy + dz * dz);
                            distance_score = current->distance_score + distance_score;
                            Vec3f pos;
                            globalIndexToPos(neighborIdx, pos);
                            double heu_score = 0;
                            if (!flag_explored) {
                                //discover a new node
                                neighborPtr->state = GridNode::OPENSET;
                                neighborPtr->father_ptr = current;
                                neighborPtr->distance_score = distance_score;
                                neighborPtr->total_score = distance_score + heu_score;
                                open_set.push(neighborPtr); //put neighbor in open set and record it.
                            } else if (distance_score < neighborPtr->distance_score) {
                                neighborPtr->father_ptr = current;
                                neighborPtr->distance_score = distance_score;
                                neighborPtr->total_score = distance_score + heu_score;
                            }
                        }
                ros::Time time_2 = ros::Time::now();
                if (!cfg_.visual_process && (time_2 - time_1).toSec() > 0.2) {
                    ROS_WARN("Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
                    return TIME_OUT;
                }
            }
            ros::Time time_2 = ros::Time::now();
            if ((time_2 - time_1).toSec() > 0.1) {
                ROS_WARN("Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).toSec(),
                         num_iter);
            }
            cout << RED << " -- [A*] Escape path searcher, cannot find path, return." << RESET << endl;
            return NO_PATH;
        }


        void ConvertNodePathToPointPath(const vector<GridNodePtr> &node_path, vec_Vec3f &point_path) {
            point_path.clear();
            for (auto ptr: node_path) {
                Vec3f pos;
                globalIndexToPos(ptr->id_g, pos);
                point_path.push_back(pos);
            }
            reverse(point_path.begin(), point_path.end());
        }


        bool
        findNearestNeighborIs(const GridType &type, const Vec3f &src_pt, Vec3f &output_pt) {
            GridType start_type;
            Vec3f cand_pt = src_pt;
            cout << sorted_pts.size() << endl;
            for (int i = 0; i < sorted_pts.size(); i++) {
                cout << sorted_pts[i].transpose() << endl;
                if (!insideLocalMap(cand_pt)) {
                    cout << "not inside local map" << endl;
                    continue;
                }
                if (md_.use_inf_map) {
                    start_type = map_ptr_->getInfGridType(cand_pt);
                } else {
                    start_type = map_ptr_->getGridType(cand_pt);
                }
                if (start_type == type) {
                    output_pt = cand_pt;
                    return true;
                } else {
                    cout << GridTypeStr[start_type] << endl;
                }
                cand_pt = src_pt + sorted_pts[i].cast<double>() * md_.resolution;
            }
            return true;
        }

        bool
        findNearestNeighborNot(const GridType &type, const Vec3f &src_pt, Vec3f &output_pt) {
            GridType start_type;
            Vec3f cand_pt = src_pt;
            for (int i = 0; i < sorted_pts.size(); i++) {
                if (!insideLocalMap(cand_pt)) {
                    continue;
                }
                if (md_.use_inf_map) {
                    start_type = map_ptr_->getInfGridType(cand_pt);
                } else {
                    start_type = map_ptr_->getGridType(cand_pt);
                }
                if (start_type != type) {
                    output_pt = cand_pt;
                    return true;
                }
                cand_pt = src_pt + sorted_pts[i].cast<double>() * md_.resolution;
            }
            return false;
        }

    private:


        void retrievePath(GridNodePtr current, vector<GridNodePtr> &path) {
            path.push_back(current);
            while (current->father_ptr != NULL) {
                current = current->father_ptr;
                path.push_back(current);
            }
        }


    };
}
