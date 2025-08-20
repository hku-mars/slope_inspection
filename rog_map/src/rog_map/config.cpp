#include <rog_map/config.h>
#include "boost/math/common_factor.hpp"

using namespace rog_map;

void ROSParamLoader::resetMapSize(ROGMapConfig &cfg) {

    int inflation_ratio = ceil(cfg.inflation_resolution / cfg.resolution);
    std::cout << GREEN << " -- [RM-Config] inflation_ratio: " << inflation_ratio << std::endl;
    cfg.inflation_resolution = cfg.resolution * inflation_ratio;
    std::cout << GREEN << " -- [RM-Config] inflation_resolution: " << cfg.inflation_resolution << std::endl;

    cfg.half_map_size_d = cfg.map_size_d / 2;

    // Size_d is only for calculate index number.
    // 1) we calculate the index number of the inf map
    int max_step = 0;
    if (!cfg.unk_inflation_en) {
        max_step = cfg.inflation_step;
    } else {
        max_step = std::max(cfg.inflation_step, cfg.unk_inflation_step);
    }
    cfg.inf_half_map_size_i = (cfg.half_map_size_d / cfg.inflation_resolution).cast<int>()
                              + (max_step + 1) * Vec3i::Ones();

    // 2) we calculate the index number of the prob map, which should be smaller than infmap
    cfg.half_map_size_i = (cfg.inf_half_map_size_i - (max_step + 1) * Vec3i::Ones()) * inflation_ratio;

    // 3) compute the frontier conter map size
    if (cfg.frontier_extraction_en) {
        cfg.fro_half_map_size_i = cfg.half_map_size_i + Vec3i::Constant(1);
    }

    // 4) re-compute the map_size_d
    cfg.map_size_d = (cfg.half_map_size_i * 2 + Vec3i::Constant(1)).cast<double>() * cfg.resolution;
    cfg.half_map_size_d = cfg.map_size_d / 2;

    // 5) compute the index of raycasting update box
    cfg.half_local_update_box_d = cfg.local_update_box_d / 2;
    cfg.half_local_update_box_i = (cfg.half_local_update_box_d / cfg.resolution).cast<int>();
    cfg.local_update_box_i = cfg.half_local_update_box_i * 2 +Vec3i::Constant(1);
    cfg.local_update_box_d = cfg.local_update_box_i.cast<double>() * cfg.resolution;







    // second, we calculate the index number of the inf map, which should be larger than prob map at least for two inf step/



//    cfg.inf_half_map_size_i = (1.0 / cfg.inflation_resolution * cfg.half_map_size_d +
//                               cfg.half_map_size_d.cwiseSign() * 0.5).cast<int>() +
//                              (cfg.inflation_step + cfg.unk_inflation_step) * 2 * Vec3i::Ones();
//
//    cfg.half_map_size_d = cfg.inf_half_map_size_i.cast<double>() * cfg.inflation_resolution;
//    cfg.map_size_d = cfg.half_map_size_d * 2;
//
//    cfg.half_map_size_i = cfg.inf_half_map_size_i * inflation_ratio;
//    cfg.fro_half_map_size_i = cfg.half_map_size_i + Vec3i::Constant(2);
//
//    cfg.inf_half_map_size_i += Vec3i::Constant(cfg.inflation_step);
//    cfg.inf_half_map_size_i += Vec3i(0, 0, cfg.inflation_step);
//
//    cfg.half_local_update_box_d = cfg.local_update_box_d / 2;
//    cfg.half_local_update_box_i = (1.0 / cfg.resolution * cfg.half_local_update_box_d +
//                                   Vec3f::Constant(0.5)).cast<int>();
//    cfg.local_update_box_i = cfg.half_local_update_box_i * 2 + Vec3i::Constant(1);
}

ROSParamLoader::ROSParamLoader(const ros::NodeHandle &nh, ROGMapConfig &cfg, const string &name_space) : nh_(nh) {
    nh_ = ros::NodeHandle("~");

    LoadParam(name_space + "/load_pcd_en", cfg.load_pcd_en, false);
    if (cfg.load_pcd_en) {
        LoadParam(name_space + "/pcd_name", cfg.pcd_name, string("map.pcd"));
    }
    LoadParam(name_space + "/safe_margin", cfg.safe_margin, 0.1);
    LoadParam(name_space + "/kd_tree_en", cfg.kd_tree_en, false);
    LoadParam(name_space + "/block_inf_pt", cfg.block_inf_pt, true);
    LoadParam(name_space + "/block_inf_point_with_inflated_occ", cfg.block_inf_point_with_inflated_occ, false);
    LoadParam(name_space + "/inf_point_flag_dis", cfg.inf_point_flag_dis, 500.0);
    cfg.sqr_inf_point_flag_dis = cfg.inf_point_flag_dis * cfg.inf_point_flag_dis;
    LoadParam(name_space + "/map_sliding/enable", cfg.map_sliding_en, true);
    LoadParam(name_space + "/map_sliding/threshold", cfg.map_sliding_thresh, -1.0);

    vector<double> temp_fix_origin;
    LoadParam(name_space + "/fix_map_origin", temp_fix_origin, vector<double>{0, 0, 0});
    if (temp_fix_origin.size() != 3) {
        ROS_ERROR("Fix map origin size is not 3!");
        exit(-1);
    } else {
        cfg.fix_map_origin = Vec3f(temp_fix_origin[0], temp_fix_origin[1], temp_fix_origin[2]);
    }

    LoadParam(name_space + "/frontier_extraction_en", cfg.frontier_extraction_en, false);

    LoadParam(name_space + "/ros_callback/enable", cfg.ros_callback_en, false);
    LoadParam(name_space + "/ros_callback/cloud_topic", cfg.cloud_topic, string("/cloud_registered"));
    LoadParam(name_space + "/ros_callback/odom_topic", cfg.odom_topic, string("/lidar_slam/odom"));
    LoadParam(name_space + "/ros_callback/odom_timeout", cfg.odom_timeout, 0.05);


    LoadParam(name_space + "/visulization/enbale", cfg.visualization_en, false);
    LoadParam(name_space + "/visulization/pub_unknown_map_en", cfg.pub_unknown_map_en, false);
    LoadParam(name_space + "/visulization/frame_id", cfg.frame_id, string("world"));
    LoadParam(name_space + "/visulization/time_rate", cfg.viz_time_rate, 0.0);
    LoadParam(name_space + "/visulization/frame_rate", cfg.viz_frame_rate, 0);
    vector<double> temp_vis_range;
    LoadParam(name_space + "/visulization/range", temp_vis_range, vector<double>{0, 0, 0});
    if (temp_vis_range.size() != 3) {
        ROS_ERROR("Visualization range size is not 3!");
        exit(-1);
    } else {
        cfg.visualization_range = Vec3f(temp_vis_range[0], temp_vis_range[1], temp_vis_range[2]);
        if (cfg.visualization_range.minCoeff() <= 0) {
            std::cout << YELLOW << " -- [ROG] Visualization range is not set, visualization is disabled"
                      << RESET << std::endl;
            cfg.visualization_en = false;
        }
    }


    LoadParam(name_space + "/resolution", cfg.resolution, cfg.resolution);
    LoadParam(name_space + "/inflation_resolution", cfg.inflation_resolution, cfg.resolution);
    /// Resize the map to ease indexing
    if (cfg.resolution > cfg.inflation_resolution) {
        ROS_ERROR("The inflation resolution should be larger than the resolution!");
        exit(-1);
    }
//    int scale = floor(cfg.inflation_resolution / cfg.resolution / 2);
//    cfg.inflation_resolution = cfg.resolution * (scale * 2 + 1);
//    ROS_ERROR("The inflation resolution is set to %f", cfg.inflation_resolution);


    /* For unk inflation */
    LoadParam(name_space + "/unk_inflation_en", cfg.unk_inflation_en, false);
    LoadParam(name_space + "/unk_inflation_step", cfg.unk_inflation_step, 1);

    LoadParam(name_space + "/inflation_step", cfg.inflation_step, 1);
    LoadParam(name_space + "/intensity_thresh", cfg.intensity_thresh, -1.0f);

    vector<double> temp_map_size;
    LoadParam(name_space + "/map_size", temp_map_size, vector<double>{10, 10, 0});
    if (temp_map_size.size() != 3) {
        ROS_ERROR("Map size size is not 3!");
        exit(-1);
    }
    cfg.map_size_d = Vec3f(temp_map_size[0], temp_map_size[1], temp_map_size[2]);


    LoadParam(name_space + "/point_filt_num", cfg.point_filt_num, 2);
    if (cfg.point_filt_num <= 0) {
        std::cout << RED << " -- [ROG] point_filt_num should be larger or equal than 1, it is set to 1 now."
                  << RESET << std::endl;
        cfg.point_filt_num = 1;
    }


    // raycasting
    LoadParam(name_space + "/raycasting/enable", cfg.raycasting_en, true);
    LoadParam(name_space + "/raycasting/batch_update_size", cfg.batch_update_size, 1);
    if (cfg.batch_update_size <= 0) {
        std::cout << RED << " -- [ROG] batch_update_size should be larger or equal than 1, it is set to 1 now."
                  << RESET << std::endl;
        cfg.batch_update_size = 1;
    }
    LoadParam(name_space + "/raycasting/inf_map_known_free_thresh", cfg.known_free_thresh, 0.70);
    LoadParam(name_space + "/raycasting/p_hit", cfg.p_hit, 0.70f);
    LoadParam(name_space + "/raycasting/p_miss", cfg.p_miss, 0.70f);
    LoadParam(name_space + "/raycasting/p_min", cfg.p_min, 0.12f);
    LoadParam(name_space + "/raycasting/p_max", cfg.p_max, 0.97f);
    LoadParam(name_space + "/raycasting/p_occ", cfg.p_occ, 0.80f);
    LoadParam(name_space + "/raycasting/p_free", cfg.p_free, 0.30f);
    LoadParam(name_space + "/raycasting/p_free", cfg.p_free, 0.30f);

    vector<double> temp_ray_range;
    LoadParam(name_space + "/raycasting/ray_range", temp_ray_range, vector<double>{0.3, 10});
    if (temp_ray_range.size() != 2) {
        ROS_ERROR("Ray range size is not 2!");
        exit(-1);
    }
    cfg.raycast_range_min = temp_ray_range[0];
    cfg.raycast_range_max = temp_ray_range[1];
    cfg.sqr_raycast_range_max = cfg.raycast_range_max * cfg.raycast_range_max;
    cfg.sqr_raycast_range_min = cfg.raycast_range_min * cfg.raycast_range_min;
    vector<double> update_box;
    LoadParam(name_space + "/raycasting/local_update_box", update_box, vector<double>{999, 999, 999});
    if (update_box.size() != 3) {
        ROS_ERROR("Update box size is not 3!");
        exit(-1);
    }
    cfg.local_update_box_d = Vec3f(update_box[0], update_box[1], update_box[2]);


    LoadParam(name_space + "/virtual_ground_height", cfg.virtual_ground_height, -0.1);
    LoadParam(name_space + "/virtual_ceil_height", cfg.virtual_ceil_height, -0.1);


    LoadParam(name_space + "/raycasting/blind_filter_en", cfg.blind_filter_en, false);
    LoadParam(name_space + "/raycasting/blind_filter_dis", cfg.blind_filter_dis, 1.0);

    resetMapSize(cfg);



    /// Probabilistic Update
#define logit(x) (log((x) / (1 - (x))))
    cfg.l_hit = logit(cfg.p_hit);
    cfg.l_miss = logit(cfg.p_miss);
    cfg.l_min = logit(cfg.p_min);
    cfg.l_max = logit(cfg.p_max);
    cfg.l_occ = logit(cfg.p_occ);
    cfg.l_free = logit(cfg.p_free);

    std::cout<<BLUE<<"\t[ROG] l_hit: "<<cfg.l_hit<<RESET<<std::endl;
    std::cout<<BLUE<<"\t[ROG] l_miss: "<<cfg.l_miss<<RESET<<std::endl;
    std::cout<<BLUE<<"\t[ROG] l_min: "<<cfg.l_min<<RESET<<std::endl;
    std::cout<<BLUE<<"\t[ROG] l_max: "<<cfg.l_max<<RESET<<std::endl;
    std::cout<<BLUE<<"\t[ROG] l_occ: "<<cfg.l_occ<<RESET<<std::endl;
    std::cout<<BLUE<<"\t[ROG] l_free: "<<cfg.l_free<<RESET<<std::endl;



    cfg.half_map_size_d = cfg.map_size_d / 2.0;

    // init spherical neighbor
    for (int dx = -cfg.inflation_step; dx <= cfg.inflation_step; dx++) {
        for (int dy = -cfg.inflation_step; dy <= cfg.inflation_step; dy++) {
            for (int dz = -cfg.inflation_step; dz <= cfg.inflation_step; dz++) {
                if (cfg.inflation_step == 1 ||
                    dx * dx + dy * dy + dz * dz <= cfg.inflation_step * cfg.inflation_step) {
                    cfg.spherical_neighbor.emplace_back(dx, dy, dz);
                }
            }
        }
    }
    std::sort(cfg.spherical_neighbor.begin(), cfg.spherical_neighbor.end(), [](const Vec3i &a, const Vec3i &b) {
        return a.norm() < b.norm();
    });

    if (cfg.unk_inflation_en) {
        cfg.unk_spherical_neighbor.clear();
        // init spherical neighbor
        for (int dx = -cfg.unk_inflation_step; dx <= cfg.unk_inflation_step; dx++) {
            for (int dy = -cfg.unk_inflation_step; dy <= cfg.unk_inflation_step; dy++) {
                for (int dz = -cfg.unk_inflation_step; dz <= cfg.unk_inflation_step; dz++) {
                    if (cfg.unk_inflation_step == 1 ||
                        dx * dx + dy * dy + dz * dz <= cfg.unk_inflation_step * cfg.unk_inflation_step) {
                        cfg.unk_spherical_neighbor.emplace_back(dx, dy, dz);
                    }
                }
            }
        }
        std::sort(cfg.unk_spherical_neighbor.begin(), cfg.unk_spherical_neighbor.end(),
                  [](const Vec3i &a, const Vec3i &b) {
                      return a.norm() < b.norm();
                  });
    }
}

template<class T>
bool ROSParamLoader::LoadParam(string param_name, T &param_value, T default_value) {
    if (nh_.getParam(param_name, param_value)) {
        printf("\033[0;32m Load param %s succes: \033[0;0m", param_name.c_str());
        std::cout << param_value << std::endl;
        return true;
    } else {
        printf("\033[0;33m Load param %s failed, use default value: \033[0;0m", param_name.c_str());
        param_value = default_value;
        std::cout << param_value << std::endl;
        return false;
    }
}

template<class T>
bool ROSParamLoader::LoadParam(string param_name, vector<T> &param_value, vector<T> default_value) {
    if (nh_.getParam(param_name, param_value)) {
        printf("\033[0;32m Load param %s succes: \033[0;0m", param_name.c_str());
        for (size_t i = 0; i < param_value.size(); i++) {
            std::cout << param_value[i] << " ";
        }
        std::cout << std::endl;
        return true;
    } else {
        printf("\033[0;33m Load param %s failed, use default value: \033[0;0m", param_name.c_str());
        param_value = default_value;
        for (size_t i = 0; i < param_value.size(); i++) {
            std::cout << param_value[i] << " ";
        }
        std::cout << std::endl;
        return false;
    }
}

template bool ROSParamLoader::LoadParam(string param_name, int &param_value, int default_value);

template bool ROSParamLoader::LoadParam(string param_name, double &param_value, double default_value);

template bool ROSParamLoader::LoadParam(string param_name, bool &param_value, bool default_value);

template bool ROSParamLoader::LoadParam(string param_name, string &param_value, string default_value);

template bool ROSParamLoader::LoadParam(string param_name, vector<int> &param_value, vector<int> default_value);

template bool ROSParamLoader::LoadParam(string param_name, vector<double> &param_value, vector<double> default_value);

template bool ROSParamLoader::LoadParam(string param_name, vector<float> &param_value, vector<float> default_value);

template bool ROSParamLoader::LoadParam(string param_name, vector<bool> &param_value, vector<bool> default_value);