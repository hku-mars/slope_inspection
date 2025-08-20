#pragma once

#include <ros/ros.h>
#include <type_utils/common_type_name.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <algorithm>

namespace rog_map {
    using namespace type_utils;
    using std::string;
    using std::vector;
#define RM_UNKNOWN_FLAG (-99999)
    typedef pcl::PointXYZINormal PclPoint;
    typedef pcl::PointCloud<PclPoint> PointCloud;


    class ROGMapConfig {
    public:
        // add 11.15 blind pt filter
        bool blind_filter_en{false};
        double blind_filter_dis{0.0};

        bool kd_tree_en{false};
        double safe_margin{0};
        bool load_pcd_en{false};
        string pcd_name{"map.pcd"};

        double resolution, inflation_resolution;
        int inflation_step;
        Vec3f local_update_box_d, half_local_update_box_d;
        Vec3i local_update_box_i, half_local_update_box_i;
        Vec3f map_size_d, half_map_size_d;
        Vec3i inf_half_map_size_i, half_map_size_i, fro_half_map_size_i;
        double virtual_ceil_height, virtual_ground_height;
        int virtual_ceil_height_id_g, virtual_ground_height_id_g;

        bool visualization_en{false}, frontier_extraction_en{false},
                raycasting_en{true}, ros_callback_en{false}, pub_unknown_map_en{false};

        bool block_inf_pt,block_inf_point_with_inflated_occ;
        double inf_point_flag_dis, sqr_inf_point_flag_dis;

        std::vector<Vec3i> spherical_neighbor;
        std::vector<Vec3i> unk_spherical_neighbor;

        /* intensity noise filter*/
        float intensity_thresh;
        /* aster properties */
        string frame_id;
        bool map_sliding_en{true};
        Vec3f fix_map_origin;
        string odom_topic, cloud_topic;
        /* probability update */
        double raycast_range_min, raycast_range_max;
        double sqr_raycast_range_min, sqr_raycast_range_max;
        int point_filt_num, batch_update_size;
        float p_hit, p_miss, p_min, p_max, p_occ, p_free;
        float l_hit, l_miss, l_min, l_max, l_occ, l_free;

        /* for unknown inflation */
        bool unk_inflation_en{false};
        int unk_inflation_step{0};



        double odom_timeout;
        Vec3f visualization_range;
        double viz_time_rate;
        int viz_frame_rate;

        double known_free_thresh;
        double map_sliding_thresh;
    };

    class ROSParamLoader {
    public:
        ros::NodeHandle nh_;

        static void resetMapSize(ROGMapConfig &cfg);

        ROSParamLoader(const ros::NodeHandle &nh, ROGMapConfig &cfg, const string &name_space = "rog_map");

        template<class T>
        bool LoadParam(string param_name, T &param_value, T default_value);

        template<class T>
        bool LoadParam(string param_name, vector<T> &param_value, vector<T> default_value);

    };

}
