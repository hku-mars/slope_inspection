#pragma once

#include <rog_map/config.h>
#include <type_utils/common_type_name.h>
#include <benchmark_utils/scope_timer.h>

#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "log/"+name))
#define PCD_FILE_DIR(name) (string(string(ROOT_DIR) + "pcd/"+name))

namespace rog_map {
    using namespace type_utils;
    using namespace benchmark_utils;
    using std::vector;
    using std::string;

    template<typename T>
    std::ostream &operator<<(std::ostream &out, const std::vector<T> &v) {
        out << "[";
        for (typename std::vector<T>::const_iterator it = v.begin(); it != v.end(); ++it) {
            out << *it;
            if (it != v.end() - 1) {
                out << ", ";
            }
        }
        out << "]";
        return out;
    }

    class SlidingMap {
    protected:
        struct SlidingConfig {
            double resolution;
            double resolution_inv;
            double sliding_thresh;
            bool map_sliding_en;
            Vec3f fix_map_origin;
            Vec3i visualization_range_i;
            Vec3i map_size_i;
            Vec3i half_map_size_i;
            int inf_step;
            int virtual_ceil_height_id_g;
            int virtual_ground_height_id_g;
            int safe_margin_i;
        } sc_;

        Vec3f local_map_origin_d_, local_map_bound_min_d_, local_map_bound_max_d_;
        Vec3i local_map_origin_i_, local_map_bound_min_i_, local_map_bound_max_i_;

    public:
        SlidingMap(const Vec3i &half_map_size_i,
                   const double &resolution,
                   const bool &map_sliding_en,
                   const double &sliding_thresh,
                   const Vec3f &fix_map_origin,
                   const double &inf_step);

        void printMapInformation();

        bool insideLocalMap(const Vec3f &pos) const;

        bool insideLocalMap(const Vec3i &id_g) const;


    protected:

        virtual void resetLocalMap() = 0;

        virtual void clearMemoryOutOfMap(const vector<int> &clear_id, const int &i) = 0;

    public:
        void mapSliding(const Vec3f &odom);

    protected:
        int getLocalIndexHash(const Vec3i &id_in) const;

        void posToGlobalIndex(const Vec3f &pos, Vec3i &id) const;

        void posToGlobalIndex(const double &pos, int &id) const;

        void globalIndexToPos(const Vec3i &id_g, Vec3f &pos) const;

        void globalIndexToLocalIndex(const Vec3i &id_g, Vec3i &id_l) const;

        void localIndexToGlobalIndex(const Vec3i &id_l, Vec3i &id_g) const;

        void localIndexToPos(const Vec3i &id_l, Vec3f &pos) const;

        void hashIdToLocalIndex(const int &hash_id,
                                Vec3i &id) const;

        void hashIdToPos(const int &hash_id,
                         Vec3f &pos) const;

        int getHashIndexFromPos(const Vec3f &pos) const;

        int getHashIndexFromGlobalIndex(const Vec3i &id_g) const;
    };
}