#pragma once

#include <rog_map/inf_map.h>
#include <rog_map/free_cnt_map.h>
#include <queue>
#include <geometry_utils/raycaster.h>
#include <geometry_utils/geometry_utils.h>
//#define USE_UNKNOWN_FLAG
namespace rog_map {

    using namespace geometry_utils;
    using std::cout;
    using std::endl;

    class ProbMap : public SlidingMap {
    public:
        // standardization query
        // Known free < l_free
        // occupied >= l_occ
        bool isKnownFree(const double &prob) const {
            return prob < cfg_.l_free;
        }

        bool isOccupied(const double &prob) const {
            return prob >= cfg_.l_occ;
        }

        // Query result
        GridType getGridType(Vec3i &id_g) const;

        bool isOccupied(const Vec3f &pos) const;

        bool isOccupiedInflate(const Vec3f &pos) const;

        bool isFree(const Vec3f &pos) const;

        bool isFreeInflate(const Vec3f &pos) const;

        bool isUnknown(const Vec3f &pos) const;

        bool isUnknownInflate(const Vec3f &pos) const;

        bool isFrontier(const Vec3f &pos) const;

        bool isFrontier(const Vec3i &id_g) const;


        GridType getGridType(const Vec3f &pos) const;

        GridType getInfGridType(const Vec3f &pos) const;

        double getMapValue(const Vec3f &pos) const;

        void boxSearch(const Vec3f &_box_min, const Vec3f &_box_max,
                       const GridType &gt, vec_E<Vec3f> &out_points) const;

        void boxSearchInflate(const Vec3f &box_min, const Vec3f &box_max,
                              const GridType &gt, vec_E<Vec3f> &out_points) const;

        void boundBoxByLocalMap(Vec3f &box_min, Vec3f &box_max) const;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    protected:
        ROGMapConfig cfg_;
        InfMap::Ptr inf_map_;
        FreeCntMap::Ptr fcnt_map_;
        /// Spherical neighborhood lookup table
        std::vector<float> occupancy_buffer_;
        bool map_empty_{true};
        struct RaycastData {
            geometry_utils::raycaster::RayCaster raycaster;
            std::queue<Vec3i> update_cache_id_g;
            std::vector<uint16_t> operation_cnt;
            std::vector<uint16_t> hit_cnt;
            Vec3f cache_box_max, cache_box_min, local_update_box_max, local_update_box_min;
            int batch_update_counter{0};
        } raycast_data_;

        vector<double> time_consuming_;
        vector<string> time_consuming_name_{"Total", "Raycast", "Update_cache", "Inflation", "PointCloudNumber",
                                            "CacheNumber", "InflationNumber"};


    public:
        typedef std::shared_ptr<ProbMap> Ptr;

        ProbMap(ROGMapConfig &cfg);

        ~ProbMap() = default;

        void updateOccPointCloud(const PointCloud &input_cloud);

        void writeTimeConsumingToLog(std::ofstream &log_file);

        void writeMapInfoToLog(std::ofstream &log_file);

        void updateProbMap(const PointCloud &cloud, const Pose &pose);

    protected:
        // warning using this function will cause memory leak if the id_g is not in the map
        bool isOccupied(const Vec3i &id_g) const;

        bool isUnknown(const Vec3i &id_g) const;

        bool isFree(const Vec3i &id_g) const;

        //====================================================================

        void clearMemoryOutOfMap(const vector<int> &clear_id, const int &i) override;

        void probabilisticMapFromCache();


        void hitPointUpdate(const Vec3f &pos, const int &hash_id, const int &hit_num);

        void missPointUpdate(const Vec3f &pos, const int &hash_id, const int &hit_num);

        void raycastProcess(const PointCloud &input_cloud, const Vec3f &cur_odom);

        void insertUpdateCandidate(const Vec3i &id_g, bool is_hit);

        void updateLocalBox(const Vec3f &cur_odom);

        void resetLocalMap() override;
    };
}