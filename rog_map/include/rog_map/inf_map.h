#pragma once

#include <rog_map/sliding_map.h>

namespace rog_map {

    class InfMap : public SlidingMap {
    public:
        // for query

        bool isOccupied(const Vec3f &pos) const;

        bool isUnknown(const Vec3f &pos) const;

        bool isFree(const Vec3f &pos) const;

    private:
        bool map_empty_{true};
        struct MapData {
            std::vector<int16_t> occupied_cnt;
            std::vector<int16_t> occ_inflate_cnt;
            std::vector<int16_t> unknown_cnt;
            std::vector<int16_t> unk_inflate_cnt;
            int sub_grid_num;
            int unk_neighbor_num;
            int occ_neighbor_num;
        } md_;
        ROGMapConfig cfg_;
        int inf_num_{0};
        double inf_t_{0.0};

    public:
        typedef std::shared_ptr<InfMap> Ptr;

        InfMap(ROGMapConfig &cfg);

        ~InfMap() = default;


        void getInflationNumAndTime(double &inf_n, double &inf_t);

        void writeMapInfoToLog(std::ofstream &log_file);

        bool updateGridCounter(const Vec3f &pos,
                               const GridType &from_type,
                               const GridType &to_type) ;

        void boxSearch(const Vec3f &box_min, const Vec3f &box_max,
                       const GridType &gt, vec_E<Vec3f> &out_points) const ;
        GridType getGridType(const Vec3f &pos) const;

        void resetLocalMap()override ;

    private:
        bool isUnknown(const Vec3i &id_g) const;

        bool isOccupied(const Vec3i &id_g) const;

        void updateInflation(const Vec3i &id_g, const bool is_hit) ;

        void updateUnkInflation(const Vec3i &id_g, const bool is_add);

        GridType getGridType(const Vec3i &id_g) const;

        void clearMemoryOutOfMap(const vector<int> &clear_id, const int &i) override ;
    };
}

