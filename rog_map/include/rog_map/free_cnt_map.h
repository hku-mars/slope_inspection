#pragma once

#include <rog_map/sliding_map.h>

namespace rog_map {

    class FreeCntMap : public SlidingMap {


    private:
        bool map_empty_{true};
        std::vector<int16_t> neighbor_free_cnt;
        ROGMapConfig cfg_;


    public:
        typedef std::shared_ptr<FreeCntMap> Ptr;

        FreeCntMap(const Vec3i &half_map_size_i,
                   const double &resolution,
                   const bool &sliding_en,
                   const double &sliding_thresh,
                   const Vec3f &fix_map_origin,
                   const int &inf_step = 0) : SlidingMap(half_map_size_i,
                                                         resolution,
                                                         sliding_en,
                                                         sliding_thresh,
                                                         fix_map_origin, inf_step) {
            int map_size = sc_.map_size_i.prod();
            neighbor_free_cnt.resize(map_size, 0);
            resetLocalMap();
            std::cout << GREEN << " -- [InfMap] Init successfully -- ." << RESET << std::endl;
            printMapInformation();
        }

        ~FreeCntMap() = default;


        void resetLocalMap() override {
            std::cout << RED << " -- [Fro-Map] Clear all local map." << std::endl;
            std::fill(neighbor_free_cnt.begin(), neighbor_free_cnt.end(), 0);
        }

        int getFreeCnt(const Vec3f &pos) {
            return neighbor_free_cnt[getHashIndexFromPos(pos)];
        }

        int getFreeCnt(const Vec3i &id_g) {
            return neighbor_free_cnt[getHashIndexFromGlobalIndex(id_g)];
        }

        void updateFrontierCounter(const Vec3i &id_g, bool add) {
            if (!insideLocalMap(id_g)) {
                return;
            }
            Vec3i neighbor_id_g;
            for (int i = -1; i <= 1; ++i) {
                neighbor_id_g[0] = id_g[0] + i;
                for (int j = -1; j <= 1; ++j) {
                    neighbor_id_g[1] = id_g[1] + j;
                    for (int k = -1; k <= 1; ++k) {
                        neighbor_id_g[2] = id_g[2] + k;
                        int hash_id = getHashIndexFromGlobalIndex(neighbor_id_g);

                        if (add) {
                            neighbor_free_cnt[hash_id] += 1;
                            // only for bug report
                            if (neighbor_free_cnt[hash_id] > 27) {
                                throw std::runtime_error("Frontier counter overflow with larger than 26");
                            }
                        } else {
                            neighbor_free_cnt[hash_id] -= 1;
                            if (neighbor_free_cnt[hash_id] < 0) {
                                throw std::runtime_error("Frontier counter overflow with smaller than 0");
                            }
                        }
                    }
                }
            }
        }

        void clearMemoryOutOfMap(const vector<int> &clear_id, const int &i) override {
            vector<int> ids{i, (i + 1) % 3, (i + 2) % 3};
            for (const auto &idd: clear_id) {
                for (int x = -sc_.half_map_size_i(ids[1]); x <= sc_.half_map_size_i(ids[1]); x++) {
                    for (int y = -sc_.half_map_size_i(ids[2]); y <= sc_.half_map_size_i(ids[2]); y++) {
                        Vec3i temp_clear_id;
                        temp_clear_id(ids[0]) = idd;
                        temp_clear_id(ids[1]) = x;
                        temp_clear_id(ids[2]) = y;
                        int addr = getLocalIndexHash(temp_clear_id);
                        neighbor_free_cnt[addr] = 0;
                    }
                }
            }
        };
    };

}

