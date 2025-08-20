#include <rog_map/inf_map.h>

using namespace rog_map;
namespace rog_map {
// Public Query Function ========================================================================
    bool InfMap::isOccupied(const Vec3f &pos) const {
        if (!insideLocalMap(pos)) return false;
        if (pos.z() > cfg_.virtual_ceil_height - cfg_.safe_margin) return false;
        if (pos.z() < cfg_.virtual_ground_height + cfg_.safe_margin) return false;
        return md_.occ_inflate_cnt[getHashIndexFromPos(pos)] > 0;
    }

    bool InfMap::isUnknown(const Vec3f &pos) const {
        if (!cfg_.unk_inflation_en) {
            return true;
        }
        Vec3i id_g, id_l;
        // 1. check virtual ceil and ground
        if (pos.z() >= cfg_.virtual_ceil_height - cfg_.safe_margin - cfg_.inflation_resolution ||
            pos.z() <= cfg_.virtual_ground_height + cfg_.safe_margin + cfg_.inflation_resolution) {
            return false;
        }
        posToGlobalIndex(pos, id_g);
        return isUnknown(id_g);
    }

    bool InfMap::isFree(const Vec3f &pos) const {
        // 1. check virtual ceil and ground
        if (pos.z() >= cfg_.virtual_ceil_height - cfg_.safe_margin - cfg_.inflation_resolution ||
            pos.z() <= cfg_.virtual_ground_height + cfg_.safe_margin + cfg_.inflation_resolution) {
            return false;
        }
        int addr = getHashIndexFromPos(pos);
        bool is_free = md_.occ_inflate_cnt[addr] == 0 && md_.unknown_cnt[addr]==0;
        if(cfg_.unk_inflation_en){
            is_free = is_free && md_.unk_inflate_cnt[addr] == 0;
        }
        return is_free;
    }

//

    InfMap::InfMap(ROGMapConfig &cfg) : SlidingMap(cfg.inf_half_map_size_i,
                                                   cfg.inflation_resolution,
                                                   cfg.map_sliding_en,
                                                   cfg.map_sliding_thresh,
                                                   cfg.fix_map_origin,
                                                   cfg.inflation_step) {

        posToGlobalIndex(cfg.visualization_range, sc_.visualization_range_i);
        posToGlobalIndex(cfg.safe_margin, sc_.safe_margin_i);
        cfg.virtual_ceil_height_id_g =
                int(cfg.virtual_ceil_height / cfg.inflation_resolution + SIGN(cfg.inflation_resolution) * 0.5) -
                cfg.inflation_step;
        cfg.virtual_ground_height_id_g =
                int(cfg.virtual_ground_height / cfg.inflation_resolution + SIGN(cfg.inflation_resolution) * 0.5) +
                cfg.inflation_step;
        cfg.virtual_ceil_height = cfg.virtual_ceil_height_id_g * cfg.inflation_resolution;
        cfg.virtual_ground_height = cfg.virtual_ground_height_id_g * cfg.inflation_resolution;


        int map_size = sc_.map_size_i.prod();
        md_.sub_grid_num = pow(static_cast<int>((cfg.inflation_resolution / cfg.resolution) + 0.5), 3);
        std::cout << GREEN << " -- [InfMap] sub_grid_num = " << md_.sub_grid_num << RESET << std::endl;
        md_.unknown_cnt.resize(map_size, md_.sub_grid_num);
        md_.occupied_cnt.resize(map_size, 0);
        md_.occ_inflate_cnt.resize(map_size, 0);
        md_.occ_neighbor_num = cfg.spherical_neighbor.size();
        if (cfg.unk_inflation_en) {
            md_.unk_neighbor_num = cfg.unk_spherical_neighbor.size();
            // Considering the all grids are unknown at the beginning
            // the unk inf cnt should be the size of inflation queue
            md_.unk_inflate_cnt.resize(map_size, md_.unk_neighbor_num);
        }
        posToGlobalIndex(cfg.visualization_range, sc_.visualization_range_i);

        resetLocalMap();
        cfg_ = cfg;
        std::cout << GREEN << " -- [InfMap] Init successfully -- ." << RESET << std::endl;
        printMapInformation();
    }

    void InfMap::getInflationNumAndTime(double &inf_n, double &inf_t) {
        inf_n = inf_num_;
        inf_num_ = 0;
        inf_t = inf_t_;
        inf_t_ = 0;
    }

    void InfMap::writeMapInfoToLog(std::ofstream &log_file) {
        log_file << "[InfMap]" << std::endl;
        log_file << "\tresolution: " << sc_.resolution << std::endl;
        log_file << "\tmap_size_i: " << sc_.map_size_i.transpose() << std::endl;
        log_file << "\tmap_size_d: " << (sc_.map_size_i.cast<double>() * sc_.resolution).transpose() << std::endl;
    }

    bool InfMap::updateGridCounter(const Vec3f &pos, const GridType &from_type, const GridType &to_type) {
        /*
         * This function update the counter in inflation map
         * all counter should be 0 ~ sub_grid_num
         * when occ_cnt > 0, this grid is considered to be occupied
         * when unk_cnt > 0, this grid is considered to be unknown?
         * */
        TimeConsuming update_t("updateGridCounter", false);
        map_empty_ = false;
        Vec3i id_l, id_g;
        posToGlobalIndex(pos, id_g);
        if (!insideLocalMap(id_g)) {
            std::cout << YELLOW << " -- [IM] updateGridCounter: out of map." << RESET << std::endl;
            return false;
        }
        globalIndexToLocalIndex(id_g, id_l);
        int addr = getLocalIndexHash(id_l);

        switch (to_type) {
            case GridType::UNKNOWN: {
                // In from unk to other, two check need to be perfromed
                // 1. unk inflation: check if all grid in known and change all neighbor's unk_inflate_cnt
                // 2. check unk to occ, if first occ, update all occ_inf_cnt.

                bool last_not_unknown;
                if (cfg_.unk_inflation_en) {
                    last_not_unknown = md_.unknown_cnt[addr] == 0;
                }
                md_.unknown_cnt[addr]++;

                // process occupied counter
                if (from_type == GridType::OCCUPIED) {
                    bool last_occupied = (md_.occupied_cnt[addr] > 0);
                    md_.occupied_cnt[addr]--;
                    bool cur_not_occupied = (md_.occupied_cnt[addr] == 0);
                    if (last_occupied && cur_not_occupied) {
                        updateInflation(id_g, false);
                    }
                    // only for bug report
                    if (md_.occupied_cnt[addr] < 0) {
                        md_.occupied_cnt[addr] = 0;
                        std::cout << RED
                                  << " -- [IM] OCCUPIED 2 UNKNOWN inf map counter less than 0, which should not happen."
                                  << RESET << std::endl;
                        std::cout << RED << " -- [IM] id_g: " << id_g.transpose() << RESET << std::endl;
                        return false;
                    }
                } else if (from_type == GridType::KNOWN_FREE) {
                    // nothing to do with known free since known free cnt = sub_grid_num - unk_cnt - occ_cnt
                } else {
                    std::cout << RED <<
                              " -- [IM] Error, from undefined type 2 UNKNOWN: inf map counter should not happen."
                              << RESET << std::endl;
                    return false;
                }

                // Add 1117 check unk inflation
                if (cfg_.unk_inflation_en) {
                    // if the grid is occupied, the unk inf cnt should be 0
                    if (last_not_unknown) {
                        // there is unk in this grid
                        updateUnkInflation(id_g, true);
                    }
                }

                if (md_.unknown_cnt[addr] > md_.sub_grid_num) {
                    md_.unknown_cnt[addr] = md_.sub_grid_num;
                    std::cout << RED
                              << " -- [IM] OCCUPIED 2 UNKNOWN, unknown counter larger than sub_grid_num, which should not happen."
                              << RESET << std::endl;
                    std::cout << RED << " -- [IM] id_g: " << id_g.transpose() << RESET << std::endl;
                    return false;
                }
                break;
            }
            case GridType::OCCUPIED: {
                // If a grid become occupied, the obstacle inflation should be considered
                bool last_not_occupied_inf = (md_.occupied_cnt[addr] == 0);
                md_.occupied_cnt[addr]++;
                bool cur_occupied_inf = md_.occupied_cnt[addr] > 0;

                if (from_type == UNKNOWN) {
                    bool last_unknown = md_.unknown_cnt[addr] > 0;
                    md_.unknown_cnt[addr]--;
                    bool cur_no_unknown = md_.unknown_cnt[addr] == 0;
                    if (cfg_.unk_inflation_en && last_unknown && cur_no_unknown) {
                        // there is unk in this grid
                        updateUnkInflation(id_g, false);
                    }

                    // only for bug report
                    if (md_.unknown_cnt[addr] < 0) {
                        md_.unknown_cnt[addr] = 0;
                        std::cout << RED
                                  << " -- [IM] From unk to occ inf map counter less than 0, which should not happen."
                                  << RESET << std::endl;
                        std::cout << RED << " -- [IM] delta id_g: " << (id_g).transpose() << RESET
                                  << std::endl;
                        return false;
                    }

                } else if (from_type == KNOWN_FREE) {
                    // nothing to do with known free since known free cnt = sub_grid_num - unk_cnt - occ_cnt
                } else {
                    std::cout << RED <<
                              " -- [IM] Error, from undefined type 2 OCCUPIED: inf map counter should not happen."
                              << RESET << std::endl;
                    return false;
                }

                if (last_not_occupied_inf && cur_occupied_inf) {
                    updateInflation(id_g, true);
                }


                // only for bug report
                if (md_.occupied_cnt[addr] > md_.sub_grid_num) {
                    std::cout << RED
                              << " -- [IM] UNKNOWN 2 OCCUPIED inf map counter larger than max num, which should not happen."
                              << RESET << std::endl;
                    std::cout << RED << " -- [IM] delta id_g: " << (id_g).transpose() << RESET
                              << std::endl;
                    md_.occupied_cnt[addr] = md_.sub_grid_num;
                    return false;
                }

                break;
            }
            case GridType::KNOWN_FREE: {
                if (from_type == UNKNOWN) {
                    bool last_unknown = md_.unknown_cnt[addr] > 0;
                    md_.unknown_cnt[addr]--;
                    bool cur_no_unknown = md_.unknown_cnt[addr] == 0;
                    if (cfg_.unk_inflation_en && last_unknown && cur_no_unknown) {
                        // there is unk in this grid
                        updateUnkInflation(id_g, false);
                    }
                    // only for bug report
                    if (md_.unknown_cnt[addr] < 0) {
                        md_.unknown_cnt[addr] = 0;
                        std::cout << RED
                                  << " -- [IM] From unk to free inf map counter less than 0, which should not happen."
                                  << RESET << std::endl;
                        std::cout << RED << " -- [IM] delta id_g: " << (id_g).transpose() << RESET
                                  << std::endl;
                        return false;
                    }
                } else if (from_type == OCCUPIED) {
                    bool last_occupied = (md_.occupied_cnt[addr] > 0);
                    md_.occupied_cnt[addr]--;
                    bool cur_not_occupied = (md_.occupied_cnt[addr] == 0);
                    if (last_occupied && cur_not_occupied) {
                        updateInflation(id_g, false);
                    }
                    // only for bug report
                    if (md_.occupied_cnt[addr] < 0) {
                        md_.occupied_cnt[addr] = 0;
                        std::cout << RED
                                  << " -- [IM] OCCUPIED 2 UNKNOWN inf map counter less than 0, which should not happen."
                                  << RESET << std::endl;
                        std::cout << RED << " -- [IM] id_g: " << id_g.transpose() << RESET << std::endl;
                        return false;
                    }
                } else {
                    std::cout << RED <<
                              " -- [IM] Error, from undefined type 2 KNOWN_FREE: inf map counter should not happen."
                              << RESET << std::endl;
                    return false;
                }
                break;
            }
            default: {
                std::cout << RED << " -- [IM] Unknown Grid type when update the inf map." << RESET << std::endl;
                return false;
            }
        }
        return true;
    }

    void
    InfMap::boxSearch(const Vec3f &box_min, const Vec3f &box_max, const GridType &gt, vec_E<Vec3f> &out_points) const {
        out_points.clear();
        if (map_empty_) {
            std::cout << RED << " -- [ROG] Map is empty, cannot perform box search." << RESET << std::endl;
            return;
        }
        Vec3i box_min_id_g, box_max_id_g;
        posToGlobalIndex(box_min, box_min_id_g);
        posToGlobalIndex(box_max, box_max_id_g);
        Vec3i box_size = box_max_id_g - box_min_id_g;
        if (gt == UNKNOWN) {
            if (!cfg_.unk_inflation_en) {
                out_points.clear();
                std::cout << RED << " -- [ROG] Unknown inflation is not enabled, cannot perform box search." << RESET
                          << std::endl;
                return;
            }
            out_points.reserve(box_size.prod());
            for (int i = box_min_id_g.x(); i <= box_max_id_g.x(); i++) {
                for (int j = box_min_id_g.y(); j <= box_max_id_g.y(); j++) {
                    for (int k = box_min_id_g.z(); k <= box_max_id_g.z(); k++) {
                        Vec3i id_g(i, j, k);
                        if (isUnknown(id_g)) {
                            Vec3f pos;
                            globalIndexToPos(id_g, pos);
                            out_points.push_back(pos);
                        }
                    }
                }
            }
        } else if (gt == OCCUPIED) {
            out_points.reserve(box_size.prod() / 3);
            for (int i = box_min_id_g.x(); i <= box_max_id_g.x(); i++) {
                for (int j = box_min_id_g.y(); j <= box_max_id_g.y(); j++) {
                    for (int k = box_min_id_g.z(); k <= box_max_id_g.z(); k++) {
                        Vec3i id_g(i, j, k);
                        if (isOccupied(id_g)) {
                            Vec3f pos;
                            globalIndexToPos(id_g, pos);
                            out_points.push_back(pos);
                        }
                    }
                }
            }
        } else {
            throw std::runtime_error(" -- [ROG-Map] Box search does not support KNOWN_FREE.");
        }

    }

    GridType InfMap::getGridType(const Vec3f &pos) const {
        Vec3i id_g, id_l;
        // 1. check virtual ceil and ground
        if (pos.z() >= cfg_.virtual_ceil_height - cfg_.safe_margin - cfg_.inflation_resolution ||
            pos.z() <= cfg_.virtual_ground_height + cfg_.safe_margin + cfg_.inflation_resolution) {
            return OCCUPIED;
        }
        posToGlobalIndex(pos, id_g);
        // 2. get true grid type
        return getGridType(id_g);
    }

    void InfMap::resetLocalMap() {
        std::cout << RED << " -- [Inf-Map] Clear all local map." << std::endl;
        std::fill(md_.unknown_cnt.begin(), md_.unknown_cnt.end(), md_.sub_grid_num);
        std::fill(md_.occupied_cnt.begin(), md_.occupied_cnt.end(), 0);
        std::fill(md_.occ_inflate_cnt.begin(), md_.occ_inflate_cnt.end(), 0);
        if (cfg_.unk_inflation_en) {
            std::fill(md_.unk_inflate_cnt.begin(), md_.unk_inflate_cnt.end(), md_.unk_neighbor_num);
        }
    }

    bool InfMap::isUnknown(const Vec3i &id_g) const {
        if (!insideLocalMap(id_g)) {
            return true;
        }
        if (!cfg_.unk_inflation_en) {
            std::cout << RED << "Cannot query unknown state of InfMap when unk_inflation_en is false." << RESET
                      << std::endl;
            return true;
        }
        Vec3i id_l;
        globalIndexToLocalIndex(id_g, id_l);
        int addr = getLocalIndexHash(id_l);
        return md_.unk_inflate_cnt[addr] > 0;
    }

    bool InfMap::isOccupied(const Vec3i &id_g) const {
        if (!insideLocalMap(id_g)) {
            return false;
        }
        return md_.occ_inflate_cnt[getHashIndexFromGlobalIndex(id_g)] > 0;
    }

    void InfMap::updateInflation(const Vec3i &id_g, const bool is_hit) {
        TimeConsuming tc("updateInflation", false);
        Vec3i id_shift, id_l;
        int addr;
        for (const auto &nei: cfg_.spherical_neighbor) {
            id_shift = id_g + nei;
            if (!insideLocalMap(id_shift)) {
                continue;
            }
            inf_num_++;
            globalIndexToLocalIndex(id_shift, id_l);
            addr = getLocalIndexHash(id_l);
            if (is_hit) {
                md_.occ_inflate_cnt[addr]++;
            } else {
                md_.occ_inflate_cnt[addr]--;
                if (md_.occ_inflate_cnt[addr] < 0) {
                    md_.occ_inflate_cnt[addr] = 0;
                    throw std::runtime_error(" -- [IM] Negative occupancy counter, which should not happened.!");
                }
            }

        }
        inf_t_ += tc.stop();
    }

    void InfMap::updateUnkInflation(const Vec3i &id_g, const bool is_add) {
        TimeConsuming tc("updateInflation", false);
        if (!cfg_.unk_inflation_en) {
            std::cout << RED << "Cannot updateUnkInflation of InfMap when unk_inflation_en is false." << RESET
                      << std::endl;
            return;
        }
        Vec3i id_shift, id_l;
        int addr;
        for (const auto &nei: cfg_.unk_spherical_neighbor) {
            id_shift = id_g + nei;
            if (!insideLocalMap(id_shift)) {
                continue;
            }
            inf_num_++;
            globalIndexToLocalIndex(id_shift, id_l);
            addr = getLocalIndexHash(id_l);
            if (is_add) {
                md_.unk_inflate_cnt[addr]++;
            } else {
                md_.unk_inflate_cnt[addr]--;
            }

            // only for bug report
            if (md_.unk_inflate_cnt[addr] < 0 || md_.unk_inflate_cnt[addr] > md_.unk_neighbor_num) {
                std::cout << "unk_inflate_cnt: " << md_.unk_inflate_cnt[addr] << " unk_neighbor_num: "
                          << md_.unk_neighbor_num << std::endl;
                throw std::runtime_error(" -- [IM] Negative occupancy counter, which should not happened.!");
                md_.unk_inflate_cnt[addr] = 0;
            }

        }
        inf_t_ += tc.stop();
    }

    GridType InfMap::getGridType(const Vec3i &id_g) const {
        if (!insideLocalMap(id_g)) {
            return OUT_OF_MAP;
        }
        Vec3i id_l;
        globalIndexToLocalIndex(id_g, id_l);
        int addr = getLocalIndexHash(id_l);
        // The Occupied is defined by inflation layer
        if (md_.occ_inflate_cnt[addr] > 0) {
            return OCCUPIED;
        } else if (cfg_.unk_inflation_en && md_.unk_inflate_cnt[addr] > 0) {
            return UNKNOWN;
        } else {
            return KNOWN_FREE;
        }
    }

    void InfMap::clearMemoryOutOfMap(const vector<int> &clear_id, const int &i) {
        vector<int> ids{i, (i + 1) % 3, (i + 2) % 3};
        for (const auto &idd: clear_id) {
            for (int x = -sc_.half_map_size_i(ids[1]); x <= sc_.half_map_size_i(ids[1]); x++) {
                for (int y = -sc_.half_map_size_i(ids[2]); y <= sc_.half_map_size_i(ids[2]); y++) {
                    Vec3i temp_clear_id;
                    temp_clear_id(ids[0]) = idd;
                    temp_clear_id(ids[1]) = x;
                    temp_clear_id(ids[2]) = y;
                    int addr = getLocalIndexHash(temp_clear_id);

                    if (md_.occupied_cnt[addr] > 0) {
                        // if this grid is currently occupied, we need to update the inflation
                        // for all the neitghbor since its a OCCUPIED to UNKNOWN lower edge
                        Vec3i id_g;
                        localIndexToGlobalIndex(temp_clear_id, id_g);
                        updateInflation(id_g, false);
                    }
                    if (cfg_.unk_inflation_en && md_.unknown_cnt[addr] == 0) {
                        // if this grid is not unknown, we need to update the inflation
                        // for all the neitghbor since it is a XXX to Unknow upper edge
                        Vec3i id_g;
                        localIndexToGlobalIndex(temp_clear_id, id_g);
                        updateUnkInflation(id_g, true);
                    }
                    // Then clear all the counter for new cells
                    // As a new ceil, this cell have no occupied cnt,
                    // no occ inf, full unknown cnt, and full unk_inflate_cnt
                    md_.occ_inflate_cnt[addr] = 0;
                    md_.occupied_cnt[addr] = 0;
                    md_.unknown_cnt[addr] = md_.sub_grid_num;
                    if (cfg_.unk_inflation_en) {
                        md_.unk_inflate_cnt[addr] = md_.unk_neighbor_num;
                    }

                }
            }
        }
    }

}