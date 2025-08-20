#include <rog_map/sliding_map.h>

using namespace rog_map;

SlidingMap::SlidingMap(const Vec3i &half_map_size_i, const double &resolution, const bool &map_sliding_en,
                       const double &sliding_thresh, const Vec3f &fix_map_origin,
                       const double &inf_step) {
    sc_.resolution = resolution;
    sc_.resolution_inv = 1.0 / resolution;
    sc_.map_sliding_en = map_sliding_en;
    sc_.sliding_thresh = sliding_thresh;
    sc_.fix_map_origin = fix_map_origin;
    sc_.inf_step = inf_step;
    sc_.half_map_size_i = half_map_size_i;
    sc_.map_size_i = 2 * sc_.half_map_size_i + Vec3i::Constant(1);

    if (!map_sliding_en) {
        local_map_origin_d_ = fix_map_origin;
        posToGlobalIndex(local_map_origin_d_, local_map_origin_i_);
    }
}

void SlidingMap::printMapInformation() {
    std::cout << GREEN << "\tresolution: " << sc_.resolution << RESET << std::endl;
    std::cout << GREEN << "\tmap_sliding_en: " << sc_.map_sliding_en << RESET << std::endl;
    std::cout << GREEN << "\tlocal_map_size_i: " << sc_.map_size_i.transpose() << RESET << std::endl;
    std::cout << GREEN << "\tlocal_map_size_d: " << sc_.map_size_i.cast<double>().transpose() * sc_.resolution << RESET
              << std::endl;
}

bool SlidingMap::insideLocalMap(const Vec3f &pos) const {
    Vec3i id_g;
    posToGlobalIndex(pos, id_g);
    return insideLocalMap(id_g);
}

bool SlidingMap::insideLocalMap(const Vec3i &id_g) const {
    if (((id_g - local_map_origin_i_).cwiseAbs() - sc_.half_map_size_i).maxCoeff() > 0) { // + sc_.inf_step > 0) {
        return false;
    }
    return true;
}

void SlidingMap::mapSliding(const Vec3f &odom) {
    TimeConsuming update_local_shift_timer("updateLocalShift", false);
    /// Compute the shifting index
    Vec3i new_origin_i;
    posToGlobalIndex(odom, new_origin_i);
    Vec3f new_origin_d = new_origin_i.cast<double>() * sc_.resolution;
    /// Compute the delta shift
    Vec3i shift_num = new_origin_i - local_map_origin_i_;
    for (long unsigned int i = 0; i < 3; i++) {
        if (fabs(shift_num[i]) > sc_.map_size_i[i]) {
            // Clear all map
            resetLocalMap();
            local_map_origin_i_ = new_origin_i;
            local_map_origin_d_ = new_origin_d;
            return;
        }
    }
    static auto normalize = [](int x, int a, int b) -> int {
        int range = b - a + 1;
        int y = (x - a) % range;
        return (y < 0 ? y + range : y) + a;
    };

    /// Clear the memory out of the map size
    for (int i = 0; i < 3; i++) {
        if (shift_num[i] == 0) {
            continue;
        }
        int min_id_g = -sc_.half_map_size_i(i) + local_map_origin_i_(i);
        int min_id_l = min_id_g % sc_.map_size_i(i);
        vector<int> clear_id;
        if (shift_num(i) > 0) {
            /// forward shift, the min id should be cut
            for (int k = 0; k < shift_num(i); k++) {
                int temp_id = min_id_l + k;
                temp_id = normalize(temp_id, -sc_.half_map_size_i(i), sc_.half_map_size_i(i));
                clear_id.push_back(temp_id);
            }
        } else {
            /// backward shift, the max should be shifted
            for (int k = -1; k >= shift_num(i); k--) {
                int temp_id = min_id_l + k;
                temp_id = normalize(temp_id, -sc_.half_map_size_i(i), sc_.half_map_size_i(i));
                clear_id.push_back(temp_id);
            }
        }

        if (clear_id.empty()) {
            continue;
        }
        clearMemoryOutOfMap(clear_id, i);
    }
    local_map_origin_i_ = new_origin_i;
    local_map_origin_d_ = new_origin_d;
}

int SlidingMap::getLocalIndexHash(const Vec3i &id_in) const {
    Vec3i id = id_in + sc_.half_map_size_i;
    return id(0) * sc_.map_size_i(1) * sc_.map_size_i(2) +
           id(1) * sc_.map_size_i(2) +
           id(2);
}

void SlidingMap::posToGlobalIndex(const Vec3f &pos, Vec3i &id) const {
    id = (sc_.resolution_inv * pos + pos.cwiseSign() * 0.5).cast<int>();
}

void SlidingMap::posToGlobalIndex(const double &pos, int &id) const {
    id = static_cast<int>((sc_.resolution_inv * pos + SIGN(pos) * 0.5));
}

void SlidingMap::globalIndexToPos(const Vec3i &id_g, Vec3f &pos) const {
    pos = id_g.cast<double>() * sc_.resolution;
}

void SlidingMap::globalIndexToLocalIndex(const Vec3i &id_g, Vec3i &id_l) const {
    for (int i = 0; i < 3; ++i) {
        id_l(i) = id_g(i) % sc_.map_size_i(i);
        id_l(i) += id_l(i) > sc_.half_map_size_i(i) ? -sc_.map_size_i(i) : 0;
        id_l(i) += id_l(i) < -sc_.half_map_size_i(i) ? sc_.map_size_i(i) : 0;
    }
}

void SlidingMap::localIndexToGlobalIndex(const Vec3i &id_l, Vec3i &id_g) const {
    for (int i = 0; i < 3; ++i) {
        int min_id_g = -sc_.half_map_size_i(i) + local_map_origin_i_(i);
        int min_id_l = min_id_g % sc_.map_size_i(i);
        min_id_l -= min_id_l > sc_.half_map_size_i(i) ? sc_.map_size_i(i) : 0;
        min_id_l += min_id_l < -sc_.half_map_size_i(i) ? sc_.map_size_i(i) : 0;
        int cur_dis_to_min_id = id_l(i) - min_id_l;
        cur_dis_to_min_id =
                (cur_dis_to_min_id) < 0 ? (sc_.map_size_i(i) + cur_dis_to_min_id) : cur_dis_to_min_id;
        int cur_id = cur_dis_to_min_id + min_id_g;
        id_g(i) = cur_id;
    }
}

void SlidingMap::localIndexToPos(const Vec3i &id_l, Vec3f &pos) const {
    for (int i = 0; i < 3; ++i) {
        int min_id_g = -sc_.half_map_size_i(i) + local_map_origin_i_(i);

        int min_id_l = min_id_g % sc_.map_size_i(i);
        min_id_l -= min_id_l > sc_.half_map_size_i(i) ? sc_.map_size_i(i) : 0;
        min_id_l += min_id_l < -sc_.half_map_size_i(i) ? sc_.map_size_i(i) : 0;

        int cur_dis_to_min_id = id_l(i) - min_id_l;
        cur_dis_to_min_id =
                (cur_dis_to_min_id) < 0 ? (sc_.map_size_i(i) + cur_dis_to_min_id) : cur_dis_to_min_id;
        int cur_id = cur_dis_to_min_id + min_id_g;
        pos(i) = cur_id * sc_.resolution;
    }
}

void SlidingMap::hashIdToLocalIndex(const int &hash_id, Vec3i &id) const {
    id(0) = hash_id / (sc_.map_size_i(1) * sc_.map_size_i(2));
    id(1) = (hash_id - id(0) * sc_.map_size_i(1) * sc_.map_size_i(2)) / sc_.map_size_i(2);
    id(2) = hash_id - id(0) * sc_.map_size_i(1) * sc_.map_size_i(2) - id(1) * sc_.map_size_i(2);
    id -= sc_.half_map_size_i;
}

void SlidingMap::hashIdToPos(const int &hash_id, Vec3f &pos) const {
    Vec3i id;
    hashIdToLocalIndex(hash_id, id);
    localIndexToPos(id, pos);
}

int SlidingMap::getHashIndexFromPos(const Vec3f &pos) const {
    Vec3i id_g, id_l;
    posToGlobalIndex(pos, id_g);
    globalIndexToLocalIndex(id_g, id_l);
    return getLocalIndexHash(id_l);
}

int SlidingMap::getHashIndexFromGlobalIndex(const Vec3i &id_g) const {
    Vec3i id_l;
    globalIndexToLocalIndex(id_g, id_l);
    return getLocalIndexHash(id_l);
}