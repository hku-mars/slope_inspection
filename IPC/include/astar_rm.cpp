#include "astar_rm.h"

namespace ipc {

double AstarClass::CalcHeu(GridNodePtr node1, GridNodePtr node2)
{
    int distance_norm = 3; // 0:Euclidean 1:Manhattan 2:L_infty 3:Diagonal 4:Dijkstra
    double h = 0;
    Eigen::Vector3i start_index = node1->index;
    Eigen::Vector3i end_index = node2->index;

    switch (distance_norm) {
        case 0: { // Euclidean
            double dx = fabs((double)(start_index(0) - end_index(0)));
            double dy = fabs((double)(start_index(1) - end_index(1)));
            double dz = fabs((double)(start_index(2) - end_index(2)));
            h = std::sqrt((std::pow(dx,2.0) + std::pow(dy,2.0) + std::pow(dz,2.0)));
            break;
        }
        case 1: { // Manhattan
            double dx = fabs((double)(start_index(0) - end_index(0)));
            double dy = fabs((double)(start_index(1) - end_index(1)));
            double dz = fabs((double)(start_index(2) - end_index(2)));
            h = dx + dy + dz;
            break;
        }
        case 2: { // L_infty
            double dx = fabs((double)(start_index(0) - end_index(0)));
            double dy = fabs((double)(start_index(1) - end_index(1)));
            double dz = fabs((double)(start_index(2) - end_index(2)));
            h = std::max(dx, std::max(dy, dz));
            break;
        }
        case 3: { // Diagonal
            double distance[3];
            distance[0] = fabs((double)(start_index(0) - end_index(0)));
            distance[1] = fabs((double)(start_index(1) - end_index(1)));
            distance[2] = fabs((double)(start_index(2) - end_index(2)));
            std::sort(distance, distance + 3);
            h = (std::sqrt(3.0) - std::sqrt(2.0))*distance[0] + (std::sqrt(3.0) - 1)*distance[1] + distance[2];
            break;
        }
        default: break;
    }

    // Tie Breaker
    h = h * tie_breaker_;
    return h;
}

void AstarClass::GetNeighbors(GridNodePtr cur, std::vector<GridNodePtr> &neighbors, std::vector<double> &costs)
{
    neighbors.clear();
    costs.clear();
    int current_x = cur->index.x();
    int current_y = cur->index.y();
    int current_z = cur->index.z();

    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            for (int k = -1; k <= 1; k++) {
                if (i == 0 && j == 0 && k == 0) continue;

                int nx = current_x + i;
                int ny = current_y + j;
                int nz = current_z + k;
                if (nx > GL_SIZE_(0)-1 || ny > GL_SIZE_(1)-1 || nz > GL_SIZE_(2)-1 || 
                    nx < 0 || ny < 0 || nz < 0) continue;
                
                Eigen::Vector3d pos = gridIndex2coord(Eigen::Vector3i(nx, ny, nz));
                // rog_map::GridType type = map_ptr_->getInfGridType(pos);
                // if (type != rog_map::KNOWN_FREE) continue;
                if (map_ptr_->isFreeInflate(pos) != true) continue;

                GridNodePtr tmp_ptr = GridNodeMap_[nx][ny][nz];
                neighbors.push_back(tmp_ptr);
                costs.push_back(std::sqrt(i*i + j*j + k*k));
            }
        }
    }
}

bool AstarClass::SearchPath(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt)
{
    ros::Time time_1 = ros::Time::now();

    if ((start_pt - end_pt).norm() <= resolution_ * 1.1) {
        ROS_ERROR("[Astar] To near! start: %f,%f,%f; end: %f,%f,%f.", 
            start_pt.x(), start_pt.y(), start_pt.z(), end_pt.x(), end_pt.y(), end_pt.z());
        return false;
    }

    // if (map_ptr_->isOccupiedInflate(start_pt)) {
    if (map_ptr_->isFreeInflate(start_pt) == false) {
    // if (map_ptr_->getInfGridType(start_pt) != rog_map::KNOWN_FREE) {
        ROS_ERROR("[Astar] Start point is in obstacle!");
        return false;
    }
    // if (map_ptr_->isOccupiedInflate(end_pt)) {
    if (map_ptr_->isFreeInflate(end_pt) == false) {
    // if (map_ptr_->getInfGridType(end_pt) != rog_map::KNOWN_FREE) {
        ROS_ERROR("[Astar] End point is in obstacle!");
        return false;
    }
    round_++;

    Eigen::Vector3i start_idx = coord2gridIndex(start_pt);
    Eigen::Vector3i end_idx   = coord2gridIndex(end_pt);
    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr   = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];
    startPtr->id = 1;
    startPtr->round = round_;
    startPtr->cameFrom = NULL;
    startPtr->gScore = 0;
    startPtr->fScore = CalcHeu(startPtr, endPtr);

    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, Compare> empty_lists;
    open_lists_.swap(empty_lists);
    open_lists_.push(startPtr);

    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;
    std::vector<GridNodePtr> neighbors;
    std::vector<double> costs;

    while (!open_lists_.empty()) {
        currentPtr = open_lists_.top();
        currentPtr->id = -1; // add current node in close_list
        open_lists_.pop();

        if (currentPtr->index == end_idx) {
            terminatePtr_ = currentPtr;
            // ROS_INFO("\033[44;37m[Astar] Time in A*  is %f ms, path cost is %f m from %f,%f,%f to %f,%f,%f.\033[0m", (ros::Time::now()-time_1).toSec()*1000.0, 
            //         currentPtr->gScore*resolution_, start_pt.x(), start_pt.y(), start_pt.z(), end_pt.x(), end_pt.y(), end_pt.z());
            return true;
        }

        GetNeighbors(currentPtr, neighbors, costs); // find neighbors and their costs

        for (int i = 0; i < neighbors.size(); i++) {
            neighborPtr = neighbors[i];
            double gh = currentPtr->gScore + costs[i];
            double fh = gh + CalcHeu(neighborPtr, endPtr);

            if (neighborPtr->round != round_) {
                neighborPtr->round = round_;
                neighborPtr->id = 0;
            }
            if (neighborPtr->id == 0) {
                neighborPtr->id = 1; // add in open list
                neighborPtr->gScore = gh;
                neighborPtr->fScore = fh;
                neighborPtr->cameFrom = currentPtr;
                open_lists_.push(neighborPtr);
            } else if (neighborPtr->id == 1) {
                if (neighborPtr->gScore > gh) { // update open list's cost
                    neighborPtr->gScore = gh;
                    neighborPtr->fScore = fh;
                    neighborPtr->cameFrom = currentPtr;
                }
            } else {
                continue;
            }
        }
        if ((ros::Time::now() - time_1).toSec() > 0.05) break;
    }
    ros::Time time_2 = ros::Time::now();
    ROS_ERROR("[Astar] A* Can not find path. Time cost %f ms. Start: %f,%f,%f, End: %f,%f,%f.", (time_2 - time_1).toSec()*1000, 
            start_pt.x(), start_pt.y(), start_pt.z(), end_pt.x(), end_pt.y(), end_pt.z());
    return false;
}

void AstarClass::GetPath(std::vector<Eigen::Vector3d> &path)
{
    path.clear();
    std::vector<GridNodePtr> gridPath;
    GridNodePtr tmp = terminatePtr_;
    while (tmp->cameFrom != NULL) {
        gridPath.push_back(tmp);
        tmp = tmp->cameFrom;
    }
    for (auto ptr : gridPath) {
        Eigen::Vector3d coord = gridIndex2coord(ptr->index);
        path.push_back(coord);
    }
    std::reverse(path.begin(), path.end());
}

void AstarClass::SimplifyPath(const std::vector<Eigen::Vector3d>& astar_path, std::vector<Eigen::Vector3d>& waypoint) {
    if (astar_path.size() <= 1) return;
    waypoint.push_back(astar_path[0]);
    if (astar_path.size() <= 2) {
        waypoint.push_back(astar_path[1]);
        return;
    }

    Eigen::Vector3d vec_last = astar_path[1] - astar_path[0];
    for (int i = 2; i < astar_path.size(); i++) {
        Eigen::Vector3d vec = astar_path[i] - astar_path[i-1];
        if (std::fabs(vec.norm()*vec_last.norm() - vec.dot(vec_last)) < 1e-5) { // The included angle of two vectors is 0
            continue;
        }
        waypoint.push_back(astar_path[i-1]);
        vec_last = vec;
    }
    waypoint.push_back(astar_path[astar_path.size() - 1]);
}

void AstarClass::FloydHandle(const std::vector<Eigen::Vector3d>& astar_path, std::vector<Eigen::Vector3d>& waypoint)
{
    waypoint.clear();
    SimplifyPath(astar_path, waypoint);
    // std::cout << "simplify size:" << waypoint.size() << std::endl;

    // generate Floyd path(twice for optimal trajectory)
    for (int time = 0; time < 3; time++) {
        for (int i = waypoint.size()-1; i > 0; i--){
            for (int j = 0; j < i-1; j++) {
                if (CheckLineFree(waypoint[i], waypoint[j]) == true) {
                    for (int k = i-1; k > j; k--) {
                        waypoint.erase(waypoint.begin()+k); // delete redundant inflection points
                    }
                    i = j;
                    break;
                }
            }
        }
    }
}

}