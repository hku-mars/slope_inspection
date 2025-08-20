#ifndef ASTAR_RM_H
#define ASTAR_RM_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <queue>

#include "rog_map/rog_map.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

namespace ipc {

constexpr double inf = 1 >> 20;
struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode {
    GridNode(){
        GridNode(Eigen::Vector3i(0, 0, 0));
    }
    GridNode(Eigen::Vector3i _index){  
		id = 0;
        round = 0;
		index = _index;
        dir = Eigen::Vector3i::Zero();
		gScore = inf;
		fScore = inf;
		cameFrom = NULL;
    }
    ~GridNode(){}

    int8_t id; // 1--> open set, -1 --> closed set
    int round;
    Eigen::Vector3i index;
    Eigen::Vector3i dir;
    double gScore;
    double fScore;
    GridNodePtr cameFrom;
};

struct Compare {
    bool operator()(const GridNodePtr a, const GridNodePtr b){
        return a->fScore > b->fScore;
    }
};

class AstarClass {
public:
    typedef std::shared_ptr<AstarClass> Ptr;
    AstarClass(){}
    ~AstarClass(){}

    void InitMap(double resolution, Eigen::Vector3d map_size, rog_map::ROGMap::Ptr rm) {
        map_ptr_ = rm;
        
        tie_breaker_ = 1.0 + 1e-5;
        round_ = 0;
        resolution_ = resolution;
        gl_low_ = -map_size / 2 - Eigen::Vector3d(resolution_/2, resolution_/2, resolution_/2);
        gl_upp_ =  map_size / 2 + Eigen::Vector3d(resolution_/2, resolution_/2, resolution_/2);
        GL_SIZE_.x() = (int)(map_size.x() / resolution_) + 1;
        GL_SIZE_.y() = (int)(map_size.y() / resolution_) + 1;
        GL_SIZE_.z() = (int)(map_size.z() / resolution_) + 1;
        std::cout << "gl_low_: " << gl_low_.transpose() << " gl_upp_: " << gl_upp_.transpose() << " GL_SIZE_: " << GL_SIZE_.transpose() << std::endl;
        
        GridNodeMap_ = new GridNodePtr ** [GL_SIZE_(0)];
        for (int i = 0; i < GL_SIZE_(0); i++) {
            GridNodeMap_[i] = new GridNodePtr * [GL_SIZE_(1)];
            for (int j = 0; j < GL_SIZE_(1); j++) {
                GridNodeMap_[i][j] = new GridNodePtr [GL_SIZE_(2)];
                for (int k = 0; k < GL_SIZE_(2); k++) {
                    Eigen::Vector3i tmpIdx(i,j,k);
                    GridNodeMap_[i][j][k] = new GridNode(tmpIdx);
                }
            }
        }
    }
    void SetCenter(Eigen::Vector3d center) {
        center_ = center;
    }
    bool CheckPointIsOccupied(const Eigen::Vector3d& pt) {
        if (map_ptr_->insideLocalMap(pt) == false) return false;
        // if (map_ptr_->isOccupiedInflate(pt) == true) return true;
        if (map_ptr_->isFreeInflate(pt) == false) return true;
        return false;
    }
    bool CheckLineFree(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
        Eigen::Vector3d vector = p2 - p1;
        int point_num = vector.norm() / resolution_;
        bool flag = true;
        for (int i = 0; i <= point_num; i++) {
            Eigen::Vector3d coor = p1 + vector * i / point_num;
            if (CheckPointIsOccupied(coor)) {
                flag = false;
                break;
            }
        }
        return flag;
    }
    bool CheckPathFree(const std::vector<Eigen::Vector3d>& path) {
        bool flag = true;
        for (int i = 0; i < path.size(); i++) {
            if (CheckPointIsOccupied(path[i])) {
                flag = false;
                break;
            }
        }
        return flag;
    }

    bool SearchPath(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
    void GetPath(std::vector<Eigen::Vector3d> &path);
    void SimplifyPath(const std::vector<Eigen::Vector3d>& astar_path, std::vector<Eigen::Vector3d>& waypoint);
    void FloydHandle(const std::vector<Eigen::Vector3d>& astar_path, std::vector<Eigen::Vector3d>& waypoint);

    double resolution_;
    Eigen::Vector3i GL_SIZE_;
    Eigen::Vector3d gl_low_, gl_upp_, center_, start_pt_, end_pt_;

private:
    inline Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i& index) {
        Eigen::Vector3d pt;
        pt(0) = index(0) * resolution_ + resolution_/2 + gl_low_(0) + center_(0);
        pt(1) = index(1) * resolution_ + resolution_/2 + gl_low_(1) + center_(1);
        pt(2) = index(2) * resolution_ + resolution_/2 + gl_low_(2) + center_(2);
        return pt;
    }
    inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d& pt) {
        Eigen::Vector3d c_pt = pt - center_;
        Eigen::Vector3i index;
        index(0) = std::min(std::max(int((c_pt(0) - gl_low_(0)) / resolution_), 0), GL_SIZE_(0)-1);
        index(1) = std::min(std::max(int((c_pt(1) - gl_low_(1)) / resolution_), 0), GL_SIZE_(1)-1);
        index(2) = std::min(std::max(int((c_pt(2) - gl_low_(2)) / resolution_), 0), GL_SIZE_(2)-1);
        return index;
    }

    double CalcHeu(GridNodePtr node1, GridNodePtr node2);
    void GetNeighbors(GridNodePtr cur, std::vector<GridNodePtr> &neighbors, std::vector<double> &costs);

    rog_map::ROGMap::Ptr map_ptr_;
    double tie_breaker_;
    int round_;
    GridNodePtr ***GridNodeMap_;
    GridNodePtr terminatePtr_;
    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, Compare> open_lists_;
};

}

#endif
