//
// Created by yunfan on 2022/6/25.
//

#pragma once

#include <ros/ros.h>
#include <geometry_utils/trajectory.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define TINYCOLORMAP_WITH_EIGEN

#include <visualization_utils/tinycolormap.hpp>
#include <type_utils/common_type_name.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/package.h>

namespace visualization_utils {
    using namespace geometry_utils;
    using namespace type_utils;
    using std::string;

    class VisualUtils {
    public:
        static void DeleteMkrArr(const ros::Publisher &pub_);

        static void DeleteMkr(const ros::Publisher &pub_);

        static void VisualizeText(const ros::Publisher &pub,
                                  const std::string &ns,
                                  const std::string &text,
                                  const Vec3f &position,
                                  const Color &c,
                                  const double &size = 0.1,
                                  const int &id = -1
        );


        static void VisualizeDrone(const ros::Publisher &pub,
                                   const Vec3f &pos,
                                   const Quatf &q,
                                   const std::string &meth_path,
                                   const std::string &ns = "drone",
                                   const double &scale = 1.0,
                                   const int &id = -1);

        static void VisualizeBoundingBox(const ros::Publisher &pub,
                                         const Vec3f &box_min,
                                         const Vec3f &box_max,
                                         const std::string &ns,
                                         const Color &color,
                                         const double &size_x = 0.1,
                                         const double &alpha = 1.0);

        static void VisualizeYawTrajectory(const ros::Publisher pub_,
                                           const Trajectory &pos_traj,
                                           const Trajectory &yaw_traj,
                                           std::string ns = "yaw_traj");

        static void VisualizeTrajectory(const ros::Publisher pub_, vec_E<Vec3f> &traj,
                                        Color color = Color::SteelBlue(),
                                        double eval_dt = 0.03,
                                        std::string ns = "traj",
                                        double size = 0.2);

        static void VisualizeTrajectory(const ros::Publisher pub_, Trajectory &traj,
                                        Color color = Color::SteelBlue(),
                                        double eval_dt = 0.03,
                                        std::string ns = "traj",
                                        double size = 0.2,
                                        bool visualize_vel = false,
                                        bool visualzie_acc = false);

        static void VisualizeTrajectoryInColorVel(const ros::Publisher &pub_,
                                                  Trajectory &traj,
                                                  double eval_dt = 0.03,
                                                  std::string ns = "color_traj",
                                                  double size = 0.2,
                                                  tinycolormap::ColormapType cmp = tinycolormap::ColormapType::Jet);

        static void VisualizePath(const ros::Publisher &pub_, const Mat3Df &path,
                                  Color color = Color::Pink(),
                                  std::string ns = "path",
                                  double pt_size = 0.1,
                                  double line_size = 0.05);

        static void VisualizePath(const ros::Publisher &pub_, const vec_E<Vec3f> &path,
                                  Color color = Color::Pink(),
                                  std::string ns = "path",
                                  double pt_size = 0.1,
                                  double line_size = 0.05);

        static void VisualizeMeshes(const ros::Publisher &pub_,
                                    vec_E<Mat3f> &meshes,
                                    Color surf_color = Color::SteelBlue(),
                                    std::string ns = "mesh");

        static void VisualizePoint(const ros::Publisher &pub_,
                                   const Vec3f &pt,
                                   Color color = Color::Pink(),
                                   std::string ns = "pt",
                                   double size = 0.1, int id = -1);

        static void VisualizeLine(const ros::Publisher &pub_, const Vec3f &p1, const Vec3f &p2,
                                  Color color_pt = Color::Pink(), Color color = Color::Orange(),
                                  std::string ns = "line",
                                  double pt_size = 0.1,
                                  double line_size = 0.05);

        static void VisualizePointsInPointCloud(const ros::Publisher pub_, const vec_E<Vec3f> &pts);

        static void VecVec3fToMkrArr(const ros::Publisher &pub_, vec_E<Vec3f> &path,
                                     Color color = Color::SteelBlue(),
                                     std::string ns = "line",
                                     double pt_size = 0.1,
                                     double line_size = 0.05);

        static void VisualizeCube(const ros::Publisher &pub_, const Vec3f &top_left,
                                  const Vec3f &but_right,
                                  Color color = Color::Purple(),
                                  double alpha = 0.3,
                                  std::string ns = "cube");


        static void VisualizeEllipsoid(const ros::Publisher &pub_, Mat3f R, Vec3f r,
                                       Vec3f p, Color color = Color::Orange());
    };
}

