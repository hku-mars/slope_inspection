//
// Created by yunfan on 2022/6/25.
//

#include <visualization_utils/visualization_utils.h>

using std::string;
using namespace geometry_utils;
using namespace visualization_utils;
using namespace type_utils;

void VisualUtils::DeleteMkrArr(const ros::Publisher &pub_) {
    visualization_msgs::Marker del;
    visualization_msgs::MarkerArray arr;
    del.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(del);
    pub_.publish(arr);
}

void VisualUtils::DeleteMkr(const ros::Publisher &pub_) {
    visualization_msgs::Marker del;
    del.action = visualization_msgs::Marker::DELETEALL;
    pub_.publish(del);
}

void VisualUtils::VisualizeText(const ros::Publisher &pub,
                                const std::string &ns,
                                const std::string &text,
                                const Vec3f &position,
                                const Color &c,
                                const double &size,
                                const int &id
) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.ns = ns.c_str();
    if (id >= 0) {
        marker.id = id;
    } else {
        static int id = 0;
        marker.id = id++;
    }
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.scale.z = size;
    marker.color = c;
    marker.text = text;
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.pose.orientation.w = 1.0;
    visualization_msgs::MarkerArray arr;
    arr.markers.push_back(marker);
    pub.publish(arr);
};


void VisualUtils::VisualizeDrone(const ros::Publisher &pub, const Vec3f &pos, const Quatf &q,
                                 const string &mesh_path, const string &ns, const double &scale,
                                 const int &id) {

    visualization_msgs::Marker meshROS;
    // Mesh model
    meshROS.header.frame_id = "world";
    meshROS.header.stamp = ros::Time::now();
    meshROS.ns = ns;
    static int mesh_id = 0;
    if (id > 0)
        meshROS.id = id;
    else
        meshROS.id = mesh_id++;
    meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
    meshROS.action = visualization_msgs::Marker::ADD;
    meshROS.pose.position.x = pos.x();
    meshROS.pose.position.y = pos.y();
    meshROS.pose.position.z = pos.z();
    meshROS.pose.orientation.w = q.w();
    meshROS.pose.orientation.x = q.x();
    meshROS.pose.orientation.y = q.y();
    meshROS.pose.orientation.z = q.z();
    meshROS.scale.x = scale;
    meshROS.scale.y = scale;
    meshROS.scale.z = scale;
    meshROS.color.a = 1.0;
    meshROS.color.r = 1.0;
    meshROS.color.g = 1.0;
    meshROS.color.b = 1.0;
    meshROS.mesh_resource = mesh_path;
    meshROS.mesh_use_embedded_materials = true;
    pub.publish(meshROS);

}

void VisualUtils::VisualizeBoundingBox(const ros::Publisher &pub,
                                       const Vec3f &box_min,
                                       const Vec3f &box_max,
                                       const string &ns,
                                       const Color &color,
                                       const double &size_x,
                                       const double &alpha) {
    Vec3f size = (box_max - box_min) / 2;
    Vec3f vis_pos_world = (box_min + box_max) / 2;
    double width = size.x();
    double length = size.y();
    double hight = size.z();
    visualization_msgs::MarkerArray mkrarr;
//Publish Bounding box
    int id = 0;
    visualization_msgs::Marker line_strip;
    line_strip.header.stamp = ros::Time::now();
    line_strip.header.frame_id = "world";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.ns = ns;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = id++; //unique id, useful when multiple markers exist.
    line_strip.type = visualization_msgs::Marker::LINE_STRIP; //marker type
    line_strip.scale.x = size_x;


    line_strip.color = color;
    line_strip.color.a = alpha; //不透明度，设0则全透明
    geometry_msgs::Point p[8];

    //vis_pos_world是目标物的坐标
    p[0].x = vis_pos_world(0) - width;
    p[0].y = vis_pos_world(1) + length;
    p[0].z = vis_pos_world(2) + hight;
    p[1].x = vis_pos_world(0) - width;
    p[1].y = vis_pos_world(1) - length;
    p[1].z = vis_pos_world(2) + hight;
    p[2].x = vis_pos_world(0) - width;
    p[2].y = vis_pos_world(1) - length;
    p[2].z = vis_pos_world(2) - hight;
    p[3].x = vis_pos_world(0) - width;
    p[3].y = vis_pos_world(1) + length;
    p[3].z = vis_pos_world(2) - hight;
    p[4].x = vis_pos_world(0) + width;
    p[4].y = vis_pos_world(1) + length;
    p[4].z = vis_pos_world(2) - hight;
    p[5].x = vis_pos_world(0) + width;
    p[5].y = vis_pos_world(1) - length;
    p[5].z = vis_pos_world(2) - hight;
    p[6].x = vis_pos_world(0) + width;
    p[6].y = vis_pos_world(1) - length;
    p[6].z = vis_pos_world(2) + hight;
    p[7].x = vis_pos_world(0) + width;
    p[7].y = vis_pos_world(1) + length;
    p[7].z = vis_pos_world(2) + hight;
    //LINE_STRIP类型仅仅将line_strip.points中相邻的两个点相连，如0和1，1和2，2和3
    for (int i = 0; i < 8; i++) {
        line_strip.points.push_back(p[i]);
    }
    //为了保证矩形框的八条边都存在：
    line_strip.points.push_back(p[0]);
    line_strip.points.push_back(p[3]);
    line_strip.points.push_back(p[2]);
    line_strip.points.push_back(p[5]);
    line_strip.points.push_back(p[6]);
    line_strip.points.push_back(p[1]);
    line_strip.points.push_back(p[0]);
    line_strip.points.push_back(p[7]);
    line_strip.points.push_back(p[4]);
    mkrarr.markers.push_back(line_strip);
    pub.publish(mkrarr);
}


void VisualUtils::VisualizeYawTrajectory(const ros::Publisher pub_,
                                         const Trajectory &pos_traj,
                                         const Trajectory &yaw_traj,
                                         string ns) {
    double t_sum = yaw_traj.getTotalDuration();
    double eval_t = 0.001;
    Vec3f cur_pos, end_point;
    tinycolormap::ColormapType cmp = tinycolormap::ColormapType::Jet;
    visualization_msgs::Marker line_list;
    visualization_msgs::MarkerArray mrkarr;
    mrkarr.markers.clear();

    while (eval_t + 1e-4 < t_sum) {
        visualization_msgs::Marker line_list;
        cur_pos = pos_traj.getPos(eval_t);
        double cur_yaw = yaw_traj.getPos(eval_t)[0];
        {
            static int cnt = 0;
            // publish lines
            visualization_msgs::Marker line_list;
            line_list.header.frame_id = "world";
            line_list.header.stamp = ros::Time::now();
            line_list.ns = ns;
            line_list.id = cnt++;
            line_list.action = visualization_msgs::Marker::ADD;
            line_list.pose.orientation.w = 1.0;
            line_list.type = visualization_msgs::Marker::ARROW;
            // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
            line_list.scale.x = 0.05;
            line_list.scale.y = 0.1;
            line_list.scale.z = 0.2;
            // Line list is blue
            // Line list is blue
            double color_id = ((eval_t / t_sum));

            Eigen::Vector3d color_mag = tinycolormap::GetColor(color_id, cmp).ConvertToEigen();
            line_list.color.r = color_mag[0];
            line_list.color.g = color_mag[1];
            line_list.color.b = color_mag[2];
            line_list.color.a = 1;
            // Create the vertices for the points and lines
            Vec3f p2 = cur_pos + Vec3f(cos(cur_yaw), sin(cur_yaw), 0);
            geometry_msgs::Point p;
            p.x = cur_pos.x();
            p.y = cur_pos.y();
            p.z = cur_pos.z();
            // The line list needs two points for each line
            line_list.points.push_back(p);
            p.x = p2.x();
            p.y = p2.y();
            p.z = p2.z();
            // The line list needs
            line_list.points.push_back(p);

            mrkarr.markers.push_back(line_list);
        }
        eval_t += 0.05;
    }
    pub_.publish(mrkarr);
}

void VisualUtils::VisualizeTrajectory(const ros::Publisher pub_, vec_E<Vec3f> &traj,
                                      Color color,
                                      double eval_dt,
                                      string ns,
                                      double size
) {
    Vec3f last_pos, cur_pos, end_point;
    visualization_msgs::Marker line_list;
    visualization_msgs::MarkerArray mrkarr;
    mrkarr.markers.clear();

    if (traj.empty()) {
        return;
    }
    last_pos = traj[0];
    for (auto cur_pos: traj) {
        visualization_msgs::Marker line_list;
        {
            static int cnt = 0;
            // publish lines
            visualization_msgs::Marker line_list;
            line_list.header.frame_id = "world";
            line_list.header.stamp = ros::Time::now();
            line_list.ns = ns;
            line_list.id = cnt++;
            line_list.action = visualization_msgs::Marker::ADD;
            line_list.pose.orientation.w = 1.0;
            line_list.type = visualization_msgs::Marker::ARROW;
            // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
            line_list.scale.x = size;
            line_list.scale.y = 0.0000001;
            line_list.scale.z = 0.0000001;
            // Line list is blue
            line_list.color = color;
            line_list.color.a = 1;
            // Create the vertices for the points and lines

            geometry_msgs::Point p;
            p.x = last_pos.x();
            p.y = last_pos.y();
            p.z = last_pos.z();
            // The line list needs two points for each line
            line_list.points.push_back(p);
            p.x = cur_pos.x();
            p.y = cur_pos.y();
            p.z = cur_pos.z();
            // The line list needs
            line_list.points.push_back(p);

            mrkarr.markers.push_back(line_list);
        }
        last_pos = cur_pos;
    }
    pub_.publish(mrkarr);
}

void VisualUtils::VisualizeTrajectory(const ros::Publisher pub_, Trajectory &traj,
                                      Color color,
                                      double eval_dt,
                                      string ns,
                                      double size,
                                      bool visualize_vel,
                                      bool visualize_acc
) {

    double t_sum = traj.getTotalDuration();
    double eval_t = 0.001;
    Vec3f last_pos, cur_pos, end_point;

    visualization_msgs::Marker line_list, vel_list, acc_list;

    line_list.header.frame_id = "world";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = ns;

    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.type = visualization_msgs::Marker::ARROW;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_list.scale.x = size;
    line_list.scale.y = 0.0000001;
    line_list.scale.z = 0.0000001;
    // Line list is blue
    line_list.color = color;
    line_list.color.a = 1;

    if (visualize_vel) {
        vel_list.header.frame_id = "world";
        vel_list.header.stamp = ros::Time::now();
        vel_list.ns = ns + "_vel";

        vel_list.action = visualization_msgs::Marker::ADD;
        vel_list.pose.orientation.w = 1.0;
        vel_list.type = visualization_msgs::Marker::ARROW;
        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        vel_list.scale.x = size;
        vel_list.scale.y = 0.0000001;
        vel_list.scale.z = 0.0000001;
        // Line list is blue
        vel_list.color.r = 1;
        vel_list.color.g = 0;
        vel_list.color.b = 0;
        vel_list.color.a = 0.5;
    }

    if (visualize_acc) {
        acc_list.header.frame_id = "world";
        acc_list.header.stamp = ros::Time::now();
        acc_list.ns = ns + "_acc";

        acc_list.action = visualization_msgs::Marker::ADD;
        acc_list.pose.orientation.w = 1.0;
        acc_list.type = visualization_msgs::Marker::ARROW;
        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        acc_list.scale.x = size;
        acc_list.scale.y = 0.0000001;
        acc_list.scale.z = 0.0000001;
        // Line list is blue
        acc_list.color.r = 0;
        acc_list.color.g = 0;
        acc_list.color.b = 1;
        acc_list.color.a = 0.5;
    }


    // Create the vertices for the points and lines
    visualization_msgs::MarkerArray mrkarr;
    mrkarr.markers.clear();

    last_pos = traj.getPos(0);
    while (eval_t + 1e-4 < t_sum) {
        static int cnt = 0;
        cur_pos = traj.getPos(eval_t);;
        line_list.id = cnt++;
        line_list.points.clear();

        geometry_msgs::Point p;
        p.x = last_pos.x();
        p.y = last_pos.y();
        p.z = last_pos.z();
        // The line list needs two points for each line
        line_list.points.push_back(p);
        p.x = cur_pos.x();
        p.y = cur_pos.y();
        p.z = cur_pos.z();
        // The line list needs
        line_list.points.push_back(p);

        mrkarr.markers.push_back(line_list);

        if (visualize_vel) {
            Vec3f cur_vel = traj.getVel(eval_t);
            double v_n = cur_vel.norm();
            Vec3f cur_vel_dir = cur_vel.cross(
                    Vec3f(0, 0, 1)).normalized();// (rot_ang.toRotationMatrix()*cur_vel ).normalized();
            Vec3f end_point = v_n * cur_vel_dir;
            vel_list.id = cnt;
            vel_list.points.clear();
            geometry_msgs::Point p;
            p.x = cur_pos.x();
            p.y = cur_pos.y();
            p.z = cur_pos.z();
            // The line list needs two points for each line
            vel_list.points.push_back(p);
            p.x += end_point.x();
            p.y += end_point.y();
            p.z += end_point.z();
            vel_list.points.push_back(p);

            mrkarr.markers.push_back(vel_list);
        }

        if (visualize_acc) {
            Vec3f cur_acc = traj.getVel(eval_t);
            double a_n = cur_acc.norm();
            Vec3f cur_acc_dir = -cur_acc.cross(
                    Vec3f(0, 0, 1)).normalized();// (rot_ang.toRotationMatrix()*cur_vel ).normalized();
            end_point = a_n * cur_acc_dir;
            acc_list.id = cnt;
            acc_list.points.clear();
            geometry_msgs::Point p;
            p.x = cur_pos.x();
            p.y = cur_pos.y();
            p.z = cur_pos.z();
            // The line list needs two points for each line
            acc_list.points.push_back(p);
            p.x += end_point.x();
            p.y += end_point.y();
            p.z += end_point.z();
            acc_list.points.push_back(p);

            mrkarr.markers.push_back(acc_list);
        }
        last_pos = cur_pos;
        eval_t += eval_dt;
    }
    pub_.publish(mrkarr);
}

void VisualUtils::VisualizeTrajectoryInColorVel(const ros::Publisher &pub_,
                                                Trajectory &traj,
                                                double eval_dt,
                                                string ns,
                                                double size,
                                                tinycolormap::ColormapType cmp) {
    double t_sum = traj.getTotalDuration();
    double eval_t = 0.001;
    Vec3f cur_vel_dir, last_pos, cur_pos, end_point;
    Vec3f cur_vel;
    visualization_msgs::Marker line_list;
    visualization_msgs::MarkerArray mrkarr;
    double vel_max = traj.getMaxVelRate();
    double map_factor = 1.0 / vel_max;
    mrkarr.markers.clear();

    last_pos = traj.getPos(0);
    while (eval_t + 1e-4 < t_sum) {
        visualization_msgs::Marker line_list;
        cur_pos = traj.getPos(eval_t);
        if ((cur_pos - last_pos).norm() < 0.1) {
            eval_t += eval_dt;
            continue;
        }
        cur_vel = traj.getVel(eval_t);
        {
            static int cnt = 0;
            // publish lines
            visualization_msgs::Marker line_list;
            line_list.header.frame_id = "world";
            line_list.header.stamp = ros::Time::now();
            line_list.ns = ns;
            line_list.id = cnt++;
            line_list.action = visualization_msgs::Marker::ADD;
            line_list.pose.orientation.w = 1.0;
            line_list.type = visualization_msgs::Marker::ARROW;
            // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
            line_list.scale.x = size;
            line_list.scale.y = 0.0000001;
            line_list.scale.z = 0.0000001;
            // Line list is blue
            double color_id = ((cur_vel.norm() * map_factor));
            Eigen::Vector3d color_mag = tinycolormap::GetColor(color_id, cmp).ConvertToEigen();
            line_list.color.r = color_mag[0];
            line_list.color.g = color_mag[1];
            line_list.color.b = color_mag[2];
            line_list.color.a = 1;
            // Create the vertices for the points and lines

            geometry_msgs::Point p;
            p.x = last_pos.x();
            p.y = last_pos.y();
            p.z = last_pos.z();
            // The line list needs two points for each line
            line_list.points.push_back(p);
            p.x = cur_pos.x();
            p.y = cur_pos.y();
            p.z = cur_pos.z();
            // The line list needs
            line_list.points.push_back(p);

            mrkarr.markers.push_back(line_list);
        }
        last_pos = cur_pos;
        eval_t += eval_dt;
    }
    pub_.publish(mrkarr);
}

void VisualUtils::VisualizePath(const ros::Publisher &pub_, const Mat3Df &path,
                                Color color,
                                string ns,
                                double pt_size,
                                double line_size) {
    visualization_msgs::Marker line_list;
    visualization_msgs::MarkerArray mkr_ary;
    if (path.size() <= 0) {
        std::cout << RED << "Try to publish empty path, return.\n" << RESET << std::endl;
        return;
    }
    Vec3f cur_pt = path.col(0), last_pt;
    static int point_id = 0;
    static int line_cnt = 0;
    for (long int i = 0; i < path.cols(); i++) {
        last_pt = cur_pt;
        cur_pt = path.col(i);

        /* Publish point */
        visualization_msgs::Marker point;
        point.header.frame_id = "world";
        point.header.stamp = ros::Time::now();
        point.ns = ns.c_str();
        point.id = point_id++;
        point.action = visualization_msgs::Marker::ADD;
        point.pose.orientation.w = 1.0;
        point.type = visualization_msgs::Marker::SPHERE;
        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        point.scale.x = pt_size;
        point.scale.y = pt_size;
        point.scale.z = pt_size;
        // Line list is blue
        point.color = color;
        point.color.a = 1.0;
        // Create the vertices for the points and lines
        geometry_msgs::Point p;
        p.x = cur_pt.x();
        p.y = cur_pt.y();
        p.z = cur_pt.z();
        point.pose.position = p;
        mkr_ary.markers.push_back(point);
        /* publish lines */
        if (i > 0) {
            geometry_msgs::Point p;
            // publish lines
            visualization_msgs::Marker line_list;
            line_list.header.frame_id = "world";
            line_list.header.stamp = ros::Time::now();
            line_list.ns = ns + "_line";
            line_list.id = line_cnt++;
            line_list.action = visualization_msgs::Marker::ADD;
            line_list.pose.orientation.w = 1.0;
            line_list.type = visualization_msgs::Marker::LINE_LIST;
            // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
            line_list.scale.x = line_size;
            // Line list is blue
            line_list.color = color;
            // Create the vertices for the points and lines

            p.x = last_pt.x();
            p.y = last_pt.y();
            p.z = last_pt.z();
            // The line list needs two points for each line
            line_list.points.push_back(p);
            p.x = cur_pt.x();
            p.y = cur_pt.y();
            p.z = cur_pt.z();
            // The line list needs
            line_list.points.push_back(p);

            mkr_ary.markers.push_back(line_list);
        }
    }
    pub_.publish(mkr_ary);
}

void VisualUtils::VisualizePath(const ros::Publisher &pub_, const vec_E<Vec3f> &path,
                                Color color,
                                string ns,
                                double pt_size,
                                double line_size) {
    visualization_msgs::Marker line_list;
    visualization_msgs::MarkerArray mkr_ary;
    if (path.size() <= 0) {
        std::cout << RED << "Try to publish empty path, return.\n" << RESET << std::endl;
        return;
    }
    Vec3f cur_pt = path[0], last_pt;
    static int point_id = 0;
    static int line_cnt = 0;
    for (size_t i = 0; i < path.size(); i++) {
        last_pt = cur_pt;
        cur_pt = path[i];

        /* Publish point */
        visualization_msgs::Marker point;
        point.header.frame_id = "world";
        point.header.stamp = ros::Time::now();
        point.ns = ns.c_str();
        point.id = point_id++;
        point.action = visualization_msgs::Marker::ADD;
        point.pose.orientation.w = 1.0;
        point.type = visualization_msgs::Marker::SPHERE;
        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        point.scale.x = pt_size;
        point.scale.y = pt_size;
        point.scale.z = pt_size;
        // Line list is blue
        point.color = color;
        point.color.a = 1.0;
        // Create the vertices for the points and lines
        geometry_msgs::Point p;
        p.x = cur_pt.x();
        p.y = cur_pt.y();
        p.z = cur_pt.z();
        point.pose.position = p;
        mkr_ary.markers.push_back(point);
        /* publish lines */
        if (i > 0) {
            geometry_msgs::Point p;
            // publish lines
            visualization_msgs::Marker line_list;
            line_list.header.frame_id = "world";
            line_list.header.stamp = ros::Time::now();
            line_list.ns = ns + "_line";
            line_list.id = line_cnt++;
            line_list.action = visualization_msgs::Marker::ADD;
            line_list.pose.orientation.w = 1.0;
            line_list.type = visualization_msgs::Marker::LINE_LIST;
            // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
            line_list.scale.x = line_size;
            // Line list is blue
            line_list.color = color;
            // Create the vertices for the points and lines

            p.x = last_pt.x();
            p.y = last_pt.y();
            p.z = last_pt.z();
            // The line list needs two points for each line
            line_list.points.push_back(p);
            p.x = cur_pt.x();
            p.y = cur_pt.y();
            p.z = cur_pt.z();
            // The line list needs
            line_list.points.push_back(p);

            mkr_ary.markers.push_back(line_list);
        }
    }
    pub_.publish(mkr_ary);
}

void VisualUtils::VisualizeMeshes(const ros::Publisher &pub_,
                                  vec_E<Mat3f> &meshes,
                                  Color surf_color,
                                  string ns) {
    // RVIZ support tris for visualization
    visualization_msgs::Marker meshMarker, edgeMarker;
    static int mesh_id = 0;
    meshMarker.id = mesh_id++;
    meshMarker.header.stamp = ros::Time::now();
    meshMarker.header.frame_id = "world";
    meshMarker.pose.orientation.w = 1.00;
    meshMarker.action = visualization_msgs::Marker::ADD;
    meshMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    meshMarker.ns = ns;
    meshMarker.color = surf_color;
    meshMarker.color.a = 0.15;
    meshMarker.scale.x = 1.0;
    meshMarker.scale.y = 1.0;
    meshMarker.scale.z = 1.0;
    geometry_msgs::Point point;

    for (long unsigned int i = 0; i < meshes.size(); i++) {
        for (int j = 0; j < 3; j++) {
            point.x = meshes[i].col(j)[0];
            point.y = meshes[i].col(j)[1];
            point.z = meshes[i].col(j)[2];
            meshMarker.points.push_back(point);
        }
    }

    pub_.publish(meshMarker);
}

void VisualUtils::VisualizePoint(const ros::Publisher &pub_,
                                 const Vec3f &pt,
                                 Color color,
                                 string ns,
                                 double size, int id) {
    visualization_msgs::MarkerArray mkr_arr;
    visualization_msgs::Marker marker_ball;
    static int cnt = 0;
    Vec3f cur_pos = pt;
    if (isnan(pt.x()) || isnan(pt.y()) || isnan(pt.z())) {
        return;
    }
    marker_ball.header.frame_id = "world";
    marker_ball.header.stamp = ros::Time::now();
    marker_ball.ns = ns.c_str();
    marker_ball.id = id >= 0 ? id : cnt++;
    marker_ball.action = visualization_msgs::Marker::ADD;
    marker_ball.pose.orientation.w = 1.0;
    marker_ball.type = visualization_msgs::Marker::SPHERE;
    marker_ball.scale.x = size;
    marker_ball.scale.y = size;
    marker_ball.scale.z = size;
    marker_ball.color = color;

    geometry_msgs::Point p;
    p.x = cur_pos.x();
    p.y = cur_pos.y();
    p.z = cur_pos.z();

    marker_ball.pose.position = p;
    mkr_arr.markers.push_back(marker_ball);
    pub_.publish(mkr_arr);
}

void
VisualUtils::VisualizeLine(const ros::Publisher &pub_, const Vec3f &p1, const Vec3f &p2, Color color_pt,
                           Color color,
                           string ns,
                           double pt_size,
                           double line_size) {
    visualization_msgs::MarkerArray mkr_arr;
    static int point_id = 0;
    static int line_cnt = 0;
    /* Publish point */
    visualization_msgs::Marker point;
    point.header.frame_id = "world";
    point.header.stamp = ros::Time::now();
    point.ns = ns.c_str();
    point.id = point_id++;
    point.action = visualization_msgs::Marker::ADD;
    point.pose.orientation.w = 1.0;
    point.type = visualization_msgs::Marker::SPHERE;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    point.scale.x = pt_size;
    point.scale.y = pt_size;
    point.scale.z = pt_size;
    // Line list is blue
    point.color = color_pt;
    // Create the vertices for the points and lines
    geometry_msgs::Point p;
    p.x = p1.x();
    p.y = p1.y();
    p.z = p1.z();
    point.pose.position = p;
    mkr_arr.markers.push_back(point);

    p.x = p2.x();
    p.y = p2.y();
    p.z = p2.z();
    point.pose.position = p;
    point.id = point_id++;
    mkr_arr.markers.push_back(point);

    // publish lines
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "world";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = ns + "line";
    line_list.id = line_cnt++;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_list.scale.x = line_size;
    // Line list is blue
    line_list.color = color;
    // Create the vertices for the points and lines

    p.x = p1.x();
    p.y = p1.y();
    p.z = p1.z();
    // The line list needs two points for each line
    line_list.points.push_back(p);
    p.x = p2.x();
    p.y = p2.y();
    p.z = p2.z();
    // The line list needs
    line_list.points.push_back(p);

    mkr_arr.markers.push_back(line_list);

    pub_.publish(mkr_arr);
}

void VisualUtils::VisualizePointsInPointCloud(const ros::Publisher pub_, const vec_E<Vec3f> &pts) {
    pcl::PointCloud<pcl::PointXYZ> pc;
    for (auto it: pts) {
        pc.push_back(pcl::PointXYZ(it.x(), it.y(), it.z()));
    }
    pc.header.frame_id = "world";
    sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg(pc, pc2);
    pc2.header.frame_id = "world";
    pc2.header.stamp = ros::Time::now();
    pub_.publish(pc2);
}

void VisualUtils::VecVec3fToMkrArr(const ros::Publisher &pub_, vec_E<Vec3f> &path,
                                   Color color,
                                   string ns,
                                   double pt_size,
                                   double line_size) {
    visualization_msgs::Marker line_list;
    visualization_msgs::MarkerArray mkr_ary;
    if (path.size() <= 0) {
        std::cout << RED << "Try to publish empty path, return.\n" << RESET << std::endl;
        return;
    }
    Vec3f cur_pt = path[0], last_pt;
    static int point_id = 0;
    static int line_cnt = 0;
    for (size_t i = 0; i < path.size(); i++) {
        last_pt = cur_pt;
        cur_pt = path[i];

/* Publish point */
        visualization_msgs::Marker point;
        point.header.frame_id = "world";
        point.header.stamp = ros::Time::now();
        point.ns = ns.c_str();
        point.id = point_id++;
        point.action = visualization_msgs::Marker::ADD;
        point.pose.orientation.w = 1.0;
        point.type = visualization_msgs::Marker::SPHERE;
// LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        point.scale.x = pt_size;
        point.scale.y = pt_size;
        point.scale.z = pt_size;
// Line list is blue
        point.color = color;
        point.color.a = 1;

// Create the vertices for the points and lines
        geometry_msgs::Point p;
        p.x = cur_pt.x();
        p.y = cur_pt.y();
        p.z = cur_pt.z();
        point.pose.position = p;
        mkr_ary.markers.push_back(point);
/* publish lines */
        if (i > 0) {
            geometry_msgs::Point p;
// publish lines
            visualization_msgs::Marker line_list;
            line_list.header.frame_id = "world";
            line_list.header.stamp = ros::Time::now();
            line_list.ns = ns + "_line";
            line_list.id = line_cnt++;
            line_list.action = visualization_msgs::Marker::ADD;
            line_list.pose.orientation.w = 1.0;
            line_list.type = visualization_msgs::Marker::LINE_LIST;
// LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
            line_list.scale.x = line_size;
// Line list is blue
            line_list.color = color;
            line_list.color.a = 1;
// Create the vertices for the points and lines

            p.x = last_pt.x();
            p.y = last_pt.y();
            p.z = last_pt.z();
// The line list needs two points for each line
            line_list.points.push_back(p);
            p.x = cur_pt.x();
            p.y = cur_pt.y();
            p.z = cur_pt.z();
// The line list needs
            line_list.points.push_back(p);

            mkr_ary.markers.push_back(line_list);
        }
    }
    pub_.publish(mkr_ary);
}

void VisualUtils::VisualizeCube(const ros::Publisher &pub_, const Vec3f &top_left,
                                const Vec3f &but_right,
                                Color color,
                                double alpha,
                                string ns) {
    Vec3f center = (top_left + but_right) / 2;
    Vec3f size = (top_left - but_right).cwiseAbs();
    visualization_msgs::Marker mkr;
    static int cube_id = 0;
    mkr.header.frame_id = "world";
    mkr.header.stamp = ros::Time::now();
    mkr.ns = ns.c_str();
    mkr.id = cube_id++;
    mkr.action = visualization_msgs::Marker::ADD;
    mkr.pose.orientation.w = 1.0;
    mkr.type = visualization_msgs::Marker::CUBE;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    mkr.scale.x = size.x();
    mkr.scale.y = size.y();
    mkr.scale.z = size.z();
    // Line list is blue
    mkr.color = color;
    mkr.color.a = alpha;
    // Create the vertices for the points and lines
    geometry_msgs::Point p;
    p.x = center.x();
    p.y = center.y();
    p.z = center.z();
    mkr.pose.position = p;

    visualization_msgs::MarkerArray mkr_arr;
    mkr_arr.markers.push_back(mkr);
    pub_.publish(mkr_arr);

}


void
VisualUtils::VisualizeEllipsoid(const ros::Publisher &pub_, Mat3f R, Vec3f r, Vec3f p, Color color) {
    visualization_msgs::Marker mkr;
    Eigen::Quaterniond q(R);
    static int id = 0;
    mkr.id = id++;
    mkr.type = visualization_msgs::Marker::SPHERE;
    mkr.header.frame_id = "world";
    mkr.header.stamp = ros::Time::now();
    mkr.ns = "ellp";
    mkr.id = id++;
    mkr.action = visualization_msgs::Marker::ADD;
    mkr.pose.orientation.w = q.w();
    mkr.pose.orientation.x = q.x();
    mkr.pose.orientation.y = q.y();
    mkr.pose.orientation.z = q.z();
    mkr.pose.position.x = p.x();
    mkr.pose.position.y = p.y();
    mkr.pose.position.z = p.z();
    mkr.scale.x = r.x() * 2;
    mkr.scale.y = r.y() * 2;
    mkr.scale.z = r.z() * 2;
    mkr.color = color;
    mkr.color.a = 0.3;
    visualization_msgs::MarkerArray mkr_arr;
    mkr_arr.markers.push_back(mkr);

    mkr.type = visualization_msgs::Marker::SPHERE;
    mkr.header.frame_id = "world";
    mkr.header.stamp = ros::Time::now();
    mkr.ns = "center";
    mkr.id = id++;
    mkr.action = visualization_msgs::Marker::ADD;
    mkr.pose.orientation.w = 1.0;
    mkr.pose.orientation.x = 0.0;
    mkr.pose.orientation.y = 0.0;
    mkr.pose.orientation.z = 0.0;

    mkr.pose.position.x = p.x();
    mkr.pose.position.y = p.y();
    mkr.pose.position.z = p.z();
    mkr.scale.x = 0.1;
    mkr.scale.y = 0.1;
    mkr.scale.z = 0.1;
    mkr.color = color;
    mkr.color.a = 1.0;
    mkr_arr.markers.push_back(mkr);

    pub_.publish(mkr_arr);

}
