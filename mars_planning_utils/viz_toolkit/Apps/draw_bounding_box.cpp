//
// Created by yunfan on 2021/8/29.
//

#include "ros/ros.h"
#include "rosbag/bag.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/ColorRGBA.h"
#include "type_utils/common_type_name.h"

using namespace type_utils;
using std::string;

static void VisualizeBoundingBox(const ros::Publisher &pub,
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
    static int id = 0;
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "fsm_node");
    ros::NodeHandle nh("~");
    double color_r, color_g, color_b;
    nh.param("color_r", color_r, 1.0);
    nh.param("color_g", color_g, 0.0);
    nh.param("color_b", color_b, 0.0);
    double bbox_max_x, bbox_max_y, bbox_max_z;
    nh.param("bbox_max_x", bbox_max_x, 1.0);
    nh.param("bbox_max_y", bbox_max_y, 1.0);
    nh.param("bbox_max_z", bbox_max_z, 1.0);
    double bbox_min_x, bbox_min_y, bbox_min_z;
    nh.param("bbox_min_x", bbox_min_x, 0.0);
    nh.param("bbox_min_y", bbox_min_y, 0.0);
    nh.param("bbox_min_z", bbox_min_z, 0.0);
    double size_x, alpha;
    nh.param("width", size_x, 0.1);
    nh.param("alpha", alpha, 1.0);


    Color color(color_r, color_g, color_b);

    ros::Publisher pub = nh.advertise<visualization_msgs::MarkerArray>("bounding_box", 1);
    ros::Rate rate(10);
    while (ros::ok()) {
        VisualizeBoundingBox(pub, Vec3f(bbox_min_x, bbox_min_y, bbox_min_z), Vec3f(bbox_max_x, bbox_max_y, bbox_max_z),
                             "test", color, size_x, alpha);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
