//
// Created by yunfan on 2021/8/29.
//

#include "ros/ros.h"
#include "rosbag/bag.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/ColorRGBA.h"
#include "type_utils/common_type_name.h"
#include "visualization_utils/visualization_utils.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "ros/package.h"
#include "nav_msgs/Odometry.h"

using namespace type_utils;
using namespace visualization_utils;
using std::string;
using std::vector;
using std::cout;
using std::endl;

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;


class PointCloudDrawer {
public:
    PointCloudDrawer() = default;

    ~PointCloudDrawer() = default;

    void AddBox(const Vec3f &box_min, const Vec3f &box_max,
                const double &resolution, PointCloud &out_pc) {
        for (double x = box_min.x(); x <= box_max.x(); x += resolution) {
            for (double y = box_min.y(); y <= box_max.y(); y += resolution) {
                for (double z = box_min.z(); z <= box_max.z(); z += resolution) {
                    PointType pt;
                    pt.x = x;
                    pt.y = y;
                    pt.z = z;
                    pt.r = 255;
                    pt.g = 255;
                    pt.b = 255;
                    out_pc.push_back(pt);
                }
            }
        }
    }

    void ColorPointCloudByHeight(const double &height_max, const double &height_min,
                                 const tinycolormap::ColormapType &cmp, PointCloud &pc) {
        auto getRatio = [&](const double &height) -> double {

            double ratio_ = (height - height_min) / (height_max - height_min);
            ratio_ = std::max(0.0, std::min(1.0, ratio_));
            return ratio_;
        };


        for (auto &p: pc) {
            double ratio = getRatio(p.z);
            auto color = tinycolormap::GetColor(ratio, cmp);
            p.r = color.r() * 255;
            p.g = color.g() * 255;
            p.b = color.b() * 255;
        }
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "draw_pointcloud_map");
    ros::NodeHandle n("~");

    PointCloud pc;
    PointCloudDrawer drawer;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // add first
    drawer.AddBox(Vec3f(5 - 0.25, -0.25, 0), Vec3f(5 + 0.25, 0.25, 5), 0.05, pc);
    // add gate
    drawer.AddBox(Vec3f(10 - 0.25, 2.5 - 0.25, 0), Vec3f(10 + 0.25, 2.5 + 0.25, 2.5), 0.05, pc);
    drawer.AddBox(Vec3f(10 - 0.25, -2.5 - 0.25, 0), Vec3f(10 + 0.25, -2.5 + 0.25, 2.5), 0.05, pc);
    drawer.AddBox(Vec3f(10 - 0.25, -2.5 - 0.25, 2.5 - 0.25), Vec3f(10 + 0.25, 2.5 + 0.25, 2.5 + 0.25), 0.05, pc);
    drawer.ColorPointCloudByHeight(0, 5, tinycolormap::ColormapType::Jet, pc);

    ros::Publisher pc_pub = n.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 1);
    ros::Rate rate(1);

    sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg(pc, pc2);
    pc2.header.frame_id = "world";
    pc2.header.stamp = ros::Time::now();
    pc2.height = 1;
    pc2.width = pc.size();

    cout<<pc.size()<<endl;

    while (ros::ok()) {
        pc_pub.publish(pc2);
        rate.sleep();
    }


    ros::waitForShutdown();
    return 0;
}
