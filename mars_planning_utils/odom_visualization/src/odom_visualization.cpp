#include <iostream>
#include <string.h>
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/Range.h"
#include "visualization_msgs/Marker.h"
#include "armadillo"
#include "quadrotor_msgs/PositionCommand.h"
#include "quadrotor_msgs/PolynomialTrajectory.h"
#include "mutex"
#include "geometry_utils/trajectory.h"
#include "visualization_utils/visualization_utils.h"

using namespace arma;
using namespace std;

static string mesh_resource;
static double color_r, color_g, color_b, color_a, cov_scale, scale;

bool cross_config = false;
bool tf45 = false;
bool cov_pos = false;
bool cov_vel = false;
bool cov_color = false;
bool origin = false;
bool isOriginSet = false;
colvec poseOrigin(6);
ros::Publisher posePub;
ros::Publisher pathPub;
ros::Publisher velPub;
ros::Publisher covPub;
ros::Publisher covVelPub;
ros::Publisher trajPub;
ros::Publisher sensorPub;
ros::Publisher meshPub;
ros::Publisher heightPub;
ros::Publisher exp_pub;
ros::Publisher safe_pub;
tf::TransformBroadcaster *broadcaster;
geometry_msgs::PoseStamped poseROS;
nav_msgs::Path pathROS, expPath, safePath;
visualization_msgs::Marker velROS;
visualization_msgs::Marker covROS;
visualization_msgs::Marker covVelROS;
visualization_msgs::Marker trajROS;
visualization_msgs::Marker sensorROS;
visualization_msgs::Marker meshROS;
sensor_msgs::Range heightROS;
string _frame_id;

mat quaternion_to_R(const colvec &q) {
    double n = norm(q, 2);
    colvec nq = q / n;

    double w = nq(0);
    double x = nq(1);
    double y = nq(2);
    double z = nq(3);
    double w2 = w * w;
    double x2 = x * x;
    double y2 = y * y;
    double z2 = z * z;
    double xy = x * y;
    double xz = x * z;
    double yz = y * z;
    double wx = w * x;
    double wy = w * y;
    double wz = w * z;

    mat R = zeros<mat>(3, 3);
    R(0, 0) = w2 + x2 - y2 - z2;
    R(1, 0) = 2 * (wz + xy);
    R(2, 0) = 2 * (xz - wy);
    R(0, 1) = 2 * (xy - wz);
    R(1, 1) = w2 - x2 + y2 - z2;
    R(2, 1) = 2 * (wx + yz);
    R(0, 2) = 2 * (wy + xz);
    R(1, 2) = 2 * (yz - wx);
    R(2, 2) = w2 - x2 - y2 + z2;
    return R;
}

mat ypr_to_R(const colvec &ypr) {
    double c, s;
    mat Rz = zeros<mat>(3, 3);
    double y = ypr(0);
    c = cos(y);
    s = sin(y);
    Rz(0, 0) = c;
    Rz(1, 0) = s;
    Rz(0, 1) = -s;
    Rz(1, 1) = c;
    Rz(2, 2) = 1;

    mat Ry = zeros<mat>(3, 3);
    double p = ypr(1);
    c = cos(p);
    s = sin(p);
    Ry(0, 0) = c;
    Ry(2, 0) = -s;
    Ry(0, 2) = s;
    Ry(2, 2) = c;
    Ry(1, 1) = 1;

    mat Rx = zeros<mat>(3, 3);
    double r = ypr(2);
    c = cos(r);
    s = sin(r);
    Rx(1, 1) = c;
    Rx(2, 1) = s;
    Rx(1, 2) = -s;
    Rx(2, 2) = c;
    Rx(0, 0) = 1;

    mat R = Rz * Ry * Rx;
    return R;
}

colvec R_to_ypr(const mat &R) {
    colvec n = R.col(0);
    colvec o = R.col(1);
    colvec a = R.col(2);

    colvec ypr(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    return ypr;
}

colvec pose_inverse(const colvec &X) {
    mat R = ypr_to_R(X.rows(3, 5));
    colvec n = R.col(0);
    colvec o = R.col(1);
    colvec a = R.col(2);

    colvec XIxyz = -trans(R) * (X.rows(0, 2));
    colvec XIypr(3);
    double XIy = atan2(o(0), n(0));
    double XIp = atan2(-a(0), n(0) * cos(XIy) + o(0) * sin(XIy));
    double XIr = atan2(n(2) * sin(XIy) - o(2) * cos(XIy), -n(1) * sin(XIy) + o(1) * cos(XIy));
    XIypr(0) = XIy;
    XIypr(1) = XIp;
    XIypr(2) = XIr;

    colvec XI = join_cols(XIxyz, XIypr);
    return XI;
}

colvec R_to_quaternion(const mat &R) {
    colvec q(4);
    double tr = R(0, 0) + R(1, 1) + R(2, 2);
    if (tr > 0) {
        double S = sqrt(tr + 1.0) * 2;
        q(0) = 0.25 * S;
        q(1) = (R(2, 1) - R(1, 2)) / S;
        q(2) = (R(0, 2) - R(2, 0)) / S;
        q(3) = (R(1, 0) - R(0, 1)) / S;
    } else if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
        double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2;
        q(0) = (R(2, 1) - R(1, 2)) / S;
        q(1) = 0.25 * S;
        q(2) = (R(0, 1) + R(1, 0)) / S;
        q(3) = (R(0, 2) + R(2, 0)) / S;
    } else if (R(1, 1) > R(2, 2)) {
        double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2;
        q(0) = (R(0, 2) - R(2, 0)) / S;
        q(1) = (R(0, 1) + R(1, 0)) / S;
        q(2) = 0.25 * S;
        q(3) = (R(1, 2) + R(2, 1)) / S;
    } else {
        double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2;
        q(0) = (R(1, 0) - R(0, 1)) / S;
        q(1) = (R(0, 2) + R(2, 0)) / S;
        q(2) = (R(1, 2) + R(2, 1)) / S;
        q(3) = 0.25 * S;
    }
    return q;
}

colvec pose_update(const colvec &X1, const colvec &X2) {
    mat R1 = ypr_to_R(X1.rows(3, 5));
    mat R2 = ypr_to_R(X2.rows(3, 5));
    mat R3 = R1 * R2;

    colvec X3xyz = X1.rows(0, 2) + R1 * X2.rows(0, 2);
    colvec X3ypr = R_to_ypr(R3);

    colvec X3 = join_cols(X3xyz, X3ypr);
    return X3;
}

struct PolyTrajData {
    quadrotor_msgs::PolynomialTrajectory msg;
    ros::Time rcv_stamp;
    ros::Time heartbeat_stamp;
    ros::Time yaw_cmd_stamp;
    ros::Time yaw_traj_stamp;

    geometry_utils::Trajectory cmd_traj;
    geometry_utils::Trajectory yaw_traj;
    double yaw_rate, yaw;

    const int YAW_COMMAND = 1, YAW_TRAJ = 2, YAW_NONE = 0;
    int yaw_mode = YAW_NONE;
    ros::Publisher mkr_pub;
    mutex traj_update_mtx;
} poly_traj_data_;


void odom_callback(const nav_msgs::Odometry::ConstPtr &msg) {
    if (msg->header.frame_id == string("null"))
        return;
    colvec pose(6);
    pose(0) = msg->pose.pose.position.x;
    pose(1) = msg->pose.pose.position.y;
    pose(2) = msg->pose.pose.position.z;
    colvec q(4);
    q(0) = msg->pose.pose.orientation.w;
    q(1) = msg->pose.pose.orientation.x;
    q(2) = msg->pose.pose.orientation.y;
    q(3) = msg->pose.pose.orientation.z;
    pose.rows(3, 5) = R_to_ypr(quaternion_to_R(q));
    colvec vel(3);
    vel(0) = msg->twist.twist.linear.x;
    vel(1) = msg->twist.twist.linear.y;
    vel(2) = msg->twist.twist.linear.z;

    if (origin && !isOriginSet) {
        isOriginSet = true;
        poseOrigin = pose;
    }
    if (origin) {
        vel = trans(ypr_to_R(pose.rows(3, 5))) * vel;
        pose = pose_update(pose_inverse(poseOrigin), pose);
        vel = ypr_to_R(pose.rows(3, 5)) * vel;
    }

    // Pose
    poseROS.header = msg->header;
    poseROS.header.stamp = msg->header.stamp;
    poseROS.header.frame_id = string("world");
    poseROS.pose.position.x = pose(0);
    poseROS.pose.position.y = pose(1);
    poseROS.pose.position.z = pose(2);
    q = R_to_quaternion(ypr_to_R(pose.rows(3, 5)));
    poseROS.pose.orientation.w = q(0);
    poseROS.pose.orientation.x = q(1);
    poseROS.pose.orientation.y = q(2);
    poseROS.pose.orientation.z = q(3);
    posePub.publish(poseROS);

    // Velocity
    colvec yprVel(3);
    yprVel(0) = atan2(vel(1), vel(0));
    yprVel(1) = -atan2(vel(2), norm(vel.rows(0, 1), 2));
    yprVel(2) = 0;
    q = R_to_quaternion(ypr_to_R(yprVel));
    velROS.header.frame_id = string("world");
    velROS.header.stamp = msg->header.stamp;
    velROS.ns = string("velocity");
    velROS.id = 0;
    velROS.type = visualization_msgs::Marker::ARROW;
    velROS.action = visualization_msgs::Marker::ADD;
    velROS.pose.position.x = pose(0);
    velROS.pose.position.y = pose(1);
    velROS.pose.position.z = pose(2);
    velROS.pose.orientation.w = q(0);
    velROS.pose.orientation.x = q(1);
    velROS.pose.orientation.y = q(2);
    velROS.pose.orientation.z = q(3);
    velROS.scale.x = norm(vel, 2);
    velROS.scale.y = 0.05;
    velROS.scale.z = 0.05;
    velROS.color.a = 1.0;
    velROS.color.r = color_r;
    velROS.color.g = color_g;
    velROS.color.b = color_b;
    velPub.publish(velROS);

    // Path
    static ros::Time prevt = msg->header.stamp;
    if ((msg->header.stamp - prevt).toSec() > 0.1) {
        prevt = msg->header.stamp;
        pathROS.header = poseROS.header;
        pathROS.poses.push_back(poseROS);
        pathPub.publish(pathROS);
    }

    // Covariance color
    double r = 1;
    double g = 1;
    double b = 1;
    bool G = msg->twist.covariance[33];
    bool V = msg->twist.covariance[34];
    bool L = msg->twist.covariance[35];
    if (cov_color) {
        r = G;
        g = V;
        b = L;
    }

    // Covariance Position
    if (cov_pos) {
        mat P(6, 6);
        for (int j = 0; j < 6; j++)
            for (int i = 0; i < 6; i++)
                P(i, j) = msg->pose.covariance[i + j * 6];
        colvec eigVal;
        mat eigVec;
        eig_sym(eigVal, eigVec, P.submat(0, 0, 2, 2));
        if (det(eigVec) < 0) {
            for (int k = 0; k < 3; k++) {
                mat eigVecRev = eigVec;
                eigVecRev.col(k) *= -1;
                if (det(eigVecRev) > 0) {
                    eigVec = eigVecRev;
                    break;
                }
            }
        }
        covROS.header.frame_id = string("world");
        covROS.header.stamp = msg->header.stamp;
        covROS.ns = string("covariance");
        covROS.id = 0;
        covROS.type = visualization_msgs::Marker::SPHERE;
        covROS.action = visualization_msgs::Marker::ADD;
        covROS.pose.position.x = pose(0);
        covROS.pose.position.y = pose(1);
        covROS.pose.position.z = pose(2);
        q = R_to_quaternion(eigVec);
        covROS.pose.orientation.w = q(0);
        covROS.pose.orientation.x = q(1);
        covROS.pose.orientation.y = q(2);
        covROS.pose.orientation.z = q(3);
        covROS.scale.x = sqrt(eigVal(0)) * cov_scale;
        covROS.scale.y = sqrt(eigVal(1)) * cov_scale;
        covROS.scale.z = sqrt(eigVal(2)) * cov_scale;
        covROS.color.a = 0.4;
        covROS.color.r = r * 0.5;
        covROS.color.g = g * 0.5;
        covROS.color.b = b * 0.5;
        covPub.publish(covROS);
    }

    // Covariance Velocity
    if (cov_vel) {
        mat P(3, 3);
        for (int j = 0; j < 3; j++)
            for (int i = 0; i < 3; i++)
                P(i, j) = msg->twist.covariance[i + j * 6];
        mat R = ypr_to_R(pose.rows(3, 5));
        P = R * P * trans(R);
        colvec eigVal;
        mat eigVec;
        eig_sym(eigVal, eigVec, P);
        if (det(eigVec) < 0) {
            for (int k = 0; k < 3; k++) {
                mat eigVecRev = eigVec;
                eigVecRev.col(k) *= -1;
                if (det(eigVecRev) > 0) {
                    eigVec = eigVecRev;
                    break;
                }
            }
        }
        covVelROS.header.frame_id = string("world");
        covVelROS.header.stamp = msg->header.stamp;
        covVelROS.ns = string("covariance_velocity");
        covVelROS.id = 0;
        covVelROS.type = visualization_msgs::Marker::SPHERE;
        covVelROS.action = visualization_msgs::Marker::ADD;
        covVelROS.pose.position.x = pose(0);
        covVelROS.pose.position.y = pose(1);
        covVelROS.pose.position.z = pose(2);
        q = R_to_quaternion(eigVec);
        covVelROS.pose.orientation.w = q(0);
        covVelROS.pose.orientation.x = q(1);
        covVelROS.pose.orientation.y = q(2);
        covVelROS.pose.orientation.z = q(3);
        covVelROS.scale.x = sqrt(eigVal(0)) * cov_scale;
        covVelROS.scale.y = sqrt(eigVal(1)) * cov_scale;
        covVelROS.scale.z = sqrt(eigVal(2)) * cov_scale;
        covVelROS.color.a = 0.4;
        covVelROS.color.r = r;
        covVelROS.color.g = g;
        covVelROS.color.b = b;
        covVelPub.publish(covVelROS);
    }

    // Color Coded Trajectory
    static colvec ppose = pose;
    static ros::Time pt = msg->header.stamp;
    ros::Time t = msg->header.stamp;
    if ((t - pt).toSec() > 0.5) {
        trajROS.header.frame_id = string("world");
        trajROS.header.stamp = ros::Time::now();
        trajROS.ns = string("trajectory");
        trajROS.type = visualization_msgs::Marker::LINE_LIST;
        trajROS.action = visualization_msgs::Marker::ADD;
        trajROS.pose.position.x = 0;
        trajROS.pose.position.y = 0;
        trajROS.pose.position.z = 0;
        trajROS.pose.orientation.w = 1;
        trajROS.pose.orientation.x = 0;
        trajROS.pose.orientation.y = 0;
        trajROS.pose.orientation.z = 0;
        trajROS.scale.x = 0.1;
        trajROS.scale.y = 0;
        trajROS.scale.z = 0;
        trajROS.color.r = 0.0;
        trajROS.color.g = 1.0;
        trajROS.color.b = 0.0;
        trajROS.color.a = 0.8;
        geometry_msgs::Point p;
        p.x = ppose(0);
        p.y = ppose(1);
        p.z = ppose(2);
        trajROS.points.push_back(p);
        p.x = pose(0);
        p.y = pose(1);
        p.z = pose(2);
        trajROS.points.push_back(p);
        std_msgs::ColorRGBA color;
        color.r = r;
        color.g = g;
        color.b = b;
        color.a = 1;
        trajROS.colors.push_back(color);
        trajROS.colors.push_back(color);
        ppose = pose;
        pt = t;
        trajPub.publish(trajROS);
    }

    // Sensor availability
    sensorROS.header.frame_id = string("world");
    sensorROS.header.stamp = msg->header.stamp;
    sensorROS.ns = string("sensor");
    sensorROS.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    sensorROS.action = visualization_msgs::Marker::ADD;
    sensorROS.pose.position.x = pose(0);
    sensorROS.pose.position.y = pose(1);
    sensorROS.pose.position.z = pose(2) + 1.0;
    sensorROS.pose.orientation.w = q(0);
    sensorROS.pose.orientation.x = q(1);
    sensorROS.pose.orientation.y = q(2);
    sensorROS.pose.orientation.z = q(3);
    string strG = G ? string(" GPS ") : string("");
    string strV = V ? string(" Vision ") : string("");
    string strL = L ? string(" Laser ") : string("");
    sensorROS.text = "| " + strG + strV + strL + " |";
    sensorROS.color.a = 1.0;
    sensorROS.color.r = 1.0;
    sensorROS.color.g = 1.0;
    sensorROS.color.b = 1.0;
    sensorROS.scale.z = 0.5;
    sensorPub.publish(sensorROS);

    // Laser height measurement
    double H = msg->twist.covariance[32];
    heightROS.header.frame_id = string("height");
    heightROS.header.stamp = msg->header.stamp;
    heightROS.radiation_type = sensor_msgs::Range::ULTRASOUND;
    heightROS.field_of_view = 5.0 * M_PI / 180.0;
    heightROS.min_range = -100;
    heightROS.max_range = 100;
    heightROS.range = H;
    heightPub.publish(heightROS);

    // Mesh model
    meshROS.header.frame_id = _frame_id;
    meshROS.header.stamp = msg->header.stamp;
    meshROS.ns = "mesh";
    meshROS.id = 0;
    meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
    meshROS.action = visualization_msgs::Marker::ADD;
    meshROS.pose.position.x = msg->pose.pose.position.x;
    meshROS.pose.position.y = msg->pose.pose.position.y;
    meshROS.pose.position.z = msg->pose.pose.position.z;
    q(0) = msg->pose.pose.orientation.w;
    q(1) = msg->pose.pose.orientation.x;
    q(2) = msg->pose.pose.orientation.y;
    q(3) = msg->pose.pose.orientation.z;
    if (cross_config) {
        colvec ypr = R_to_ypr(quaternion_to_R(q));
        ypr(0) += 45.0 * M_PI / 180.0;
        q = R_to_quaternion(ypr_to_R(ypr));
    }
    meshROS.pose.orientation.w = q(0);
    meshROS.pose.orientation.x = q(1);
    meshROS.pose.orientation.y = q(2);
    meshROS.pose.orientation.z = q(3);
    meshROS.scale.x = scale;
    meshROS.scale.y = scale;
    meshROS.scale.z = scale;
    meshROS.color.a = color_a;
    meshROS.color.r = color_r;
    meshROS.color.g = color_g;
    meshROS.color.b = color_b;
    meshROS.mesh_resource = mesh_resource;
    meshROS.mesh_use_embedded_materials = true;
    meshPub.publish(meshROS);

    // TF for raw sensor visualization
    if (tf45) {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(pose(0), pose(1), pose(2)));
        transform.setRotation(tf::Quaternion(q(1), q(2), q(3), q(0)));

        tf::Transform transform45;
        transform45.setOrigin(tf::Vector3(0, 0, 0));
        colvec y45 = zeros<colvec>(3);
        y45(0) = 45.0 * M_PI / 180;
        colvec q45 = R_to_quaternion(ypr_to_R(y45));
        transform45.setRotation(tf::Quaternion(q45(1), q45(2), q45(3), q45(0)));

        tf::Transform transform90;
        transform90.setOrigin(tf::Vector3(0, 0, 0));
        colvec p90 = zeros<colvec>(3);
        p90(1) = 90.0 * M_PI / 180;
        colvec q90 = R_to_quaternion(ypr_to_R(p90));
        transform90.setRotation(tf::Quaternion(q90(1), q90(2), q90(3), q90(0)));

        broadcaster->sendTransform(tf::StampedTransform(transform, msg->header.stamp, string("world"), string("base")));
        broadcaster->sendTransform(
                tf::StampedTransform(transform45, msg->header.stamp, string("base"), string("laser")));
        broadcaster->sendTransform(
                tf::StampedTransform(transform45, msg->header.stamp, string("base"), string("vision")));
        broadcaster->sendTransform(
                tf::StampedTransform(transform90, msg->header.stamp, string("base"), string("height")));
    }

    if (!poly_traj_data_.cmd_traj.empty()) {
        expPath.poses.clear();
        safePath.poses.clear();
        double eval_t = msg->header.stamp.toSec() - poly_traj_data_.cmd_traj.start_WT;
        geometry_utils::Trajectory &cmd_traj = poly_traj_data_.cmd_traj;
        double dur = poly_traj_data_.cmd_traj.getTotalDuration();
        Eigen::Vector3d last_snap, snap, pos;
        last_snap = cmd_traj.getSnap(eval_t);
        eval_t += 0.01;
        bool safe_begin{false};
        for (; eval_t < dur; eval_t += 0.01) {
            snap = cmd_traj.getSnap(eval_t);
            double diff = (snap - last_snap).norm();
            cout<<"diff: "<<diff<<endl;
            if( diff > 5){
                safe_begin = true;
            }
            last_snap = snap;
            pos = cmd_traj.getPos(eval_t);
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = _frame_id;
            pose.header.stamp = msg->header.stamp;
            pose.pose.position.x = pos(0);
            pose.pose.position.y = pos(1);
            pose.pose.position.z = pos(2);
            if(!safe_begin){
                expPath.poses.push_back(pose);
            }else{
                safePath.poses.push_back(pose);
            }
        }
        expPath.header.frame_id = _frame_id;
        expPath.header.stamp = msg->header.stamp;
        exp_pub.publish(expPath);
        safePath.header.frame_id = _frame_id;
        safePath.header.stamp = msg->header.stamp;
        safe_pub.publish(safePath);
    }

}

void cmd_callback(const quadrotor_msgs::PositionCommand cmd) {
    if (cmd.header.frame_id == string("null"))
        return;

    colvec pose(6);
    pose(0) = cmd.position.x;
    pose(1) = cmd.position.y;
    pose(2) = cmd.position.z;
    colvec q(4);
    q(0) = 1.0;
    q(1) = 0.0;
    q(2) = 0.0;
    q(3) = 0.0;
    pose.rows(3, 5) = R_to_ypr(quaternion_to_R(q));

    // Mesh model
    meshROS.header.frame_id = _frame_id;
    meshROS.header.stamp = cmd.header.stamp;
    meshROS.ns = "mesh";
    meshROS.id = 0;
    meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
    meshROS.action = visualization_msgs::Marker::ADD;
    meshROS.pose.position.x = cmd.position.x;
    meshROS.pose.position.y = cmd.position.y;
    meshROS.pose.position.z = cmd.position.z;

    if (cross_config) {
        colvec ypr = R_to_ypr(quaternion_to_R(q));
        ypr(0) += 45.0 * M_PI / 180.0;
        q = R_to_quaternion(ypr_to_R(ypr));
    }
    meshROS.pose.orientation.w = q(0);
    meshROS.pose.orientation.x = q(1);
    meshROS.pose.orientation.y = q(2);
    meshROS.pose.orientation.z = q(3);
    meshROS.scale.x = 2.0;
    meshROS.scale.y = 2.0;
    meshROS.scale.z = 2.0;
    meshROS.color.a = color_a;
    meshROS.color.r = color_r;
    meshROS.color.g = color_g;
    meshROS.color.b = color_b;
    meshROS.mesh_use_embedded_materials = true;
    meshROS.mesh_resource = mesh_resource;
    meshPub.publish(meshROS);
}

void poly_callback(const quadrotor_msgs::PolynomialTrajectoryConstPtr &pMsg) {
    lock_guard<mutex> lock(poly_traj_data_.traj_update_mtx);
    using namespace geometry_utils;
    /// First decode the type info;
    bool IS_EMER_STOP = pMsg->type & quadrotor_msgs::PolynomialTrajectory::EMER_STOP;
    /// Early return if the message is emergency stop
    if (IS_EMER_STOP) {
        cout << RED << " -- [MPC] Emergency Stop Received" << RESET << endl;
        return;
    }

    bool IS_YAW_TRAJ = pMsg->type & quadrotor_msgs::PolynomialTrajectory::YAW_TRAJ;
    bool IS_YAW_COMMAND = pMsg->type & quadrotor_msgs::PolynomialTrajectory::YAW_COMMAND;
    bool IS_HEART_BEAT = pMsg->type & quadrotor_msgs::PolynomialTrajectory::HEART_BEAT;
    bool IS_POSITION_TRAJ = pMsg->type & quadrotor_msgs::PolynomialTrajectory::POSITION_TRAJ;

    poly_traj_data_.msg = *pMsg;
    poly_traj_data_.rcv_stamp = ros::Time::now();

    if (IS_HEART_BEAT) {
        poly_traj_data_.heartbeat_stamp = ros::Time::now();
        if (!IS_POSITION_TRAJ && !IS_YAW_TRAJ && !IS_YAW_COMMAND && !IS_EMER_STOP) {
            /// Early return if the message is a pure heart beat
            return;
        }
    }
    visualization_utils::VisualUtils::DeleteMkrArr(poly_traj_data_.mkr_pub);

    if (IS_POSITION_TRAJ) {
        if (poly_traj_data_.msg.order_pos != 7) {
            ROS_ERROR(" -- [MPC] PolynomialTrajectory order is not 7, which is not supported");
            return;
        }

        int coef_size = (poly_traj_data_.msg.order_pos + 1) * poly_traj_data_.msg.piece_num_pos;
        if (poly_traj_data_.msg.coef_pos_x.size() != coef_size ||
            poly_traj_data_.msg.coef_pos_y.size() != coef_size ||
            poly_traj_data_.msg.coef_pos_z.size() != coef_size) {
            ROS_ERROR(" -- [MPC] PolynomialTrajectory position coef size is not correct.");
            return;
        }

        Eigen::Matrix<double, 3, 8> coeff;
        Eigen::VectorXd eigenVec(8);  // 创建Eigen向量
        poly_traj_data_.cmd_traj.clear();
        for (int i = 0; i < poly_traj_data_.msg.piece_num_pos; i++) {
            // 使用Eigen的Map函数将std::vector数据映射到Eigen向量
            Eigen::Map<Eigen::VectorXd>(&eigenVec[0], 8) = Eigen::Map<Eigen::VectorXd>(
                    &poly_traj_data_.msg.coef_pos_x[i * 8], 8);
            coeff.row(0) = eigenVec.transpose();
            Eigen::Map<Eigen::VectorXd>(&eigenVec[0], 8) = Eigen::Map<Eigen::VectorXd>(
                    &poly_traj_data_.msg.coef_pos_y[i * 8], 8);
            coeff.row(1) = eigenVec.transpose();
            Eigen::Map<Eigen::VectorXd>(&eigenVec[0], 8) = Eigen::Map<Eigen::VectorXd>(
                    &poly_traj_data_.msg.coef_pos_z[i * 8], 8);
            coeff.row(2) = eigenVec.transpose();
            double t = poly_traj_data_.msg.time_pos[i];
            poly_traj_data_.cmd_traj.emplace_back(t, coeff);
        }
        poly_traj_data_.cmd_traj.start_WT = poly_traj_data_.msg.start_WT_pos.toSec();
        poly_traj_data_.cmd_traj.Visualize(poly_traj_data_.mkr_pub);
    }

    if (IS_YAW_TRAJ) {
        if (poly_traj_data_.msg.order_yaw != 7 &&
            poly_traj_data_.msg.order_yaw != 5) {
            ROS_ERROR(" -- [MPC] PolynomialTrajectory order is not 7, which is not supported\n");
            return;
        }

        int coef_size = (poly_traj_data_.msg.order_yaw + 1) * poly_traj_data_.msg.piece_num_yaw;
        if (poly_traj_data_.msg.coef_yaw.size() != coef_size) {
            ROS_ERROR(" -- [MPC] PolynomialTrajectory yaw coef size is not correct");
            return;
        }
        int mat_col_sizee = poly_traj_data_.msg.order_yaw + 1;
        Eigen::MatrixXd coeff(3, mat_col_sizee);
        coeff.setZero();
        Eigen::VectorXd eigenVec(mat_col_sizee);  // 创建Eigen向量
        poly_traj_data_.yaw_traj.clear();
        for (int i = 0; i < poly_traj_data_.msg.piece_num_yaw; i++) {
            // 使用Eigen的Map函数将std::vector数据映射到Eigen向量
            Eigen::Map<Eigen::VectorXd>(&eigenVec[0], mat_col_sizee) = Eigen::Map<Eigen::VectorXd>(
                    &poly_traj_data_.msg.coef_yaw[i * mat_col_sizee], mat_col_sizee);
            coeff.row(0) = eigenVec.transpose();
            double t = poly_traj_data_.msg.time_yaw[i];
            poly_traj_data_.yaw_traj.emplace_back(t, coeff);
        }
        poly_traj_data_.yaw_traj.start_WT = poly_traj_data_.msg.start_WT_yaw.toSec();
        poly_traj_data_.yaw_traj_stamp = ros::Time::now();
        poly_traj_data_.yaw_mode = poly_traj_data_.YAW_TRAJ;

        visualization_utils::VisualUtils::VisualizeYawTrajectory(poly_traj_data_.mkr_pub, poly_traj_data_.cmd_traj,
                                                                 poly_traj_data_.yaw_traj);
    }

    if (IS_YAW_COMMAND) {
        poly_traj_data_.yaw_rate = poly_traj_data_.msg.yaw_rate;
        poly_traj_data_.yaw = poly_traj_data_.msg.yaw;
        poly_traj_data_.yaw_mode = poly_traj_data_.YAW_COMMAND;
        poly_traj_data_.yaw_cmd_stamp = ros::Time::now();
    }

    if (!IS_YAW_TRAJ && !IS_YAW_COMMAND) {
        poly_traj_data_.yaw_mode = poly_traj_data_.YAW_NONE;
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_visualization");
    ros::NodeHandle n("~");

    n.param("mesh_resource", mesh_resource, std::string("package://odom_visualization/meshes/f250.dae"));
    n.param("color/r", color_r, 1.0);
    n.param("color/g", color_g, 0.0);
    n.param("color/b", color_b, 0.0);
    n.param("color/a", color_a, 1.0);
    n.param("origin", origin, false);
    n.param("robot_scale", scale, 2.0);
    n.param("frame_id", _frame_id, string("world"));

    n.param("cross_config", cross_config, false);
    n.param("tf45", tf45, false);
    n.param("covariance_scale", cov_scale, 100.0);
    n.param("covariance_position", cov_pos, false);
    n.param("covariance_velocity", cov_vel, false);
    n.param("covariance_color", cov_color, false);

    ros::Subscriber sub_odom = n.subscribe("odom", 100, odom_callback);
//    ros::Subscriber sub_cmd = n.subscribe("cmd", 100, cmd_callback);
//    ros::Subscriber sub_poly = n.subscribe("/planning_cmd/poly_traj", 100, poly_callback);
    posePub = n.advertise<geometry_msgs::PoseStamped>("pose", 100, true);
    pathPub = n.advertise<nav_msgs::Path>("path", 100, true);
    velPub = n.advertise<visualization_msgs::Marker>("velocity", 100, true);
    covPub = n.advertise<visualization_msgs::Marker>("covariance", 100, true);
    covVelPub = n.advertise<visualization_msgs::Marker>("covariance_velocity", 100, true);
    trajPub = n.advertise<visualization_msgs::Marker>("trajectory", 100, true);
    sensorPub = n.advertise<visualization_msgs::Marker>("sensor", 100, true);
    meshPub = n.advertise<visualization_msgs::Marker>("robot", 100, true);
    heightPub = n.advertise<sensor_msgs::Range>("height", 100, true);
    exp_pub = n.advertise<nav_msgs::Path>("exp_path", 100, true);
    safe_pub = n.advertise<nav_msgs::Path>("safe_path", 100, true);

    tf::TransformBroadcaster b;
    broadcaster = &b;

    ros::spin();

    return 0;
}
