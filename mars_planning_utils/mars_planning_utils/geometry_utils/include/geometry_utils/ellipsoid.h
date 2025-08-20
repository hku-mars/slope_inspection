#pragma once


#include <type_utils/common_type_name.h>
#include <geometry_utils/geometry_utils.h>
#include <visualization_utils/visualization_utils.h>

namespace geometry_utils {
    class Ellipsoid {
    private:
        /// If the ellipsoid is empty
        bool undefined{true};

        /// The ellipsoid is defined by shape C and center d
        Mat3f C_{}, C_inv_{};
        Mat3f R_{};
        Vec3f r_{}, d_{};

    public:
        Ellipsoid() {}

//        void operator=(const Ellipsoid &other) {
//            std::cout << "Copy assignment called" << std::endl;
//            undefined = false;
//            this->R_ = other.R();
//            this->r_ = other.r();
//            this->d_ = other.d();
//            this->C_ = R_ * r_.asDiagonal() * R_.transpose();
//            this->C_inv_ = C_.inverse();
//        }
//
//        // 拷贝构造函数
//        Ellipsoid(const Ellipsoid &other) {
//            std::cout << "Copy constructor called" << std::endl;
//            undefined = false;
//            this->R_ = other.R();
//            this->r_ = other.r();
//            this->d_ = other.d();
//            this->C_ = R_ * r_.asDiagonal() * R_.transpose();
//            this->C_inv_ = C_.inverse();
//        }

        Ellipsoid(const Mat3f &C, const Vec3f &d);

        Ellipsoid(const Mat3f &R, const Vec3f &r, const Vec3f &d);

        /// If this ellipsoid is empty
        bool empty() const;

        double pointDistaceToEllipsoid(const Vec3f &pt, Vec3f & closest_pt_on_ellip) const;

        /// Find the closestPoint in a point set
        int nearestPointId(const Eigen::Matrix3Xd &pc) const;

        /// Find the closestPoint in a point set
        Vec3f nearestPoint(const Eigen::Matrix3Xd &pc) const;

        /// Find the closestPoint in a point set
        double nearestPointDis(const Eigen::Matrix3Xd &pc, int &np_id) const;

        /// Get the shape of the ellipsoid
        Mat3f C() const;

        /// Get the center of the ellipsoid
        Vec3f d() const;

        Mat3f R() const;

        Vec3f r() const;



        /// Convert a point to the ellipsoid frame
        Vec3f toEllipsoidFrame(const Vec3f &pt_w) const;

        /// Convert a set of points to the ellipsoid frame
        Eigen::Matrix3Xd toEllipsoidFrame(const Eigen::Matrix3Xd &pc_w) const;

        /// Convert a point to the world frame
        Vec3f toWorldFrame(const Vec3f &pt_e) const;

        /// Convert a set of points to the world frame
        Eigen::Matrix3Xd toWorldFrame(const Eigen::Matrix3Xd &pc_e) const;

        /// Convert a plane to the ellipsoid frame
        Eigen::Vector4d toEllipsoidFrame(const Eigen::Vector4d &plane_w) const;

        /// Convert a plane to the ellipsoid frame
        Eigen::Vector4d toWorldFrame(const Eigen::Vector4d &plane_e) const;

        /// Convert a set of planes to ellipsoid frame
        Eigen::MatrixX4d toEllipsoidFrame(const Eigen::MatrixX4d &planes_w) const;

        /// Convert a set of planes to ellipsoid frame
        Eigen::MatrixX4d toWorldFrame(const Eigen::MatrixX4d &planes_e) const;

        /// Calculate the distance of a point in world frame
        double dist(const Vec3f &pt_w) const;

        /// Calculate the distance of a point in world frame
        Eigen::VectorXd dist(const Eigen::Matrix3Xd &pc_w) const;

        bool noPointsInside(vec_Vec3f &pc, const Eigen::Matrix3d R,
                            const Vec3f &r, const Vec3f &p) const;

        bool pointsInside(const Eigen::Matrix3Xd &pc,
                          Mat3Df &out,
                          int &min_pt_id) const;

        /// Check if the point is inside, non-exclusive
        bool inside(const Vec3f &pt) const;

//
        void Visualize(const ros::Publisher &pub,
                       const std::string & ns = "ellipsoid",
                       Color color = Color(Color::Orange(), 0.3)) {
            visualization_msgs::Marker mkr;
            Eigen::Quaterniond q(R_);
            static int id = 0;
            mkr.id = id++;
            mkr.type = visualization_msgs::Marker::SPHERE;
            mkr.header.frame_id = "world";
            mkr.header.stamp = ros::Time::now();
            mkr.ns = ns;
            mkr.id = id++;
            mkr.action = visualization_msgs::Marker::ADD;
            mkr.pose.orientation.w = q.w();
            mkr.pose.orientation.x = q.x();
            mkr.pose.orientation.y = q.y();
            mkr.pose.orientation.z = q.z();
            mkr.pose.position.x = d_.x();
            mkr.pose.position.y = d_.y();
            mkr.pose.position.z = d_.z();
            mkr.scale.x = r_.x() * 2;
            mkr.scale.y = r_.y() * 2;
            mkr.scale.z = r_.z() * 2;
            mkr.color = color;
            visualization_msgs::MarkerArray mkr_arr;
            mkr_arr.markers.push_back(mkr);
            pub.publish(mkr_arr);
        }

    };

}
