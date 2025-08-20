/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#pragma once

#include <type_utils/common_type_name.h>
#include <geometry_utils/quickhull.h>
#include <math_utils/sdlp.h>
#include <tf/tf.h>
#include <Eigen/Eigen>

#include <cfloat>
#include <cstdint>
#include <set>
#include <chrono>

namespace geometry_utils {
    using namespace type_utils;
    using namespace math_utils;
    using namespace Eigen;

    ///============ 2023-06-30: add by yunfan ============///
    double DistancePointEllipse(double e0, double e1, double y0, double y1, double &x0, double &x1);

    double
    DistancePointEllipsoid(double e0, double e1, double e2, double y0, double y1, double y2, double &x0, double &x1,
                           double &x2);


    ///============ 2023-06-13: add by Gene ============///
    template<typename Scalar_t>
    Eigen::Matrix<Scalar_t, 3, 1> quaternion_to_yrp(const Eigen::Quaternion<Scalar_t> &q_);

    ///============ 2023-06-13: add by Yunfan ============///
    Vec4f translatePlane(const Vec4f &plane, const Vec3f &translation);

    ///============ 2023-05-23: add by Yunfan ============///
    void normalizeNextYaw(const double &last_yaw, double &yaw);

    ///============ 2023-3-12: add by Yunfan ============///
    void convertFlatOutputToAttAndOmg(const Vec3f &p,
                                      const Vec3f &v,
                                      const Vec3f &a,
                                      const Vec3f &j,
                                      const double &yaw,
                                      const double &yaw_dot,
                                      Vec3f &rpy,
                                      Vec3f &omg,
                                      double &aT
    );


    ///============ 2022-12-10: add by Yunfan ============///
    bool pointInsidePolytope(const Vec3f &point, const PolyhedronH &polytope,
                             double margin = 1e-6);

    ///============ 2022-12-5: add by Yunfan ============///
    double pointLineSegmentDistance(const Vec3f &p, const Vec3f &a, const Vec3f &b);

    double computePathLength(const type_utils::vec_E<Vec3f> &path);

    ///============ 2022-11-18 ====================================================================================
    int inline GetIntersection(float fDst1, float fDst2, Vec3f P1, Vec3f P2, Vec3f &Hit);

    int inline InBox(Vec3f Hit, Vec3f B1, Vec3f B2, const int Axis);

    //The box in this article is Axis-Aligned and so can be defined by only two 3D points:
    // B1 - the smallest values of X, Y, Z
    //        B2 - the largest values of X, Y, Z
    // returns true if line (L1, L2) intersects with the box (B1, B2)
    // returns intersection point in Hit
    int lineIntersectBox(Vec3f L1, Vec3f L2, Vec3f B1, Vec3f B2, Vec3f &Hit);

    Vec3f lineBoxIntersectPoint(const Vec3f &pt, const Vec3f &pos,
                                const Vec3f &box_min, const Vec3f &box_max);

    ///================================================================================================


    Eigen::Matrix3d RotationFromVec3(const Eigen::Vector3d &v);

    // 通过三点获得一个平面
    void FromPointsToPlane(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3,
                           Eigen::Vector4d &hPoly);

    void GetFovCheckPlane(const Eigen::Matrix3d R, const Eigen::Vector3d t, Eigen::MatrixX4d &fov_planes,
                          std::vector<Eigen::Matrix3d> &fov_pts);

    void GetFovPlanes(const Eigen::Matrix3d R, const Eigen::Vector3d t, Eigen::MatrixX4d &fov_planes,
                      std::vector<Eigen::Matrix3d> &fov_pts);


    double findInteriorDist(const Eigen::MatrixX4d &hPoly,
                            Eigen::Vector3d &interior);

    // Each row of hPoly is defined by h0, h1, h2, h3 as
    // h0*x + h1*y + h2*z + h3 <= 0
    bool findInterior(const Eigen::MatrixX4d &hPoly,
                      Eigen::Vector3d &interior);

    bool overlap(const Eigen::MatrixX4d &hPoly0,
                 const Eigen::MatrixX4d &hPoly1,
                 const double eps = 1.0e-6);

    struct filterLess {
        inline bool operator()(const Eigen::Vector3d &l,
                               const Eigen::Vector3d &r) {
            return l(0) < r(0) ||
                   (l(0) == r(0) &&
                    (l(1) < r(1) ||
                     (l(1) == r(1) &&
                      l(2) < r(2))));
        }
    };

    void filterVs(const Eigen::Matrix3Xd &rV,
                  const double &epsilon,
                  Eigen::Matrix3Xd &fV);

    // Each row of hPoly is defined by h0, h1, h2, h3 as
    // h0*x + h1*y + h2*z + h3 <= 0
    // proposed epsilon is 1.0e-6
    void enumerateVs(const Eigen::MatrixX4d &hPoly,
                     const Eigen::Vector3d &inner,
                     Eigen::Matrix3Xd &vPoly,
                     const double epsilon = 1.0e-6);

    // Each row of hPoly is defined by h0, h1, h2, h3 as
    // h0*x + h1*y + h2*z + h3 <= 0
    // proposed epsilon is 1.0e-6
    bool enumerateVs(const Eigen::MatrixX4d &hPoly,
                     Eigen::Matrix3Xd &vPoly,
                     const double epsilon = 1.0e-6);


    template<typename Scalar_t>
    Scalar_t toRad(const Scalar_t &x);

    template<typename Scalar_t>
    Scalar_t toDeg(const Scalar_t &x);

    template<typename Scalar_t>
    Eigen::Matrix<Scalar_t, 3, 3> rotx(Scalar_t t);

    template<typename Scalar_t>
    Eigen::Matrix<Scalar_t, 3, 3> roty(Scalar_t t);

    template<typename Scalar_t>
    Eigen::Matrix<Scalar_t, 3, 3> rotz(Scalar_t t);

    template<typename Derived>
    Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr_to_R(const Eigen::DenseBase<Derived> &ypr);

    template<typename Derived>
    Eigen::Matrix<typename Derived::Scalar, 3, 3>
    vec_to_R(const Eigen::MatrixBase<Derived> &v1, const Eigen::MatrixBase<Derived> &v2);

    template<typename Derived>
    Eigen::Matrix<typename Derived::Scalar, 3, 1> R_to_ypr(const Eigen::DenseBase<Derived> &R);

    template<typename Derived>
    Eigen::Quaternion<typename Derived::Scalar> ypr_to_quaternion(const Eigen::DenseBase<Derived> &ypr);

    template<typename Scalar_t>
    Eigen::Matrix<Scalar_t, 3, 1> quaternion_to_ypr(const Eigen::Quaternion<Scalar_t> &q_);

    template<typename Scalar_t>
    Scalar_t get_yaw_from_quaternion(const Eigen::Quaternion<Scalar_t> &q);

    template<typename Scalar_t>
    Eigen::Quaternion<Scalar_t> yaw_to_quaternion(Scalar_t yaw);

    template<typename Scalar_t>
    Scalar_t normalize_angle(Scalar_t a);

    template<typename Scalar_t>
    Scalar_t angle_add(Scalar_t a, Scalar_t b);

    template<typename Scalar_t>
    Scalar_t yaw_add(Scalar_t a, Scalar_t b);

    template<typename Derived>
    Eigen::Matrix<typename Derived::Scalar, 3, 3> get_skew_symmetric(const Eigen::DenseBase<Derived> &v);

    template<typename Derived>
    Eigen::Matrix<typename Derived::Scalar, 3, 1> from_skew_symmetric(const Eigen::DenseBase<Derived> &M);


}