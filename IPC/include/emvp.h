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

#include <math_utils/lbfgs.h>
#include <math_utils/sdlp.h>
#include <geometry_utils/geometry_utils.h>
#include <geometry_utils/polytope.h>
#include <geometry_utils/ellipsoid.h>
#include <benchmark_utils/scope_timer.h>

#include <Eigen/SVD>
#include <Eigen/Dense>

namespace ipc {
    using namespace geometry_utils;
    using namespace type_utils;
    using std::cout;
    using std::endl;
    using std::string;
    using std::vector;

    class emvp{
    private:
/**
 * @brief chol3d
 *	An implementation of 3D cholesky decomposition
 *   https://en.wikipedia.org/wiki/Cholesky_decomposition
 */
        static void chol3d(const Eigen::Matrix3d &A,
                           Eigen::Matrix3d &L);

/**
 * @brief smoothedL1
 *	A smooth L1 loss for high-accuracy constrain.
 *   https://mohitjainweb.files.wordpress.com/2018/03/smoothl1loss.pdf
 */
        static bool smoothedL1(const double &mu,
                               const double &x,
                               double &f,
                               double &df);

/**
 * @brief costMVIE
 *
 *   The cost function of Maximum Volume Inscribe Ellipsoid problem
 *
 */
        static double costMVIE(void *data,
                               const Eigen::VectorXd &x,
                               Eigen::VectorXd &grad);

/**
 * @brief maxVolInsEllipsoid
 * @param hPoly X rows of hyperplane
 * @param offset_x offset added to the long semi-axis, default is 0
 */
        static bool maxVolInsEllipsoid(const Eigen::MatrixX4d &hPoly,
                                       Ellipsoid &E,
                                       const bool &_fix_p = false);

/**
 * @brief findEllipsoid: find maximum ellipsoid with RILS
 * @param pc the obstacle points
 * @param a the start point of the line segment seed
 * @param b the end point of the line segment seed
 * @param out_ell the output ellipsoid
 * @param r_robot the robot_size, decide if the polytope need to be shrink
 * @param _fix_p decide if the ellipsoid center need to be optimized
 * @param iterations number of the alternating optimization
 */

        static void findEllipsoid(
                const Eigen::Matrix3Xd &pc,
                const Eigen::Vector3d &a,
                const Eigen::Vector3d &b,
                Ellipsoid &out_ell,
                const double &robot_r = 0.0);

        static void findTangentPlaneOfSphere(const Eigen::Vector3d &center, const double &r,
                                             const Eigen::Vector3d &pass_point,
                                             const Eigen::Vector3d &seed_p,
                                             Eigen::Vector4d &outter_plane);

    public:
        emvp() = default;
        ~emvp() = default;
/**
 * @brief embed maximum volume polytope
 * @param bd bounding box with 6 faces
 * @param pc the obstacle points
 * @param a the start point of the line segment seed
 * @param b the end point of the line segment seed
 * @param hPoly the output polytope
 * @param r_robot the robot_size, decide if the polytope need to be shrink
 * @param _fix_p decide if the ellipsoid center need to be optimized
 * @param iterations number of the alternating optimization
 */
        static bool convexDecomposition(const Eigen::MatrixX4d &bd,
                         const Eigen::Matrix3Xd &pc,
                         const Eigen::Vector3d &a,
                         const Eigen::Vector3d &b,
                         Polytope &out_poly,
                         const double r_robot = 0.0,
                         const bool _fix_p = false,
                         const int iterations = 3);
    };

}
