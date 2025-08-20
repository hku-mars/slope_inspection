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
#include <optimization_utils/optimization_utils.h>
#include <optimization_utils/mvie.h>
#include <memory>

namespace ciri {
    using namespace optimization_utils;
    using namespace geometry_utils;
    using namespace type_utils;
    using namespace std;

    class CIRI {
    private:
        double robot_r_{0};
        int iter_num_{1};

        Ellipsoid sphere_template_;
        Polytope optimized_polytope_;

    private:

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
        void findEllipsoid(
                const Eigen::Matrix3Xd &pc,
                const Eigen::Vector3d &a,
                const Eigen::Vector3d &b,
                Ellipsoid &out_ell);

        static void findTangentPlaneOfSphere(const Eigen::Vector3d &center, const double &r,
                                             const Eigen::Vector3d &pass_point,
                                             const Eigen::Vector3d &seed_p,
                                             Eigen::Vector4d &outter_plane);

    public:
        CIRI() = default;

        ~CIRI() = default;

        typedef shared_ptr<CIRI> Ptr;

        void setupParams(double robot_r, int iter_num);

        RET_CODE comvexDecomposition(const Eigen::MatrixX4d &bd,
                                     const Eigen::Matrix3Xd &pc,
                                     const Eigen::Vector3d &a,
                                     const Eigen::Vector3d &b);

        RET_CODE comvexDecomposition(const Eigen::MatrixX4d &bd,
                                     const Eigen::Matrix3Xd &pc,
                                     const vector<double> & pc_radius,
                                     const Eigen::Vector3d &a,
                                     const Eigen::Vector3d &b);


        void getPolytope(Polytope &optimized_poly);
    };
}