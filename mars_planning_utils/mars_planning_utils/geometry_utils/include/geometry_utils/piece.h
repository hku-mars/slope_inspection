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

#include <cmath>
#include <cfloat>
#include <vector>
#include <algorithm>
#include <type_utils/common_type_name.h>
#include <math_utils/root_finder.h>
#include <geometry_utils/banded_system.h>


namespace geometry_utils {
    using namespace type_utils;

    class Piece {
    public:
        // [ t^5 t^4 t^3 t^2 t 1]
//  typedef Eigen::Matrix<double, 3, 5 + 1> CoefficientMat5;
//  typedef Eigen::Matrix<double, 3, 5> VelCoefficientMat5;
//  typedef Eigen::Matrix<double, 3, 5 - 1> AccCoefficientMat5;
//
//  typedef Eigen::Matrix<double, 3, 7 + 1> CoefficientMat7;
//  typedef Eigen::Matrix<double, 3, 7> VelCoefficientMat7;
//  typedef Eigen::Matrix<double, 3, 7 - 1> AccCoefficientMat7;

    private:
        double duration;
        Eigen::MatrixXd coeffMat;
        int D;

    public:
        Piece() = default;

        Piece(double dur, const Eigen::MatrixXd &cMat)
                : duration(dur), coeffMat(cMat), D(cMat.cols() - 1) {};

        int getDim() const;

        int getDegree() const;

        double getDuration() const;

        void setDuration(double dur);

        const Eigen::MatrixXd &getCoeffMat() const;

        Eigen::Vector3d getPos(const double &t) const;

        Eigen::Vector3d getVel(const double &t) const;

        Eigen::Vector3d getAcc(const double &t) const;

        Eigen::Vector3d getJer(const double &t) const;

        Eigen::Vector3d getSnap(const double &t) const;

        Eigen::MatrixXd getState(const double &t) const;

        Eigen::MatrixXd normalizePosCoeffMat() const;

        Eigen::MatrixXd normalizeVelCoeffMat() const;

        Eigen::MatrixXd normalizeAccCoeffMat() const;

        double getMaxVelRate() const;

        double getMaxAccRate() const;

        bool checkMaxVelRate(const double &maxVelRate) const;

        bool checkMaxAccRate(const double &maxAccRate) const;
    };
}

