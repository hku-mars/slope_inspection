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

#include <geometry_utils/piece.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define TINYCOLORMAP_WITH_EIGEN

#include <visualization_utils/tinycolormap.hpp>

namespace geometry_utils {
    using namespace type_utils;

    class Trajectory {

    private:
        typedef std::vector<Piece> Pieces;
        Pieces pieces;

    public:
        double start_WT{0}; // start wall time

        Trajectory() = default;

        Trajectory(const std::vector<double> &durs,
                   const std::vector<Eigen::MatrixXd> &cMats);

        Trajectory operator+(const Trajectory &traj_in) {
            Trajectory new_traj;
            new_traj.pieces = pieces;
            new_traj.pieces.insert(new_traj.pieces.end(), traj_in.pieces.begin(), traj_in.pieces.end());
            new_traj.start_WT = start_WT;
            return new_traj;
        }


        bool empty();

        int getPieceNum() const;

        vec_Vec3f getWaypoints() const;

        Eigen::VectorXd getDurations() const;

        double getTotalDuration() const;

        const Piece &operator[](int i) const {
            return pieces[i];
        }

        Piece &operator[](int i) {
            return pieces[i];
        }

        void clear();

        typename Pieces::const_iterator begin() const {
            return pieces.begin();
        }

        typename Pieces::const_iterator end() const {
            return pieces.end();
        }

        typename Pieces::iterator begin() {
            return pieces.begin();
        }

        typename Pieces::iterator end() {
            return pieces.end();
        }

        void reserve(const int &n);

        void emplace_back(const Piece &piece);

        void emplace_back(const double &dur,
                          const Eigen::MatrixXd &cMat);

        void append(const Trajectory &traj);

        int locatePieceIdx(double &t) const;

        double getWaypointTT(const int &watpoint_id) const;

        Eigen::Vector3d getPos(double t) const;

        Eigen::Vector3d getVel(double t) const;

        Eigen::Vector3d getAcc(double t) const;

        Eigen::Vector3d getJer(double t) const;

        Mat3Df getState(double t) const;

        Vec3f getSnap(double t) const;

        Eigen::Vector3d getJuncPos(int juncIdx) const;

        Eigen::Vector3d getJuncVel(int juncIdx) const;

        Eigen::Vector3d getJuncAcc(int juncIdx) const;

        bool getPartialTrajectoryByID(const int &start_id, const int &end_id, Trajectory &out_traj) const;

        bool getPartialTrajectoryByTime(const double &start_t, const double &end_t, Trajectory &out_traj) const;

        double getMaxVelRate() const;

        double getMaxAccRate() const;

        bool checkMaxVelRate(const double &maxVelRate) const;

        bool checkMaxAccRate(const double &maxAccRate) const;

        void Visualize(const ros::Publisher &pub_,
                       const std::string &name_space = "traj",
                       const Color &color = Color::Chartreuse(),
                       const double &size = 0.2,
                       const bool &show_waypoint = false,
                       const bool &color_in_vel = false,
                       const tinycolormap::ColormapType &cmp = tinycolormap::ColormapType::Jet) const;

        void printProfile() const;

    };
}

