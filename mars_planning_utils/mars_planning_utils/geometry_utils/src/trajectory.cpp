#include <geometry_utils/trajectory.h>

using namespace geometry_utils;
// Trasjectory==================================================

Trajectory::Trajectory(const std::vector<double> &durs,
                       const std::vector<Eigen::MatrixXd> &cMats) {
    int N = std::min(durs.size(), cMats.size());
    pieces.reserve(N);
    for (int i = 0; i < N; i++) {
        pieces.emplace_back(durs[i], cMats[i]);
    }
}

vec_Vec3f Trajectory::getWaypoints() const {
    vec_Vec3f pts;
    for (size_t i = 0; i < pieces.size(); i++) {
        pts.push_back(pieces[i].getPos(0));
    }
    pts.push_back(getPos(getTotalDuration()));
    return pts;
}

bool Trajectory::empty() {
    return pieces.empty();
}

int Trajectory::getPieceNum() const {
    return pieces.size();
}

Eigen::VectorXd Trajectory::getDurations() const {
    int N = getPieceNum();
    Eigen::VectorXd durations(N);
    for (int i = 0; i < N; i++) {
        durations(i) = pieces[i].getDuration();
    }
    return durations;
}

double Trajectory::getTotalDuration() const {
    int N = getPieceNum();
    double totalDuration = 0.0;
    for (int i = 0; i < N; i++) {
        totalDuration += pieces[i].getDuration();
    }
    return totalDuration;
}

void Trajectory::clear() {
    pieces.clear();
    return;
}

void Trajectory::reserve(const int &n) {
    pieces.reserve(n);
    return;
}

void Trajectory::emplace_back(const Piece &piece) {
    pieces.emplace_back(piece);
    return;
}

void Trajectory::emplace_back(const double &dur,
                              const Eigen::MatrixXd &cMat) {
    pieces.emplace_back(dur, cMat);
    return;
}

void Trajectory::append(const Trajectory &traj) {
    pieces.insert(pieces.end(), traj.begin(), traj.end());
    return;
}

int Trajectory::locatePieceIdx(double &t) const {
    int N = getPieceNum();
    int idx;
    double dur;
    for (idx = 0;
         idx < N &&
         t > (dur = pieces[idx].getDuration());
         idx++) {
        t -= dur;
    }
    if (idx == N) {
        idx--;
        t += pieces[idx].getDuration();
    }
    return idx;
}

double Trajectory::getWaypointTT(const int &watpoint_id) const {
    double t = 0.0;
    for (int i = 0; i <= watpoint_id; i++) {
        t += pieces[i].getDuration();
    }
    return t;
}

Eigen::Vector3d Trajectory::getPos(double t) const {

    int pieceIdx = locatePieceIdx(t);
    return pieces[pieceIdx].getPos(t);
}

Eigen::Vector3d Trajectory::getVel(double t) const {
    int pieceIdx = locatePieceIdx(t);
    return pieces[pieceIdx].getVel(t);
}

Eigen::Vector3d Trajectory::getAcc(double t) const {
    int pieceIdx = locatePieceIdx(t);
    return pieces[pieceIdx].getAcc(t);
}

Eigen::Vector3d Trajectory::getJer(double t) const {
    int pieceIdx = locatePieceIdx(t);
    return pieces[pieceIdx].getJer(t);
}


Vec3f Trajectory::getSnap(double t) const {
    int pieceIdx = locatePieceIdx(t);
    return pieces[pieceIdx].getSnap(t);
}

Mat3Df Trajectory::getState(double t) const {
    int pieceIdx = locatePieceIdx(t);
    return pieces[pieceIdx].getState(t);
}

Eigen::Vector3d Trajectory::getJuncPos(int juncIdx) const {
    if (juncIdx != getPieceNum()) {
        return pieces[juncIdx].getCoeffMat().col(pieces[juncIdx].getDegree());
    } else {
        return pieces[juncIdx - 1].getPos(pieces[juncIdx - 1].getDuration());
    }
}

Eigen::Vector3d Trajectory::getJuncVel(int juncIdx) const {
    if (juncIdx != getPieceNum()) {
        return pieces[juncIdx].getCoeffMat().col(pieces[juncIdx].getDegree() - 1);
    } else {
        return pieces[juncIdx - 1].getVel(pieces[juncIdx - 1].getDuration());
    }
}

Eigen::Vector3d Trajectory::getJuncAcc(int juncIdx) const {
    if (juncIdx != getPieceNum()) {
        return pieces[juncIdx].getCoeffMat().col(pieces[juncIdx].getDegree() - 2) * 2.0;
    } else {
        return pieces[juncIdx - 1].getAcc(pieces[juncIdx - 1].getDuration());
    }
}

bool Trajectory::getPartialTrajectoryByID(const int &start_id, const int &end_id, Trajectory &out_traj) const {
    out_traj.clear();
    int end_id_ = end_id;
    if (end_id_ == -1) {
        end_id_ = pieces.size();
    }

    if (start_id < 0 || end_id_ >= pieces.size() || start_id >= end_id_) {
        return false;
    }

    for (int i = start_id; i < end_id_; i++) {
        out_traj.emplace_back(pieces[i]);
    }
    return true;
}

bool Trajectory::getPartialTrajectoryByTime(const double &start_TT, const double &end_TT,
                                            Trajectory &out_traj) const {
    /*
     * syms c0 c1 c2 c3 c4 c5 c6 c7 real;
          syms tn t0 real;

          C = [c0 c1 c2 c3 c4 c5];
          t = (tn+t0);
          T = [1 t t^2 t^3 t^4 t^5];
          p = C*T'
          pe = expand(p)


          [c,t]=coeffs(pe,tn)%
          c'

          C = [c0 c1 c2 c3 c4 c5 c6 c7];
          t = (tn+t0);
          T = [1 t t^2 t^3 t^4 t^5 t^6 t^7];
          p = C*T'
          pe = expand(p)


          [c,t]=coeffs(pe,tn)%
          c'
     * */
    double total_dur = getTotalDuration();
    if (start_TT < 0 || start_TT >= total_dur) {
        std::cout << RED << "Partial traj end_t error. [Start_TT]: " << start_TT
                  << " [Total Dur]: " << total_dur << RESET << std::endl;
    }
    if (end_TT <= 0 || end_TT > total_dur) {
        std::cout << RED << "Partial traj end_t error. [end_TT]: " << end_TT
                  << " [Total Dur]: " << total_dur <<
                  RESET << std::endl;
        return false;
    }
    if (end_TT <= start_TT) {
        std::cout << RED << "Time duration wrong: start_t: " << start_TT <<
                  "; end_t: " << end_TT << RESET << std::endl;
        return false;
    }

    out_traj.clear();
    if (start_TT == 0) {
        // 只需要修改终点时间即可
//            print("Only change end time.\n");
        double t0 = end_TT;
        int pieceIdx = locatePieceIdx(t0);
//            print("pieceIdx = {}, t0 = {}.\n",pieceIdx,t0);
        if (pieceIdx == 0) {
            Piece new_pie = pieces[pieceIdx];
            new_pie.setDuration(t0);
            out_traj.emplace_back(new_pie);
        } else {
            for (int i = 0; i < pieceIdx; i++) {
                out_traj.emplace_back(pieces[i]);
            }
            Piece new_pie = pieces[pieceIdx];
            new_pie.setDuration(t0);
            out_traj.emplace_back(new_pie);
        }
        out_traj.start_WT = start_WT;
        return true;
    }

    // Get the start piece id
    double t0 = start_TT;
    double local_end_t = end_TT;
    int pieceIdx = locatePieceIdx(t0);
    if (pieces[pieceIdx].getDegree() == 5) {
        int pieceEndIdx = locatePieceIdx(local_end_t);
        double t02 = t0 * t0;
        double t03 = t02 * t0;
        double t04 = t03 * t0;
        double t05 = t04 * t0;
        // Cut current traj;
        Eigen::MatrixXd coef_mat = pieces[pieceIdx].getCoeffMat();
        Eigen::Matrix<double, 6, 6> cvt_M;
        cvt_M << 1, 0, 0, 0, 0, 0,
                5 * t0, 1, 0, 0, 0, 0,
                10 * t02, 4 * t0, 1, 0, 0, 0,
                10 * t03, 6 * t02, 3 * t0, 1, 0, 0,
                5 * t04, 4 * t03, 3 * t02, 2 * t0, 1, 0,
                t05, 1 * t04, 1 * t03, t02, t0, 1;

        coef_mat = coef_mat * cvt_M.transpose();
        double p1_t = std::min(pieces[pieceIdx].getDuration() - t0, end_TT - start_TT);
        Piece new_pie(p1_t, coef_mat);
        out_traj.pieces.push_back(new_pie);
        if (pieceIdx == pieceEndIdx) {
            out_traj.start_WT = start_WT + start_TT;
            return true;
        }
        // input the rest traj;
        for (int i = pieceIdx + 1; i < pieceEndIdx; i++) {
            out_traj.pieces.push_back(pieces[i]);
        }
        Eigen::MatrixXd end_coef = pieces[pieceEndIdx].getCoeffMat();
        Piece new_pie_end(local_end_t, end_coef);
        out_traj.pieces.push_back(new_pie_end);
        out_traj.start_WT = start_WT + start_TT;
        return true;
    } else if (pieces[pieceIdx].getDegree() == 7) {
        int pieceEndIdx = locatePieceIdx(local_end_t);
        double t02 = t0 * t0;
        double t03 = t02 * t0;
        double t04 = t03 * t0;
        double t05 = t04 * t0;
        double t06 = t03 * t03;
        double t07 = t03 * t04;

        // Cut current traj;
        Eigen::MatrixXd coef_mat = pieces[pieceIdx].getCoeffMat();
        Eigen::Matrix<double, 8, 8> cvt_M;
        cvt_M << 1, 0, 0, 0, 0, 0, 0, 0,
                7 * t0, 1, 0, 0, 0, 0, 0, 0,
                21 * t02, 6 * t0, 1, 0, 0, 0, 0, 0,
                35 * t03, 15 * t02, 5 * t0, 1, 0, 0, 0, 0,
                35 * t04, 20 * t03, 10 * t02, 4 * t0, 1, 0, 0, 0,
                21 * t05, 15 * t04, 10 * t03, 6 * t02, 3 * t0, 1, 0, 0,
                7 * t06, 6 * t05, 5 * t04, 4 * t03, 3 * t02, 2 * t0, 1, 0,
                t07, t06, 1 * t05, t04, t03, t02, t0, 1;

        coef_mat = coef_mat * cvt_M.transpose();
        double p1_t = std::min(pieces[pieceIdx].getDuration() - t0, end_TT - start_TT);
        Piece new_pie(p1_t, coef_mat);
        out_traj.pieces.push_back(new_pie);
        if (pieceIdx == pieceEndIdx) {
            out_traj.start_WT = start_WT + start_TT;
            return true;
        }
        // input the rest traj;
        for (int i = pieceIdx + 1; i < pieceEndIdx; i++) {
            out_traj.pieces.push_back(pieces[i]);
        }
        Eigen::MatrixXd end_coef = pieces[pieceEndIdx].getCoeffMat();
        Piece new_pie_end(local_end_t, end_coef);
        out_traj.pieces.push_back(new_pie_end);
        out_traj.start_WT = start_WT + start_TT;
        return true;

    } else {
        std::cout << "[getPartialTrajectory] ERROR, the piece degree is neither 5 n or 7" << std::endl;
        Trajectory out_empty;
        return false;
    }
}

double Trajectory::getMaxVelRate() const {
    int N = getPieceNum();
    double maxVelRate = -INFINITY;
    double tempNorm;
    for (int i = 0; i < N; i++) {
        tempNorm = pieces[i].getMaxVelRate();
        maxVelRate = maxVelRate < tempNorm ? tempNorm : maxVelRate;
    }
    return maxVelRate;
}

double Trajectory::getMaxAccRate() const {
    int N = getPieceNum();
    double maxAccRate = -INFINITY;
    double tempNorm;
    for (int i = 0; i < N; i++) {
        tempNorm = pieces[i].getMaxAccRate();
        maxAccRate = maxAccRate < tempNorm ? tempNorm : maxAccRate;
    }
    return maxAccRate;
}

bool Trajectory::checkMaxVelRate(const double &maxVelRate) const {
    int N = getPieceNum();
    bool feasible = true;
    for (int i = 0; i < N && feasible; i++) {
        feasible = feasible && pieces[i].checkMaxVelRate(maxVelRate);
    }
    return feasible;
}

bool Trajectory::checkMaxAccRate(const double &maxAccRate) const {
    int N = getPieceNum();
    bool feasible = true;
    for (int i = 0; i < N && feasible; i++) {
        feasible = feasible && pieces[i].checkMaxAccRate(maxAccRate);
    }
    return feasible;
}

void Trajectory::printProfile() const {
    std::cout << GREEN << "[Trajectory::printProfile] The trajectory has " << getPieceNum() << " pieces" << RESET
              << std::endl;
    std::cout << GREEN << "\tMaximum Velocity Rate: " << getMaxVelRate() << RESET << std::endl;
    std::cout << GREEN << "\tMaximum Acceleration Rate: " << getMaxAccRate() << RESET << std::endl;
    std::cout << GREEN << "\tTotal Duration: " << getTotalDuration() << RESET << std::endl;
}

void Trajectory::Visualize(const ros::Publisher &pub_,
                           const std::string &name_space,
                           const Color &color,
                           const double &size,
                           const bool &show_waypoint,
                           const bool &color_in_vel,
                           const tinycolormap::ColormapType &cmp) const {
    // * 1) Visualize the color
    double t_sum = getTotalDuration();
    if (t_sum < 1e-2 || isnan(t_sum) || pieces.empty()) {
        std::cout << REDPURPLE << "[Trajectory::Visualize] ERROR, the total duration is too small" << RESET
                  << std::endl;
        return;
    }
    double eval_t = 0.001;
    Vec3f cur_vel_dir, last_pos, cur_pos, end_point;
    Vec3f cur_vel;
    visualization_msgs::Marker line_list;
    visualization_msgs::MarkerArray mrkarr;
    double vel_max = getMaxVelRate();
    double map_factor = 1.0 / vel_max;
    mrkarr.markers.clear();
    last_pos = getPos(0);
    line_list.header.frame_id = "world";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = name_space;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.type = visualization_msgs::Marker::ARROW;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_list.scale.x = size;
    line_list.scale.y = 0.0000001;
    line_list.scale.z = 0.0000001;
    while (eval_t + 1e-4 < t_sum) {
        cur_pos = getPos(eval_t);
        if ((cur_pos - last_pos).norm() < size / 2.00) {
            eval_t += 0.03;
            continue;
        }
        cur_vel = getVel(eval_t);
        {

            // publish lines
            static int cnt = 0;
            line_list.id = cnt++;
            line_list.points.clear();
            // Line list is blue
            Eigen::Vector3d color_mag;

            if (color_in_vel) {
                double color_id = ((cur_vel.norm() * map_factor));
                color_mag = tinycolormap::GetColor(color_id, cmp).ConvertToEigen();
            } else {
                color_mag.x() = color.r;
                color_mag.y() = color.g;
                color_mag.z() = color.b;
            }

            line_list.color.r = static_cast<float>(color_mag[0]);
            line_list.color.g = static_cast<float >(color_mag[1]);
            line_list.color.b = static_cast<float >(color_mag[2]);
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
        eval_t += 0.03;
    }
    // * 2) Publish the waypoint
    if (show_waypoint) {
        visualization_msgs::Marker marker_ball;
        static int ball_id = 0;
        marker_ball.header.frame_id = "world";
        marker_ball.header.stamp = ros::Time::now();
        marker_ball.ns = name_space + "_waypoints";
        marker_ball.action = visualization_msgs::Marker::ADD;
        marker_ball.pose.orientation.w = 1.0;
        marker_ball.type = visualization_msgs::Marker::SPHERE;
        marker_ball.scale.x = size * 3;
        marker_ball.scale.y = size * 3;
        marker_ball.scale.z = size * 3;
        marker_ball.color = color;
        marker_ball.color.a = 1.0;
        geometry_msgs::Point p;
        for (long unsigned int i = 0; i < pieces.size() - 1; i++) {
            cur_pos = pieces[i].getPos(pieces[i].getDuration());
            p.x = cur_pos.x();
            p.y = cur_pos.y();
            p.z = cur_pos.z();
            marker_ball.pose.position = p;
            marker_ball.id = ball_id++;
            mrkarr.markers.push_back(marker_ball);
        }
    }
    pub_.publish(mrkarr);
}