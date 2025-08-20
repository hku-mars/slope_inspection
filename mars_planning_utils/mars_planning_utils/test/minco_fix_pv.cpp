//
// Created by yunfan on 11/25/22.
//

#include "type_utils/common_type_name.h"
#include <geometry_utils/trajectory.h>
#include <math_utils/lbfgs.h>
#include <visualization_utils/matplotlibcpp.h>

namespace plt = matplotlibcpp;
using namespace std;
using namespace math_utils;
using namespace geometry_utils;
using namespace type_utils;
// MINCO for s=4 and non-uniform time


class PlotUtils {
public:
    static void PlotTrajToControlVariables(Trajectory &traj, const double dt = 0.01) {
        double eval_t = 0.0, total_t = traj.getTotalDuration();
        vector<double> wx, wy, wz, x, y, z, vx, vy, vz, ax, ay, az, aT, ts, jx, jy, jz, rs, ps, ys, v_norm, a_norm;
        wx.reserve(static_cast<int>(total_t / dt));
        wy.reserve(static_cast<int>(total_t / dt));
        wz.reserve(static_cast<int>(total_t / dt));
        vx.reserve(static_cast<int>(total_t / dt));
        vy.reserve(static_cast<int>(total_t / dt));
        vz.reserve(static_cast<int>(total_t / dt));
        ax.reserve(static_cast<int>(total_t / dt));
        ay.reserve(static_cast<int>(total_t / dt));
        az.reserve(static_cast<int>(total_t / dt));
        aT.reserve(static_cast<int>(total_t / dt));
        ts.reserve(static_cast<int>(total_t / dt));
        jx.reserve(static_cast<int>(total_t / dt));
        jy.reserve(static_cast<int>(total_t / dt));
        jz.reserve(static_cast<int>(total_t / dt));
        rs.reserve(static_cast<int>(total_t / dt));
        ps.reserve(static_cast<int>(total_t / dt));
        ys.reserve(static_cast<int>(total_t / dt));
        v_norm.reserve(static_cast<int>(total_t / dt));
        a_norm.reserve(static_cast<int>(total_t / dt));
        double g = 9.81;
        double yaw = 0;
        double yaw_dot = 0;
        double a_T;
        Vec3f omega, vel, acc, jerk, rpy;
        while (1) {
            if (eval_t > total_t) {
                eval_t = total_t;
                acc = traj.getAcc(eval_t);
                vel = traj.getVel(eval_t);
                jerk = traj.getJer(eval_t);
                v_norm.push_back(vel.norm());
                a_norm.push_back(acc.norm());
                aT.push_back(a_T);
                wx.push_back(omega.x());
                wy.push_back(omega.y());
                wz.push_back(omega.z());
                vx.push_back(vel.x());
                vy.push_back(vel.y());
                vz.push_back(vel.z());
                ax.push_back(acc.x());
                ay.push_back(acc.y());
                az.push_back(acc.z());
                jx.push_back(jerk.x());
                jy.push_back(jerk.y());
                jz.push_back(jerk.z());
                rs.push_back(rpy.x());
                ps.push_back(rpy.y());
                ys.push_back(rpy.z());
                ts.push_back(eval_t);
                eval_t += dt;
                break;
            }
            acc = traj.getAcc(eval_t);
            vel = traj.getVel(eval_t);
            jerk = traj.getJer(eval_t);
            v_norm.push_back(vel.norm());
            a_norm.push_back(acc.norm());
            aT.push_back(a_T);
            wx.push_back(omega.x());
            wy.push_back(omega.y());
            wz.push_back(omega.z());
            vx.push_back(vel.x());
            vy.push_back(vel.y());
            vz.push_back(vel.z());
            ax.push_back(acc.x());
            ay.push_back(acc.y());
            az.push_back(acc.z());
            jx.push_back(jerk.x());
            jy.push_back(jerk.y());
            jz.push_back(jerk.z());
            rs.push_back(rpy.x());
            ps.push_back(rpy.y());
            ys.push_back(rpy.z());
            ts.push_back(eval_t);
            eval_t += dt;
        }
        cout << "max vel: " << *max_element(v_norm.begin(), v_norm.end()) << endl;
//        /* Plot vel*/
//        plt::subplot(4, 2, 1);
//        plt::plot(ts);
        plt::plot(ts, vx);
        plt::plot(ts, ax);
        plt::plot(ts, jx);
//        plt::plot(ts, vy);
//        plt::plot(ts, vz);
//        plt::title("Vel (m/s)");
////        /* Plot acc*/
//        plt::subplot(4, 2, 2);
//        plt::plot(ts, ax);
//        plt::plot(ts, ay);
//        plt::plot(ts, az);
//        plt::title("Acc (m/s^2)");
//
//        /* Plot rpy*/
//        plt::subplot(4, 2, 3);
//        plt::plot(ts, rs);
//        plt::plot(ts, ps);
//        plt::plot(ts, ys);
//        plt::title("Angle ");
//
//
//        /* Plot omega*/
//        plt::subplot(4, 2, 4);
//        plt::plot(ts, wx);
//        plt::plot(ts, wy);
//        plt::plot(ts, wz);
//        plt::title("Omega (rad/s)");
//        /* Plot aT*/
//        plt::subplot(4, 2, 5);
//        plt::plot(ts, aT);
//        plt::title("aT ");
//
////        /* Plot jerk*/
//        plt::subplot(4, 2, 6);
//        plt::plot(ts, jx);
//        plt::plot(ts, jy);
//        plt::plot(ts, jz);
//        plt::title("Jerk ");
////
////        /* Plot v norm*/
//        plt::subplot(4, 2, 7);
//        plt::plot(ts, v_norm);
//        plt::title("Vel norm ");
//
//        /* Plot a norm */
//        plt::subplot(4, 2, 8);
//        plt::plot(ts, a_norm);
//        plt::title("Acc norm ");
        plt::grid(true);
        plt::show();
    }
};


class MINCO_S4NU {
public:
    MINCO_S4NU() = default;

    ~MINCO_S4NU() { A.destroy(); }

private:
    int N{0};
    Eigen::Matrix<double, 3, 4> headPVAJ;
    Eigen::Matrix<double, 3, 4> tailPVAJ;
    BandedSystem A;
    Eigen::MatrixX3d b;
    Eigen::VectorXd T1;
    Eigen::VectorXd T2;
    Eigen::VectorXd T3;
    Eigen::VectorXd T4;
    Eigen::VectorXd T5;
    Eigen::VectorXd T6;
    Eigen::VectorXd T7;

public:
    void setConditions(const Eigen::Matrix<double, 3, 4> &headState,
                       const Eigen::Matrix<double, 3, 4> &tailState,
                       const int &pieceNum) {
        N = pieceNum;
        headPVAJ = headState;
        tailPVAJ = tailState;
        A.create(8 * N, 8, 8);
        b.resize(8 * N, 3);
        T1.resize(N);
        T2.resize(N);
        T3.resize(N);
        T4.resize(N);
        T5.resize(N);
        T6.resize(N);
        T7.resize(N);
        return;
    }

    void visualizeA(const Eigen::MatrixXd &inPs,
                    const Eigen::MatrixXd &inAs,
                    const Eigen::MatrixXd &inVs,
                    const Eigen::VectorXd &ts) {
        MatDf A(8 * N, 8 * N);
        A.setZero();
        T1 = ts;
        T2 = T1.cwiseProduct(T1);
        T3 = T2.cwiseProduct(T1);
        T4 = T2.cwiseProduct(T2);
        T5 = T4.cwiseProduct(T1);
        T6 = T4.cwiseProduct(T2);
        T7 = T4.cwiseProduct(T3);


        b.setZero();
        // F0
        A(0, 0) = 1.0;
        A(1, 1) = 1.0;
        A(2, 2) = 2.0;
        A(3, 3) = 6.0;
        b.row(0) = headPVAJ.col(0).transpose();
        b.row(1) = headPVAJ.col(1).transpose();
        b.row(2) = headPVAJ.col(2).transpose();
        b.row(3) = headPVAJ.col(3).transpose();

        for (int i = 0; i < N - 1; i++) {

            A(8 * i + 4, 8 * i) = 1.0;
            A(8 * i + 4, 8 * i + 1) = T1(i);
            A(8 * i + 4, 8 * i + 2) = T2(i);
            A(8 * i + 4, 8 * i + 3) = T3(i);
            A(8 * i + 4, 8 * i + 4) = T4(i);
            A(8 * i + 4, 8 * i + 5) = T5(i);
            A(8 * i + 4, 8 * i + 6) = T6(i);
            A(8 * i + 4, 8 * i + 7) = T7(i);

            // beta 2
//            A(8 * i + 5, 8 * i + 1) = 1.0;
//            A(8 * i + 5, 8 * i + 2) = 2.0 * T1(i);
//            A(8 * i + 5, 8 * i + 3) = 3.0 * T2(i);
//            A(8 * i + 5, 8 * i + 4) = 4.0 * T3(i);
//            A(8 * i + 5, 8 * i + 5) = 5.0 * T4(i);
//            A(8 * i + 5, 8 * i + 6) = 6.0 * T5(i);
//            A(8 * i + 5, 8 * i + 7) = 7.0 * T6(i);


            // beta 3
            A(8 * i + 5, 8 * i + 2) = 2.0;
            A(8 * i + 5, 8 * i + 3) = 6.0 * T1(i);
            A(8 * i + 5, 8 * i + 4) = 12.0 * T2(i);
            A(8 * i + 5, 8 * i + 5) = 20.0 * T3(i);
            A(8 * i + 5, 8 * i + 6) = 30.0 * T4(i);
            A(8 * i + 5, 8 * i + 7) = 42.0 * T5(i);


            // - beta 1
            A(8 * i + 7, 8 * i) = 1.0;
            A(8 * i + 7, 8 * i + 1) = T1(i);
            A(8 * i + 7, 8 * i + 2) = T2(i);
            A(8 * i + 7, 8 * i + 3) = T3(i);
            A(8 * i + 7, 8 * i + 4) = T4(i);
            A(8 * i + 7, 8 * i + 5) = T5(i);
            A(8 * i + 7, 8 * i + 6) = T6(i);
            A(8 * i + 7, 8 * i + 7) = T7(i);
            A(8 * i + 7, 8 * i + 8) = -1.0;

            A(8 * i + 8, 8 * i + 1) = 1.0;
            A(8 * i + 8, 8 * i + 2) = 2.0 * T1(i);
            A(8 * i + 8, 8 * i + 3) = 3.0 * T2(i);
            A(8 * i + 8, 8 * i + 4) = 4.0 * T3(i);
            A(8 * i + 8, 8 * i + 5) = 5.0 * T4(i);
            A(8 * i + 8, 8 * i + 6) = 6.0 * T5(i);
            A(8 * i + 8, 8 * i + 7) = 7.0 * T6(i);
            A(8 * i + 8, 8 * i + 9) = -1.0;

            A(8 * i + 9, 8 * i + 2) = 2.0;
            A(8 * i + 9, 8 * i + 3) = 6.0 * T1(i);
            A(8 * i + 9, 8 * i + 4) = 12.0 * T2(i);
            A(8 * i + 9, 8 * i + 5) = 20.0 * T3(i);
            A(8 * i + 9, 8 * i + 6) = 30.0 * T4(i);
            A(8 * i + 9, 8 * i + 7) = 42.0 * T5(i);
            A(8 * i + 9, 8 * i + 10) = -2.0;

            // beta 3
            A(8 * i + 10, 8 * i + 3) = 6.0;
            A(8 * i + 10, 8 * i + 4) = 24.0 * T1(i);
            A(8 * i + 10, 8 * i + 5) = 60.0 * T2(i);
            A(8 * i + 10, 8 * i + 6) = 120.0 * T3(i);
            A(8 * i + 10, 8 * i + 7) = 210.0 * T4(i);
            A(8 * i + 10, 8 * i + 11) = -6.0;

            // beta4
            A(8 * i + 11, 8 * i + 4) = 24.0;
            A(8 * i + 11, 8 * i + 5) = 120.0 * T1(i);
            A(8 * i + 11, 8 * i + 6) = 360.0 * T2(i);
            A(8 * i + 11, 8 * i + 7) = 840.0 * T3(i);
            A(8 * i + 11, 8 * i + 12) = -24.0;

            A(8 * i + 6, 8 * i + 5) = 120.0;
            A(8 * i + 6, 8 * i + 6) = 720.0 * T1(i);
            A(8 * i + 6, 8 * i + 7) = 2520.0 * T2(i);
            A(8 * i + 6, 8 * i + 13) = -120.0;


//            // beta 1

            // beta 1
//            A(8 * i + 4, 8 * i) = 1.0;
//            A(8 * i + 4, 8 * i + 1) = T1(i);
//            A(8 * i + 4, 8 * i + 2) = T2(i);
//            A(8 * i + 4, 8 * i + 3) = T3(i);
//            A(8 * i + 4, 8 * i + 4) = T4(i);
//            A(8 * i + 4, 8 * i + 5) = T5(i);
//            A(8 * i + 4, 8 * i + 6) = T6(i);
//            A(8 * i + 4, 8 * i + 7) = T7(i);
//
//            // beta 1
//            A(8 * i + 5, 8 * i) = 1.0;
//            A(8 * i + 5, 8 * i + 1) = T1(i);
//            A(8 * i +5, 8 * i + 2) = T2(i);
//            A(8 * i + 5, 8 * i + 3) = T3(i);
//            A(8 * i + 5, 8 * i + 4) = T4(i);
//            A(8 * i + 5, 8 * i + 5) = T5(i);
//            A(8 * i + 5, 8 * i + 6) = T6(i);
//            A(8 * i + 5, 8 * i + 7) = T7(i);
//            A(8 * i + 5, 8 * i + 8) = -1.0;
//
//            A(8 * i + 6, 8 * i + 1) = 1.0;
//            A(8 * i + 6, 8 * i + 2) = 2.0 * T1(i);
//            A(8 * i + 6, 8 * i + 3) = 3.0 * T2(i);
//            A(8 * i + 6, 8 * i + 4) = 4.0 * T3(i);
//            A(8 * i + 6, 8 * i + 5) = 5.0 * T4(i);
//            A(8 * i + 6, 8 * i + 6) = 6.0 * T5(i);
//            A(8 * i + 6, 8 * i + 7) = 7.0 * T6(i);
//            A(8 * i + 6, 8 * i + 9) = -1.0;
//            // beta 2
//            A(8 * i + 7, 8 * i + 2) = 2.0;
//            A(8 * i + 7, 8 * i + 3) = 6.0 * T1(i);
//            A(8 * i + 7, 8 * i + 4) = 12.0 * T2(i);
//            A(8 * i + 7, 8 * i + 5) = 20.0 * T3(i);
//            A(8 * i + 7, 8 * i + 6) = 30.0 * T4(i);
//            A(8 * i + 7, 8 * i + 7) = 42.0 * T5(i);
//            A(8 * i + 7, 8 * i + 10) = -2.0;
//            // beta 3
//            A(8 * i + 8, 8 * i + 3) = 6.0;
//            A(8 * i + 8, 8 * i + 4) = 24.0 * T1(i);
//            A(8 * i + 8, 8 * i + 5) = 60.0 * T2(i);
//            A(8 * i + 8, 8 * i + 6) = 120.0 * T3(i);
//            A(8 * i + 8, 8 * i + 7) = 210.0 * T4(i);
//            A(8 * i + 8, 8 * i + 11) = -6.0;
//
//            // F1
//            A(8 * i + 9, 8 * i + 4) = 24.0;
//            A(8 * i + 9, 8 * i + 5) = 120.0 * T1(i);
//            A(8 * i + 9, 8 * i + 6) = 360.0 * T2(i);
//            A(8 * i + 9, 8 * i + 7) = 840.0 * T3(i);
//            A(8 * i + 9, 8 * i + 12) = -24.0;
//
//            A(8 * i + 10, 8 * i + 5) = 120.0;
//            A(8 * i + 10, 8 * i + 6) = 720.0 * T1(i);
//            A(8 * i + 10, 8 * i + 7) = 2520.0 * T2(i);
//            A(8 * i + 10, 8 * i + 13) = -120.0;
//
//            A(8 * i + 11, 8 * i + 6) = 720.0;
//            A(8 * i + 11, 8 * i + 7) = 5040.0 * T1(i);
//            A(8 * i + 11, 8 * i + 14) = -720.0;
//
//            // beta 1

            b.row(8 * i + 4) = inPs.col(i).transpose();
//            b.row(8 * i + 5) = inVs.col(i).transpose();
            b.row(8 * i + 5) = inAs.col(i).transpose();


        }

        A(8 * N - 4, 8 * N - 8) = 1.0;
        A(8 * N - 4, 8 * N - 7) = T1(N - 1);
        A(8 * N - 4, 8 * N - 6) = T2(N - 1);
        A(8 * N - 4, 8 * N - 5) = T3(N - 1);
        A(8 * N - 4, 8 * N - 4) = T4(N - 1);
        A(8 * N - 4, 8 * N - 3) = T5(N - 1);
        A(8 * N - 4, 8 * N - 2) = T6(N - 1);
        A(8 * N - 4, 8 * N - 1) = T7(N - 1);
        A(8 * N - 3, 8 * N - 7) = 1.0;
        A(8 * N - 3, 8 * N - 6) = 2.0 * T1(N - 1);
        A(8 * N - 3, 8 * N - 5) = 3.0 * T2(N - 1);
        A(8 * N - 3, 8 * N - 4) = 4.0 * T3(N - 1);
        A(8 * N - 3, 8 * N - 3) = 5.0 * T4(N - 1);
        A(8 * N - 3, 8 * N - 2) = 6.0 * T5(N - 1);
        A(8 * N - 3, 8 * N - 1) = 7.0 * T6(N - 1);
        A(8 * N - 2, 8 * N - 6) = 2.0;
        A(8 * N - 2, 8 * N - 5) = 6.0 * T1(N - 1);
        A(8 * N - 2, 8 * N - 4) = 12.0 * T2(N - 1);
        A(8 * N - 2, 8 * N - 3) = 20.0 * T3(N - 1);
        A(8 * N - 2, 8 * N - 2) = 30.0 * T4(N - 1);
        A(8 * N - 2, 8 * N - 1) = 42.0 * T5(N - 1);
        A(8 * N - 1, 8 * N - 5) = 6.0;
        A(8 * N - 1, 8 * N - 4) = 24.0 * T1(N - 1);
        A(8 * N - 1, 8 * N - 3) = 60.0 * T2(N - 1);
        A(8 * N - 1, 8 * N - 2) = 120.0 * T3(N - 1);
        A(8 * N - 1, 8 * N - 1) = 210.0 * T4(N - 1);

        b.row(8 * N - 4) = tailPVAJ.col(0).transpose();
        b.row(8 * N - 3) = tailPVAJ.col(1).transpose();
        b.row(8 * N - 2) = tailPVAJ.col(2).transpose();
        b.row(8 * N - 1) = tailPVAJ.col(3).transpose();

        cout << b << endl;
        cout << A << endl;
        b = A.lu().solve(b);

    }

    void setParameters(const Eigen::MatrixXd &inPs,
                       const Eigen::MatrixXd &inAs,
                       const Eigen::MatrixXd &inVs,
                       const Eigen::VectorXd &ts) {
        visualizeA(inPs, inVs, inAs, ts);
        return;
        T1 = ts;
        T2 = T1.cwiseProduct(T1);
        T3 = T2.cwiseProduct(T1);
        T4 = T2.cwiseProduct(T2);
        T5 = T4.cwiseProduct(T1);
        T6 = T4.cwiseProduct(T2);
        T7 = T4.cwiseProduct(T3);

        A.reset();
        b.setZero();

        A(0, 0) = 1.0;
        A(1, 1) = 1.0;
        A(2, 2) = 2.0;
        A(3, 3) = 6.0;
        b.row(0) = headPVAJ.col(0).transpose();
        b.row(1) = headPVAJ.col(1).transpose();
        b.row(2) = headPVAJ.col(2).transpose();
        b.row(3) = headPVAJ.col(3).transpose();

        for (int i = 0; i < N - 1; i++) {
            // beta 1
            A(8 * i + 4, 8 * i) = 1.0;
            A(8 * i + 4, 8 * i + 1) = T1(i);
            A(8 * i + 4, 8 * i + 2) = T2(i);
            A(8 * i + 4, 8 * i + 3) = T3(i);
            A(8 * i + 4, 8 * i + 4) = T4(i);
            A(8 * i + 4, 8 * i + 5) = T5(i);
            A(8 * i + 4, 8 * i + 6) = T6(i);
            A(8 * i + 4, 8 * i + 7) = T7(i);

            // beta 1
            A(8 * i + 5, 8 * i) = 1.0;
            A(8 * i + 5, 8 * i + 1) = T1(i);
            A(8 * i +5, 8 * i + 2) = T2(i);
            A(8 * i + 5, 8 * i + 3) = T3(i);
            A(8 * i + 5, 8 * i + 4) = T4(i);
            A(8 * i + 5, 8 * i + 5) = T5(i);
            A(8 * i + 5, 8 * i + 6) = T6(i);
            A(8 * i + 5, 8 * i + 7) = T7(i);
            A(8 * i + 5, 8 * i + 8) = -1.0;

            A(8 * i + 6, 8 * i + 1) = 1.0;
            A(8 * i + 6, 8 * i + 2) = 2.0 * T1(i);
            A(8 * i + 6, 8 * i + 3) = 3.0 * T2(i);
            A(8 * i + 6, 8 * i + 4) = 4.0 * T3(i);
            A(8 * i + 6, 8 * i + 5) = 5.0 * T4(i);
            A(8 * i + 6, 8 * i + 6) = 6.0 * T5(i);
            A(8 * i + 6, 8 * i + 7) = 7.0 * T6(i);
            A(8 * i + 6, 8 * i + 9) = -1.0;
            // beta 2
            A(8 * i + 7, 8 * i + 2) = 2.0;
            A(8 * i + 7, 8 * i + 3) = 6.0 * T1(i);
            A(8 * i + 7, 8 * i + 4) = 12.0 * T2(i);
            A(8 * i + 7, 8 * i + 5) = 20.0 * T3(i);
            A(8 * i + 7, 8 * i + 6) = 30.0 * T4(i);
            A(8 * i + 7, 8 * i + 7) = 42.0 * T5(i);
            A(8 * i + 7, 8 * i + 10) = -2.0;
            // beta 3
            A(8 * i + 8, 8 * i + 3) = 6.0;
            A(8 * i + 8, 8 * i + 4) = 24.0 * T1(i);
            A(8 * i + 8, 8 * i + 5) = 60.0 * T2(i);
            A(8 * i + 8, 8 * i + 6) = 120.0 * T3(i);
            A(8 * i + 8, 8 * i + 7) = 210.0 * T4(i);
            A(8 * i + 8, 8 * i + 11) = -6.0;

            // F1
            A(8 * i + 9, 8 * i + 4) = 24.0;
            A(8 * i + 9, 8 * i + 5) = 120.0 * T1(i);
            A(8 * i + 9, 8 * i + 6) = 360.0 * T2(i);
            A(8 * i + 9, 8 * i + 7) = 840.0 * T3(i);
            A(8 * i + 9, 8 * i + 12) = -24.0;

            A(8 * i + 10, 8 * i + 5) = 120.0;
            A(8 * i + 10, 8 * i + 6) = 720.0 * T1(i);
            A(8 * i + 10, 8 * i + 7) = 2520.0 * T2(i);
            A(8 * i + 10, 8 * i + 13) = -120.0;

            A(8 * i + 11, 8 * i + 6) = 720.0;
            A(8 * i + 11, 8 * i + 7) = 5040.0 * T1(i);
            A(8 * i + 11, 8 * i + 14) = -720.0;

            // beta 1

            b.row(8 * i + 4) = inPs.col(i).transpose();

//            b.row(8 * i + 5) = inAs.col(i).transpose();
//            b.row(8 * i + 6) = inVs.col(i).transpose();

//            A(8 * i + 4, 8 * i + 4) = 24.0;
//            A(8 * i + 4, 8 * i + 5) = 120.0 * T1(i);
//            A(8 * i + 4, 8 * i + 6) = 360.0 * T2(i);
//            A(8 * i + 4, 8 * i + 7) = 840.0 * T3(i);
//            A(8 * i + 4, 8 * i + 12) = -24.0;
//
//            A(8 * i + 5, 8 * i + 5) = 120.0;
//            A(8 * i + 5, 8 * i + 6) = 720.0 * T1(i);
//            A(8 * i + 5, 8 * i + 7) = 2520.0 * T2(i);
//            A(8 * i + 5, 8 * i + 13) = -120.0;
//
//            A(8 * i + 6, 8 * i + 6) = 720.0;
//            A(8 * i + 6, 8 * i + 7) = 5040.0 * T1(i);
//            A(8 * i + 6, 8 * i + 14) = -720.0;
//
//            // beta 1
//            A(8 * i + 7, 8 * i) = 1.0;
//            A(8 * i + 7, 8 * i + 1) = T1(i);
//            A(8 * i + 7, 8 * i + 2) = T2(i);
//            A(8 * i + 7, 8 * i + 3) = T3(i);
//            A(8 * i + 7, 8 * i + 4) = T4(i);
//            A(8 * i + 7, 8 * i + 5) = T5(i);
//            A(8 * i + 7, 8 * i + 6) = T6(i);
//            A(8 * i + 7, 8 * i + 7) = T7(i);
//            // beta 1
//            A(8 * i + 8, 8 * i) = 1.0;
//            A(8 * i + 8, 8 * i + 1) = T1(i);
//            A(8 * i + 8, 8 * i + 2) = T2(i);
//            A(8 * i + 8, 8 * i + 3) = T3(i);
//            A(8 * i + 8, 8 * i + 4) = T4(i);
//            A(8 * i + 8, 8 * i + 5) = T5(i);
//            A(8 * i + 8, 8 * i + 6) = T6(i);
//            A(8 * i + 8, 8 * i + 7) = T7(i);
//            A(8 * i + 8, 8 * i + 8) = -1.0;
//            // beta 1
//            A(8 * i + 9, 8 * i + 1) = 1.0;
//            A(8 * i + 9, 8 * i + 2) = 2.0 * T1(i);
//            A(8 * i + 9, 8 * i + 3) = 3.0 * T2(i);
//            A(8 * i + 9, 8 * i + 4) = 4.0 * T3(i);
//            A(8 * i + 9, 8 * i + 5) = 5.0 * T4(i);
//            A(8 * i + 9, 8 * i + 6) = 6.0 * T5(i);
//            A(8 * i + 9, 8 * i + 7) = 7.0 * T6(i);
//            A(8 * i + 9, 8 * i + 9) = -1.0;
//            // beta 2
//            A(8 * i + 10, 8 * i + 2) = 2.0;
//            A(8 * i + 10, 8 * i + 3) = 6.0 * T1(i);
//            A(8 * i + 10, 8 * i + 4) = 12.0 * T2(i);
//            A(8 * i + 10, 8 * i + 5) = 20.0 * T3(i);
//            A(8 * i + 10, 8 * i + 6) = 30.0 * T4(i);
//            A(8 * i + 10, 8 * i + 7) = 42.0 * T5(i);
//            A(8 * i + 10, 8 * i + 10) = -2.0;
//            // beta 3
//            A(8 * i + 11, 8 * i + 3) = 6.0;
//            A(8 * i + 11, 8 * i + 4) = 24.0 * T1(i);
//            A(8 * i + 11, 8 * i + 5) = 60.0 * T2(i);
//            A(8 * i + 11, 8 * i + 6) = 120.0 * T3(i);
//            A(8 * i + 11, 8 * i + 7) = 210.0 * T4(i);
//            A(8 * i + 11, 8 * i + 11) = -6.0;
//
//
////            b.row(8 * i + 5) = inAs.col(i).transpose();
////            b.row(8 * i + 6) = inVs.col(i).transpose();
//            b.row(8 * i + 7) = inPs.col(i).transpose();
        }

        A(8 * N - 4, 8 * N - 8) = 1.0;
        A(8 * N - 4, 8 * N - 7) = T1(N - 1);
        A(8 * N - 4, 8 * N - 6) = T2(N - 1);
        A(8 * N - 4, 8 * N - 5) = T3(N - 1);
        A(8 * N - 4, 8 * N - 4) = T4(N - 1);
        A(8 * N - 4, 8 * N - 3) = T5(N - 1);
        A(8 * N - 4, 8 * N - 2) = T6(N - 1);
        A(8 * N - 4, 8 * N - 1) = T7(N - 1);
        A(8 * N - 3, 8 * N - 7) = 1.0;
        A(8 * N - 3, 8 * N - 6) = 2.0 * T1(N - 1);
        A(8 * N - 3, 8 * N - 5) = 3.0 * T2(N - 1);
        A(8 * N - 3, 8 * N - 4) = 4.0 * T3(N - 1);
        A(8 * N - 3, 8 * N - 3) = 5.0 * T4(N - 1);
        A(8 * N - 3, 8 * N - 2) = 6.0 * T5(N - 1);
        A(8 * N - 3, 8 * N - 1) = 7.0 * T6(N - 1);
        A(8 * N - 2, 8 * N - 6) = 2.0;
        A(8 * N - 2, 8 * N - 5) = 6.0 * T1(N - 1);
        A(8 * N - 2, 8 * N - 4) = 12.0 * T2(N - 1);
        A(8 * N - 2, 8 * N - 3) = 20.0 * T3(N - 1);
        A(8 * N - 2, 8 * N - 2) = 30.0 * T4(N - 1);
        A(8 * N - 2, 8 * N - 1) = 42.0 * T5(N - 1);
        A(8 * N - 1, 8 * N - 5) = 6.0;
        A(8 * N - 1, 8 * N - 4) = 24.0 * T1(N - 1);
        A(8 * N - 1, 8 * N - 3) = 60.0 * T2(N - 1);
        A(8 * N - 1, 8 * N - 2) = 120.0 * T3(N - 1);
        A(8 * N - 1, 8 * N - 1) = 210.0 * T4(N - 1);

        b.row(8 * N - 4) = tailPVAJ.col(0).transpose();
        b.row(8 * N - 3) = tailPVAJ.col(1).transpose();
        b.row(8 * N - 2) = tailPVAJ.col(2).transpose();
        b.row(8 * N - 1) = tailPVAJ.col(3).transpose();


        cout << b << endl;

        A.factorizeLU();
        A.solve(b);

        return;
    }

    void getTrajectory(Trajectory &traj) const {
        traj.clear();
        traj.reserve(N);
        for (int i = 0; i < N; i++) {
            traj.emplace_back(T1(i),
                              b.block<8, 3>(8 * i, 0)
                                      .transpose()
                                      .rowwise()
                                      .reverse());
        }
        return;
    }

};


int main() {

    MINCO_S4NU minco;
    StatePVAJ headPVAJ, tailPVAJ;
    headPVAJ.setZero();
    tailPVAJ.setZero();
    headPVAJ.col(0) = Eigen::Vector3d(0.0, 0.0, 0.0);
    tailPVAJ.col(0) = Eigen::Vector3d(1.0, 0.0, 0.0);
    Vec3f inPos = Vec3f(0.5, 0.0, 0.0);
    Vec3f inVel = Vec3f(-15.5, 0.0, 0.0);
    Vec3f inAcc = Vec3f(-15.5, 0.0, 0.0);


    minco.setConditions(headPVAJ, tailPVAJ, 2);
    Vec2f ts;
    ts << 1, 1;
    minco.setParameters(inPos, inVel, inAcc, ts);
    Trajectory traj;
    minco.getTrajectory(traj);
    cout << traj.getVel(1).transpose() << endl;
    cout << traj.getAcc(1).transpose() << endl;
    cout << traj.getJer(1).transpose() << endl;
    cout << GREEN << "Traj opt success, duration: " << traj.getTotalDuration() << RESET << endl;
    PlotUtils::PlotTrajToControlVariables(traj, 0.01);

    return 0;
}