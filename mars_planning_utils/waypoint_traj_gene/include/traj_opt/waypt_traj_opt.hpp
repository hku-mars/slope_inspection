#ifndef WAYPOINT_TRAJ_OPTIMIZER
#define WAYPOINT_TRAJ_OPTIMIZER

#include <Eigen/Eigen>

#include <cmath>
#include <cfloat>
#include <iostream>
#include <vector>
#include <Eigen/Eigen>

#include "math_utils/lbfgs.h"
#include "benchmark_utils/scope_timer.h"
#include "geometry_utils/geometry_utils.h"
#include "type_utils/common_type_name.h"
#include "geometry_utils/polytope.h"
#include "visualization_utils/visualization_utils.h"
#include "traj_opt/config.hpp"
#include <optimization_utils/optimization_utils.h>
#include <optimization_utils/minco.h>

namespace traj_opt {
    using namespace geometry_utils;
    using namespace visualization_utils;
    using namespace type_utils;
    using namespace benchmark_utils;
    using namespace optimization_utils;
    using std::cout;
    using std::endl;
    using std::string;
    using std::vector;

    class GcopterWayptS3 {
    public:
        GcopterWayptS3() = default;

        ~GcopterWayptS3() {}

        int iter_num;
        double evaluate_cost_dt;
    private:
        MINCO_S3NU minco;

        double rho;
        double scale_factor;
        Eigen::Matrix3d headPVA;
        Eigen::Matrix3d tailPVA;

        int pieceN;

        int spatialDim;
        int temporalDim;

        double smoothEps;
        int integralRes;
        Eigen::VectorXd magnitudeBd;
        Eigen::VectorXd penaltyWt;

        bool block_energy_cost;
        lbfgs::lbfgs_parameter_t lbfgs_params;

        Eigen::Matrix3Xd points;
        Eigen::Matrix3Xd waypoints;
        Eigen::Matrix3Xd free_points;
        Eigen::VectorXd times;

        Eigen::Matrix3Xd gradByPoints;
        Eigen::VectorXd gradByTimes;
        Eigen::MatrixX3d partialGradByCoeffs;
        Eigen::VectorXd partialGradByTimes;

        bool block_snap_cost;
        Eigen::VectorXd fix_times;


    private:

        // magnitudeBounds = [v_max, a_max, omg_max, theta_max, thrust_min, thrust_max, pos_margin]^T
        // penaltyWeights = [pos_weight, vel_weight, acc_weight, omg_weight, theta_weight, thrust_weight]^T
        // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
        //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
        static inline void attachPenaltyFunctional(const Eigen::VectorXd &T,
                                                   const Eigen::MatrixX3d &coeffs,
                                                   const double &smoothFactor,
                                                   const int &integralResolution,
                                                   const Eigen::VectorXd &magnitudeBounds,
                                                   const Eigen::VectorXd &penaltyWeights,
                                                   double &cost,
                                                   Eigen::VectorXd &gradT,
                                                   Eigen::MatrixX3d &gradC) {
            const double vmax = magnitudeBounds[0];
            const double amax = magnitudeBounds[1];
            const double jmax = magnitudeBounds[2];

            const double vmaxSqr = vmax * vmax;
            const double amaxSqr = amax * amax;
            const double jmaxSqr = jmax * jmax;

            const double weightVel = penaltyWeights[0];
            const double weightAcc = penaltyWeights[1];
            const double weightJer = penaltyWeights[2];

            Eigen::Vector3d pos, vel, acc, jer, sna;
            Eigen::Vector3d totalGradPos, totalGradVel, totalGradAcc, totalGradJer;

            double step, alpha;
            double s1, s2, s3, s4, s5;
            Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
            Eigen::Vector3d outerNormal;

            double violaVel, violaVelPena, violaVelPenaD;
            double violaAcc, violaAccPena, violaAccPenaD;
            double violaJer, violaJerPena, violaJerPenaD;
            Eigen::Matrix<double, 6, 3> gradViolaVc, gradViolaAc, gradViolaJc;
            double gradViolaVt, gradViolaAt, gradViolaJt;
            double node, pena;
            const int pieceNum = T.size();
            const double integralFrac = 1.0 / integralResolution;
            pena = 0.0;
            for (int i = 0; i < pieceNum; i++) {
                const Eigen::Matrix<double, 6, 3> &c = coeffs.block<6, 3>(i * 6, 0);
                step = T(i) * integralFrac;
                for (int j = 0; j <= integralResolution; j++) {
                    s1 = j * step;
                    s2 = s1 * s1;
                    s3 = s2 * s1;
                    s4 = s2 * s2;
                    s5 = s4 * s1;
                    beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5;
                    beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4) =
                            4.0 * s3, beta1(5) = 5.0 * s4;
                    beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2, beta2(
                            5) = 20.0 * s3;
                    beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1, beta3(5) =
                            60.0 * s2;
                    beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0, beta4(5) =
                            120.0 * s1;
                    vel = c.transpose() * beta1;
                    acc = c.transpose() * beta2;
                    jer = c.transpose() * beta3;
                    sna = c.transpose() * beta4;
                    alpha = j * integralFrac;

                    violaVel = vel.squaredNorm() - vmaxSqr;
                    violaAcc = acc.squaredNorm() - amaxSqr;
                    violaJer = jer.squaredNorm() - jmaxSqr;
                    node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;

                    if (weightVel > 0 && gcopter::smoothedL1(violaVel, smoothFactor, violaVelPena, violaVelPenaD)) {

                        gradViolaVc = 2.0 * beta1 * vel.transpose();
                        gradViolaVt = 2.0 * alpha * vel.transpose() * acc;
                        gradC.block<6, 3>(i * 6, 0) += node * step * weightVel * violaVelPenaD * gradViolaVc;
                        gradT(i) += node * (weightVel * violaVelPenaD * gradViolaVt * step +
                                            weightVel * violaVelPena * integralFrac);
                        pena += node * step * weightVel * violaVelPena;
                    }

                    if (weightAcc > 0 && gcopter::smoothedL1(violaAcc, smoothFactor, violaAccPena, violaAccPenaD)) {

                        gradViolaAc = 2.0 * beta2 * acc.transpose();
                        gradViolaAt = 2.0 * alpha * acc.transpose() * jer;
                        gradC.block<6, 3>(i * 6, 0) += node * step * weightAcc * violaAccPenaD * gradViolaAc;
                        gradT(i) += node * (weightAcc * violaAccPenaD * gradViolaAt * step +
                                            weightAcc * violaAccPena * integralFrac);
                        pena += node * step * weightAcc * violaAccPena;
                    }

                    if (weightJer > 0.0 && gcopter::smoothedL1(violaJer, smoothFactor, violaJerPena, violaJerPenaD)) {
                        gradViolaJc = 2.0 * beta3 * jer.transpose();
                        gradViolaJt = 2.0 * alpha * jer.transpose() * sna;
                        gradC.block<6, 3>(i * 6, 0) += node * step * weightJer * violaJerPenaD * gradViolaJc;
                        gradT(i) += node * (weightJer * violaJerPenaD * gradViolaJt * step +
                                            weightJer * violaJerPena * integralFrac);
                        pena += node * step * weightJer * violaJerPena;
                    }
                }

            }
            cost += pena;
            return;
        }

        static inline void forwardT(const Eigen::VectorXd &tau,
                                    Eigen::VectorXd &T) {
            const int sizeTau = tau.size();
            T.resize(sizeTau);
            for (int i = 0; i < sizeTau; i++) {
                T(i) = tau(i) > 0.0
                       ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
                       : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
            }
            return;
        }

        template<typename EIGENVEC>
        static inline void backwardT(const Eigen::VectorXd &T,
                                     EIGENVEC &tau) {
            const int sizeT = T.size();
            tau.resize(sizeT);
            for (int i = 0; i < sizeT; i++) {
                tau(i) = T(i) > 1.0
                         ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
                         : (1.0 - sqrt(2.0 / T(i) - 1.0));
            }

            return;
        }

        template<typename EIGENVEC>
        static inline void backwardGradT(const Eigen::VectorXd &tau,
                                         const Eigen::VectorXd &gradT,
                                         EIGENVEC &gradTau) {
            const int sizeTau = tau.size();
            gradTau.resize(sizeTau);
            double denSqrt;
            for (int i = 0; i < sizeTau; i++) {
                if (tau(i) > 0) {
                    gradTau(i) = gradT(i) * (tau(i) + 1.0);
                } else {
                    denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
                    gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
                }
            }

            return;
        }

        static inline double costFunctional(void *ptr,
                                            const Eigen::VectorXd &x,
                                            Eigen::VectorXd &g) {
            TimeConsuming t_("cost functional", false);
            GcopterWayptS3 &obj = *(GcopterWayptS3 *) ptr;
            const int dimTau = obj.temporalDim;
            const int dimXi = obj.spatialDim;
            const double weightT = obj.rho;
            Eigen::Map<const Eigen::VectorXd> tau(x.data(), dimTau);
            Eigen::Map<const Eigen::VectorXd> xi(x.data() + dimTau, dimXi);
            Eigen::Map<Eigen::VectorXd> gradTau(g.data(), dimTau);
            Eigen::Map<Eigen::VectorXd> gradXi(g.data() + dimTau, dimXi);
            obj.iter_num++;
            forwardT(tau, obj.times);
            for (int i = 0; i < obj.free_points.cols(); i++) {
                obj.free_points.col(i).x() = xi(3 * i);
                obj.free_points.col(i).y() = xi(3 * i + 1);
                obj.free_points.col(i).z() = xi(3 * i + 2);
                obj.points.col(i * 2) = obj.free_points.col(i);
            }
            double cost;
            obj.minco.setParameters(obj.points, obj.times);

            cost = 0;
            obj.partialGradByCoeffs.setZero();
            obj.partialGradByTimes.setZero();
            if (!obj.block_energy_cost) {
                obj.minco.getEnergy(cost);
                obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
                obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);
            }
            attachPenaltyFunctional(obj.times, obj.minco.getCoeffs(),
                                    obj.smoothEps, obj.integralRes,
                                    obj.magnitudeBd, obj.penaltyWt,
                                    cost, obj.partialGradByTimes, obj.partialGradByCoeffs);
            obj.minco.propogateGrad(obj.partialGradByCoeffs, obj.partialGradByTimes,
                                    obj.gradByPoints, obj.gradByTimes);
//            print(fg(color::yellow), "MINCO time cost {}\n",  weightT * obj.times.sum());
            cost += weightT * obj.times.sum();
//            print(fg(color::yellow), "MINCO Internal cost {}\n", cost);
            obj.gradByTimes.array() += weightT;

            backwardGradT(tau, obj.gradByTimes, gradTau);
            int idx = 0;
            for (int i = 0; i < obj.gradByPoints.cols(); i++) {
                if (i % 2 == 0) {
                    gradXi.segment(idx * 3, 3) = obj.gradByPoints.col(i);
                    idx++;
                }
            }
            obj.evaluate_cost_dt += t_.stop();
            return cost;
        }

    public:
        // magnitudeBounds = [v_max, acc_max, omg_max, theta_max, thrust_min, thrust_max, pos_margin]^T
        // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
        inline bool setup(const double &timeWeight,
                          const Eigen::Matrix3d &initialPVA,
                          const Eigen::Matrix3d &terminalPVA,
                          const Eigen::Matrix3Xd &_waypoints,
                          const double &smoothingFactor,
                          const int &integralResolution,
                          const Eigen::VectorXd &magnitudeBounds,
                          const Eigen::VectorXd &penaltyWeights,
                          const double _scale_factor = 1.0,
                          const bool _block_energy_cost = false) {
            block_energy_cost = _block_energy_cost;
            scale_factor = _scale_factor;
            rho = timeWeight;
            headPVA = initialPVA;
            tailPVA = terminalPVA;
            waypoints = _waypoints;

            headPVA.col(1) /= scale_factor;
            headPVA.col(2) /= (scale_factor * scale_factor);
            tailPVA.col(1) /= scale_factor;
            tailPVA.col(2) /= (scale_factor * scale_factor);


            smoothEps = smoothingFactor;
            integralRes = integralResolution;
            magnitudeBd = magnitudeBounds;
            penaltyWt = penaltyWeights;

            magnitudeBd[0] = magnitudeBd[0] / scale_factor;
            magnitudeBd[1] = magnitudeBd[1] / (scale_factor * scale_factor);

            pieceN = (waypoints.cols() + 1) * 2;
            temporalDim = pieceN;
            spatialDim = (waypoints.cols() + 1) * 3;
            // Setup for MINCO_S3NU, FlatnessMap, and L-BFGS solver
            minco.setConditions(headPVA, tailPVA, pieceN);

            // Allocate temp variables
            points.resize(3, 2 * waypoints.cols() + 1);
            free_points.resize(3, waypoints.cols() + 1);
            times.resize(pieceN);
            gradByPoints.resize(3, waypoints.cols() + 1);
            gradByTimes.resize(temporalDim);
            partialGradByCoeffs.resize(6 * pieceN, 3);
            partialGradByTimes.resize(pieceN);

            return true;
        }

        inline void setInitial() {
            // 初始化时间分配直接拉满到速度
            const double allocationSpeed = magnitudeBd[0];

            if (waypoints.size() == 0) {
                // 如果只固定起点和终点，则只有一个自由waypoint
                free_points.resize(3, 1);
                // 将自由点设置为起点和终点的中间
                free_points = (tailPVA.col(0) + headPVA.col(0)) / 2;
                // 轨迹内部所有waypoint等于自由点
                points.col(0) = free_points.col(0);
                // 时间分配为最大速度跑直线
                times(0) = (tailPVA.col(0) - headPVA.col(0)).norm() / 2 / allocationSpeed;
                times(1) = times(0);
            } else {
                // 如果有固定的waypoint了
                free_points.resize(3, waypoints.cols() + 1);
                // 第一个自由点的坐标为第一个waypoint和起点中间
                free_points.col(0) = (waypoints.col(0) + headPVA.col(0)) / 2;
                // 中间的自由点坐标为两个相邻waypoint。
                for (int i = 0; i < waypoints.cols() - 1; i++) {
                    Vec3f midpt = (waypoints.col(i) + waypoints.col(i + 1)) / 2;
                    free_points.col(i + 1) = midpt;
                }

                free_points.rightCols(1) = (waypoints.rightCols(1) + tailPVA.col(0)) / 2;

                // 随后填充所有内部点。
                for (int i = 0; i < waypoints.cols(); i++) {
                    points.col(2 * (i)) = free_points.col(i);
                    points.col(2 * (i) + 1) = waypoints.col(i);
                }

                points.rightCols(1) = free_points.rightCols(1);

                // 计算时间分配
                Eigen::Vector3d lastP, curP, delta;
                curP = headPVA.col(0);
                for (int i = 0; i < temporalDim - 1; i++) {
                    lastP = curP;
                    curP = points.col(i);
                    delta = curP - lastP;
                    times(i) = delta.norm() / allocationSpeed;
                }
                delta = points.rightCols(1) - tailPVA.col(0);
                times(temporalDim - 1) = delta.norm() / allocationSpeed;
            }
            return;
        }

        inline double optimize(Trajectory &traj,
                               const double &relCostTol) {
            Eigen::VectorXd x(temporalDim + spatialDim);
            Eigen::Map<Eigen::VectorXd> tau(x.data(), temporalDim);
            Eigen::Map<Eigen::VectorXd> xi(x.data() + temporalDim, spatialDim);

            setInitial();
            for (int i = 0; i < free_points.cols(); i++) {
                xi.block<3, 1>(3 * i, 0) = free_points.col(i);
            }
            backwardT(times, tau);
            cout << times.transpose() << endl;
            iter_num = 0;
            evaluate_cost_dt = 0.0;
            double minCostFunctional;
            lbfgs_params.mem_size = 256;
            lbfgs_params.past = 3;
            lbfgs_params.min_step = 1.0e-32;
            lbfgs_params.g_epsilon = 0.0;
            lbfgs_params.delta = relCostTol;
            int ret = lbfgs::lbfgs_optimize(x,
                                            minCostFunctional,
                                            &GcopterWayptS3::costFunctional,
                                            nullptr,
                                            nullptr,
                                            this,
                                            lbfgs_params);
            if (ret >= 0) {
                times /= scale_factor;
                headPVA.col(1) *= scale_factor;
                headPVA.col(2) *= (scale_factor * scale_factor);
                tailPVA.col(1) *= scale_factor;
                tailPVA.col(2) *= (scale_factor * scale_factor);

                minco.setConditions(headPVA, tailPVA, temporalDim);
                minco.setParameters(points, times);
                minco.getTrajectory(traj);
            } else {
                traj.clear();
                minCostFunctional = INFINITY;
                cout << RED " -- [MINCO] TrajOpt failed: " << lbfgs::lbfgs_strerror(ret) << RESET << endl;
            }
            return minCostFunctional;
        }


    };

    class GcopterWayptS4 {
    public:
        GcopterWayptS4() = default;

        ~GcopterWayptS4() {}

        int iter_num;
        double evaluate_cost_dt;
    private:
        MINCO_S4NU minco;

        double rho;
        double scale_factor;
        StatePVAJ headPVAJ;
        StatePVAJ tailPVAJ;

        int pieceN;

        int spatialDim;
        int temporalDim;

        double smoothEps;
        int integralRes;
        Eigen::VectorXd magnitudeBd;
        Eigen::VectorXd penaltyWt;

        bool block_energy_cost;
        lbfgs::lbfgs_parameter_t lbfgs_params;

        Eigen::Matrix3Xd points;
        Eigen::Matrix3Xd waypoints;
        Eigen::Matrix3Xd free_points;
        Eigen::VectorXd times;

        Eigen::Matrix3Xd gradByPoints;
        Eigen::VectorXd gradByTimes;
        Eigen::MatrixX3d partialGradByCoeffs;
        Eigen::VectorXd partialGradByTimes;

        bool block_snap_cost;
        Eigen::VectorXd fix_times;


    private:

        // magnitudeBounds = [v_max, a_max, omg_max, theta_max, thrust_min, thrust_max, pos_margin]^T
        // penaltyWeights = [pos_weight, vel_weight, acc_weight, omg_weight, theta_weight, thrust_weight]^T
        // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
        //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
        static inline void attachPenaltyFunctional(const Eigen::VectorXd &T,
                                                   const Eigen::MatrixX3d &coeffs,
                                                   const double &smoothFactor,
                                                   const int &integralResolution,
                                                   const Eigen::VectorXd &magnitudeBounds,
                                                   const Eigen::VectorXd &penaltyWeights,
                                                   double &cost,
                                                   Eigen::VectorXd &gradT,
                                                   Eigen::MatrixX3d &gradC) {
            const double vmax = magnitudeBounds[0];
            const double amax = magnitudeBounds[1];
            const double jmax = magnitudeBounds[2];

            const double vmaxSqr = vmax * vmax;
            const double amaxSqr = amax * amax;
            const double jmaxSqr = jmax * jmax;

            const double weightVel = penaltyWeights[0];
            const double weightAcc = penaltyWeights[1];
            const double weightJer = penaltyWeights[2];

            Eigen::Vector3d pos, vel, acc, jer, sna;
            Eigen::Vector3d totalGradPos, totalGradVel, totalGradAcc, totalGradJer;

            double step, alpha;
            double s1, s2, s3, s4, s5, s6, s7;
            Eigen::Matrix<double, 8, 1> beta0, beta1, beta2, beta3, beta4;
            Eigen::Vector3d outerNormal;

            double violaVel, violaVelPena, violaVelPenaD;
            double violaAcc, violaAccPena, violaAccPenaD;
            double violaJer, violaJerPena, violaJerPenaD;
            Eigen::Matrix<double, 8, 3> gradViolaVc, gradViolaAc, gradViolaJc;
            double gradViolaVt, gradViolaAt, gradViolaJt;
            double node, pena;
            const int pieceNum = T.size();
            const double integralFrac = 1.0 / integralResolution;
            pena = 0.0;
            for (int i = 0; i < pieceNum; i++) {
                const Eigen::Matrix<double, 8, 3> &c = coeffs.block<8, 3>(i * 8, 0);
                step = T(i) * integralFrac;
                for (int j = 0; j <= integralResolution; j++) {
                    s1 = j * step;
                    s2 = s1 * s1;
                    s3 = s2 * s1;
                    s4 = s2 * s2;
                    s5 = s4 * s1;
                    s6 = s4 * s2;
                    s7 = s4 * s3;
                    beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5, beta0(
                            6) = s6, beta0(7) = s7;

                    beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4) =
                            4.0 * s3, beta1(5) = 5.0 * s4, beta1(6) = 6.0 * s5, beta1(7) = 7.0 * s6;

                    beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2, beta2(
                            5) = 20.0 * s3, beta2(6) = 30.0 * s4, beta2(7) = 42.0 * s5;

                    beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1, beta3(5) =
                            60.0 * s2, beta3(6) = 120.0 * s3, beta3(7) = 210.0 * s4;

                    beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0, beta4(5) =
                            120.0 * s1, beta4(6) = 360.0 * s2, beta4(7) = 840.0 * s3;

                    pos = c.transpose() * beta0;
                    vel = c.transpose() * beta1;
                    acc = c.transpose() * beta2;
                    jer = c.transpose() * beta3;
                    sna = c.transpose() * beta4;
                    alpha = j * integralFrac;

                    violaVel = vel.squaredNorm() - vmaxSqr;
                    violaAcc = acc.squaredNorm() - amaxSqr;
                    violaJer = jer.squaredNorm() - jmaxSqr;
                    node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;

                    if (weightVel > 0 && gcopter::smoothedL1(violaVel, smoothFactor, violaVelPena, violaVelPenaD)) {

                        gradViolaVc = 2.0 * beta1 * vel.transpose();
                        gradViolaVt = 2.0 * alpha * vel.transpose() * acc;
                        gradC.block<8, 3>(i * 8, 0) += node * step * weightVel * violaVelPenaD * gradViolaVc;
                        gradT(i) += node * (weightVel * violaVelPenaD * gradViolaVt * step +
                                            weightVel * violaVelPena * integralFrac);
                        pena += node * step * weightVel * violaVelPena;
                    }

                    if (weightAcc > 0 && gcopter::smoothedL1(violaAcc, smoothFactor, violaAccPena, violaAccPenaD)) {

                        gradViolaAc = 2.0 * beta2 * acc.transpose();
                        gradViolaAt = 2.0 * alpha * acc.transpose() * jer;
                        gradC.block<8, 3>(i * 8, 0) += node * step * weightAcc * violaAccPenaD * gradViolaAc;
                        gradT(i) += node * (weightAcc * violaAccPenaD * gradViolaAt * step +
                                            weightAcc * violaAccPena * integralFrac);
                        pena += node * step * weightAcc * violaAccPena;
                    }

                    if (weightJer > 0.0 && gcopter::smoothedL1(violaJer, smoothFactor, violaJerPena, violaJerPenaD)) {
                        gradViolaJc = 2.0 * beta3 * jer.transpose();
                        gradViolaJt = 2.0 * alpha * jer.transpose() * sna;
                        gradC.block<8, 3>(i * 8, 0) += node * step * weightJer * violaJerPenaD * gradViolaJc;
                        gradT(i) += node * (weightJer * violaJerPenaD * gradViolaJt * step +
                                            weightJer * violaJerPena * integralFrac);
                        pena += node * step * weightJer * violaJerPena;
                    }
                }

            }
            cost += pena;
            return;
        }

        static inline void forwardT(const Eigen::VectorXd &tau,
                                    Eigen::VectorXd &T) {
            const int sizeTau = tau.size();
            T.resize(sizeTau);
            for (int i = 0; i < sizeTau; i++) {
                T(i) = tau(i) > 0.0
                       ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
                       : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
            }
            return;
        }

        template<typename EIGENVEC>
        static inline void backwardT(const Eigen::VectorXd &T,
                                     EIGENVEC &tau) {
            const int sizeT = T.size();
            tau.resize(sizeT);
            for (int i = 0; i < sizeT; i++) {
                tau(i) = T(i) > 1.0
                         ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
                         : (1.0 - sqrt(2.0 / T(i) - 1.0));
            }

            return;
        }

        template<typename EIGENVEC>
        static inline void backwardGradT(const Eigen::VectorXd &tau,
                                         const Eigen::VectorXd &gradT,
                                         EIGENVEC &gradTau) {
            const int sizeTau = tau.size();
            gradTau.resize(sizeTau);
            double denSqrt;
            for (int i = 0; i < sizeTau; i++) {
                if (tau(i) > 0) {
                    gradTau(i) = gradT(i) * (tau(i) + 1.0);
                } else {
                    denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
                    gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
                }
            }

            return;
        }

        static inline double costFunctional(void *ptr,
                                            const Eigen::VectorXd &x,
                                            Eigen::VectorXd &g) {
            TimeConsuming t_("cost functional", false);
            GcopterWayptS4 &obj = *(GcopterWayptS4 *) ptr;
            const int dimTau = obj.temporalDim;
            const int dimXi = obj.spatialDim;
            const double weightT = obj.rho;
            Eigen::Map<const Eigen::VectorXd> tau(x.data(), dimTau);
            Eigen::Map<const Eigen::VectorXd> xi(x.data() + dimTau, dimXi);
            Eigen::Map<Eigen::VectorXd> gradTau(g.data(), dimTau);
            Eigen::Map<Eigen::VectorXd> gradXi(g.data() + dimTau, dimXi);
            obj.iter_num++;
            forwardT(tau, obj.times);
            for (int i = 0; i < obj.free_points.cols(); i++) {
                obj.free_points.col(i).x() = xi(3 * i);
                obj.free_points.col(i).y() = xi(3 * i + 1);
                obj.free_points.col(i).z() = xi(3 * i + 2);
                obj.points.col(i * 2) = obj.free_points.col(i);
            }
            double cost;
            obj.minco.setParameters(obj.points, obj.times);

            cost = 0;
            obj.partialGradByCoeffs.setZero();
            obj.partialGradByTimes.setZero();
            if (!obj.block_energy_cost) {
                obj.minco.getEnergy(cost);
                obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
                obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);
            }
            attachPenaltyFunctional(obj.times, obj.minco.getCoeffs(),
                                    obj.smoothEps, obj.integralRes,
                                    obj.magnitudeBd, obj.penaltyWt,
                                    cost, obj.partialGradByTimes, obj.partialGradByCoeffs);
            obj.minco.propogateGrad(obj.partialGradByCoeffs, obj.partialGradByTimes,
                                    obj.gradByPoints, obj.gradByTimes);
//            print(fg(color::yellow), "MINCO time cost {}\n",  weightT * obj.times.sum());
            cost += weightT * obj.times.sum();
//            print(fg(color::yellow), "MINCO Internal cost {}\n", cost);
            obj.gradByTimes.array() += weightT;

            backwardGradT(tau, obj.gradByTimes, gradTau);
            int idx = 0;
            for (int i = 0; i < obj.gradByPoints.cols(); i++) {
                if (i % 2 == 0) {
                    gradXi.segment(idx * 3, 3) = obj.gradByPoints.col(i);
                    idx++;
                }
            }
            obj.evaluate_cost_dt += t_.stop();
            return cost;
        }

    public:
        // magnitudeBounds = [v_max, acc_max, omg_max, theta_max, thrust_min, thrust_max, pos_margin]^T
        // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
        inline bool setup(const double &timeWeight,
                          const StatePVAJ &initialPVAJ,
                          const StatePVAJ &terminalPVAJ,
                          const Eigen::Matrix3Xd &_waypoints,
                          const double &smoothingFactor,
                          const int &integralResolution,
                          const Eigen::VectorXd &magnitudeBounds,
                          const Eigen::VectorXd &penaltyWeights,
                          const double _scale_factor = 1.0,
                          const bool _block_energy_cost = false) {
            block_energy_cost = _block_energy_cost;
            scale_factor = _scale_factor;
            rho = timeWeight;
            headPVAJ = initialPVAJ;
            tailPVAJ = terminalPVAJ;
            waypoints = _waypoints;

            headPVAJ.col(1) /= scale_factor;
            headPVAJ.col(2) /= (scale_factor * scale_factor);
            headPVAJ.col(3) /= (scale_factor * scale_factor * scale_factor);

            tailPVAJ.col(1) /= scale_factor;
            tailPVAJ.col(2) /= (scale_factor * scale_factor);
            tailPVAJ.col(3) /= (scale_factor * scale_factor * scale_factor);


            smoothEps = smoothingFactor;
            integralRes = integralResolution;
            magnitudeBd = magnitudeBounds;
            penaltyWt = penaltyWeights;

            magnitudeBd[0] = magnitudeBd[0] / scale_factor;
            magnitudeBd[1] = magnitudeBd[1] / (scale_factor * scale_factor);
            magnitudeBd[2] = magnitudeBd[2] / (scale_factor * scale_factor * scale_factor);


            pieceN = (waypoints.cols() + 1) * 2;
            temporalDim = pieceN;
            spatialDim = (waypoints.cols() + 1) * 3;
            // Setup for MINCO_S3NU, FlatnessMap, and L-BFGS solver
            minco.setConditions(headPVAJ, tailPVAJ, pieceN);

            // Allocate temp variables
            points.resize(3, 2 * waypoints.cols() + 1);
            free_points.resize(3, waypoints.cols() + 1);
            times.resize(pieceN);
            gradByPoints.resize(3, waypoints.cols() + 1);
            gradByTimes.resize(temporalDim);
            partialGradByCoeffs.resize(8 * pieceN, 3);
            partialGradByTimes.resize(pieceN);

            return true;
        }

        inline void setInitial() {
            // 初始化时间分配直接拉满到速度
            const double allocationSpeed = magnitudeBd[0];

            if (waypoints.size() == 0) {
                // 如果只固定起点和终点，则只有一个自由waypoint
                free_points.resize(3, 1);
                // 将自由点设置为起点和终点的中间
                free_points = (tailPVAJ.col(0) + headPVAJ.col(0)) / 2;
                // 轨迹内部所有waypoint等于自由点
                points.col(0) = free_points.col(0);
                // 时间分配为最大速度跑直线
                times(0) = (tailPVAJ.col(0) - headPVAJ.col(0)).norm() / 2 / allocationSpeed;
                times(1) = times(0);
            } else {
                // 如果有固定的waypoint了
                free_points.resize(3, waypoints.cols() + 1);
                // 第一个自由点的坐标为第一个waypoint和起点中间
                free_points.col(0) = (waypoints.col(0) + headPVAJ.col(0)) / 2;
                // 中间的自由点坐标为两个相邻waypoint。
                for (int i = 0; i < waypoints.cols() - 1; i++) {
                    Vec3f midpt = (waypoints.col(i) + waypoints.col(i + 1)) / 2;
                    free_points.col(i + 1) = midpt;
                }

                free_points.rightCols(1) = (waypoints.rightCols(1) + tailPVAJ.col(0)) / 2;

                // 随后填充所有内部点。
                for (int i = 0; i < waypoints.cols(); i++) {
                    points.col(2 * (i)) = free_points.col(i);
                    points.col(2 * (i) + 1) = waypoints.col(i);
                }

                points.rightCols(1) = free_points.rightCols(1);

                // 计算时间分配
                Eigen::Vector3d lastP, curP, delta;
                curP = headPVAJ.col(0);
                for (int i = 0; i < temporalDim - 1; i++) {
                    lastP = curP;
                    curP = points.col(i);
                    delta = curP - lastP;
                    times(i) = delta.norm() / allocationSpeed;
                }
                delta = points.rightCols(1) - tailPVAJ.col(0);
                if (delta.norm() == 0) {
                    cout << "delta norm is zero" << endl;
                    cout << free_points.rightCols(1).transpose() << endl;
                    cout << points.rightCols(1).transpose() << endl;
                    cout << tailPVAJ.col(0).transpose() << endl;
                    cout << "delta norm is zero" << endl;
                    cout << waypoints.transpose() << endl;
                }
                times(temporalDim - 1) = delta.norm() / allocationSpeed;
            }
            return;
        }

        inline double optimize(Trajectory &traj,
                               const double &relCostTol) {
            Eigen::VectorXd x(temporalDim + spatialDim);
            Eigen::Map<Eigen::VectorXd> tau(x.data(), temporalDim);
            Eigen::Map<Eigen::VectorXd> xi(x.data() + temporalDim, spatialDim);

            setInitial();
            for (int i = 0; i < free_points.cols(); i++) {
                xi.block<3, 1>(3 * i, 0) = free_points.col(i);
            }
            backwardT(times, tau);
            cout << times.transpose() << endl;
            iter_num = 0;
            evaluate_cost_dt = 0.0;
            double minCostFunctional;
            lbfgs_params.mem_size = 256;
            lbfgs_params.past = 3;
            lbfgs_params.min_step = 1.0e-32;
            lbfgs_params.g_epsilon = 0.0;
            lbfgs_params.delta = relCostTol;
            int ret = lbfgs::lbfgs_optimize(x,
                                            minCostFunctional,
                                            &GcopterWayptS4::costFunctional,
                                            nullptr,
                                            nullptr,
                                            this,
                                            lbfgs_params);
            if (ret >= 0) {
                times /= scale_factor;
                headPVAJ.col(1) *= scale_factor;
                headPVAJ.col(2) *= (scale_factor * scale_factor);
                headPVAJ.col(3) *= (scale_factor * scale_factor * scale_factor);
                tailPVAJ.col(1) *= scale_factor;
                tailPVAJ.col(2) *= (scale_factor * scale_factor);
                tailPVAJ.col(3) *= (scale_factor * scale_factor * scale_factor);

                minco.setConditions(headPVAJ, tailPVAJ, temporalDim);
                minco.setParameters(points, times);
                minco.getTrajectory(traj);
            } else {
                traj.clear();
                minCostFunctional = INFINITY;
                cout << RED << " -- [MINCO] TrajOpt failed, " << lbfgs::lbfgs_strerror(ret) << RESET << endl;
            }
            return minCostFunctional;
        }


    };

    class WayptTrajOpt {
    private:
        GcopterWayptS3 min_jert_opt_;
        GcopterWayptS4 min_snap_opt_;
        TrajOptConfig cfg_;
        ros::NodeHandle nh_;
        VecDf magnitude_bound, penna_weights;
    public:

        WayptTrajOpt(const ros::NodeHandle &nh,
                     TrajOptConfig &cfg) {
            cfg_ = cfg;
            nh_ = nh;
            magnitude_bound.resize(3);
            magnitude_bound(0) = cfg_.max_vel;
            magnitude_bound(1) = cfg_.max_acc;
            magnitude_bound(2) = cfg_.max_jerk;

            penna_weights.resize(3);
            penna_weights(0) = cfg_.penna_vel;
            penna_weights(1) = cfg_.penna_acc;
            penna_weights(2) = cfg_.penna_jerk;
        }

        typedef std::shared_ptr<WayptTrajOpt> Ptr;

        bool optimize(const StatePVAJ &init_state,
                      const StatePVAJ &goal_state,
                      const Mat3Df &way_pts,
                      Trajectory &out_traj) {

            if (cfg_.energy_cost_type == 3) {
                StatePVA istate, fstate;
                istate = init_state.block<3, 3>(0, 0);
                fstate = goal_state.block<3, 3>(0, 0);

                if (!min_jert_opt_.setup(cfg_.penna_t,
                                         istate,
                                         fstate,
                                         way_pts,
                                         cfg_.smooth_eps,
                                         cfg_.integral_reso,
                                         magnitude_bound,
                                         penna_weights
                )) {
                    cout << RED <<
                         " -- [WayptOpt] Error, failed to setup optimization problem, force return." << RESET << endl;
                    return false;
                }

                double res = cfg_.opt_accuracy;
                res = min_jert_opt_.optimize(out_traj, res);
                if (isinf(res)) {
                    cout << RED << " -- [WayptOpt] Error, optimization failed." << RESET << endl;
                    return false;
                }
            } else if (cfg_.energy_cost_type == 4) {
                if (!min_snap_opt_.setup(cfg_.penna_t,
                                         init_state,
                                         goal_state,
                                         way_pts,
                                         cfg_.smooth_eps,
                                         cfg_.integral_reso,
                                         magnitude_bound,
                                         penna_weights
                )) {
                    cout << RED <<
                         " -- [WayptOpt] Error, failed to setup optimization problem, force return." << RESET << endl;
                    return false;
                }

                double res = cfg_.opt_accuracy;
                res = min_snap_opt_.optimize(out_traj, res);
                if (isinf(res)) {
                    cout << RED << " -- [WayptOpt] Error, optimization failed." << RESET << endl;
                    return false;
                }
            }

            return true;
        }

    };


}
#endif // WAYPOINT_TRAJ_OPTIMIZER