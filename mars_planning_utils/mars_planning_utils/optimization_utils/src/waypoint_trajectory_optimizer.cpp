#include <optimization_utils/waypoint_trajectory_optimizer.h>


namespace optimization_utils {


    double GcopterWayptS3::optimize(Trajectory &traj, const double &relCostTol) {
        Eigen::VectorXd x(temporalDim + spatialDim);
        Eigen::Map<Eigen::VectorXd> tau(x.data(), temporalDim);
        Eigen::Map<Eigen::VectorXd> xi(x.data() + temporalDim, spatialDim);

        setInitial();
        for (int i = 0; i < free_points.cols(); i++) {
            xi.block<3, 1>(3 * i, 0) = free_points.col(i);
        }
        gcopter::backwardMapTToTau(times, tau);
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

    bool GcopterWayptS3::setup(const double &timeWeight, const StatePVA &initialPVA,
                               const StatePVA &terminalPVA, const Eigen::Matrix3Xd &_waypoints,
                               const double &smoothingFactor, const int &integralResolution,
                               const Eigen::VectorXd &magnitudeBounds, const Eigen::VectorXd &penaltyWeights,
                               const double _scale_factor, const bool _block_energy_cost) {
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

    void GcopterWayptS3::setInitial() {
        // 初始化时间分配直接拉满到速度
        const double allocationSpeed = magnitudeBd[0];
        const double allocationAcc = magnitudeBd[1];

        if (waypoints.size() == 0) {
            // 如果只固定起点和终点，则只有一个自由waypoint
            free_points.resize(3, 1);
            // 将自由点设置为起点和终点的中间
            free_points = (tailPVA.col(0) + headPVA.col(0)) / 2;
            // 轨迹内部所有waypoint等于自由点
            points.col(0) = free_points.col(0);
            // 时间分配为最大速度跑直线
            times(0) =max((tailPVA.col(0) - headPVA.col(0)).norm() / allocationSpeed,
                          (headPVA.col(1) - tailPVA.col(1)).norm() / allocationAcc)  / 2 ;
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

    double GcopterWayptS3::costFunctional(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g) {
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
        gcopter::forwardMapTauToT(tau, obj.times);
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

        gcopter::propagateGradientTToTau(tau, obj.gradByTimes, gradTau);
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


    void GcopterWayptS3::attachPenaltyFunctional(const Eigen::VectorXd &T, const Eigen::MatrixX3d &coeffs,
                                                 const double &smoothFactor, const int &integralResolution,
                                                 const Eigen::VectorXd &magnitudeBounds,
                                                 const Eigen::VectorXd &penaltyWeights, double &cost,
                                                 Eigen::VectorXd &gradT, Eigen::MatrixX3d &gradC) {
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

    void GcopterWayptS4::attachPenaltyFunctional(const Eigen::VectorXd &T, const Eigen::MatrixX3d &coeffs,
                                                 const double &smoothFactor, const int &integralResolution,
                                                 const Eigen::VectorXd &magnitudeBounds,
                                                 const Eigen::VectorXd &penaltyWeights, double &cost,
                                                 Eigen::VectorXd &gradT, Eigen::MatrixX3d &gradC) {
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

    double GcopterWayptS4::costFunctional(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g) {
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
        gcopter::forwardMapTauToT(tau, obj.times);
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

        gcopter::propagateGradientTToTau(tau, obj.gradByTimes, gradTau);
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

    void GcopterWayptS4::setInitial() {
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

    bool GcopterWayptS4::setup(const double &timeWeight, const StatePVAJ &initialPVAJ, const StatePVAJ &terminalPVAJ,
                               const Eigen::Matrix3Xd &_waypoints, const double &smoothingFactor,
                               const int &integralResolution, const Eigen::VectorXd &magnitudeBounds,
                               const Eigen::VectorXd &penaltyWeights, const double _scale_factor,
                               const bool _block_energy_cost) {
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

    double GcopterWayptS4::optimize(Trajectory &traj, const double &relCostTol) {
        Eigen::VectorXd x(temporalDim + spatialDim);
        Eigen::Map<Eigen::VectorXd> tau(x.data(), temporalDim);
        Eigen::Map<Eigen::VectorXd> xi(x.data() + temporalDim, spatialDim);

        setInitial();
        for (int i = 0; i < free_points.cols(); i++) {
            xi.block<3, 1>(3 * i, 0) = free_points.col(i);
        }
        gcopter::backwardMapTToTau(times, tau);
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


}
