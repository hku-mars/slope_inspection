#pragma once

#include <optimization_utils/minco.h>
#include <math_utils/lbfgs.h>
#include <benchmark_utils/scope_timer.h>
#include <geometry_utils/geometry_utils.h>
#include <type_utils/common_type_name.h>
#include <geometry_utils/polytope.h>
#include <optimization_utils/optimization_utils.h>


namespace optimization_utils {
    using namespace geometry_utils;
    using namespace visualization_utils;
    using namespace type_utils;
    using namespace benchmark_utils;
    using namespace std;

    typedef Gcopter<Eigen::Map<Eigen::VectorXd>> gcopter;

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
        StatePVA headPVA, tailPVA;

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
        static void attachPenaltyFunctional(const Eigen::VectorXd &T,
                                            const Eigen::MatrixX3d &coeffs,
                                            const double &smoothFactor,
                                            const int &integralResolution,
                                            const Eigen::VectorXd &magnitudeBounds,
                                            const Eigen::VectorXd &penaltyWeights,
                                            double &cost,
                                            Eigen::VectorXd &gradT,
                                            Eigen::MatrixX3d &gradC);


        static double costFunctional(void *ptr,
                                     const Eigen::VectorXd &x,
                                     Eigen::VectorXd &g);

        void setInitial();

    public:
        // magnitudeBounds = [v_max, acc_max, omg_max, theta_max, thrust_min, thrust_max, pos_margin]^T
        // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
        bool setup(const double &timeWeight,
                   const Eigen::Matrix3d &initialPVA,
                   const Eigen::Matrix3d &terminalPVA,
                   const Eigen::Matrix3Xd &_waypoints,
                   const double &smoothingFactor,
                   const int &integralResolution,
                   const Eigen::VectorXd &magnitudeBounds,
                   const Eigen::VectorXd &penaltyWeights,
                   const double _scale_factor = 1.0,
                   const bool _block_energy_cost = false);


        double optimize(Trajectory &traj,
                        const double &relCostTol);


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
        static void attachPenaltyFunctional(const Eigen::VectorXd &T,
                                            const Eigen::MatrixX3d &coeffs,
                                            const double &smoothFactor,
                                            const int &integralResolution,
                                            const Eigen::VectorXd &magnitudeBounds,
                                            const Eigen::VectorXd &penaltyWeights,
                                            double &cost,
                                            Eigen::VectorXd &gradT,
                                            Eigen::MatrixX3d &gradC);

        static double costFunctional(void *ptr,
                                     const Eigen::VectorXd &x,
                                     Eigen::VectorXd &g);

        void setInitial();

    public:
        // magnitudeBounds = [v_max, acc_max, omg_max, theta_max, thrust_min, thrust_max, pos_margin]^T
        // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
        bool setup(const double &timeWeight,
                   const StatePVAJ &initialPVAJ,
                   const StatePVAJ &terminalPVAJ,
                   const Eigen::Matrix3Xd &_waypoints,
                   const double &smoothingFactor,
                   const int &integralResolution,
                   const Eigen::VectorXd &magnitudeBounds,
                   const Eigen::VectorXd &penaltyWeights,
                   const double _scale_factor = 1.0,
                   const bool _block_energy_cost = false);


        double optimize(Trajectory &traj,
                        const double &relCostTol);

    };

}
