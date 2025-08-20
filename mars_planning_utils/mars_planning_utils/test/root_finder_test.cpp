//
// Created by yunfan on 11/25/22.
//

#include <type_utils/common_type_name.h>
#include <benchmark_utils/scope_timer.h>
#include <geometry_utils/trajectory.h>
#include <geometry_utils/raycaster.h>
#include <geometry_utils/geometry_utils.h>
#include <geometry_utils/polytope.h>
#include <visualization_utils/visualization_utils.h>
#include <optimization_utils/waypoint_trajectory_optimizer.h>
#include <visualization_utils/visualization_utils.h>
//
// Created by yunfan on 21/5/2023.
//

#include "math_utils/lbfgs.h"
#include "math_utils/root_finder.h"
#include <Eigen/Eigen>
#include <iostream>
#include <iomanip>
#include <iomanip>

using namespace math_utils;
using namespace optimization_utils;

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_mars_planning_utils");
    ros::NodeHandle nh("~");

    ros::Publisher mkrarr_pub = nh.advertise<visualization_msgs::MarkerArray>("marker_array", 1);

    GcopterWayptS4 optimizer;

    StatePVAJ head_state, tail_state;
    head_state.setZero();
    tail_state.setZero();
    Mat3Df waypoitns;
    waypoitns.setZero(3, 2);
    waypoitns.col(0) = Vec3f(3, 0, 0);
    waypoitns.col(1) = Vec3f(6, 0, 0);
    tail_state.col(0) = Vec3f(10.0, 0, 0);
    VecDf magenitude_bound(3);
    magenitude_bound << 1, 3, -1;
    VecDf pennalty_weight(3);
    pennalty_weight << 1e9, 1e8, 1e8;


    optimizer.setup(1e6,
                    head_state,
                    tail_state,
                    waypoitns,
                    0.01,
                    10,
                    magenitude_bound,
                    pennalty_weight,
                    1.0,
                    false
    );
    Trajectory out_traj;
    double ret_cost;

    {
        TimeConsuming ttt("normal minco optimize");
        ret_cost = optimizer.optimize(out_traj, 1e-5);
    }


    sleep(1);


    VisualUtils::VisualizeTrajectory(mkrarr_pub, out_traj, Color::SteelBlue(), 0.1, "traj", 0.1, true, true);
    double max_vel;
    {
        TimeConsuming ttt("getMaxVelRate");
        max_vel = out_traj.getMaxVelRate();
    }

    double duration = out_traj.getDurations()[0];
    double jizhidian = 0;
    {
        TimeConsuming ttt("getExtremePoint");
        Eigen::MatrixXd nVelCoeffMat = out_traj[0].normalizeVelCoeffMat();
        Eigen::VectorXd coeff = math_utils::RootFinder::polySqr(nVelCoeffMat.row(0)) +
                                math_utils::RootFinder::polySqr(nVelCoeffMat.row(1)) +
                                math_utils::RootFinder::polySqr(nVelCoeffMat.row(2));

        int N = coeff.size();
        int n = N - 1;
        for (int i = 0; i < N; i++) {
            coeff(i) *= n;
            n--;
        }
        std::set<double> candidates = math_utils::RootFinder::solvePolynomial(coeff.head(N - 1), -0.11, 1.11,
                                                                              FLT_EPSILON / duration);
    }




    cout << std::setprecision(9);
    cout << GREEN << "Maximum veclocity: " << max_vel << RESET << endl;
    cout << GREEN << "Maximum acc: " << out_traj.getMaxAccRate() << RESET << endl;
    cout << std::setprecision(9) << GREEN << "Cost: " << ret_cost << RESET << endl;
    ros::spin();


    return 0;
}