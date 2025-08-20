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

//
// Created by yunfan on 21/5/2023.
//

#include "math_utils/lbfgs.h"
#include <Eigen/Eigen>
#include <iostream>
#include <iomanip>

using namespace math_utils;

class MinimizationExample {
public:
    int run(const int N) {
        double finalCost;
        Eigen::VectorXd x(N);

        /* Set the initial guess */
        for (int i = 0; i < N; i += 2) {
            x(i) = -1.2;
            x(i + 1) = 1.0;
        }

        /* Set the minimization parameters */
        lbfgs::lbfgs_parameter_t params;
        params.g_epsilon = 1.0e-8;
        params.past = 3;
        params.delta = 1.0e-8;

        /* Start minimization */
        int ret = lbfgs::lbfgs_optimize(x,
                                        finalCost,
                                        costFunction,
                                        nullptr,
                                        monitorProgress,
                                        this,
                                        params);

        /* Report the result. */
        std::cout << std::setprecision(4)
                  << "================================" << std::endl
                  << "L-BFGS Optimization Returned: " << ret << std::endl
                  << "Minimized Cost: " << finalCost << std::endl
                  << "Optimal Variables: " << std::endl
                  << x.transpose() << std::endl;

        return ret;
    }

private:
    static double costFunction(void *instance,
                               const Eigen::VectorXd &x,
                               Eigen::VectorXd &g) {
        const int n = x.size();
        double fx = 0.0;
        for (int i = 0; i < n; i += 2) {
            const double t1 = 1.0 - x(i);
            const double t2 = 10.0 * (x(i + 1) - x(i) * x(i));
            g(i + 1) = 20.0 * t2;
            g(i) = -2.0 * (x(i) * g(i + 1) + t1);
            fx += t1 * t1 + t2 * t2;
        }
        return fx;
    }

    static int monitorProgress(void *instance,
                               const Eigen::VectorXd &x,
                               const Eigen::VectorXd &g,
                               const double fx,
                               const double step,
                               const int k,
                               const int ls) {
        std::cout << std::setprecision(4)
                  << "================================" << std::endl
                  << "Iteration: " << k << std::endl
                  << "Function Value: " << fx << std::endl
                  << "Gradient Inf Norm: " << g.cwiseAbs().maxCoeff() << std::endl
                  << "Variables: " << std::endl
                  << x.transpose() << std::endl;
        return 0;
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "test_mars_planning_utils");
    ros::NodeHandle nh("~");
    benchmark_utils::TimeConsuming tc("test", 100);

    double out = geometry_utils::toDeg(1.0);
    std::cout << out << std::endl;
    MinimizationExample example;
    example.run(200);
    geometry_utils::Trajectory traj;
    std::cout << traj.empty() << std::endl;

    geometry_utils::raycaster::RayCaster raycaster;
    raycaster.setInput(type_utils::Vec3f(0, 0, 0), type_utils::Vec3f(1, 1, 1));

    geometry_utils::Polytope polytope;
    std::cout << polytope.empty() << std::endl;

    ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("test", 1);
    visualization_utils::VisualUtils::VisualizePoint(pub, type_utils::Vec3f(0, 0, 0));
    return 0;
}