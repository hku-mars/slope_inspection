//
// Created by yunfan on 2022/6/25.
//

#pragma once

#include <geometry_utils/trajectory.h>
#include <math_utils/lbfgs.h>


namespace optimization_utils {

    using namespace geometry_utils;
    using namespace type_utils;

    template<typename EIGENVEC>
    class Gcopter {
    public:
        Gcopter() = default;

        ~Gcopter() = default;

        static bool smoothedL1(const double &x,
                               const double &mu,
                               double &f,
                               double &df);


        static void forwardMapTauToT(const Eigen::VectorXd &tau,
                                     Eigen::VectorXd &T);

        static void backwardMapTToTau(const Eigen::VectorXd &T, EIGENVEC &tau);

        static void propagateGradientTToTau(const Eigen::VectorXd &tau,
                                            const Eigen::VectorXd &gradT,
                                            EIGENVEC &gradTau);

        static void mapIntervalToInf(const double &lower_bound,
                                     const double &upper_bound,
                                     const double &inter,
                                     double &inf);

        static void mapInfToInterval(const double &lower_bound,
                                     const double &upper_bound,
                                     const double &inf,
                                     double &inter);

        static void propagateGradIntervalToInf(const double &lower_bound,
                                               const double &upper_bound,
                                               const double &inf,
                                               const double &grad_inter,
                                               double &grad_inf);


        // For corridor optimization
        static void normRetrictionLayer(const Eigen::VectorXd &xi,
                                        const PolyhedronV &vPoly,
                                        double &cost,
                                        EIGENVEC &gradXi);

        static void backwardGradP(const Eigen::VectorXd &xi, const PolyhedronV &vPoly, const Eigen::Matrix3Xd &gradP,
                                  EIGENVEC &gradXi);

        static void backwardGradP(const Eigen::VectorXd &xi,
                                         const Eigen::VectorXi &vIdx,
                                         const PolyhedraV &vPolys,
                                         const Eigen::Matrix3Xd &gradP,
                                         EIGENVEC &gradXi);

        static void backwardP(const Eigen::Matrix3Xd &points, const PolyhedronV &vPoly, EIGENVEC &xi);

        static void forwardP(const Eigen::VectorXd &xi, const PolyhedronV &vPoly, Eigen::Matrix3Xd &P);

        static double costTinyNLS(void *ptr,
                                  const Eigen::VectorXd &xi,
                                  Eigen::VectorXd &gradXi);

        static void forwardP(const Eigen::VectorXd &xi,
                             const Eigen::VectorXi &vIdx,
                             const PolyhedraV &vPolys,
                             Eigen::Matrix3Xd &P);

        static void backwardP(const Eigen::Matrix3Xd &P,
                              const Eigen::VectorXi &vIdx,
                              const PolyhedraV &vPolys,
                              EIGENVEC &xi);

        static void normRetrictionLayer(const Eigen::VectorXd &xi,
                                        const Eigen::VectorXi &vIdx,
                                        const PolyhedraV &vPolys,
                                        double &cost,
                                        EIGENVEC &gradXi);
    };

    typedef Gcopter<Eigen::Map<Eigen::VectorXd>> gcopter;
}

