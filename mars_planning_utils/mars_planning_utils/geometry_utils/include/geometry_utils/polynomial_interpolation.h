#pragma once

#include <geometry_utils/trajectory.h>

namespace geometry_utils {
    namespace poly_interpo {
        template<int dim>
        static Trajectory minimumAccInterpolation(const Eigen::Matrix<double, dim, 2> &init_state,
                                                  const Eigen::Matrix<double, dim, 2> &goal_state,
                                                  const Eigen::Matrix<double, dim, -1> &waypoint,
                                                  const VecDf &ts) {

            Eigen::VectorXd T1;
            Eigen::VectorXd T2;
            Eigen::VectorXd T3;
            T1 = ts;
            T2 = T1.cwiseProduct(T1);
            T3 = T2.cwiseProduct(T1);
            int N = static_cast<int>(ts.size());
            BandedSystem A;
            MatDf b(4 * N, dim);
            A.create(4 * N, 4, 4);
            b.setZero();

            A(0, 0) = 1.0;
            A(1, 1) = 1.0;
            b.row(0) = init_state.col(0).transpose();
            b.row(1) = init_state.col(1).transpose();

            for (int i = 0; i < N - 1; i++) {
                A(4 * i + 2, 4 * i + 2) = 2.0;
                A(4 * i + 2, 4 * i + 3) = 6.0 * T1(i);
                A(4 * i + 2, 4 * i + 6) = -2.0;
                A(4 * i + 3, 4 * i) = 1.0;
                A(4 * i + 3, 4 * i + 1) = T1(i);
                A(4 * i + 3, 4 * i + 2) = T2(i);
                A(4 * i + 3, 4 * i + 3) = T3(i);
                A(4 * i + 4, 4 * i) = 1.0;
                A(4 * i + 4, 4 * i + 1) = T1(i);
                A(4 * i + 4, 4 * i + 2) = T2(i);
                A(4 * i + 4, 4 * i + 3) = T3(i);
                A(4 * i + 4, 4 * i + 4) = -1.0;
                A(4 * i + 5, 4 * i + 1) = 1.0;
                A(4 * i + 5, 4 * i + 2) = 2.0 * T1(i);
                A(4 * i + 5, 4 * i + 3) = 3.0 * T2(i);
                A(4 * i + 5, 4 * i + 5) = -1.0;

                b.row(4 * i + 3) = waypoint.col(i).transpose();
            }

            A(4 * N - 2, 4 * N - 4) = 1.0;
            A(4 * N - 2, 4 * N - 3) = T1(N - 1);
            A(4 * N - 2, 4 * N - 2) = T2(N - 1);
            A(4 * N - 2, 4 * N - 1) = T3(N - 1);
            A(4 * N - 1, 4 * N - 3) = 1.0;
            A(4 * N - 1, 4 * N - 2) = 2 * T1(N - 1);
            A(4 * N - 1, 4 * N - 1) = 3 * T2(N - 1);

            b.row(4 * N - 2) = goal_state.col(0).transpose();
            b.row(4 * N - 1) = goal_state.col(1).transpose();

            A.factorizeLU();
            A.solve(b);
            // b N * 8 , dim
            // c N * 8 , 3
            Trajectory traj;
            traj.reserve(N);
            MatDf c(N * 8, 3);
            c.setZero();
            for (int i = 0; i < dim; i++) {
                for (int k = 0; k < N; k++) {
                    c.block<4,1>(k*8,i)= b.block<4,1>(k*4,i);
                }
            }
            for (int i = 0; i < N; i++) {
                traj.emplace_back(T1(i), c.block<8, 3>(8 * i, 0).transpose().rowwise().reverse());
            }
            return traj;
        }

        template<int dim>
        static Trajectory minimumSnapInterpolation(const Eigen::Matrix<double, dim, 4> &init_state,
                                                   const Eigen::Matrix<double, dim, 4> &goal_state,
                                                   const Eigen::Matrix<double, dim, -1> &waypoint,
                                                   const VecDf &ts) {

            Eigen::VectorXd T1;
            Eigen::VectorXd T2;
            Eigen::VectorXd T3;
            Eigen::VectorXd T4;
            Eigen::VectorXd T5;
            Eigen::VectorXd T6;
            Eigen::VectorXd T7;
            int N = static_cast<int>(ts.size());
            BandedSystem A;
            MatDf b(8 * N, dim);
            T1 = ts;
            T2 = T1.cwiseProduct(T1);
            T3 = T2.cwiseProduct(T1);
            T4 = T2.cwiseProduct(T2);
            T5 = T4.cwiseProduct(T1);
            T6 = T4.cwiseProduct(T2);
            T7 = T4.cwiseProduct(T3);

            A.create(8 * N, 8, 8);
            b.setZero();

            A(0, 0) = 1.0;
            A(1, 1) = 1.0;
            A(2, 2) = 2.0;
            A(3, 3) = 6.0;
            b.row(0) = init_state.col(0).transpose();
            b.row(1) = init_state.col(1).transpose();
            b.row(2) = init_state.col(2).transpose();
            b.row(3) = init_state.col(3).transpose();

            for (int i = 0; i < N - 1; i++) {
                A(8 * i + 4, 8 * i + 4) = 24.0;
                A(8 * i + 4, 8 * i + 5) = 120.0 * T1(i);
                A(8 * i + 4, 8 * i + 6) = 360.0 * T2(i);
                A(8 * i + 4, 8 * i + 7) = 840.0 * T3(i);
                A(8 * i + 4, 8 * i + 12) = -24.0;

                A(8 * i + 5, 8 * i + 5) = 120.0;
                A(8 * i + 5, 8 * i + 6) = 720.0 * T1(i);
                A(8 * i + 5, 8 * i + 7) = 2520.0 * T2(i);
                A(8 * i + 5, 8 * i + 13) = -120.0;

                A(8 * i + 6, 8 * i + 6) = 720.0;
                A(8 * i + 6, 8 * i + 7) = 5040.0 * T1(i);
                A(8 * i + 6, 8 * i + 14) = -720.0;

                A(8 * i + 7, 8 * i) = 1.0;
                A(8 * i + 7, 8 * i + 1) = T1(i);
                A(8 * i + 7, 8 * i + 2) = T2(i);
                A(8 * i + 7, 8 * i + 3) = T3(i);
                A(8 * i + 7, 8 * i + 4) = T4(i);
                A(8 * i + 7, 8 * i + 5) = T5(i);
                A(8 * i + 7, 8 * i + 6) = T6(i);
                A(8 * i + 7, 8 * i + 7) = T7(i);

                A(8 * i + 8, 8 * i) = 1.0;
                A(8 * i + 8, 8 * i + 1) = T1(i);
                A(8 * i + 8, 8 * i + 2) = T2(i);
                A(8 * i + 8, 8 * i + 3) = T3(i);
                A(8 * i + 8, 8 * i + 4) = T4(i);
                A(8 * i + 8, 8 * i + 5) = T5(i);
                A(8 * i + 8, 8 * i + 6) = T6(i);
                A(8 * i + 8, 8 * i + 7) = T7(i);
                A(8 * i + 8, 8 * i + 8) = -1.0;

                A(8 * i + 9, 8 * i + 1) = 1.0;
                A(8 * i + 9, 8 * i + 2) = 2.0 * T1(i);
                A(8 * i + 9, 8 * i + 3) = 3.0 * T2(i);
                A(8 * i + 9, 8 * i + 4) = 4.0 * T3(i);
                A(8 * i + 9, 8 * i + 5) = 5.0 * T4(i);
                A(8 * i + 9, 8 * i + 6) = 6.0 * T5(i);
                A(8 * i + 9, 8 * i + 7) = 7.0 * T6(i);
                A(8 * i + 9, 8 * i + 9) = -1.0;

                A(8 * i + 10, 8 * i + 2) = 2.0;
                A(8 * i + 10, 8 * i + 3) = 6.0 * T1(i);
                A(8 * i + 10, 8 * i + 4) = 12.0 * T2(i);
                A(8 * i + 10, 8 * i + 5) = 20.0 * T3(i);
                A(8 * i + 10, 8 * i + 6) = 30.0 * T4(i);
                A(8 * i + 10, 8 * i + 7) = 42.0 * T5(i);
                A(8 * i + 10, 8 * i + 10) = -2.0;

                A(8 * i + 11, 8 * i + 3) = 6.0;
                A(8 * i + 11, 8 * i + 4) = 24.0 * T1(i);
                A(8 * i + 11, 8 * i + 5) = 60.0 * T2(i);
                A(8 * i + 11, 8 * i + 6) = 120.0 * T3(i);
                A(8 * i + 11, 8 * i + 7) = 210.0 * T4(i);
                A(8 * i + 11, 8 * i + 11) = -6.0;

                b.row(8 * i + 7) = waypoint.col(i).transpose();
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

            b.row(8 * N - 4) = goal_state.col(0).transpose();
            b.row(8 * N - 3) = goal_state.col(1).transpose();
            b.row(8 * N - 2) = goal_state.col(2).transpose();
            b.row(8 * N - 1) = goal_state.col(3).transpose();

            A.factorizeLU();
            A.solve(b);
            // b N * 8 , dim
            // c N * 8 , 3
            Trajectory traj;
            traj.reserve(N);
            MatDf c(N * 8, 3);
            c.setZero();
            for (int i = 0; i < dim; i++) {
                c.col(i) = b.col(i);
            }
            for (int i = 0; i < N; i++) {
                traj.emplace_back(T1(i), c.block<8, 3>(8 * i, 0).transpose().rowwise().reverse());
            }
            return traj;
        }

        template<int dim>
        static Trajectory minimumJerkInterpolation(const Eigen::Matrix<double, dim, 3> &init_state,
                                                   const Eigen::Matrix<double, dim, 3> &goal_state,
                                                   const Eigen::Matrix<double, dim, -1> &waypoint,
                                                   const VecDf &ts) {

            Eigen::VectorXd T1;
            Eigen::VectorXd T2;
            Eigen::VectorXd T3;
            Eigen::VectorXd T4;
            Eigen::VectorXd T5;
            int N = static_cast<int>(ts.size());
            BandedSystem A;
            MatDf b(6 * N, dim);
            T1 = ts;
            T2 = T1.cwiseProduct(T1);
            T3 = T2.cwiseProduct(T1);
            T4 = T2.cwiseProduct(T2);
            T5 = T4.cwiseProduct(T1);


            A.create(6 * N, 6, 6);
            b.setZero();

            A(0, 0) = 1.0;
            A(1, 1) = 1.0;
            A(2, 2) = 2.0;

            b.row(0) = init_state.col(0).transpose();
            b.row(1) = init_state.col(1).transpose();
            b.row(2) = init_state.col(2).transpose();

            for (int i = 0; i < N - 1; i++) {
                A(6 * i + 3, 6 * i + 3) = 6.0;
                A(6 * i + 3, 6 * i + 4) = 24.0 * T1(i);
                A(6 * i + 3, 6 * i + 5) = 60.0 * T2(i);
                A(6 * i + 3, 6 * i + 9) = -6.0;
                A(6 * i + 4, 6 * i + 4) = 24.0;
                A(6 * i + 4, 6 * i + 5) = 120.0 * T1(i);
                A(6 * i + 4, 6 * i + 10) = -24.0;
                A(6 * i + 5, 6 * i) = 1.0;
                A(6 * i + 5, 6 * i + 1) = T1(i);
                A(6 * i + 5, 6 * i + 2) = T2(i);
                A(6 * i + 5, 6 * i + 3) = T3(i);
                A(6 * i + 5, 6 * i + 4) = T4(i);
                A(6 * i + 5, 6 * i + 5) = T5(i);
                A(6 * i + 6, 6 * i) = 1.0;
                A(6 * i + 6, 6 * i + 1) = T1(i);
                A(6 * i + 6, 6 * i + 2) = T2(i);
                A(6 * i + 6, 6 * i + 3) = T3(i);
                A(6 * i + 6, 6 * i + 4) = T4(i);
                A(6 * i + 6, 6 * i + 5) = T5(i);
                A(6 * i + 6, 6 * i + 6) = -1.0;
                A(6 * i + 7, 6 * i + 1) = 1.0;
                A(6 * i + 7, 6 * i + 2) = 2 * T1(i);
                A(6 * i + 7, 6 * i + 3) = 3 * T2(i);
                A(6 * i + 7, 6 * i + 4) = 4 * T3(i);
                A(6 * i + 7, 6 * i + 5) = 5 * T4(i);
                A(6 * i + 7, 6 * i + 7) = -1.0;
                A(6 * i + 8, 6 * i + 2) = 2.0;
                A(6 * i + 8, 6 * i + 3) = 6 * T1(i);
                A(6 * i + 8, 6 * i + 4) = 12 * T2(i);
                A(6 * i + 8, 6 * i + 5) = 20 * T3(i);
                A(6 * i + 8, 6 * i + 8) = -2.0;

                b.row(6 * i + 5) = waypoint.col(i).transpose();
            }

            A(6 * N - 3, 6 * N - 6) = 1.0;
            A(6 * N - 3, 6 * N - 5) = T1(N - 1);
            A(6 * N - 3, 6 * N - 4) = T2(N - 1);
            A(6 * N - 3, 6 * N - 3) = T3(N - 1);
            A(6 * N - 3, 6 * N - 2) = T4(N - 1);
            A(6 * N - 3, 6 * N - 1) = T5(N - 1);
            A(6 * N - 2, 6 * N - 5) = 1.0;
            A(6 * N - 2, 6 * N - 4) = 2 * T1(N - 1);
            A(6 * N - 2, 6 * N - 3) = 3 * T2(N - 1);
            A(6 * N - 2, 6 * N - 2) = 4 * T3(N - 1);
            A(6 * N - 2, 6 * N - 1) = 5 * T4(N - 1);
            A(6 * N - 1, 6 * N - 4) = 2;
            A(6 * N - 1, 6 * N - 3) = 6 * T1(N - 1);
            A(6 * N - 1, 6 * N - 2) = 12 * T2(N - 1);
            A(6 * N - 1, 6 * N - 1) = 20 * T3(N - 1);

            b.row(6 * N - 3) = goal_state.col(0).transpose();
            b.row(6 * N - 2) = goal_state.col(1).transpose();
            b.row(6 * N - 1) = goal_state.col(2).transpose();

            A.factorizeLU();
            A.solve(b);
            // b N * 8 , dim
            // c N * 8 , 3
            Trajectory traj;
            traj.reserve(N);
            MatDf c(N * 6, 3);
            c.setZero();
            for (int i = 0; i < dim; i++) {
                c.col(i) = b.col(i);
            }
            for (int i = 0; i < N; i++) {
                traj.emplace_back(T1(i), c.block<6, 3>
                        (6 * i, 0).transpose().rowwise().reverse());
            }
            return traj;
        }
    }
}