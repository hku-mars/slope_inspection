/* 
 * Copyright (c) 1990 Michael E. Hohmeyer, 
 *       hohmeyer@icemcfd.com
 * Permission is granted to modify and re-distribute this code in any manner
 * as long as this notice is preserved.  All standard disclaimers apply.
 * 
 * R. Seidel's algorithm for solving LPs (linear programs.)
 */

/* 
 * Copyright (c) 2021 Zhepei Wang,
 *       wangzhepei@live.com
 * 1. Bug fix in "move_to_front" function that "prev[m]" is illegally accessed
 *    while "prev" originally has only m ints. It is fixed by allocating a 
 *    "prev" with m + 1 ints.  
 * 2. Add Eigen interface.
 * 3. Resursive template.
 * Permission is granted to modify and re-distribute this code in any manner
 * as long as this notice is preserved.  All standard disclaimers apply.
 * 
 * Ref: Seidel, R. (1991), "Small-dimensional linear programming and convex 
 *      hulls made easy", Discrete & Computational Geometry 6 (1): 423–434, 
 *      doi:10.1007/BF02574699
 */


#pragma once

#include <Eigen/Eigen>
#include <cmath>
#include <random>

namespace math_utils {
    constexpr double eps = 1.0e-12;

    namespace sdlp {

        enum {
            /* minimum attained */
            MINIMUM = 0,
            /* no feasible region */
            INFEASIBLE,
            /* unbounded solution */
            UNBOUNDED,
            /* only a vertex in the solution set */
            AMBIGUOUS,
        };

        double dot2(const double a[2],
                           const double b[2]);

        double cross2(const double a[2],
                             const double b[2]);

        bool unit2(const double a[2],
                          double b[2]);

        /* unitize a d + 1 dimensional point */
        template<int d>
        bool unit(double *a);

        /* optimize the unconstrained objective */
        template<int d>
        int lp_no_con(const double *n_vec,
                             const double *d_vec,
                             double *opt);

        /* returns the plane index that is in i's place */
        int move_to_front(const int i,
                                 int *next,
                                 int *prev);

        void lp_min_lin_rat(const bool degen,
                                   const double cw_vec[2],
                                   const double ccw_vec[2],
                                   const double n_vec[2],
                                   const double d_vec[2],
                                   double opt[2]);

        int wedge(const double (*halves)[2],
                         const int m,
                         int *next,
                         int *prev,
                         double cw_vec[2],
                         double ccw_vec[2],
                         bool *degen);

        /* return the minimum on the projective line */
        int lp_base_case(const double (*halves)[2], /* halves --- half lines */
                                const int m,               /* m      --- terminal marker */
                                const double n_vec[2],     /* n_vec  --- numerator funciton */
                                const double d_vec[2],     /* d_vec  --- denominator function */
                                double opt[2],             /* opt    --- optimum  */
                                int *next,                 /* next, prev  --- double linked list of indices */
                                int *prev);

        /* find the largest coefficient in a plane */
        template<int d>
        void findimax(const double *pln,
                             int *imax);

        template<int d>
        void vector_up(const double *equation,
                              const int ivar,
                              const double *low_vector,
                              double *vector);

        template<int d>
        void vector_down(const double *elim_eqn,
                                const int ivar,
                                const double *old_vec,
                                double *new_vec);

        template<int d>
        void plane_down(const double *elim_eqn,
                               const int ivar,
                               const double *old_plane,
                               double *new_plane);

        template<int d>
        int linfracprog(const double *halves, /* halves  --- half spaces */
                               const int max_size,   /* max_size --- size of halves array */
                               const int m,          /* m       --- terminal marker */
                               const double *n_vec,  /* n_vec   --- numerator vector */
                               const double *d_vec,  /* d_vec   --- denominator vector */
                               double *opt,          /* opt     --- optimum */
                               double *work,         /* work    --- work space (see below) */
                               int *next,            /* next    --- array of indices into halves */
                               int *prev);           /* prev    --- array of indices into halves */
        /*
        **
        ** half-spaces are in the form
        ** halves[i][0]*x[0] + halves[i][1]*x[1] + 
        ** ... + halves[i][d-1]*x[d-1] + halves[i][d]*x[d] >= 0
        **
        ** coefficients should be normalized
        ** half-spaces should be in random order
        ** the order of the half spaces is 0, next[0] next[next[0]] ...
        ** and prev[next[i]] = i
        **
        ** halves: (max_size)x(d+1)
        **
        ** the optimum has been computed for the half spaces
        ** 0 , next[0], next[next[0]] , ... , prev[0]
        ** the next plane that needs to be tested is 0
        **
        ** m is the index of the first plane that is NOT on the list
        ** i.e. m is the terminal marker for the linked list.
        **
        ** the objective function is dot(x,nvec)/dot(x,dvec)
        ** if you want the program to solve standard d dimensional linear programming
        ** problems then n_vec = ( x0, x1, x2, ..., xd-1, 0)
        ** and           d_vec = (  0,  0,  0, ...,    0, 1)
        ** and halves[0] = (0, 0, ... , 1)
        **
        ** work points to (max_size+3)*(d+2)*(d-1)/2 double space
        */

        template<>
        int linfracprog<1>(const double *halves,
                                  const int max_size,
                                  const int m,
                                  const double *n_vec,
                                  const double *d_vec,
                                  double *opt,
                                  double *work,
                                  int *next,
                                  int *prev);

        inline void rand_permutation(const int n,
                                     int *p);

        template<int d>
        double linprog(const Eigen::Matrix<double, d, 1> &c,
                              const Eigen::Matrix<double, -1, d> &A,
                              const Eigen::Matrix<double, -1, 1> &b,
                              Eigen::Matrix<double, d, 1> &x);
        /*
        **  min cTx, s.t. Ax<=b
        **  dim(x) << dim(b)
        */


    }

}// namespace sdlp