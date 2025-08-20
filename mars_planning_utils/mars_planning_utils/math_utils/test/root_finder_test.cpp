#include <iostream>
#include <iomanip>
#include <random>
#include <ctime>

#include "math_utils/root_finder.h"
#include "JenkinsTraub/jt.h"

using namespace std;
using namespace Eigen;
using namespace math_utils;
// Use Mersenne Twister as PRNG
std::mt19937_64 gen;
std::uniform_real_distribution<double> uniformReal(0.0, 1.0);

void testRootFinder(int order)
{
    int numRounds = 300000;
    double tol = 1e-8;
    int dispPrecision = 12;

    // Randomly geenrate polynomial whose all real roots lie in an interval
    set<double> realRoots;
    VectorXd coeffs(1);
    coeffs << 1;
    double tempRoot;
    for (int i = 1; i <= order; i++)
    {
        VectorXd b(2);
        tempRoot = (uniformReal(gen) - 0.5) * 10000;
        b << 1, -tempRoot;
        coeffs = RootFinder::polyConv(coeffs, b);
        realRoots.insert(tempRoot);
    }

    // Show ground truth
    cout << "------------------ Benchmark Companion Matrix Method, Roots Isolation Method, and Jenkins-Traub Algorithm ------------------" << endl;

    cout << "Ground Truth: " << endl;
    for (auto it = realRoots.begin(); it != realRoots.end(); it++)
    {
        cout << setprecision(dispPrecision) << *it << " ";
    }
    cout << endl
         << endl;

    // Running root finder for multiple rounds
    set<double> rts1, rts2, rts3;
    VectorXd coeffsReversed = coeffs.reverse();
    VectorXcd rts3Vec;
    clock_t ini = clock();
    for (int i = 0; i < numRounds; i++)
    {
        //Find all roots by eigen values of companion matrix
        rts1 = RootFinder::solvePolynomial(coeffs, -INFINITY, INFINITY, tol, false);
    }
    clock_t mid1 = clock();
    for (int i = 0; i < numRounds; i++)
    {
        //Find all roots by real roots isolation, i.e., in (-inf, inf)
        rts2 = RootFinder::solvePolynomial(coeffs, -INFINITY, INFINITY, tol);
    }
    clock_t mid2 = clock();
    for (int i = 0; i < numRounds; i++)
    {
        //Find all roots by Jenkins-Traub algorithm
        jt::findRootsJenkinsTraub(coeffsReversed, &rts3Vec);
    }
    clock_t fin = clock();

    // Results
    cout << "Average Execution Time for Companion Matrix Method: " << endl
         << (mid1 - ini) * 1.0 / numRounds / CLOCKS_PER_SEC << " secs" << endl
         << endl;

    cout << "Average Execution Time for Real Roots Isolation Method: " << endl
         << (mid2 - mid1) * 1.0 / numRounds / CLOCKS_PER_SEC << " secs" << endl
         << endl;

    cout << "Average Execution Time for Jenkins-Traub Algorithm: " << endl
         << (fin - mid2) * 1.0 / numRounds / CLOCKS_PER_SEC << " secs" << endl
         << endl;

    cout << "Roots Found by Companion Matrix Method: " << endl;
    for (auto it = rts1.begin(); it != rts1.end(); it++)
    {
        cout << setprecision(dispPrecision) << *it << " ";
    }
    cout << endl
         << endl;
    cout << "Roots Found by Real Roots Isolation Method: " << endl;
    for (auto it = rts2.begin(); it != rts2.end(); it++)
    {
        cout << setprecision(dispPrecision) << *it << " ";
    }
    cout << endl
         << endl;
    cout << "Roots Found by Jenkins-Traub Algorithm: " << endl;
    for (int i = 0; i < rts3Vec.size(); i++)
    {
        if (rts3Vec(i).imag() < tol)
        {
            rts3.insert(rts3Vec(i).real());
        }
    }
    for (auto it = rts3.begin(); it != rts3.end(); it++)
    {
        cout << setprecision(dispPrecision) << *it << " ";
    }
    cout << endl
         << endl;
}

int main(int argc, char **argv)
{

    testRootFinder(8);

    return 0;
}