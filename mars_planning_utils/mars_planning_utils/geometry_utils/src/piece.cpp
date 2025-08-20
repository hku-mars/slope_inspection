#include <geometry_utils/piece.h>


using namespace geometry_utils;

// Piece==================================================

int Piece::getDim() const {
    return 3;
}

int Piece::getDegree() const {
    return D;
}

double Piece::getDuration() const {
    return duration;
}

void Piece::setDuration(double dur) {
    duration = dur;
}

const Eigen::MatrixXd &Piece::getCoeffMat() const {
    return coeffMat;
}

Eigen::Vector3d Piece::getPos(const double &t) const {
    Eigen::Vector3d pos(0.0, 0.0, 0.0);
    double tn = 1.0;
    for (int i = D; i >= 0; i--) {
        pos += tn * coeffMat.col(i);
        tn *= t;
    }
    return pos;
}

Eigen::Vector3d Piece::getVel(const double &t) const {
    Eigen::Vector3d vel(0.0, 0.0, 0.0);
    double tn = 1.0;
    int n = 1;
    for (int i = D - 1; i >= 0; i--) {
        vel += n * tn * coeffMat.col(i);
        tn *= t;
        n++;
    }
    return vel;
}

Eigen::Vector3d Piece::getAcc(const double &t) const {
    Eigen::Vector3d acc(0.0, 0.0, 0.0);
    double tn = 1.0;
    int m = 1;
    int n = 2;
    for (int i = D - 2; i >= 0; i--) {
        acc += m * n * tn * coeffMat.col(i);
        tn *= t;
        m++;
        n++;
    }
    return acc;
}

Eigen::Vector3d Piece::getJer(const double &t) const {
    Eigen::Vector3d jer(0.0, 0.0, 0.0);
    double tn = 1.0;
    int l = 1;
    int m = 2;
    int n = 3;
    for (int i = D - 3; i >= 0; i--) {
        jer += l * m * n * tn * coeffMat.col(i);
        tn *= t;
        l++;
        m++;
        n++;
    }
    return jer;
}

Eigen::Vector3d Piece::getSnap(const double &t) const {
    Eigen::Vector3d sna(0.0, 0.0, 0.0);
    double tn = 1.0;
    int l = 1;
    int m = 2;
    int n = 3;
    int j = 4;
    for (int i = D - 4; i >= 0; i--) {
        sna += l * m * n * j * tn * coeffMat.col(i);
        tn *= t;
        l++;
        m++;
        n++;
        j++;
    }
    return sna;
}


Eigen::MatrixXd Piece::normalizePosCoeffMat() const {
    Eigen::MatrixXd nPosCoeffsMat(3, D + 1);
    double t = 1.0;
    for (int i = D; i >= 0; i--) {
        nPosCoeffsMat.col(i) = coeffMat.col(i) * t;
        t *= duration;
    }
    return nPosCoeffsMat;
}

Eigen::MatrixXd Piece::normalizeVelCoeffMat() const {
    Eigen::MatrixXd nVelCoeffMat(3, D);
    int n = 1;
    double t = duration;
    for (int i = D - 1; i >= 0; i--) {
        nVelCoeffMat.col(i) = n * coeffMat.col(i) * t;
        t *= duration;
        n++;
    }
    return nVelCoeffMat;
}

Eigen::MatrixXd Piece::normalizeAccCoeffMat() const {
    Eigen::MatrixXd nAccCoeffMat(3, D - 1);
    int n = 2;
    int m = 1;
    double t = duration * duration;
    for (int i = D - 2; i >= 0; i--) {
        nAccCoeffMat.col(i) = n * m * coeffMat.col(i) * t;
        n++;
        m++;
        t *= duration;
    }
    return nAccCoeffMat;
}

double Piece::getMaxVelRate() const {
    Eigen::MatrixXd nVelCoeffMat = normalizeVelCoeffMat();
    Eigen::VectorXd coeff = math_utils::RootFinder::polySqr(nVelCoeffMat.row(0)) +
                            math_utils::RootFinder::polySqr(nVelCoeffMat.row(1)) +
                            math_utils::RootFinder::polySqr(nVelCoeffMat.row(2));
    int N = coeff.size();
    int n = N - 1;
    for (int i = 0; i < N; i++) {
        coeff(i) *= n;
        n--;
    }
//        print("begin solve poly. {}\n", coeff.head(N - 1).squaredNorm());
    if (coeff.head(N - 1).squaredNorm() < 1e-6) {
        return getVel(0.0).norm();
    } else {
        double l = -0.0625;
        double r = 1.0625;
        int cnt = 0;
        while (fabs(math_utils::RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON) {
            l = 0.5 * l;
            if (cnt++ > 100) {
                std::cout << "math_utils::RootFinder stuck in inf-loop" << std::endl;
            }
        }
        cnt = 0;
        while (fabs(math_utils::RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON) {
            r = 0.5 * (r + 1.0);
            if (cnt++ > 100) {
                std::cout << "math_utils::RootFinder stuck in inf-loop" << std::endl;
            }
        }
        std::set<double> candidates = math_utils::RootFinder::solvePolynomial(coeff.head(N - 1), l, r,
                                                                              FLT_EPSILON / duration);
        candidates.insert(0.0);
        candidates.insert(1.0);
        double maxVelRateSqr = -INFINITY;
        double tempNormSqr;
        for (std::set<double>::const_iterator it = candidates.begin();
             it != candidates.end();
             it++) {
            if (0.0 <= *it && 1.0 >= *it) {
                tempNormSqr = getVel((*it) * duration).squaredNorm();
                maxVelRateSqr = maxVelRateSqr < tempNormSqr ? tempNormSqr : maxVelRateSqr;
            }
        }
        return sqrt(maxVelRateSqr);
    }
}

double Piece::getMaxAccRate() const {
    Eigen::MatrixXd nAccCoeffMat = normalizeAccCoeffMat();
    Eigen::VectorXd coeff = math_utils::RootFinder::polySqr(nAccCoeffMat.row(0)) +
                            math_utils::RootFinder::polySqr(nAccCoeffMat.row(1)) +
                            math_utils::RootFinder::polySqr(nAccCoeffMat.row(2));
    int N = coeff.size();
    int n = N - 1;
    for (int i = 0; i < N; i++) {
        coeff(i) *= n;
        n--;
    }
    if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON) {
        return getAcc(0.0).norm();
    } else {
        double l = -0.0625;
        double r = 1.0625;
        while (fabs(math_utils::RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON) {
            l = 0.5 * l;
        }
        while (fabs(math_utils::RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON) {
            r = 0.5 * (r + 1.0);
        }
        std::set<double> candidates = math_utils::RootFinder::solvePolynomial(coeff.head(N - 1), l, r,
                                                                              FLT_EPSILON / duration);
        candidates.insert(0.0);
        candidates.insert(1.0);
        double maxAccRateSqr = -INFINITY;
        double tempNormSqr;
        for (std::set<double>::const_iterator it = candidates.begin();
             it != candidates.end();
             it++) {
            if (0.0 <= *it && 1.0 >= *it) {
                tempNormSqr = getAcc((*it) * duration).squaredNorm();
                maxAccRateSqr = maxAccRateSqr < tempNormSqr ? tempNormSqr : maxAccRateSqr;
            }
        }
        return sqrt(maxAccRateSqr);
    }
}

Eigen::MatrixXd Piece::getState(const double &t) const {
    MatDf out_mat;
    out_mat.resize(3, 4);
    out_mat << getPos(t), getVel(t), getAcc(t), getJer(t);
    return out_mat;
}

bool Piece::checkMaxVelRate(const double &maxVelRate) const {
    double sqrMaxVelRate = maxVelRate * maxVelRate;
    if (getVel(0.0).squaredNorm() >= sqrMaxVelRate ||
        getVel(duration).squaredNorm() >= sqrMaxVelRate) {
        return false;
    } else {
        Eigen::MatrixXd nVelCoeffMat = normalizeVelCoeffMat();
        Eigen::VectorXd coeff = math_utils::RootFinder::polySqr(nVelCoeffMat.row(0)) +
                                math_utils::RootFinder::polySqr(nVelCoeffMat.row(1)) +
                                math_utils::RootFinder::polySqr(nVelCoeffMat.row(2));
        double t2 = duration * duration;
        coeff.tail<1>()(0) -= sqrMaxVelRate * t2;
        return math_utils::RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
    }
}

bool Piece::checkMaxAccRate(const double &maxAccRate) const {
    double sqrMaxAccRate = maxAccRate * maxAccRate;
    if (getAcc(0.0).squaredNorm() >= sqrMaxAccRate ||
        getAcc(duration).squaredNorm() >= sqrMaxAccRate) {
        return false;
    } else {
        Eigen::MatrixXd nAccCoeffMat = normalizeAccCoeffMat();
        Eigen::VectorXd coeff = math_utils::RootFinder::polySqr(nAccCoeffMat.row(0)) +
                                math_utils::RootFinder::polySqr(nAccCoeffMat.row(1)) +
                                math_utils::RootFinder::polySqr(nAccCoeffMat.row(2));
        double t2 = duration * duration;
        double t4 = t2 * t2;
        coeff.tail<1>()(0) -= sqrMaxAccRate * t4;
        return math_utils::RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
    }
}
