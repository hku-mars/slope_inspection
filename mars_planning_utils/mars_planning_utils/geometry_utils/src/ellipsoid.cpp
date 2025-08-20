#include <geometry_utils/ellipsoid.h>

using namespace geometry_utils;

bool Ellipsoid::empty() const {
    return undefined;
}

Ellipsoid::Ellipsoid(const Mat3f &C, const Vec3f &d) : C_(C), d_(d) {
    undefined = false;
    C_inv_ = C_.inverse();

    Eigen::JacobiSVD<Eigen::Matrix3d, Eigen::FullPivHouseholderQRPreconditioner> svd(C_, Eigen::ComputeFullU);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Vector3d S = svd.singularValues();
    if (U.determinant() < 0.0) {
        R_.col(0) = U.col(1);
        R_.col(1) = U.col(0);
        R_.col(2) = U.col(2);
        r_(0) = S(1);
        r_(1) = S(0);
        r_(2) = S(2);
    } else {
        R_ = U;
        r_ = S;
    }
}

Ellipsoid::Ellipsoid(const Mat3f &R, const Vec3f &r, const Vec3f &d)
        : R_(R), r_(r), d_(d) {
    undefined = false;
    C_ = R_ * r_.asDiagonal() * R_.transpose();
    C_inv_ = C_.inverse();
}

double Ellipsoid::pointDistaceToEllipsoid(const Vec3f &pt, Vec3f &closest_pt_on_ellip) const {
    /// step one: transform the point to the ellipsoid frame
    Vec3f pt_ellip_frame = R_.transpose() * (pt - d_);
    double dist = geometry_utils::DistancePointEllipsoid(r_(0), r_(1), r_(2),
                                                         pt_ellip_frame.x(),
                                                         pt_ellip_frame.y(),
                                                         pt_ellip_frame.z(),
                                                         closest_pt_on_ellip.x(),
                                                         closest_pt_on_ellip.y(),
                                                         closest_pt_on_ellip.z());
    /// step two: transform the closest point back to the world frame
    closest_pt_on_ellip = R_ * closest_pt_on_ellip + d_;
    return dist;
}

int Ellipsoid::nearestPointId(const Eigen::Matrix3Xd &pc) const {
    Eigen::VectorXd dists = (C_inv_ * (pc.colwise() - d_)).colwise().norm();
    int np_id;
    dists.minCoeff(&np_id);
    return np_id;
}

Vec3f Ellipsoid::nearestPoint(const Eigen::Matrix3Xd &pc) const {
    Eigen::VectorXd dists = (C_inv_ * (pc.colwise() - d_)).colwise().norm();
    int np_id;
    dists.minCoeff(&np_id);
    return pc.col(np_id);
}

double Ellipsoid::nearestPointDis(const Eigen::Matrix3Xd &pc, int &np_id) const {
    Eigen::VectorXd dists = (C_inv_ * (pc.colwise() - d_)).colwise().norm();
    double np_dist = dists.minCoeff(&np_id);
    return np_dist;
}

Mat3f Ellipsoid::C() const {
    return C_;
}

Vec3f Ellipsoid::d() const {
    return d_;
}

Mat3f Ellipsoid::R() const {
    return R_;
}

Vec3f Ellipsoid::r() const {
    return r_;
}


Vec3f Ellipsoid::toEllipsoidFrame(const Vec3f &pt_w) const {
    return C_inv_ * (pt_w - d_);
}

Eigen::Matrix3Xd Ellipsoid::toEllipsoidFrame(const Matrix3Xd &pc_w) const {
    return C_inv_ * (pc_w.colwise() - d_);
}

Vec3f Ellipsoid::toWorldFrame(const Vec3f &pt_e) const {
    return C_ * pt_e + d_;
}

Eigen::Matrix3Xd Ellipsoid::toWorldFrame(const Eigen::Matrix3Xd &pc_e) const {
    return (C_ * pc_e).colwise() + d_;
}

Eigen::Vector4d Ellipsoid::toEllipsoidFrame(const Eigen::Vector4d &plane_w) const {
    Eigen::Vector4d plane_e;
    plane_e.head(3) = plane_w.head(3).transpose() * C_;
    plane_e(3) = plane_w(3) + plane_w.head(3).dot(d_);
    return plane_e;
}

Eigen::Vector4d Ellipsoid::toWorldFrame(const Eigen::Vector4d &plane_e) const {
    Eigen::Vector4d plane_w;
    plane_w.head(3) = plane_e.head(3).transpose() * C_inv_;
    plane_w(3) = plane_e(3) - plane_w.head(3).dot(d_);
    return plane_w;
}

Eigen::MatrixX4d Ellipsoid::toEllipsoidFrame(const Eigen::MatrixX4d &planes_w) const {
    Eigen::MatrixX4d planes_e(planes_w.rows(), planes_w.cols());
    planes_e.leftCols(3) = planes_w.leftCols(3) * C_;
    planes_e.rightCols(1) = planes_w.rightCols(1) + planes_w.leftCols(3) * d_;
    return planes_e;
}

Eigen::MatrixX4d Ellipsoid::toWorldFrame(const Eigen::MatrixX4d &planes_e) const {
    Eigen::MatrixX4d planes_w(planes_e.rows(), planes_e.cols());
    planes_w.leftCols(3) = planes_e.leftCols(3) * C_inv_;
    planes_w.rightCols(1) = planes_e.rightCols(1) + planes_w.leftCols(3) * d_;
    return planes_w;
}

double Ellipsoid::dist(const Vec3f &pt_w) const {
    return (C_inv_ * (pt_w - d_)).norm();
}

Eigen::VectorXd Ellipsoid::dist(const Eigen::Matrix3Xd &pc_w) const {
    return (C_inv_ * (pc_w.colwise() - d_)).colwise().norm();
}

bool Ellipsoid::noPointsInside(vec_Vec3f &pc, const Eigen::Matrix3d R, const Vec3f &r, const Vec3f &p) const {
    Eigen::Matrix3d C_inv;
    C_inv = r.cwiseInverse().asDiagonal() * R.transpose();
    for (auto pt_w: pc) {
        double d = (C_inv * (pt_w - p)).norm();
        if (d <= 1) {
            return false;
        }
    }
    return true;
}

bool Ellipsoid::pointsInside(const Eigen::Matrix3Xd &pc, Mat3Df &out, int &min_pt_id) const {
    Eigen::VectorXd vec = (C_inv_ * (pc.colwise() - d_)).colwise().norm();
    vec_E<Vec3f> pts;
    pts.reserve(pc.cols());
    int cnt = 0;
    min_pt_id = 0;
    double min_dis = std::numeric_limits<double>::max();
    for (long int i = 0; i < vec.size(); i++) {
        if (vec(i) <= 1) {
            pts.push_back(pc.col(i));
            if (vec(i) <= min_dis) {
                min_pt_id = cnt;
                min_dis = vec(i);
            }
            cnt++;
        }
    }
    if (!pts.empty()) {
        out = Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>>(pts[0].data(), 3, pts.size());
        return true;
    } else {
        return false;
    }
}

bool Ellipsoid::inside(const Vec3f &pt) const {
    return dist(pt) <= 1;
}


