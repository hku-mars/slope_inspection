#include <geometry_utils/geometry_utils.h>

using namespace type_utils;
namespace geometry_utils {
    // reference: https://www.geometrictools.com/Documentation/DistancePointEllipseEllipsoid.pdf
    double DistancePointEllipse(double e0, double e1, double y0, double y1, double &x0, double &x1) {
        double distance;
        double record_sign[2] = {1, 1};
        constexpr double eps = 1e-8;
        if (y0 < 0) {
            record_sign[0] = -1;
            y0 = -y0;
        }
        if (y1 < 0) {
            record_sign[1] = -1;
            y1 = -y1;
        }

        auto getRoot = [&](double r0, double z0, double z1, double g) {
            double n0 = r0 * z0;
            double s0 = z1 - 1, s1 = (g < 0 ? 0 : sqrt(n0 * n0 + z1 * z1) - 1);
            double s = 0;
            for (int i = 0; i < 10; ++i) {
                s = (s0 + s1) / 2;
                if (s == s0 || s == s1) {
                    break;
                }
                double ratio0 = n0 / (s + r0), ratio1 = z1 / (s + 1);
                g = ratio0 * ratio0 + ratio1 * ratio1 - 1;
                if (g > 0) {
                    s0 = s;
                } else if (g < 0) {
                    s1 = s;
                } else {
                    break;
                }
            }
            return s;
        };

        if (y1 > eps) {
            if (y0 > eps) {
                double z0 = y0 / e0, z1 = y1 / e1, g = z0 * z0 + z1 * z1 - 1;
                if (g != 0) {
                    double r0 = e0 * e0 / e1 / e1, sbar = getRoot(r0, z0, z1, g);
                    x0 = r0 * y0 / (sbar + r0);
                    x1 = y1 / (sbar + 1);
                    distance = sqrt((x0 - y0) * (x0 - y0) + (x1 - y1) * (x1 - y1));
                } else {
                    x0 = y0;
                    x1 = y1;
                    distance = 0;
                }
            } else { // y0 == 0
                x0 = 0;
                x1 = e1;
                distance = fabs(y1 - e1);
            }
        } else { // y1 == 0
            double numer0 = e0 * y0, denom0 = e0 * e0 - e1 * e1;
            if (numer0 < denom0) {
                double xde0 = numer0 / denom0;
                x0 = e0 * xde0;
                x1 = e1 * sqrt(1 - xde0 * xde0);
                distance = sqrt((x0 - y0) * (x0 - y0) + x1 * x1);
            } else {
                x0 = e0;
                x1 = 0;
                distance = fabs(y0 - e0);
            }
        }
        x0 *= record_sign[0];
        x1 *= record_sign[1];

        return distance;
    }

    double
    DistancePointEllipsoid(double e0, double e1, double e2, double y0, double y1, double y2, double &x0, double &x1,
                           double &x2) {
        auto getRoot = [&](double r0, double r1, double z0, double z1, double z2, double g) {
            double n0 = r0 * z0, n1 = r1 * z1;
            double s0 = z2 - 1, s1 = (g < 0 ? 0 : sqrt(n0 * n0 + n1 * n1 + z2 * z2) - 1);
            double s = 0;
            const int maxIterations = 10;
            for (int i = 0; i < maxIterations; ++i) {
                s = (s0 + s1) / 2;
                if (s == s0 || s == s1) {
                    break;
                }
                double ratio0 = n0 / (s + r0), ratio1 = n1 / (s + r1), ratio2 = z2 / (s + 1);
                g = (ratio0 * ratio0) + (ratio1 * ratio1) + (ratio2 * ratio2) - 1;
                if (g > 0) {
                    s0 = s;
                } else if (g < 0) {
                    s1 = s;
                } else {
                    break;
                }
            }
            return s;
        };
        constexpr double eps = 1e-8;
        double distance;
        double record_sign[3] = {1, 1, 1};

        if (y0 < 0) {
            record_sign[0] = -1;
            y0 = -y0;
        }
        if (y1 < 0) {
            record_sign[1] = -1;
            y1 = -y1;
        }
        if (y2 < 0) {
            record_sign[2] = -1;
            y2 = -y2;
        }

        if (y2 > eps) {
            if (y1 > eps) {
                if (y0 > eps) {
                    double z0 = y0 / e0, z1 = y1 / e1, z2 = y2 / e2;
                    double g = sqrt(z0 * z0 + z1 * z1 + z2 * z2) - 1;

                    if (g != 0) {
                        double r0 = e0 * e0 / e2 / e2, r1 = e1 * e1 / e2 / e2;
                        double sbar = getRoot(r0, r1, z0, z1, z2, g);

                        x0 = r0 * y0 / (sbar + r0);
                        x1 = r1 * y1 / (sbar + r1);
                        x2 = y2 / (sbar + 1);

                        distance = sqrt((x0 - y0) * (x0 - y0) +
                                        (x1 - y1) * (x1 - y1) +
                                        (x2 - y2) * (x2 - y2));
                    } else {
                        x0 = y0;
                        x1 = y1;
                        x2 = y2;
                        distance = 0;
                    }
                } else // y0 == 0
                {
                    x0 = 0;
                    distance = DistancePointEllipse(e1, e2, y1, y2, x1, x2);
                }
            } else // y1 == 0
            {
                if (y0 > 0) {
                    x1 = 0;
                    distance = DistancePointEllipse(e0, e2, y0, y2, x0, x2);
                } else // y0 == 0
                {
                    x0 = 0;
                    x1 = 0;
                    x2 = e2;
                    distance = fabs(y2 - e2);
                }
            }
        } else // y2 == 0
        {
            double denom0 = e0 * e0 - e2 * e2, denom1 = e1 * e1 - e2 * e2;
            double numer0 = e0 * y0, numer1 = e1 * y1;
            bool computed = false;

            if (numer0 < denom0 && numer1 < denom1) {
                double xde0 = numer0 / denom0, xde1 = numer1 / denom1;
                double discr = 1 - xde0 * xde0 - xde1 * xde1;

                if (discr > 0) {
                    x0 = e0 * xde0;
                    x1 = e1 * xde1;
                    x2 = e2 * sqrt(discr);

                    distance = sqrt((x0 - y0) * (x0 - y0) +
                                    (x1 - y1) * (x1 - y1) +
                                    x2 * x2);
                    computed = true;
                }
            }
            if (!computed) {
                x2 = 0;
                distance = DistancePointEllipse(e0, e1, y0, y1, x0, x1);
            }
        }
        x0 *= record_sign[0];
        x1 *= record_sign[1];
        x2 *= record_sign[2];
        return distance;
    }
}


template<typename Scalar_t>
Eigen::Matrix<Scalar_t, 3, 1> geometry_utils::quaternion_to_yrp(const Eigen::Quaternion<Scalar_t> &q_) {
    Eigen::Quaternion<Scalar_t> q = q_.normalized();

    Eigen::Matrix<Scalar_t, 3, 1> yrp;
    yrp(1) = asin(2 * (q.w() * q.x() + q.y() * q.z()));
    yrp(2) = -atan2(2 * (q.x() * q.z() - q.w() * q.y()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));
    yrp(0) = -atan2(2 * (q.x() * q.y() - q.w() * q.z()), 1 - 2 * (q.x() * q.x() + q.z() * q.z()));

    return yrp;
}

template Eigen::Matrix<double, 3, 1> geometry_utils::quaternion_to_yrp<double>(const Eigen::Quaternion<double> &q_);

template Eigen::Matrix<float, 3, 1> geometry_utils::quaternion_to_yrp<float>(const Eigen::Quaternion<float> &q_);


Vec4f geometry_utils::translatePlane(const Vec4f &plane, const Vec3f &translation) {
    Vec4f translatedPlane;
    translatedPlane.head(3) = plane.head(3);
    translatedPlane(3) = plane(3) - plane.head(3).dot(translation);
    return translatedPlane;
}

void geometry_utils::normalizeNextYaw(const double &last_yaw, double &yaw) {
    double diff = last_yaw - yaw;
    if (isnan(yaw)) {
        yaw = last_yaw;
        return;
    }

    while (fabs(diff) > M_PI) {
        if (diff > 0) {
            yaw += 2 * M_PI;
        } else {
            yaw -= 2 * M_PI;
        }
        diff = last_yaw - yaw;
    }
}

///============ 2023-3-12: add by Yunfan ============///
void geometry_utils::convertFlatOutputToAttAndOmg(const Vec3f &p,
                                                  const Vec3f &v,
                                                  const Vec3f &a,
                                                  const Vec3f &j,
                                                  const double &yaw,
                                                  const double &yaw_dot,
                                                  Vec3f &rpy,
                                                  Vec3f &omg,
                                                  double &aT
) {
    static const Vec3f grav = 9.80f * Vec3f(0, 0, 1);
    aT = (grav + a).norm();
    Vec3f xB, yB, zB;
    Vec3f xC(cos(yaw), sin(yaw), 0);

    zB = (grav + a).normalized();
    yB = ((zB).cross(xC)).normalized();
    xB = yB.cross(zB);
    Mat3f R;
    R << xB, yB, zB;
    Quatf q(R);
    double r, pit, y;
    tf::Quaternion tf_q;
    tf_q.setW(q.w());
    tf_q.setX(q.x());
    tf_q.setY(q.y());
    tf_q.setZ(q.z());
    tf::Matrix3x3(tf_q).getRPY(r, pit, y);
    rpy(0) = r;
    rpy(1) = pit;
    rpy(2) = y;
    Vec3f omega;
    Vec3f hw = (j - (zB.dot(j) * zB)) / aT;
    omega(0) = hw.dot(yB);
    omega(1) = hw.dot(xB);
    omega(2) = yaw_dot * (zB.dot(Vec3f(0, 0, 1)));
    omg = omega;
}


///============ 2022-12-10: add by Yunfan ============///
bool geometry_utils::pointInsidePolytope(const Vec3f &point, const PolyhedronH &polytope,
                                         double margin) {
    Eigen::Vector4d pt_e;
    pt_e.head(3) = point;
    pt_e(3) = 1;
    if ((polytope * pt_e).maxCoeff() > margin) {
        return false;
    }
    return true;
}

///============ 2022-12-5: add by Yunfan ============///
double geometry_utils::pointLineSegmentDistance(const Vec3f &p, const Vec3f &a, const Vec3f &b) {
    Vec3f ab = b - a;
    Vec3f ap = p - a;
    Vec3f bp = p - b;
    double e = ap.dot(ab);
    if (e <= 0) return ap.norm();
    double f = ab.dot(ab);
    if (e >= f) return bp.norm();
    return ap.cross(ab).norm() / ab.norm();
}

double geometry_utils::computePathLength(const type_utils::vec_E<Vec3f> &path) {
    if (path.size() < 2) {
        return 0.0;
    }
    double len = 0.0;
    for (size_t i = 0; i < path.size() - 1; i++) {
        len += (path[i] - path[i + 1]).norm();
    }
    return len;
}

///============ 2022-11-18 ====================================================================================
int geometry_utils::GetIntersection(float fDst1, float fDst2, Vec3f P1, Vec3f P2, Vec3f &Hit) {
    if ((fDst1 * fDst2) >= 0.0f) return 0;
    if (fDst1 == fDst2) return 0;
    Hit = P1 + (P2 - P1) * (-fDst1 / (fDst2 - fDst1));
    return 1;
}

int geometry_utils::InBox(Vec3f Hit, Vec3f B1, Vec3f B2, const int Axis) {
    if (Axis == 1 && Hit.z() > B1.z() && Hit.z() < B2.z() && Hit.y() > B1.y() && Hit.y() < B2.y()) return 1;
    if (Axis == 2 && Hit.z() > B1.z() && Hit.z() < B2.z() && Hit.x() > B1.x() && Hit.x() < B2.x()) return 1;
    if (Axis == 3 && Hit.x() > B1.x() && Hit.x() < B2.x() && Hit.y() > B1.y() && Hit.y() < B2.y()) return 1;
    return 0;
}

//The box in this article is Axis-Aligned and so can be defined by only two 3D points:
// B1 - the smallest values of X, Y, Z
//        B2 - the largest values of X, Y, Z
// returns true if line (L1, L2) intersects with the box (B1, B2)
// returns intersection point in Hit
int geometry_utils::lineIntersectBox(Vec3f L1, Vec3f L2, Vec3f B1, Vec3f B2, Vec3f &Hit) {
    if (L2.x() < B1.x() && L1.x() < B1.x()) return false;
    if (L2.x() > B2.x() && L1.x() > B2.x()) return false;
    if (L2.y() < B1.y() && L1.y() < B1.y()) return false;
    if (L2.y() > B2.y() && L1.y() > B2.y()) return false;
    if (L2.z() < B1.z() && L1.z() < B1.z()) return false;
    if (L2.z() > B2.z() && L1.z() > B2.z()) return false;

    // inside box seems intersect
//        if (L1.x() > B1.x() && L1.x() < B2.x() &&
//            L1.y() > B1.y() && L1.y() < B2.y() &&
//            L1.z() > B1.z() && L1.z() < B2.z()) {
//            Hit = L1;
//            return true;
//        }

    if ((GetIntersection(L1.x() - B1.x(), L2.x() - B1.x(), L1, L2, Hit) && InBox(Hit, B1, B2, 1))
        || (GetIntersection(L1.y() - B1.y(), L2.y() - B1.y(), L1, L2, Hit) && InBox(Hit, B1, B2, 2))
        || (GetIntersection(L1.z() - B1.z(), L2.z() - B1.z(), L1, L2, Hit) && InBox(Hit, B1, B2, 3))
        || (GetIntersection(L1.x() - B2.x(), L2.x() - B2.x(), L1, L2, Hit) && InBox(Hit, B1, B2, 1))
        || (GetIntersection(L1.y() - B2.y(), L2.y() - B2.y(), L1, L2, Hit) && InBox(Hit, B1, B2, 2))
        || (GetIntersection(L1.z() - B2.z(), L2.z() - B2.z(), L1, L2, Hit) && InBox(Hit, B1, B2, 3)))
        return true;

    return false;
}

type_utils::Vec3f geometry_utils::lineBoxIntersectPoint(const Vec3f &pt, const Vec3f &pos,
                                                        const Vec3f &box_min, const Vec3f &box_max) {
    Eigen::Vector3d diff = pt - pos;
    Eigen::Vector3d max_tc = box_max - pos;
    Eigen::Vector3d min_tc = box_min - pos;

    double min_t = 1000000;

    for (int i = 0; i < 3; ++i) {
        if (fabs(diff[i]) > 0) {

            double t1 = max_tc[i] / diff[i];
            if (t1 > 0 && t1 < min_t)
                min_t = t1;

            double t2 = min_tc[i] / diff[i];
            if (t2 > 0 && t2 < min_t)
                min_t = t2;
        }
    }

    return pos + (min_t - 1e-3) * diff;
}

///================================================================================================

Eigen::Matrix3d geometry_utils::RotationFromVec3(const Eigen::Vector3d &v) {
    // zero roll
    Eigen::Vector3d rpy(0, std::atan2(-v(2), v.topRows<2>().norm()),
                        std::atan2(v(1), v(0)));
    Eigen::Quaterniond qx(cos(rpy(0) / 2), sin(rpy(0) / 2), 0, 0);
    Eigen::Quaterniond qy(cos(rpy(1) / 2), 0, sin(rpy(1) / 2), 0);
    Eigen::Quaterniond qz(cos(rpy(2) / 2), 0, 0, sin(rpy(2) / 2));
    return Eigen::Matrix3d(qz * qy * qx);
}

// 通过三点获得一个平面
void geometry_utils::FromPointsToPlane(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3,
                                       Eigen::Vector4d &hPoly) {
    // Each row of hPoly is defined by h0, h1, h2, h3 as
    // h0*x + h1*y + h2*z + h3 <= 0
    hPoly(0) = ((p2.y() - p1.y()) * (p3.z() - p1.z()) - (p2.z() - p1.z()) * (p3.y() - p1.y()));
    hPoly(1) = ((p2.z() - p1.z()) * (p3.x() - p1.x()) - (p2.x() - p1.x()) * (p3.z() - p1.z()));
    hPoly(2) = ((p2.x() - p1.x()) * (p3.y() - p1.y()) - (p2.y() - p1.y()) * (p3.x() - p1.x()));
    hPoly(3) = (0 - (hPoly(0) * p1.x() + hPoly(1) * p1.y() + hPoly(2) * p1.z()));
}

void geometry_utils::GetFovCheckPlane(const Eigen::Matrix3d R, const Eigen::Vector3d t, Eigen::MatrixX4d &fov_planes,
                                      std::vector<Eigen::Matrix3d> &fov_pts) {
    // 只使用上下两个切面来约束。
    fov_planes.resize(2, 4);
    fov_pts.clear();
    static Eigen::Vector3d inner_pt(1, 0, 0);
    static constexpr double rad60 = (35.0 / 180.0 * M_PI);
    static constexpr double radm5 = (-5.0 / 180.0 * M_PI);
    static double z60 = sin(rad60) * 5;
    static double r60 = cos(rad60) * 5;
    static double zm5 = sin(radm5) * 5;
    static double rm5 = cos(radm5) * 5;

    Eigen::Matrix3Xd four_pts(3, 4);
    four_pts.col(0) = Eigen::Vector3d(r60, -3, z60);
    four_pts.col(1) = Eigen::Vector3d(r60, 3, z60);
    four_pts.col(2) = Eigen::Vector3d(rm5, -3, zm5);
    four_pts.col(3) = Eigen::Vector3d(rm5, 3, zm5);
    four_pts = (R * four_pts).colwise() + t;
    Eigen::Vector3d fov_inner_pt = R * inner_pt + t;
    Eigen::Vector4d temp;
    FromPointsToPlane(four_pts.col(0), four_pts.col(1), t, temp);
    if (temp.head(3).dot(fov_inner_pt) + temp(3) > 0) {
        temp = -temp;
    }

    Eigen::Matrix3d temp_p;
    temp_p << four_pts.col(0), four_pts.col(1), t;
    fov_pts.push_back(temp_p);
    temp_p << four_pts.col(2), four_pts.col(3), t;
    fov_pts.push_back(temp_p);
    fov_planes.row(0) = temp;
    FromPointsToPlane(four_pts.col(2), four_pts.col(3), t, temp);
    if (temp.head(3).dot(fov_inner_pt) + temp(3) > 0) {
        temp = -temp;
    }

    fov_planes.row(1) = temp;
}

void geometry_utils::GetFovPlanes(const Eigen::Matrix3d R, const Eigen::Vector3d t, Eigen::MatrixX4d &fov_planes,
                                  std::vector<Eigen::Matrix3d> &fov_pts) {
    fov_planes.resize(10, 4);
    fov_pts.clear();
    // 1) 获得前三个面
    static const double sqrt2 = sqrt(2);
    static Eigen::Vector3d center(0, 0, 0);
    static Eigen::Vector3d inner_pt(0, 0, -1);
    static Eigen::Vector3d inner_pt2(0, 0, 1);
    static constexpr double rad60 = (35.0 / 180.0 * M_PI);
    static double z = sin(rad60) * 5;
    static double r = cos(rad60) * 5;
    Eigen::Matrix3Xd ten_pts(3, 10);
    ten_pts.col(0) = (Eigen::Vector3d(0, r, z));
    ten_pts.col(1) = (Eigen::Vector3d(r / sqrt2, r / sqrt2, z));
    ten_pts.col(2) = (Eigen::Vector3d(r, 0, z));
    ten_pts.col(3) = (Eigen::Vector3d(r / sqrt2, -r / sqrt2, z));
    ten_pts.col(4) = (Eigen::Vector3d(0, -r, z));


    // 2) 获得下三个面
    constexpr double rad85 = (55.0 / 180.0 * M_PI);
    static double z85 = -cos(rad85) * 5;
    static double r85 = sin(rad85) * 5;
    ten_pts.col(5) = (Eigen::Vector3d(0, r85, z85));
    ten_pts.col(6) = (Eigen::Vector3d(r85 / sqrt2, r85 / sqrt2, z85));
    ten_pts.col(7) = (Eigen::Vector3d(r85, 0, z85));
    ten_pts.col(8) = (Eigen::Vector3d(r85 / sqrt2, -r85 / sqrt2, z85));
    ten_pts.col(9) = (Eigen::Vector3d(0, -r85, z85));

    // 3) 旋转所有点
    ten_pts = (R * ten_pts).colwise() + t;
    Eigen::Vector3d fov_inner_pt = R * inner_pt + t;
    Eigen::Vector3d fov_inner_pt2 = R * inner_pt2 + t;
    Eigen::Matrix3d fov_pt;
    for (int i = 0; i < 4; i++) {
        Eigen::Vector4d temp;
        FromPointsToPlane(ten_pts.col(i), ten_pts.col(i + 1), t, temp);
        fov_pt << ten_pts.col(i), ten_pts.col(i + 1), t;
        fov_pts.push_back(fov_pt);
        if (temp.head(3).dot(fov_inner_pt) + temp(3) > 0) {
            temp = -temp;
        }
        fov_planes.row(i) = temp;
    }
    for (int i = 5; i < 9; i++) {
        Eigen::Vector4d temp;
        FromPointsToPlane(ten_pts.col(i), ten_pts.col(i + 1), t, temp);
        fov_pt << ten_pts.col(i), ten_pts.col(i + 1), t;
        fov_pts.push_back(fov_pt);
        if (temp.head(3).dot(fov_inner_pt2) + temp(3) > 0) {
            temp = -temp;
        }
        fov_planes.row(i) = temp;
    }

    // 4）转为平面方程输出
}


double geometry_utils::findInteriorDist(const Eigen::MatrixX4d &hPoly,
                                        Eigen::Vector3d &interior) {
    const int m = hPoly.rows();

    Eigen::MatrixX4d A(m, 4);
    Eigen::VectorXd b(m);
    Eigen::Vector4d c, x;
    const Eigen::ArrayXd hNorm = hPoly.leftCols<3>().rowwise().norm();
    A.leftCols<3>() = hPoly.leftCols<3>().array().colwise() / hNorm;
    A.rightCols<1>().setConstant(1.0);
    b = -hPoly.rightCols<1>().array() / hNorm;
    c.setZero();
    c(3) = -1.0;

    const double minmaxsd = sdlp::linprog<4>(c, A, b, x);
    interior = x.head<3>();
    return -minmaxsd;
}

// Each row of hPoly is defined by h0, h1, h2, h3 as
// h0*x + h1*y + h2*z + h3 <= 0
bool geometry_utils::findInterior(const Eigen::MatrixX4d &hPoly,
                                  Eigen::Vector3d &interior) {
    const int m = hPoly.rows();

    Eigen::MatrixX4d A(m, 4);
    Eigen::VectorXd b(m);
    Eigen::Vector4d c, x;
    const Eigen::ArrayXd hNorm = hPoly.leftCols<3>().rowwise().norm();
    A.leftCols<3>() = hPoly.leftCols<3>().array().colwise() / hNorm;
    A.rightCols<1>().setConstant(1.0);
    b = -hPoly.rightCols<1>().array() / hNorm;
    c.setZero();
    c(3) = -1.0;

    const double minmaxsd = sdlp::linprog<4>(c, A, b, x);
    interior = x.head<3>();

    return minmaxsd < 0.0 && !std::isinf(minmaxsd);
}

bool geometry_utils::overlap(const Eigen::MatrixX4d &hPoly0,
                             const Eigen::MatrixX4d &hPoly1,
                             const double eps) {
    const int m = hPoly0.rows();
    const int n = hPoly1.rows();
    Eigen::MatrixX4d A(m + n, 4);
    Eigen::Vector4d c, x;
    Eigen::VectorXd b(m + n);
    A.leftCols<3>().topRows(m) = hPoly0.leftCols<3>();
    A.leftCols<3>().bottomRows(n) = hPoly1.leftCols<3>();
    A.rightCols<1>().setConstant(1.0);
    b.topRows(m) = -hPoly0.rightCols<1>();
    b.bottomRows(n) = -hPoly1.rightCols<1>();
    c.setZero();
    c(3) = -1.0;

    const double minmaxsd = sdlp::linprog<4>(c, A, b, x);

    return minmaxsd < -eps && !std::isinf(minmaxsd);
}

void geometry_utils::filterVs(const Eigen::Matrix3Xd &rV,
                              const double &epsilon,
                              Eigen::Matrix3Xd &fV) {
    const double mag = std::max(fabs(rV.maxCoeff()), fabs(rV.minCoeff()));
    const double res = mag * std::max(fabs(epsilon) / mag, DBL_EPSILON);
    std::set<Eigen::Vector3d, filterLess> filter;
    fV = rV;
    int offset = 0;
    Eigen::Vector3d quanti;
    for (int i = 0; i < rV.cols(); i++) {
        quanti = (rV.col(i) / res).array().round();
        if (filter.find(quanti) == filter.end()) {
            filter.insert(quanti);
            fV.col(offset) = rV.col(i);
            offset++;
        }
    }
    fV = fV.leftCols(offset).eval();
    return;
}

// Each row of hPoly is defined by h0, h1, h2, h3 as
// h0*x + h1*y + h2*z + h3 <= 0
// proposed epsilon is 1.0e-6
void geometry_utils::enumerateVs(const Eigen::MatrixX4d &hPoly,
                                 const Eigen::Vector3d &inner,
                                 Eigen::Matrix3Xd &vPoly,
                                 const double epsilon) {
    const Eigen::VectorXd b = -hPoly.rightCols<1>() - hPoly.leftCols<3>() * inner;
    const Eigen::Matrix<double, 3, -1, Eigen::ColMajor> A =
            (hPoly.leftCols<3>().array().colwise() / b.array()).transpose();

    QuickHull<double> qh;
    const double qhullEps = std::min(epsilon, defaultEps<double>());
    // CCW is false because the normal in quickhull towards interior
    const auto cvxHull = qh.getConvexHull(A.data(), A.cols(), false, true, qhullEps);
    const auto &idBuffer = cvxHull.getIndexBuffer();
    const int hNum = idBuffer.size() / 3;
    Eigen::Matrix3Xd rV(3, hNum);
    Eigen::Vector3d normal, point, edge0, edge1;
    for (int i = 0; i < hNum; i++) {
        point = A.col(idBuffer[3 * i + 1]);
        edge0 = point - A.col(idBuffer[3 * i]);
        edge1 = A.col(idBuffer[3 * i + 2]) - point;
        normal = edge0.cross(edge1); //cross in CW gives an outter normal
        rV.col(i) = normal / normal.dot(point);
    }
    filterVs(rV, epsilon, vPoly);
    vPoly = (vPoly.array().colwise() + inner.array()).eval();
}

// Each row of hPoly is defined by h0, h1, h2, h3 as
// h0*x + h1*y + h2*z + h3 <= 0
// proposed epsilon is 1.0e-6
bool geometry_utils::enumerateVs(const Eigen::MatrixX4d &hPoly,
                                 Eigen::Matrix3Xd &vPoly,
                                 const double epsilon) {
    Eigen::Vector3d inner;
    if (findInterior(hPoly, inner)) {
        enumerateVs(hPoly, inner, vPoly, epsilon);
        return true;
    }
    return false;
}


template<typename Scalar_t>
Scalar_t geometry_utils::toRad(const Scalar_t &x) {
    return x / 180.0 * M_PI;
}

template<typename Scalar_t>
Scalar_t geometry_utils::toDeg(const Scalar_t &x) {
    return x * 180.0 / M_PI;
}

template<typename Scalar_t>
Eigen::Matrix<Scalar_t, 3, 3> geometry_utils::rotx(Scalar_t t) {
    Eigen::Matrix<Scalar_t, 3, 3> R;
    R(0, 0) = 1.0;
    R(0, 1) = 0.0;
    R(0, 2) = 0.0;
    R(1, 0) = 0.0;
    R(1, 1) = std::cos(t);
    R(1, 2) = -std::sin(t);
    R(2, 0) = 0.0;
    R(2, 1) = std::sin(t);
    R(2, 2) = std::cos(t);

    return R;
}

template<typename Scalar_t>
Eigen::Matrix<Scalar_t, 3, 3> geometry_utils::roty(Scalar_t t) {
    Eigen::Matrix<Scalar_t, 3, 3> R;
    R(0, 0) = std::cos(t);
    R(0, 1) = 0.0;
    R(0, 2) = std::sin(t);
    R(1, 0) = 0.0;
    R(1, 1) = 1.0;
    R(1, 2) = 0;
    R(2, 0) = -std::sin(t);
    R(2, 1) = 0.0;
    R(2, 2) = std::cos(t);

    return R;
}

template<typename Scalar_t>
Eigen::Matrix<Scalar_t, 3, 3> geometry_utils::rotz(Scalar_t t) {
    Eigen::Matrix<Scalar_t, 3, 3> R;
    R(0, 0) = std::cos(t);
    R(0, 1) = -std::sin(t);
    R(0, 2) = 0.0;
    R(1, 0) = std::sin(t);
    R(1, 1) = std::cos(t);
    R(1, 2) = 0.0;
    R(2, 0) = 0.0;
    R(2, 1) = 0.0;
    R(2, 2) = 1.0;

    return R;
}

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> geometry_utils::ypr_to_R(const Eigen::DenseBase<Derived> &ypr) {
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
    EIGEN_STATIC_ASSERT(Derived::RowsAtCompileTime == 3, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
    EIGEN_STATIC_ASSERT(Derived::ColsAtCompileTime == 1, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

    typename Derived::Scalar c, s;

    Eigen::Matrix<typename Derived::Scalar, 3, 3> Rz = Eigen::Matrix<typename Derived::Scalar, 3, 3>::Zero();
    typename Derived::Scalar y = ypr(0);
    c = cos(y);
    s = sin(y);
    Rz(0, 0) = c;
    Rz(1, 0) = s;
    Rz(0, 1) = -s;
    Rz(1, 1) = c;
    Rz(2, 2) = 1;

    Eigen::Matrix<typename Derived::Scalar, 3, 3> Ry = Eigen::Matrix<typename Derived::Scalar, 3, 3>::Zero();
    typename Derived::Scalar p = ypr(1);
    c = cos(p);
    s = sin(p);
    Ry(0, 0) = c;
    Ry(2, 0) = -s;
    Ry(0, 2) = s;
    Ry(2, 2) = c;
    Ry(1, 1) = 1;

    Eigen::Matrix<typename Derived::Scalar, 3, 3> Rx = Eigen::Matrix<typename Derived::Scalar, 3, 3>::Zero();
    typename Derived::Scalar r = ypr(2);
    c = cos(r);
    s = sin(r);
    Rx(1, 1) = c;
    Rx(2, 1) = s;
    Rx(1, 2) = -s;
    Rx(2, 2) = c;
    Rx(0, 0) = 1;

    Eigen::Matrix<typename Derived::Scalar, 3, 3> R = Rz * Ry * Rx;
    return R;
}

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3>
geometry_utils::vec_to_R(const Eigen::MatrixBase<Derived> &v1, const Eigen::MatrixBase<Derived> &v2) {
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
    EIGEN_STATIC_ASSERT(Derived::RowsAtCompileTime == 3, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
    EIGEN_STATIC_ASSERT(Derived::ColsAtCompileTime == 1, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

    Eigen::Quaternion<typename Derived::Scalar> q = Eigen::Quaternion<typename Derived::Scalar>::FromTwoVectors(
            v1,
            v2);

    return q.matrix();
}

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 1> geometry_utils::R_to_ypr(const Eigen::DenseBase<Derived> &R) {
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
    EIGEN_STATIC_ASSERT(Derived::RowsAtCompileTime == 3, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
    EIGEN_STATIC_ASSERT(Derived::ColsAtCompileTime == 3, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

    Eigen::Matrix<typename Derived::Scalar, 3, 1> n = R.col(0);
    Eigen::Matrix<typename Derived::Scalar, 3, 1> o = R.col(1);
    Eigen::Matrix<typename Derived::Scalar, 3, 1> a = R.col(2);

    Eigen::Matrix<typename Derived::Scalar, 3, 1> ypr(3);
    typename Derived::Scalar y = atan2(n(1), n(0));
    typename Derived::Scalar p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    typename Derived::Scalar r =
            atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    return ypr;
}

template<typename Derived>
Eigen::Quaternion<typename Derived::Scalar> geometry_utils::ypr_to_quaternion(const Eigen::DenseBase<Derived> &ypr) {
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
    EIGEN_STATIC_ASSERT(Derived::RowsAtCompileTime == 3, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
    EIGEN_STATIC_ASSERT(Derived::ColsAtCompileTime == 1, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

    const typename Derived::Scalar cy = cos(ypr(0) / 2.0);
    const typename Derived::Scalar sy = sin(ypr(0) / 2.0);
    const typename Derived::Scalar cp = cos(ypr(1) / 2.0);
    const typename Derived::Scalar sp = sin(ypr(1) / 2.0);
    const typename Derived::Scalar cr = cos(ypr(2) / 2.0);
    const typename Derived::Scalar sr = sin(ypr(2) / 2.0);

    Eigen::Quaternion<typename Derived::Scalar> q;

    q.w() = cr * cp * cy + sr * sp * sy;
    q.x() = sr * cp * cy - cr * sp * sy;
    q.y() = cr * sp * cy + sr * cp * sy;
    q.z() = cr * cp * sy - sr * sp * cy;

    return q;
}

template<typename Scalar_t>
Eigen::Matrix<Scalar_t, 3, 1> geometry_utils::quaternion_to_ypr(const Eigen::Quaternion<Scalar_t> &q_) {
    Eigen::Quaternion<Scalar_t> q = q_.normalized();

    Eigen::Matrix<Scalar_t, 3, 1> ypr;
    ypr(2) = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));
    ypr(1) = asin(2 * (q.w() * q.y() - q.z() * q.x()));
    ypr(0) = atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));

    return ypr;
}

template<typename Scalar_t>
Scalar_t geometry_utils::get_yaw_from_quaternion(const Eigen::Quaternion<Scalar_t> &q) {
    return quaternion_to_ypr(q)(0);
}

template<typename Scalar_t>
Eigen::Quaternion<Scalar_t> geometry_utils::yaw_to_quaternion(Scalar_t yaw) {
    return Eigen::Quaternion<Scalar_t>(rotz(yaw));
}

template<typename Scalar_t>
Scalar_t geometry_utils::normalize_angle(Scalar_t a) {
    int cnt = 0;
    while (true) {
        cnt++;

        if (a < -M_PI) {
            a += M_PI * 2.0;
        } else if (a > M_PI) {
            a -= M_PI * 2.0;
        }

        if (-M_PI <= a && a <= M_PI) {
            break;
        };

        assert(cnt < 10 && "[uav_utils/geometry_msgs] INVALID INPUT ANGLE");
    }

    return a;
}

template<typename Scalar_t>
Scalar_t geometry_utils::angle_add(Scalar_t a, Scalar_t b) {
    Scalar_t c = a + b;
    c = normalize_angle(c);
    assert(-M_PI <= c && c <= M_PI);
    return c;
}

template<typename Scalar_t>
Scalar_t geometry_utils::yaw_add(Scalar_t a, Scalar_t b) {
    return angle_add(a, b);
}

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> geometry_utils::get_skew_symmetric(const Eigen::DenseBase<Derived> &v) {
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
    EIGEN_STATIC_ASSERT(Derived::RowsAtCompileTime == 3, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
    EIGEN_STATIC_ASSERT(Derived::ColsAtCompileTime == 1, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

    Eigen::Matrix<typename Derived::Scalar, 3, 3> M;
    M.setZero();
    M(0, 1) = -v(2);
    M(0, 2) = v(1);
    M(1, 0) = v(2);
    M(1, 2) = -v(0);
    M(2, 0) = -v(1);
    M(2, 1) = v(0);
    return M;
}

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 1> geometry_utils::from_skew_symmetric(const Eigen::DenseBase<Derived> &M) {
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
    EIGEN_STATIC_ASSERT(Derived::RowsAtCompileTime == 3, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
    EIGEN_STATIC_ASSERT(Derived::ColsAtCompileTime == 3, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

    Eigen::Matrix<typename Derived::Scalar, 3, 1> v;
    v(0) = M(2, 1);
    v(1) = -M(2, 0);
    v(2) = M(1, 0);

    assert(v.isApprox(Eigen::Matrix<typename Derived::Scalar, 3, 1>(-M(1, 2), M(0, 2), -M(0, 1))));

    return v;
}


template Eigen::Matrix<double, 3, 1>
geometry_utils::from_skew_symmetric<Eigen::Matrix3d>(const Eigen::DenseBase<Eigen::Matrix3d> &M);

template Eigen::Matrix<double, 3, 3>
geometry_utils::get_skew_symmetric<Eigen::Vector3d>(const Eigen::DenseBase<Eigen::Vector3d> &v);

template double geometry_utils::toDeg(const double &x);

template double geometry_utils::toRad(const double &x);

template Eigen::Matrix<double, 3, 3> geometry_utils::rotx(double t);

template Eigen::Matrix<double, 3, 3> geometry_utils::roty(double t);

template Eigen::Matrix<double, 3, 3> geometry_utils::rotz(double t);

template Eigen::Matrix<double, 3, 3> geometry_utils::ypr_to_R(const Eigen::DenseBase<Eigen::Vector3d> &ypr);

template Eigen::Matrix<double, 3, 3>
geometry_utils::vec_to_R(const Eigen::MatrixBase<Eigen::Vector3d> &v1, const Eigen::MatrixBase<Eigen::Vector3d> &v2);

template Eigen::Matrix<double, 3, 1> geometry_utils::R_to_ypr(const Eigen::DenseBase<Eigen::Matrix3d> &R);

template Eigen::Quaternion<double> geometry_utils::ypr_to_quaternion(const Eigen::DenseBase<Eigen::Vector3d> &ypr);

template Eigen::Matrix<double, 3, 1> geometry_utils::quaternion_to_ypr(const Eigen::Quaternion<double> &q_);

template double geometry_utils::get_yaw_from_quaternion(const Eigen::Quaternion<double> &q);

template Eigen::Quaternion<double> geometry_utils::yaw_to_quaternion(double yaw);

template double geometry_utils::normalize_angle(double a);

template double geometry_utils::angle_add(double a, double b);

template double geometry_utils::yaw_add(double a, double b);


template Eigen::Matrix<float, 3, 1>
geometry_utils::from_skew_symmetric<Eigen::Matrix3f>(const Eigen::DenseBase<Eigen::Matrix3f> &M);

template Eigen::Matrix<float, 3, 3>
geometry_utils::get_skew_symmetric<Eigen::Vector3f>(const Eigen::DenseBase<Eigen::Vector3f> &v);

template float geometry_utils::toDeg(const float &x);

template float geometry_utils::toRad(const float &x);

template Eigen::Matrix<float, 3, 3> geometry_utils::rotx(float t);

template Eigen::Matrix<float, 3, 3> geometry_utils::roty(float t);

template Eigen::Matrix<float, 3, 3> geometry_utils::rotz(float t);

template Eigen::Matrix<float, 3, 3> geometry_utils::ypr_to_R(const Eigen::DenseBase<Eigen::Vector3f> &ypr);

template Eigen::Matrix<float, 3, 3>
geometry_utils::vec_to_R(const Eigen::MatrixBase<Eigen::Vector3f> &v1, const Eigen::MatrixBase<Eigen::Vector3f> &v2);

template Eigen::Matrix<float, 3, 1> geometry_utils::R_to_ypr(const Eigen::DenseBase<Eigen::Matrix3f> &R);

template Eigen::Quaternion<float> geometry_utils::ypr_to_quaternion(const Eigen::DenseBase<Eigen::Vector3f> &ypr);

template Eigen::Matrix<float, 3, 1> geometry_utils::quaternion_to_ypr(const Eigen::Quaternion<float> &q_);

template float geometry_utils::get_yaw_from_quaternion(const Eigen::Quaternion<float> &q);

template Eigen::Quaternion<float> geometry_utils::yaw_to_quaternion(float yaw);

template float geometry_utils::normalize_angle(float a);

template float geometry_utils::angle_add(float a, float b);

template float geometry_utils::yaw_add(float a, float b);


