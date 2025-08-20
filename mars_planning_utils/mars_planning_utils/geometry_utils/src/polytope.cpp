#include <geometry_utils/polytope.h>

using namespace geometry_utils;

Polytope::Polytope() {
    undefined = true;
}

Polytope::Polytope(MatD4f _planes) {
    planes = _planes;
    undefined = false;
}

bool Polytope::empty() {
    return undefined;
}

bool Polytope::HaveSeedLine() {
    return have_seed_line;
}

void Polytope::SetSeedLine(const std::pair<Vec3f, Vec3f> &_seed_line, double r) {
    robot_r = r;
    seed_line = _seed_line;
    have_seed_line = true;
}

int Polytope::SurfNum() const {
    if (undefined) {
        return 0;
    }
    return planes.rows();
}

Vec3f Polytope::CrossCenter(const Polytope &b) const {
    MatD4f curIH;
    curIH.resize(this->SurfNum() + b.SurfNum(), 4);
    curIH << this->planes, b.GetPlanes();
    Mat3Df curIV; // 走廊的顶点
    if (!geometry_utils::enumerateVs(curIH, curIV)) {
        printf(" -- [processCorridor] Failed to get Overlap enumerateVs .\n");
        return Vec3f(-999, -999, -999);
    }
    double x = (curIV.row(0).maxCoeff() + curIV.row(0).minCoeff()) * 0.5;
    double y = (curIV.row(1).maxCoeff() + curIV.row(1).minCoeff()) * 0.5;
    double z = (curIV.row(2).maxCoeff() + curIV.row(2).minCoeff()) * 0.5;
    return Vec3f(x, y, z);
}


Polytope Polytope::CrossWith(const Polytope &b) const {
    MatD4f curIH;
    curIH.resize(this->SurfNum() + b.SurfNum(), 4);
    curIH << this->planes, b.GetPlanes();
    Polytope out;
    out.SetPlanes(curIH);
    return out;
}

bool Polytope::HaveOverlapWith(Polytope cmp, double eps) {
    return geometry_utils::overlap(this->planes, cmp.GetPlanes(), eps);
}


MatD4f Polytope::GetPlanes() const {
    return planes;
}

void Polytope::Reset() {
    undefined = true;
    is_known_free = false;
    have_seed_line = false;
}

bool Polytope::IsKnownFree() {
    if (undefined) {
        return false;
    }
    return is_known_free;
}

void Polytope::SetKnownFree(bool is_free) {
    is_known_free = is_free;
}

void Polytope::SetPlanes(MatD4f _planes) {
    planes = _planes;
    undefined = false;
}

void Polytope::SetEllipsoid(const Ellipsoid &ellip) {
    ellipsoid_ = ellip;
}

bool Polytope::PointIsInside(const Vec3f &pt, const double margin) const {
    if (undefined) {
        return false;
    }
    if (planes.rows() == 0 || isnan(planes.sum())) {
        std::cout << RED << "ill polytope, force return." << RESET << std::endl;
    }
    Eigen::Vector4d pt_e;
    pt_e.head(3) = pt;
    pt_e(3) = 1;
    if ((planes * pt_e).maxCoeff() > margin) {
        return false;
    }
    return true;
}


vec_E<Vec3f> Polytope::SortPtsInClockWise(vec_E<Vec3f> &pts, Vec3f normal) {
    Vec3f center(0, 0, 0);
    for (auto pt: pts) {
        center += pt;
    }
    center = center / pts.size();
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Vec3f(0, 0, 1), normal);
    center = q.matrix() * center;
    std::vector<std::pair<double, Vec3f>> pts_valued;
    pts_valued.resize(pts.size());
    Vec3f temp_p;
    for (size_t i = 0; i < pts.size(); i++) {
        temp_p = q.matrix() * pts[i];
        double theta = atan2(temp_p(1) - center(1), temp_p(0) - center(0));
        pts_valued[i] = std::make_pair(theta, pts[i]);
    }

    std::sort(
            pts_valued.begin(), pts_valued.end(),
            [](const std::pair<double, Vec3f> &i,
               const std::pair<double, Vec3f> &j) { return i.first < j.first; });
    vec_E<Vec3f> pts_sorted(pts_valued.size());
    for (size_t i = 0; i < pts_valued.size(); i++)
        pts_sorted[i] = pts_valued[i].second;
    return pts_sorted;
}

double Polytope::GetVolume() const {
    // 首先，我们需要获取多面体的顶点
    Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vPoly;
    MatD4f planes = GetPlanes();
    if (!geometry_utils::enumerateVs(planes, vPoly)) {
        printf("Failed to compute volume: cannot enumerate vertices.\n");
        return 0;
    }

    // 使用QuickHull库计算凸包
    geometry_utils::QuickHull<double> qh;
    const auto convexHull = qh.getConvexHull(vPoly.data(), vPoly.cols(), false, true);
    const auto &indexBuffer = convexHull.getIndexBuffer();

    // 确保我们至少有四个顶点，这样才能构成一个四面体
    if (indexBuffer.size() < 4) {
        printf("Not enough vertices to compute volume.\n");
        return 0;
    }

    // 计算多面体的重心
    Vec3f centroid = Vec3f::Zero();
    for (long int i = 0; i < vPoly.cols(); i++) {
        centroid += vPoly.col(i);
    }
    centroid /= static_cast<double>(vPoly.cols());

    // 对每个三角形面，计算其四面体相对于重心的体积
    double volume = 0.0;
    for (size_t i = 0; i < indexBuffer.size(); i += 3) {
        Vec3f A = vPoly.col(indexBuffer[i]);
        Vec3f B = vPoly.col(indexBuffer[i + 1]);
        Vec3f C = vPoly.col(indexBuffer[i + 2]);

        // 使用标量三重积计算四面体的有向体积
        double tetrahedronVolume = (A - centroid).dot((B - centroid).cross(C - centroid)) / 6.0;
        volume += std::abs(tetrahedronVolume);
    }

    return volume;
}

void Polytope::Visualize(const ros::Publisher &pub, const std::string &ns, const bool &use_random_color,
                         const Color &surf_color, const Color &edge_color, const Color &vertex_color,
                         const double &alpha, const double &edge_width) {
    if (isnan(planes.sum())) {
        printf(" -- [Poly] ERROR, try to visualize polytope with NaN, force return.\n");
        return;
    }
    // Due to the fact that H-representation cannot be directly visualized
    // We first conduct vertex enumeration of them, then apply quickhull
    // to obtain triangle meshs of polyhedra
    Eigen::Matrix3Xd mesh(3, 0), curTris(3, 0), oldTris(3, 0);

//        ellipsoid_.Visualize(pub);
//        LineSegment l(seed_line.first, seed_line.second);
//        l.Visualize(pub);

    oldTris = mesh;
    Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vPoly;
    MatD4f planes = GetPlanes();
    if (!geometry_utils::enumerateVs(planes, vPoly)) {
        printf(" -- [WARN] Trying to publish ill polytope.\n");
        return;
    }

    geometry_utils::QuickHull<double> tinyQH;
    const auto polyHull = tinyQH.getConvexHull(vPoly.data(), vPoly.cols(), false, true);
    const auto &idxBuffer = polyHull.getIndexBuffer();
    int hNum = idxBuffer.size() / 3;

    curTris.resize(3, hNum * 3);
    for (int i = 0; i < hNum * 3; i++) {
        curTris.col(i) = vPoly.col(idxBuffer[i]);
    }
    mesh.resize(3, oldTris.cols() + curTris.cols());
    mesh.leftCols(oldTris.cols()) = oldTris;
    mesh.rightCols(curTris.cols()) = curTris;

    if (isnan(mesh.sum())) {
        printf(" -- [WARN] Trying to publish ill polytope.\n");
        return;
    }

    // RVIZ support tris for visualization
    visualization_msgs::MarkerArray mkr_arr;
    visualization_msgs::Marker meshMarker, edgeMarker, seedMarker;
    // static int mesh_id = 0;
    // static int edge_id = 0;
    int mesh_id = 0;
    int edge_id = 0;

    static std::random_device rd;
    static std::mt19937 mt(rd());
    static std::uniform_real_distribution<double> dist(0.0, 1.0);
    Color random_color(dist(mt), dist(mt), dist(mt));
    if (have_seed_line) {
        /// * 1) Publish Seed line
        // static int point_id = 0;
        // static int line_cnt = 0;
        int point_id = 0;
        int line_cnt = 0;
        /* Publish point */
        visualization_msgs::Marker point;
        point.header.frame_id = "world";
        point.header.stamp = ros::Time::now();
        point.ns = ns + " pt";
        point.id = point_id++;
        point.action = visualization_msgs::Marker::ADD;
        point.pose.orientation.w = 1.0;
        point.type = visualization_msgs::Marker::SPHERE;
        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        point.scale.x = 0.2;
        point.scale.y = 0.2;
        point.scale.z = 0.2;
        // Line list is blue
        if (use_random_color) {
            point.color = random_color;// Color::Blue();
        } else {
            point.color = edge_color;
        }

        point.color.a = 1.0;
        // Create the vertices for the points and lines
        geometry_msgs::Point p;
        p.x = seed_line.first.x();
        p.y = seed_line.first.y();
        p.z = seed_line.first.z();
        point.pose.position = p;
        mkr_arr.markers.push_back(point);

        p.x = seed_line.second.x();
        p.y = seed_line.second.y();
        p.z = seed_line.second.z();
        point.pose.position = p;
        point.id = point_id++;
        mkr_arr.markers.push_back(point);

        // publish lines
        visualization_msgs::Marker line_list;
        line_list.header.frame_id = "world";
        line_list.header.stamp = ros::Time::now();
        line_list.ns = ns + " line";
        line_list.id = line_cnt++;
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_list.scale.x = 0.1;
        // Line list is blue
        if (use_random_color) {
            line_list.color = random_color;// Color::Blue();
        } else {
            line_list.color = edge_color;
        }
        line_list.color.a = 1.0;
        // Create the vertices for the points and lines

        p.x = seed_line.first.x();
        p.y = seed_line.first.y();
        p.z = seed_line.first.z();
        // The line list needs two points for each line
        line_list.points.push_back(p);
        p.x = seed_line.second.x();
        p.y = seed_line.second.y();
        p.z = seed_line.second.z();
        // The line list needs
        line_list.points.push_back(p);

        mkr_arr.markers.push_back(line_list);
    }

    geometry_msgs::Point point;
    meshMarker.id = mesh_id++;
    meshMarker.header.stamp = ros::Time::now();
    meshMarker.header.frame_id = "world";
    meshMarker.pose.orientation.w = 1.00;
    meshMarker.action = visualization_msgs::Marker::ADD;
    meshMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    meshMarker.ns = ns + " mesh";
    meshMarker.color = surf_color;
    meshMarker.color.a = alpha;
    meshMarker.scale.x = 1.0;
    meshMarker.scale.y = 1.0;
    meshMarker.scale.z = 1.0;

    edgeMarker = meshMarker;
    edgeMarker.header.stamp = ros::Time::now();
    edgeMarker.header.frame_id = "world";
    edgeMarker.id = edge_id++;
    edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
    edgeMarker.ns = ns + " edge";
    if (use_random_color) {
        edgeMarker.color = random_color;// Color::Blue();
    } else {
        edgeMarker.color = edge_color;
    }
    edgeMarker.color.a = 1.00;
    edgeMarker.scale.x = edge_width;


    long unsigned int ptnum = mesh.cols();

    for (long unsigned int i = 0; i < ptnum; i++) {
        point.x = mesh(0, i);
        point.y = mesh(1, i);
        point.z = mesh(2, i);
        meshMarker.points.push_back(point);
    }


    /// 接下来遍历选择定点属于哪些面
    vec_E<Vec3f> edges;

    for (long int i = 0; i < planes.rows(); i++) {
        edges.clear();
        Eigen::VectorXd temp = (planes.row(i).head(3) * vPoly);
        Eigen::VectorXd d(temp.size());
        d.setConstant(planes(i, 3));
        temp = temp + d;
        for (int j = 0; j < temp.size(); j++) {
            if (std::abs(temp(j)) < epsilon_) {
                edges.push_back(vPoly.col(j));
            }
        }
        if (edges.size() < 2) {
            continue;
        }
        /// 对每个面的定点逆时针排序
        vec_E<Vec3f> pts_sorted = SortPtsInClockWise(edges, planes.row(i).head(3));
        int pts_num = pts_sorted.size();
        for (int k = 0; k < pts_num - 1; k++) {
            point.x = pts_sorted[k].x();
            point.y = pts_sorted[k].y();
            point.z = pts_sorted[k].z();
            edgeMarker.points.push_back(point);
            point.x = pts_sorted[k + 1].x();
            point.y = pts_sorted[k + 1].y();
            point.z = pts_sorted[k + 1].z();
            edgeMarker.points.push_back(point);
        }
        point.x = pts_sorted[0].x();
        point.y = pts_sorted[0].y();
        point.z = pts_sorted[0].z();
        edgeMarker.points.push_back(point);
        point.x = pts_sorted[pts_num - 1].x();
        point.y = pts_sorted[pts_num - 1].y();
        point.z = pts_sorted[pts_num - 1].z();
        edgeMarker.points.push_back(point);
    }
    mkr_arr.markers.push_back(meshMarker);
    mkr_arr.markers.push_back(edgeMarker);
    pub.publish(mkr_arr);
}