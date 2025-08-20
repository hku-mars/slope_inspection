#pragma once

#include <type_utils/common_type_name.h>
#include <geometry_utils/geometry_utils.h>
#include <geometry_utils/ellipsoid.h>
#include <visualization_utils/visualization_utils.h>

namespace geometry_utils {
    class Polytope {
    private:
        bool undefined{true};
        bool is_known_free{false};

        MatD4f planes;
        bool have_seed_line{false};
    public:
        double overlap_depth_with_last_one{0};
        Vec3f interior_pt_with_last_one;
        Ellipsoid ellipsoid_;
        std::pair<Vec3f, Vec3f> seed_line;
        double robot_r;

        Polytope();

        Polytope(MatD4f _planes);

        bool empty();

        bool HaveSeedLine();

        void SetSeedLine(const std::pair<Vec3f, Vec3f> &_seed_line, double r = 0);

        int SurfNum() const;

        Polytope CrossWith(const Polytope &b) const ;

        Vec3f CrossCenter(const Polytope &b) const ;

        bool HaveOverlapWith(Polytope cmp, double eps = 1e-6);

        MatD4f GetPlanes() const;

        void Reset();

        bool IsKnownFree();

        void SetKnownFree(bool is_free);

        void SetPlanes(MatD4f _planes);

        void SetEllipsoid(const Ellipsoid &ellip);

        bool PointIsInside(const Vec3f &pt, const double margin = 0.01) const;

        /// Sort a set of points in a plane.
        static vec_E<Vec3f> SortPtsInClockWise(vec_E<Vec3f> &pts, Vec3f normal);

        double GetVolume() const ;

        double volume() const {
            return GetVolume();
        }

        void Visualize(const ros::Publisher &pub,
                       const std::string &ns = "mesh",
                       const bool &use_random_color = false,
                       const Color &surf_color = Color::SteelBlue(),
                       const Color &edge_color = Color::Black(),
                       const Color &vertex_color = Color::Red(),
                       const double &alpha = 0.15,
                       const double &edge_width = 0.1);
    };

    typedef std::vector<Polytope> PolytopeVec;
}

