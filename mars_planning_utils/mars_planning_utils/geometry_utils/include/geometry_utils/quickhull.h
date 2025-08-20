// Public domain code from Antti Kuukka
// https://github.com/akuukka/quickhull
// Many thanks to him.

#pragma once

#include <iostream>
#include <cmath>
#include <limits>
#include <memory>
#include <algorithm>
#include <array>
#include <vector>
#include <deque>
#include <unordered_map>
#include <fstream>
#include <cassert>

namespace geometry_utils {

    // ----------------------- BasicData Part -----------------------

    template<typename T>
    class Vector3 {
    public:
        Vector3() = default;

        Vector3(const T &x, const T &y, const T &z) : x(x), y(y), z(z) {}

        T x, y, z;

        T dotProduct(const Vector3 &other) const {
            return x * other.x + y * other.y + z * other.z;
        }

        Vector3 operator-(const Vector3 &other) const {
            return Vector3(x - other.x, y - other.y, z - other.z);
        }

        Vector3 operator+(const Vector3 &other) const {
            return Vector3(x + other.x, y + other.y, z + other.z);
        }

        Vector3 &operator+=(const Vector3 &other) {
            x += other.x;
            y += other.y;
            z += other.z;
            return *this;
        }

        Vector3 &operator-=(const Vector3 &other) {
            x -= other.x;
            y -= other.y;
            z -= other.z;
            return *this;
        }

        Vector3 &operator*=(const T &c) {
            x *= c;
            y *= c;
            z *= c;
            return *this;
        }

        Vector3 &operator/=(const T &c) {
            x /= c;
            y /= c;
            z /= c;
            return *this;
        }

        Vector3 operator-() const {
            return Vector3(-x, -y, -z);
        }

        template<typename S>
        Vector3 operator*(const S &c) const {
            return Vector3(x * c, y * c, z * c);
        }

        template<typename S>
        Vector3 operator/(const S &c) const {
            return Vector3(x / c, y / c, z / c);
        }

        T getLengthSquared() const {
            return x * x + y * y + z * z;
        }

        bool operator!=(const Vector3 &o) const {
            return x != o.x || y != o.y || z != o.z;
        }

        T getSquaredDistanceTo(const Vector3 &other) const {
            const T dx = x - other.x;
            const T dy = y - other.y;
            const T dz = z - other.z;
            return dx * dx + dy * dy + dz * dz;
        }
    };

    // Overload also << operator for easy printing of debug data
    template<typename T>
    std::ostream &operator<<(std::ostream &os, const Vector3<T> &vec) {
        os << "(" << vec.x << "," << vec.y << "," << vec.z << ")";
        return os;
    }

    template<typename T>
    Vector3<T> operator*(T c, const Vector3<T> &v) {
        return Vector3<T>(v.x * c, v.y * c, v.z * c);
    }

    template<typename T>
    class Plane {
    public:
        Vector3<T> m_N;

        // Signed distance (if normal is of length 1) to the plane from origin
        T m_D;

        // Normal length squared
        T m_sqrNLength;

        bool isPointOnPositiveSide(const Vector3<T> &Q) const {
            T d = m_N.dotProduct(Q) + m_D;
            if (d >= 0)
                return true;
            return false;
        }

        Plane() = default;

        // Construct a plane using normal N and any point P on the plane
        Plane(const Vector3<T> &N, const Vector3<T> &P)
                : m_N(N), m_D(-N.dotProduct(P)),
                  m_sqrNLength(m_N.x * m_N.x + m_N.y * m_N.y + m_N.z * m_N.z) {}
    };

    template<typename T>
    struct Ray {
        const Vector3<T> m_S;
        const Vector3<T> m_V;
        const T m_VInvLengthSquared;

        Ray(const Vector3<T> &S, const Vector3<T> &V)
                : m_S(S), m_V(V), m_VInvLengthSquared(T{1} / m_V.getLengthSquared()) {}
    };

    template<typename S>
    class Pool {
        std::vector<std::unique_ptr<S>> m_data;

    public:
        void clear() {
            m_data.clear();
        }

        void reclaim(std::unique_ptr<S> &ptr) {
            m_data.push_back(std::move(ptr));
        }

        std::unique_ptr<S> get() {
            if (m_data.empty()) {
                return std::unique_ptr<S>(new S());
            }
            auto it = m_data.end() - 1;
            std::unique_ptr<S> r = std::move(*it);
            m_data.erase(it);
            return r;
        }
    };

    template<typename T>
    class VertexDataSource {
        const Vector3<T> *m_ptr;
        size_t m_count;

    public:
        VertexDataSource(const Vector3<T> *ptr, size_t count)
                : m_ptr(ptr), m_count(count) {}

        VertexDataSource(const std::vector<Vector3<T>> &vec)
                : m_ptr(&vec[0]), m_count(vec.size()) {}

        VertexDataSource()
                : m_ptr(nullptr), m_count(0) {}

        VertexDataSource &operator=(const VertexDataSource &other) = default;

        size_t size() const {
            return m_count;
        }

        const Vector3<T> &operator[](size_t index) const {
            return m_ptr[index];
        }

        const Vector3<T> *begin() const {
            return m_ptr;
        }

        const Vector3<T> *end() const {
            return m_ptr + m_count;
        }
    };

    // ----------------------- MeshBuilder Part -----------------------

    template<typename T>
    class MeshBuilder {
    public:
        struct HalfEdge {
            size_t m_endVertex;
            size_t m_opp;
            size_t m_face;
            size_t m_next;

            void disable();

            bool isDisabled() const;
        };

        struct Face {
            size_t m_he;
            Plane<T> m_P;
            T m_mostDistantPointDist;
            size_t m_mostDistantPoint;
            size_t m_visibilityCheckedOnIteration;
            std::uint8_t m_isVisibleFaceOnCurrentIteration: 1;
            std::uint8_t m_inFaceStack: 1;

            // Bit for each half edge assigned to this face,
            // each being 0 or 1 depending on whether the
            // edge belongs to horizon edge
            std::uint8_t m_horizonEdgesOnCurrentIteration: 3;
            std::unique_ptr<std::vector<size_t>> m_pointsOnPositiveSide;

            Face() : m_he(std::numeric_limits<size_t>::max()),
                     m_P(),
                     m_mostDistantPointDist(0),
                     m_mostDistantPoint(0),
                     m_visibilityCheckedOnIteration(0),
                     m_isVisibleFaceOnCurrentIteration(0),
                     m_inFaceStack(0),
                     m_horizonEdgesOnCurrentIteration(0) {}

            void disable();

            bool isDisabled() const;
        };

        // Mesh data
        std::vector<Face> m_faces;
        std::vector<HalfEdge> m_halfEdges;

        // When the mesh is modified and faces and half edges are
        // removed from it, we do not actually remove them from
        // the container vectors. Insted, they are marked as disabled
        // which means that the indices can be reused when we need to
        // add new faces and half edges to the mesh. We store the
        // free indices in the following vectors.
        std::vector<size_t> m_disabledFaces, m_disabledHalfEdges;

        size_t addFace();

        size_t addHalfEdge();

        // Mark a face as disabled and return a pointer
        // to the points that were on the positive of it.
        std::unique_ptr<std::vector<size_t>> disableFace(size_t faceIndex);

        void disableHalfEdge(size_t heIndex);

        MeshBuilder() = default;

        // Create a mesh with initial tetrahedron ABCD.
        // Dot product of AB with the normal of triangle
        // ABC should be negative.
        void setup(size_t a, size_t b, size_t c, size_t d);

        std::array<size_t, 3> getVertexIndicesOfFace(const Face &f) const;

        std::array<size_t, 2> getVertexIndicesOfHalfEdge(const HalfEdge &he) const;

        std::array<size_t, 3> getHalfEdgeIndicesOfFace(const Face &f) const;
    };


    // ----------------------- mu Part -----------------------

    namespace mu {

        template<typename T>
        T getSquaredDistanceBetweenPointAndRay(const Vector3<T> &p, const Ray<T> &r) {
            const Vector3<T> s = p - r.m_S;
            T t = s.dotProduct(r.m_V);
            return s.getLengthSquared() - t * t * r.m_VInvLengthSquared;
        }

        // Note that the unit of distance returned is relative
        // to plane's normal's length (divide by N.getNormalized()
        // if needed to get the "real" distance).
        template<typename T>
        T getSignedDistanceToPlane(const Vector3<T> &v, const Plane<T> &p) {
            return p.m_N.dotProduct(v) + p.m_D;
        }

        template<typename T>
        Vector3<T> getTriangleNormal(const Vector3<T> &a,
                                     const Vector3<T> &b,
                                     const Vector3<T> &c) {
            // We want to get (a-c).crossProduct(b-c)
            // without constructing temp vectors
            T x = a.x - c.x;
            T y = a.y - c.y;
            T z = a.z - c.z;
            T rhsx = b.x - c.x;
            T rhsy = b.y - c.y;
            T rhsz = b.z - c.z;
            T px = y * rhsz - z * rhsy;
            T py = z * rhsx - x * rhsz;
            T pz = x * rhsy - y * rhsx;
            return Vector3<T>(px, py, pz);
        }

    } // namespace mu

    // ----------------------- HalfEdgeMesh Part -----------------------

    template<typename FloatType, typename IndexType>
    class HalfEdgeMesh {
    public:
        struct HalfEdge {
            IndexType m_endVertex;
            IndexType m_opp;
            IndexType m_face;
            IndexType m_next;
        };

        struct Face {
            // Index of one of the half edges of this face
            IndexType m_halfEdgeIndex;
        };

        std::vector<Vector3<FloatType>> m_vertices;
        std::vector<Face> m_faces;
        std::vector<HalfEdge> m_halfEdges;

        HalfEdgeMesh(const MeshBuilder<FloatType> &builderObject,
                     const VertexDataSource<FloatType> &vertexData);
    };

    // ----------------------- Convex Hull Part -----------------------

    template<typename T>
    class ConvexHull {
        std::unique_ptr<std::vector<Vector3<T>>> m_optimizedVertexBuffer;
        VertexDataSource<T> m_vertices;
        std::vector<size_t> m_indices;

    public:
        ConvexHull() = default;

        // Copy constructor
        ConvexHull(const ConvexHull &o);

        ConvexHull &operator=(const ConvexHull &o);


        ConvexHull(ConvexHull &&o);

        ConvexHull &operator=(ConvexHull &&o);

        // Construct vertex and index buffers from
        // half edge mesh and pointcloud
        ConvexHull(const MeshBuilder<T> &mesh,
                   const VertexDataSource<T> &pointCloud,
                   bool CCW,
                   bool useOriginalIndices);

        std::vector<size_t> &getIndexBuffer();

        const std::vector<size_t> &getIndexBuffer() const;

        VertexDataSource<T> &getVertexBuffer();

        const VertexDataSource<T> &getVertexBuffer() const;

        // Export the mesh to a Waveform OBJ file
        void writeWaveformOBJ(const std::string &filename,
                              const std::string &objectName = "quickhull") const;
    };

    // ----------------------- Quick Hull Part -----------------------

    // Implementation of the 3D QuickHull algorithm by Antti Kuukka
    //
    // No copyrights. What follows is 100% Public Domain.
    //
    //
    //
    // INPUT:  a list of points in 3D space (for example, vertices of a 3D mesh)
    //
    // OUTPUT: a ConvexHull object which provides vertex and index buffers of
    // the generated convex hull as a triangle mesh.
    //
    //
    //
    // The implementation is thread-safe if each thread is using its own
    // QuickHull object.
    //
    //
    // SUMMARY OF THE ALGORITHM:
    //         - Create initial simplex (tetrahedron) using extreme points.
    //           We have four faces now and they form a convex mesh M.
    //         - For each point, assign them to the first face for which they
    //           are on the positive side of (so each point is assigned to at
    //           most one face). Points inside the initial tetrahedron are left
    //           behind now and no longer affect the calculations.
    //         - Add all faces that have points assigned to them to Face Stack.
    //         - Iterate until Face Stack is empty:
    //              - Pop topmost face F from the stack
    //              - From the points assigned to F, pick the point P that is
    //                farthest away from the plane defined by F.
    //              - Find all faces of M that have P on their positive side.
    //                Let us call these the "visible faces".
    //              - Because of the way M is constructed, these faces are connected.
    //                Solve their horizon edge loop.
    //				- "Extrude to P": Create new faces by connecting P with the points
    //                 belonging to the horizon edge. Add the new faces to M and remove
    //                 the visible faces from M.
    //              - Each point that was assigned to visible faces is now assigned
    //                to at most one of the newly created faces.
    //              - Those new faces that have points assigned to them are added
    //                to the top of Face Stack.
    //          - M is now the convex hull.
    //
    // TO DO:
    //  - Implement a proper 2D QuickHull and use that to solve the degenerate 2D case
    //    (when all the points lie on the same plane in 3D space).

    struct DiagnosticsData {
        // How many times QuickHull failed to solve the horizon edge. Failures lead to
        // degenerated convex hulls.
        size_t m_failedHorizonEdges;

        DiagnosticsData() : m_failedHorizonEdges(0) {}
    };

    template<typename T>
    T defaultEps();

    template<typename T>
    class QuickHull {
    public:
        QuickHull() = default;

        ~QuickHull() = default;

    private:
        using vec3 = Vector3<T>;

        T m_epsilon{}, m_epsilonSquared{}, m_scale{};
        bool m_planar{};
        std::vector<vec3> m_planarPointCloudTemp;
        VertexDataSource<T> m_vertexData;
        MeshBuilder<T> m_mesh;
        std::array<size_t, 6> m_extremeValues{};
        DiagnosticsData m_diagnostics;

        // Temporary variables used during iteration process
        std::vector<size_t> m_newFaceIndices;
        std::vector<size_t> m_newHalfEdgeIndices;
        std::vector<std::unique_ptr<std::vector<size_t>>> m_disabledFacePointVectors;
        std::vector<size_t> m_visibleFaces;
        std::vector<size_t> m_horizonEdges;

        struct FaceData {
            size_t m_faceIndex;
            // If the face turns out not to be visible, this half edge will be marked as horizon edge
            size_t m_enteredFromHalfEdge;

            FaceData(size_t fi, size_t he) : m_faceIndex(fi), m_enteredFromHalfEdge(he) {}
        };

        std::vector<FaceData> m_possiblyVisibleFaces;
        std::deque<size_t> m_faceList;

        // Create a half edge mesh representing the base tetrahedron from which the QuickHull
        // iteration proceeds. m_extremeValues must be properly set up when this is called.
        void setupInitialTetrahedron();

        // Given a list of half edges, try to rearrange them so
        // that they form a loop. Return true on success.
        bool reorderHorizonEdges(std::vector<size_t> &horizonEdges);

        // Find indices of extreme values (max x, min x, max y, min y, max z, min z) for the given point cloud
        std::array<size_t, 6> getExtremeValues();

        // Compute scale of the vertex data.
        T getScale(const std::array<size_t, 6> &extremeValues);

        // Each face contains a unique pointer to a vector of indices.
        // However, many - often most - faces do not have any points
        // on the positive side of them especially at the the end of
        // the iteration. When a face is removed from the mesh, its
        // associated point vector, if such exists, is moved to the
        // index vector pool, and when we need to add new faces with
        // points on the positive side to the mesh, we reuse these
        // vectors. This reduces the amount of std::vectors we have
        // to deal with, and impact on performance is remarkable.
        Pool<std::vector<size_t>> m_indexVectorPool;

        std::unique_ptr<std::vector<size_t>> getIndexVectorFromPool();

        void reclaimToIndexVectorPool(std::unique_ptr<std::vector<size_t>> &ptr);

        // Associates a point with a face if the point resides on the positive
        // side of the plane. Returns true if the points was on the positive side.
        bool addPointToFace(typename MeshBuilder<T>::Face &f, size_t pointIndex);

        // This will update m_mesh from which we create the ConvexHull
        // object that getConvexHull function returns
        void createConvexHalfEdgeMesh();

        // Constructs the convex hull into a MeshBuilder object
        // which can be converted to a ConvexHull or Mesh object
        void buildMesh(const VertexDataSource<T> &pointCloud, bool CCW, bool useOriginalIndices, T eps);

        // The public getConvexHull functions will
        // setup a VertexDataSource object and call this
        // The public getConvexHull functions will setup a VertexDataSource object and call this
        ConvexHull<T> getConvexHull(const VertexDataSource<T> &pointCloud, bool CCW, bool useOriginalIndices, T eps);

    public:
        // Computes convex hull for a given point cloud.
        // Params:
        //   pointCloud: a vector of of 3D points
        //   CCW: whether the output mesh triangles should have CCW orientation
        //   useOriginalIndices: should the output mesh use same vertex indices
        //                       as the original point cloud. If this is false,
        //                       then we generate a new vertex buffer which contains
        //                       only the vertices that are part of the convex hull.
        //   eps: minimum distance to a plane to consider a point being on
        //        positive of it (for a point cloud with scale 1)
        ConvexHull<T> getConvexHull(const std::vector<Vector3<T>> &pointCloud,
                                    bool CCW,
                                    bool useOriginalIndices,
                                    T eps = defaultEps<T>());


        // Computes convex hull for a given point cloud.
        // Params:
        //   vertexData: pointer to the first 3D point of the point cloud
        //   vertexCount: number of vertices in the point cloud
        //   CCW: whether the output mesh triangles should have CCW orientation
        //   useOriginalIndices: should the output mesh use same vertex indices
        //                       as the original point cloud. If this is false,
        //                       then we generate a new vertex buffer which contains
        //                       only the vertices that are part of the convex hull.
        //   eps: minimum distance to a plane to consider a point being on positive
        //        side of it (for a point cloud with scale 1)
        ConvexHull<T> getConvexHull(const Vector3<T> *vertexData,
                                    size_t vertexCount,
                                    bool CCW,
                                    bool useOriginalIndices,
                                    T eps = defaultEps<T>());

        // Computes convex hull for a given point cloud. This function
        // assumes that the vertex data resides in memory
        // in the following format: x_0,y_0,z_0,x_1,y_1,z_1,...
        // Params:
        //   vertexData: pointer to the X component of the first point of the point cloud.
        //   vertexCount: number of vertices in the point cloud
        //   CCW: whether the output mesh triangles should have CCW orientation
        //   useOriginalIndices: should the output mesh use same vertex indices as the
        //                       original point cloud. If this is false, then we generate
        //                       a new vertex buffer which contains only the vertices that
        //                       are part of the convex hull.
        //   eps: minimum distance to a plane to consider a point being on positive side
        //        of it (for a point cloud with scale 1)
        ConvexHull<T> getConvexHull(const T *vertexData,
                                    size_t vertexCount,
                                    bool CCW,
                                    bool useOriginalIndices,
                                    T eps = defaultEps<T>());

        // Computes convex hull for a given point cloud. This function assumes
        // that the vertex data resides in memory
        // in the following format: x_0,y_0,z_0,x_1,y_1,z_1,...
        // Params:
        //   vertexData: pointer to the X component of the first point of the point cloud.
        //   vertexCount: number of vertices in the point cloud
        //   CCW: whether the output mesh triangles should have CCW orientation
        //   eps: minimum distance to a plane to consider a point being on positive side
        //        of it (for a point cloud with scale 1)
        // Returns:
        //   Convex hull of the point cloud as a mesh object with half edge structure.
        HalfEdgeMesh<T, size_t> getConvexHullAsMesh(const T *vertexData,
                                                    size_t vertexCount,
                                                    bool CCW,
                                                    T eps = defaultEps<T>());

        // Get diagnostics about last generated convex hull
        const DiagnosticsData &getDiagnostics();

    };


}