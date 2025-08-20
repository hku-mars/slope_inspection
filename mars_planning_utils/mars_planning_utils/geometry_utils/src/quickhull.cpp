#include <geometry_utils/quickhull.h>
#include <typeinfo>

namespace geometry_utils {

    template<typename T>
    ConvexHull<T>::ConvexHull(const ConvexHull &o) {
        m_indices = o.m_indices;
        if (o.m_optimizedVertexBuffer) {
            m_optimizedVertexBuffer.reset(new std::vector<Vector3<T>>
                                                  (*o.m_optimizedVertexBuffer));
            m_vertices = VertexDataSource<T>(*m_optimizedVertexBuffer);
        } else {
            m_vertices = o.m_vertices;
        }
    }

    template<typename T>
    ConvexHull<T>::ConvexHull(ConvexHull<T> &&o) {
        m_indices = std::move(o.m_indices);
        if (o.m_optimizedVertexBuffer) {
            m_optimizedVertexBuffer = std::move(o.m_optimizedVertexBuffer);
            o.m_vertices = VertexDataSource<T>();
            m_vertices = VertexDataSource<T>(*m_optimizedVertexBuffer);
        } else {
            m_vertices = o.m_vertices;
        }
    }

    template<typename T>
    ConvexHull<T>::ConvexHull(const MeshBuilder<T> &mesh, const VertexDataSource<T> &pointCloud, bool CCW,
                              bool useOriginalIndices) {
        if (!useOriginalIndices) {
            m_optimizedVertexBuffer.reset(new std::vector<Vector3<T>>
                                                  ());
        }

        std::vector<std::uint8_t> faceProcessed(mesh.m_faces.size(), 0);
        std::vector<size_t> faceStack;
        // Map vertex indices from original point cloud
        // to the new mesh vertex indices
        std::unordered_map<size_t, size_t> vertexIndexMapping;
        for (size_t i = 0; i < mesh.m_faces.size(); i++) {
            if (!mesh.m_faces[i].isDisabled()) {
                faceStack.push_back(i);
                break;
            }
        }
        if (faceStack.size() == 0) {
            return;
        }

        const size_t iCCW = CCW ? 1 : 0;
        const size_t finalMeshFaceCount = mesh.m_faces.size() - mesh.m_disabledFaces.size();
        m_indices.reserve(finalMeshFaceCount * 3);

        while (faceStack.size()) {
            auto it = faceStack.end() - 1;
            size_t top = *it;
            assert(!mesh.m_faces[top].isDisabled());
            faceStack.erase(it);
            if (faceProcessed[top]) {
                continue;
            } else {
                faceProcessed[top] = 1;
                auto halfEdges = mesh.getHalfEdgeIndicesOfFace(mesh.m_faces[top]);
                size_t adjacent[] = {mesh.m_halfEdges[mesh.m_halfEdges[halfEdges[0]].m_opp].m_face,
                                     mesh.m_halfEdges[mesh.m_halfEdges[halfEdges[1]].m_opp].m_face,
                                     mesh.m_halfEdges[mesh.m_halfEdges[halfEdges[2]].m_opp].m_face};
                for (auto a: adjacent) {
                    if (!faceProcessed[a] && !mesh.m_faces[a].isDisabled()) {
                        faceStack.push_back(a);
                    }
                }
                auto vertices = mesh.getVertexIndicesOfFace(mesh.m_faces[top]);
                if (!useOriginalIndices) {
                    for (auto &v: vertices) {
                        auto itV = vertexIndexMapping.find(v);
                        if (itV == vertexIndexMapping.end()) {
                            m_optimizedVertexBuffer->push_back(pointCloud[v]);
                            vertexIndexMapping[v] = m_optimizedVertexBuffer->size() - 1;
                            v = m_optimizedVertexBuffer->size() - 1;
                        } else {
                            v = itV->second;
                        }
                    }
                }
                m_indices.push_back(vertices[0]);
                m_indices.push_back(vertices[1 + iCCW]);
                m_indices.push_back(vertices[2 - iCCW]);
            }
        }

        if (!useOriginalIndices) {
            m_vertices = VertexDataSource<T>(*m_optimizedVertexBuffer);
        } else {
            m_vertices = pointCloud;
        }
    }

    template<typename T>
    std::vector<size_t> &ConvexHull<T>::getIndexBuffer() {
        return m_indices;
    }

    template<typename T>
    const std::vector<size_t> &ConvexHull<T>::getIndexBuffer() const {
        return m_indices;
    }

    template<typename T>
    VertexDataSource<T> &ConvexHull<T>::getVertexBuffer() {
        return m_vertices;
    }

    template<typename T>
    const VertexDataSource<T> &ConvexHull<T>::getVertexBuffer() const {
        return m_vertices;
    }

    template<typename T>
    void ConvexHull<T>::writeWaveformOBJ(const std::string &filename, const std::string &objectName) const {
        std::ofstream objFile;
        objFile.open(filename);
        objFile << "o " << objectName << "\n";
        for (const auto &v: getVertexBuffer()) {
            objFile << "v " << v.x << " " << v.y << " " << v.z << "\n";
        }
        const auto &indBuf = getIndexBuffer();
        size_t triangleCount = indBuf.size() / 3;
        for (size_t i = 0; i < triangleCount; i++) {
            objFile << "f " << indBuf[i * 3] + 1 << " "
                    << indBuf[i * 3 + 1] + 1 << " "
                    << indBuf[i * 3 + 2] + 1 << "\n";
        }
        objFile.close();
    }

    template<typename T>
    ConvexHull<T> &ConvexHull<T>::operator=(const ConvexHull<T> &o) {
        if (&o == this) {
            return *this;
        }
        m_indices = o.m_indices;
        if (o.m_optimizedVertexBuffer) {
            m_optimizedVertexBuffer.reset(new std::vector<Vector3<T>>
                                                  (*o.m_optimizedVertexBuffer));
            m_vertices = VertexDataSource<T>(*m_optimizedVertexBuffer);
        } else {
            m_vertices = o.m_vertices;
        }
        return *this;
    }

    template<typename T>
    ConvexHull<T> &ConvexHull<T>::operator=(ConvexHull<T> &&o) {
        if (&o == this) {
            return *this;
        }
        m_indices = std::move(o.m_indices);
        if (o.m_optimizedVertexBuffer) {
            m_optimizedVertexBuffer = std::move(o.m_optimizedVertexBuffer);
            o.m_vertices = VertexDataSource<T>();
            m_vertices = VertexDataSource<T>(*m_optimizedVertexBuffer);
        } else {
            m_vertices = o.m_vertices;
        }
        return *this;
    }


    template<typename T, typename IndexType>
    HalfEdgeMesh<T, IndexType>::HalfEdgeMesh(
            const MeshBuilder<T> &builderObject,
            const VertexDataSource<T> &vertexData
    ) {
        std::unordered_map<IndexType, IndexType> faceMapping;
        std::unordered_map<IndexType, IndexType> halfEdgeMapping;
        std::unordered_map<IndexType, IndexType> vertexMapping;

        size_t i = 0;
        for (const auto &face: builderObject.m_faces) {
            if (!face.isDisabled()) {
                m_faces.push_back({static_cast<IndexType>(face.m_he)});
                faceMapping[i] = m_faces.size() - 1;

                const auto heIndices = builderObject.getHalfEdgeIndicesOfFace(face);
                for (const auto heIndex: heIndices) {
                    const IndexType vertexIndex = builderObject.m_halfEdges[heIndex].m_endVertex;
                    if (vertexMapping.count(vertexIndex) == 0) {
                        m_vertices.push_back(vertexData[vertexIndex]);
                        vertexMapping[vertexIndex] = m_vertices.size() - 1;
                    }
                }
            }
            i++;
        }

        i = 0;
        for (const auto &halfEdge: builderObject.m_halfEdges) {
            if (!halfEdge.isDisabled()) {
                m_halfEdges.push_back({static_cast<IndexType>(halfEdge.m_endVertex),
                                       static_cast<IndexType>(halfEdge.m_opp),
                                       static_cast<IndexType>(halfEdge.m_face),
                                       static_cast<IndexType>(halfEdge.m_next)});
                halfEdgeMapping[i] = m_halfEdges.size() - 1;
            }
            i++;
        }

        for (auto &face: m_faces) {
            assert(halfEdgeMapping.count(face.m_halfEdgeIndex) == 1);
            face.m_halfEdgeIndex = halfEdgeMapping[face.m_halfEdgeIndex];
        }

        for (auto &he: m_halfEdges) {
            he.m_face = faceMapping[he.m_face];
            he.m_opp = halfEdgeMapping[he.m_opp];
            he.m_next = halfEdgeMapping[he.m_next];
            he.m_endVertex = vertexMapping[he.m_endVertex];
        }
    }


    template<>
    float defaultEps() {
        return 0.0001f;
    }

    template<>
    double defaultEps() {
        return 0.0000001;
    }

    /*
     * Implementation of the algorithm
     */

    template<typename T>
    ConvexHull<T>
    QuickHull<T>::getConvexHull(const VertexDataSource<T> &pointCloud, bool CCW, bool useOriginalIndices, T epsilon) {
        buildMesh(pointCloud, CCW, useOriginalIndices, epsilon);
        return ConvexHull<T>(m_mesh, m_vertexData, CCW, useOriginalIndices);
    }


    template<typename T>
    ConvexHull<T> QuickHull<T>::getConvexHull(const std::vector<Vector3<T>>
                                              &pointCloud,
                                              bool CCW,
                                              bool useOriginalIndices, T
                                              epsilon) {
        VertexDataSource<T> vertexDataSource(pointCloud);
        return
                getConvexHull(vertexDataSource, CCW, useOriginalIndices, epsilon
                );
    }

    template<typename T>
    ConvexHull<T>
    QuickHull<T>::getConvexHull(const Vector3<T> *vertexData, size_t vertexCount, bool CCW, bool useOriginalIndices,
                                T epsilon) {
        VertexDataSource<T> vertexDataSource(vertexData, vertexCount);
        return getConvexHull(vertexDataSource, CCW, useOriginalIndices, epsilon);
    }

    template<typename T>
    ConvexHull<T>
    QuickHull<T>::getConvexHull(const T *vertexData, size_t vertexCount, bool CCW, bool useOriginalIndices, T epsilon) {
        VertexDataSource<T> vertexDataSource((const vec3 *) vertexData, vertexCount);
        return getConvexHull(vertexDataSource, CCW, useOriginalIndices, epsilon);
    }


    template<typename FloatType>
    HalfEdgeMesh<FloatType, size_t>
    QuickHull<FloatType>::getConvexHullAsMesh(const FloatType *vertexData, size_t vertexCount, bool CCW,
                                              FloatType epsilon) {
        VertexDataSource<FloatType> vertexDataSource((const vec3 *) vertexData, vertexCount);
        buildMesh(vertexDataSource, CCW, false, epsilon);
        return HalfEdgeMesh<FloatType, size_t>(m_mesh, m_vertexData);
    }

    template<typename T>
    void QuickHull<T>::buildMesh(const VertexDataSource<T> &pointCloud, bool CCW, bool useOriginalIndices, T epsilon) {
        // CCW is unused for now
        (void) CCW;
        // useOriginalIndices is unused for now
        (void) useOriginalIndices;

        if (pointCloud.size() == 0) {
            m_mesh = MeshBuilder<T>();
            return;
        }
        m_vertexData = pointCloud;

        // Very first: find extreme values and use them to compute the scale of the point cloud.
        m_extremeValues = getExtremeValues();
        m_scale = getScale(m_extremeValues);

        // Epsilon we use depends on the scale
        m_epsilon = epsilon * m_scale;
        m_epsilonSquared = m_epsilon * m_epsilon;

        // Reset diagnostics
        m_diagnostics = DiagnosticsData();

        m_planar = false; // The planar case happens when all the points appear to lie on a two dimensional subspace of R^3.
        createConvexHalfEdgeMesh();
        if (m_planar) {
            const size_t extraPointIndex = m_planarPointCloudTemp.size() - 1;
            for (auto &he: m_mesh.m_halfEdges) {
                if (he.m_endVertex == extraPointIndex) {
                    he.m_endVertex = 0;
                }
            }
            m_vertexData = pointCloud;
            m_planarPointCloudTemp.clear();
        }
    }


    template<typename T>
    void QuickHull<T>::createConvexHalfEdgeMesh() {
        m_visibleFaces.clear();
        m_horizonEdges.clear();
        m_possiblyVisibleFaces.clear();

        // Compute base tetrahedron
        setupInitialTetrahedron();
        assert(m_mesh.m_faces.size() == 4);

        // Init face stack with those faces that have points assigned to them
        m_faceList.clear();
        for (size_t i = 0; i < 4; i++) {
            auto &f = m_mesh.m_faces[i];
            if (f.m_pointsOnPositiveSide && f.m_pointsOnPositiveSide->size() > 0) {
                m_faceList.push_back(i);
                f.m_inFaceStack = 1;
            }
        }

        // Process faces until the face list is empty.
        size_t iter = 0;
        while (!m_faceList.empty()) {
            iter++;
            if (iter == std::numeric_limits<size_t>::max()) {
                // Visible face traversal marks visited faces with iteration counter (to mark that the face has been visited on this iteration) and the max value represents unvisited faces. At this point we have to reset iteration counter. This shouldn't be an
                // issue on 64 bit machines.
                iter = 0;
            }

            const size_t topFaceIndex = m_faceList.front();
            m_faceList.pop_front();

            auto &tf = m_mesh.m_faces[topFaceIndex];
            tf.m_inFaceStack = 0;

            assert(!tf.m_pointsOnPositiveSide || tf.m_pointsOnPositiveSide->size() > 0);
            if (!tf.m_pointsOnPositiveSide || tf.isDisabled()) {
                continue;
            }

            // Pick the most distant point to this triangle plane as the point to which we extrude
            const vec3 &activePoint = m_vertexData[tf.m_mostDistantPoint];
            const size_t activePointIndex = tf.m_mostDistantPoint;

            // Find out the faces that have our active point on their positive side (these are the "visible faces"). The face on top of the stack of course is one of them. At the same time, we create a list of horizon edges.
            m_horizonEdges.clear();
            m_possiblyVisibleFaces.clear();
            m_visibleFaces.clear();
            m_possiblyVisibleFaces.emplace_back(topFaceIndex, std::numeric_limits<size_t>::max());
            while (m_possiblyVisibleFaces.size()) {
                const auto faceData = m_possiblyVisibleFaces.back();
                m_possiblyVisibleFaces.pop_back();
                auto &pvf = m_mesh.m_faces[faceData.m_faceIndex];
                assert(!pvf.isDisabled());

                if (pvf.m_visibilityCheckedOnIteration == iter) {
                    if (pvf.m_isVisibleFaceOnCurrentIteration) {
                        continue;
                    }
                } else {
                    const Plane<T> &P = pvf.m_P;
                    pvf.m_visibilityCheckedOnIteration = iter;
                    const T d = P.m_N.dotProduct(activePoint) + P.m_D;
                    if (d > 0) {
                        pvf.m_isVisibleFaceOnCurrentIteration = 1;
                        pvf.m_horizonEdgesOnCurrentIteration = 0;
                        m_visibleFaces.push_back(faceData.m_faceIndex);
                        for (auto heIndex: m_mesh.getHalfEdgeIndicesOfFace(pvf)) {
                            if (m_mesh.m_halfEdges[heIndex].m_opp != faceData.m_enteredFromHalfEdge) {
                                m_possiblyVisibleFaces.emplace_back(
                                        m_mesh.m_halfEdges[m_mesh.m_halfEdges[heIndex].m_opp].m_face, heIndex);
                            }
                        }
                        continue;
                    }
                    assert(faceData.m_faceIndex != topFaceIndex);
                }

                // The face is not visible. Therefore, the halfedge we came from is part of the horizon edge.
                pvf.m_isVisibleFaceOnCurrentIteration = 0;
                m_horizonEdges.push_back(faceData.m_enteredFromHalfEdge);
                // Store which half edge is the horizon edge. The other half edges of the face will not be part of the final mesh so their data slots can by recycled.
                const auto halfEdges = m_mesh.getHalfEdgeIndicesOfFace(
                        m_mesh.m_faces[m_mesh.m_halfEdges[faceData.m_enteredFromHalfEdge].m_face]);
                const std::int8_t ind = (halfEdges[0] == faceData.m_enteredFromHalfEdge) ? 0 : (halfEdges[1] ==
                                                                                                faceData.m_enteredFromHalfEdge
                                                                                                ? 1 : 2);
                m_mesh.m_faces[m_mesh.m_halfEdges[faceData.m_enteredFromHalfEdge].m_face].m_horizonEdgesOnCurrentIteration |= (
                        1 << ind);
            }
            const size_t horizonEdgeCount = m_horizonEdges.size();

            // Order horizon edges so that they form a loop. This may fail due to numerical instability in which case we give up trying to solve horizon edge for this point and accept a minor degeneration in the convex hull.
            if (!reorderHorizonEdges(m_horizonEdges)) {
                m_diagnostics.m_failedHorizonEdges++;
                std::cerr << "Failed to solve horizon edge." << std::endl;
                auto it = std::find(tf.m_pointsOnPositiveSide->begin(), tf.m_pointsOnPositiveSide->end(),
                                    activePointIndex);
                tf.m_pointsOnPositiveSide->erase(it);
                if (tf.m_pointsOnPositiveSide->size() == 0) {
                    reclaimToIndexVectorPool(tf.m_pointsOnPositiveSide);
                }
                continue;
            }

            // Except for the horizon edges, all half edges of the visible faces can be marked as disabled. Their data slots will be reused.
            // The faces will be disabled as well, but we need to remember the points that were on the positive side of them - therefore
            // we save pointers to them.
            m_newFaceIndices.clear();
            m_newHalfEdgeIndices.clear();
            m_disabledFacePointVectors.clear();
            size_t disableCounter = 0;
            for (auto faceIndex: m_visibleFaces) {
                auto &disabledFace = m_mesh.m_faces[faceIndex];
                auto halfEdges = m_mesh.getHalfEdgeIndicesOfFace(disabledFace);
                for (size_t j = 0; j < 3; j++) {
                    if ((disabledFace.m_horizonEdgesOnCurrentIteration & (1 << j)) == 0) {
                        if (disableCounter < horizonEdgeCount * 2) {
                            // Use on this iteration
                            m_newHalfEdgeIndices.push_back(halfEdges[j]);
                            disableCounter++;
                        } else {
                            // Mark for reusal on later iteration step
                            m_mesh.disableHalfEdge(halfEdges[j]);
                        }
                    }
                }
                // Disable the face, but retain pointer to the points that were on the positive side of it. We need to assign those points
                // to the new faces we create shortly.
                auto t = std::move(m_mesh.disableFace(faceIndex));
                if (t) {
                    assert(t->size()); // Because we should not assign point vectors to faces unless needed...
                    m_disabledFacePointVectors.push_back(std::move(t));
                }
            }
            if (disableCounter < horizonEdgeCount * 2) {
                const size_t newHalfEdgesNeeded = horizonEdgeCount * 2 - disableCounter;
                for (size_t i = 0; i < newHalfEdgesNeeded; i++) {
                    m_newHalfEdgeIndices.push_back(m_mesh.addHalfEdge());
                }
            }

            // Create new faces using the edgeloop
            for (size_t i = 0; i < horizonEdgeCount; i++) {
                const size_t AB = m_horizonEdges[i];

                auto horizonEdgeVertexIndices = m_mesh.getVertexIndicesOfHalfEdge(m_mesh.m_halfEdges[AB]);
                size_t A, B, C;
                A = horizonEdgeVertexIndices[0];
                B = horizonEdgeVertexIndices[1];
                C = activePointIndex;

                const size_t newFaceIndex = m_mesh.addFace();
                m_newFaceIndices.push_back(newFaceIndex);

                const size_t CA = m_newHalfEdgeIndices[2 * i + 0];
                const size_t BC = m_newHalfEdgeIndices[2 * i + 1];

                m_mesh.m_halfEdges[AB].m_next = BC;
                m_mesh.m_halfEdges[BC].m_next = CA;
                m_mesh.m_halfEdges[CA].m_next = AB;

                m_mesh.m_halfEdges[BC].m_face = newFaceIndex;
                m_mesh.m_halfEdges[CA].m_face = newFaceIndex;
                m_mesh.m_halfEdges[AB].m_face = newFaceIndex;

                m_mesh.m_halfEdges[CA].m_endVertex = A;
                m_mesh.m_halfEdges[BC].m_endVertex = C;

                auto &newFace = m_mesh.m_faces[newFaceIndex];

                const Vector3<T> planeNormal = mu::getTriangleNormal(m_vertexData[A], m_vertexData[B], activePoint);
                newFace.m_P = Plane<T>(planeNormal, activePoint);
                newFace.m_he = AB;

                m_mesh.m_halfEdges[CA].m_opp = m_newHalfEdgeIndices[i > 0 ? i * 2 - 1 : 2 * horizonEdgeCount - 1];
                m_mesh.m_halfEdges[BC].m_opp = m_newHalfEdgeIndices[((i + 1) * 2) % (horizonEdgeCount * 2)];
            }

            // Assign points that were on the positive side of the disabled faces to the new faces.
            for (auto &disabledPoints: m_disabledFacePointVectors) {
                assert(disabledPoints);
                for (const auto &point: *(disabledPoints)) {
                    if (point == activePointIndex) {
                        continue;
                    }
                    for (size_t j = 0; j < horizonEdgeCount; j++) {
                        if (addPointToFace(m_mesh.m_faces[m_newFaceIndices[j]], point)) {
                            break;
                        }
                    }
                }
                // The points are no longer needed: we can move them to the vector pool for reuse.
                reclaimToIndexVectorPool(disabledPoints);
            }

            // Increase face stack size if needed
            for (const auto newFaceIndex: m_newFaceIndices) {
                auto &newFace = m_mesh.m_faces[newFaceIndex];
                if (newFace.m_pointsOnPositiveSide) {
                    assert(newFace.m_pointsOnPositiveSide->size() > 0);
                    if (!newFace.m_inFaceStack) {
                        m_faceList.push_back(newFaceIndex);
                        newFace.m_inFaceStack = 1;
                    }
                }
            }
        }

        // Cleanup
        m_indexVectorPool.clear();
    }

/*
 * Private helper functions
 */

    template<typename T>
    std::array<size_t, 6> QuickHull<T>::getExtremeValues() {
        std::array<size_t, 6> outIndices{0, 0, 0, 0, 0, 0};
        T extremeVals[6] = {m_vertexData[0].x, m_vertexData[0].x, m_vertexData[0].y, m_vertexData[0].y,
                            m_vertexData[0].z,
                            m_vertexData[0].z};
        const size_t vCount = m_vertexData.size();
        for (size_t i = 1; i < vCount; i++) {
            const Vector3<T> &pos = m_vertexData[i];
            if (pos.x > extremeVals[0]) {
                extremeVals[0] = pos.x;
                outIndices[0] = i;
            } else if (pos.x < extremeVals[1]) {
                extremeVals[1] = pos.x;
                outIndices[1] = i;
            }
            if (pos.y > extremeVals[2]) {
                extremeVals[2] = pos.y;
                outIndices[2] = i;
            } else if (pos.y < extremeVals[3]) {
                extremeVals[3] = pos.y;
                outIndices[3] = i;
            }
            if (pos.z > extremeVals[4]) {
                extremeVals[4] = pos.z;
                outIndices[4] = i;
            } else if (pos.z < extremeVals[5]) {
                extremeVals[5] = pos.z;
                outIndices[5] = i;
            }
        }
        return outIndices;
    }

    template<typename T>
    bool QuickHull<T>::reorderHorizonEdges(std::vector<size_t> &horizonEdges) {
        const size_t horizonEdgeCount = horizonEdges.size();
        for (size_t i = 0; i < horizonEdgeCount - 1; i++) {
            const size_t endVertex = m_mesh.m_halfEdges[horizonEdges[i]].m_endVertex;
            bool foundNext = false;
            for (size_t j = i + 1; j < horizonEdgeCount; j++) {
                const size_t beginVertex = m_mesh.m_halfEdges[m_mesh.m_halfEdges[horizonEdges[j]].m_opp].m_endVertex;
                if (beginVertex == endVertex) {
                    std::swap(horizonEdges[i + 1], horizonEdges[j]);
                    foundNext = true;
                    break;
                }
            }
            if (!foundNext) {
                return false;
            }
        }
        assert(m_mesh.m_halfEdges[horizonEdges[horizonEdges.size() - 1]].m_endVertex ==
               m_mesh.m_halfEdges[m_mesh.m_halfEdges[horizonEdges[0]].m_opp].m_endVertex);
        return true;
    }

    template<typename T>
    T QuickHull<T>::getScale(const std::array<size_t, 6> &extremeValues) {
        T s = 0;
        for (size_t i = 0; i < 6; i++) {
            const T *v = (const T *) (&m_vertexData[extremeValues[i]]);
            v += i / 2;
            auto a = std::abs(*v);
            if (a > s) {
                s = a;
            }
        }
        return s;
    }

    template<typename T>
    void QuickHull<T>::setupInitialTetrahedron() {
        const size_t vertexCount = m_vertexData.size();

        // If we have at most 4 points, just return a degenerate tetrahedron:
        if (vertexCount <= 4) {
            size_t v[4] = {0, std::min((size_t) 1, vertexCount - 1), std::min((size_t) 2, vertexCount - 1),
                           std::min((size_t) 3, vertexCount - 1)};
            const Vector3<T> N = mu::getTriangleNormal(m_vertexData[v[0]], m_vertexData[v[1]], m_vertexData[v[2]]);
            const Plane<T> trianglePlane(N, m_vertexData[v[0]]);
            if (trianglePlane.isPointOnPositiveSide(m_vertexData[v[3]])) {
                std::swap(v[0], v[1]);
            }
            return m_mesh.setup(v[0], v[1], v[2], v[3]);
        }

        // Find two most distant extreme points.
        T maxD = m_epsilonSquared;
        std::pair<size_t, size_t> selectedPoints;
        for (size_t i = 0; i < 6; i++) {
            for (size_t j = i + 1; j < 6; j++) {
                const T d = m_vertexData[m_extremeValues[i]].getSquaredDistanceTo(m_vertexData[m_extremeValues[j]]);
                if (d > maxD) {
                    maxD = d;
                    selectedPoints = {m_extremeValues[i], m_extremeValues[j]};
                }
            }
        }
        if (maxD == m_epsilonSquared) {
            // A degenerate case: the point cloud seems to consists of a single point
            return m_mesh.setup(0,
                                std::min((size_t) 1, vertexCount - 1),
                                std::min((size_t) 2, vertexCount - 1),
                                std::min((size_t) 3, vertexCount - 1));
        }
        assert(selectedPoints.first != selectedPoints.second);

        // Find the most distant point to the line between the two chosen extreme points.
        const Ray<T> r(m_vertexData[selectedPoints.first],
                       (m_vertexData[selectedPoints.second] - m_vertexData[selectedPoints.first]));
        maxD = m_epsilonSquared;
        size_t maxI = std::numeric_limits<size_t>::max();
        const size_t vCount = m_vertexData.size();
        for (size_t i = 0; i < vCount; i++) {
            const T distToRay = mu::getSquaredDistanceBetweenPointAndRay(m_vertexData[i], r);
            if (distToRay > maxD) {
                maxD = distToRay;
                maxI = i;
            }
        }
        if (maxD == m_epsilonSquared) {
            // It appears that the point cloud belongs to a 1 dimensional subspace of R^3: convex hull has no volume => return a thin triangle
            // Pick any point other than selectedPoints.first and selectedPoints.second as the third point of the triangle
            auto it = std::find_if(m_vertexData.begin(), m_vertexData.end(), [&](const vec3 &ve) {
                return ve != m_vertexData[selectedPoints.first] && ve != m_vertexData[selectedPoints.second];
            });
            const size_t thirdPoint = (it == m_vertexData.end()) ? selectedPoints.first : std::distance(
                    m_vertexData.begin(), it);
            it = std::find_if(m_vertexData.begin(), m_vertexData.end(), [&](const vec3 &ve) {
                return ve != m_vertexData[selectedPoints.first] && ve != m_vertexData[selectedPoints.second] &&
                       ve != m_vertexData[thirdPoint];
            });
            const size_t fourthPoint = (it == m_vertexData.end()) ? selectedPoints.first : std::distance(
                    m_vertexData.begin(), it);
            return m_mesh.setup(selectedPoints.first, selectedPoints.second, thirdPoint, fourthPoint);
        }

        // These three points form the base triangle for our tetrahedron.
        assert(selectedPoints.first != maxI && selectedPoints.second != maxI);
        std::array<size_t, 3> baseTriangle{selectedPoints.first, selectedPoints.second, maxI};
        const Vector3<T> baseTriangleVertices[] = {m_vertexData[baseTriangle[0]], m_vertexData[baseTriangle[1]],
                                                   m_vertexData[baseTriangle[2]]};

        // Next step is to find the 4th vertex of the tetrahedron. We naturally choose the point farthest away from the triangle plane.
        maxD = m_epsilon;
        maxI = 0;
        const Vector3<T> N = mu::getTriangleNormal(baseTriangleVertices[0], baseTriangleVertices[1],
                                                   baseTriangleVertices[2]);
        Plane<T> trianglePlane(N, baseTriangleVertices[0]);
        for (size_t i = 0; i < vCount; i++) {
            const T d = std::abs(mu::getSignedDistanceToPlane(m_vertexData[i], trianglePlane));
            if (d > maxD) {
                maxD = d;
                maxI = i;
            }
        }
        if (maxD == m_epsilon) {
            // All the points seem to lie on a 2D subspace of R^3. How to handle this? Well, let's add one extra point to the point cloud so that the convex hull will have volume.
            m_planar = true;
            const vec3 N1 = mu::getTriangleNormal(baseTriangleVertices[1], baseTriangleVertices[2],
                                                  baseTriangleVertices[0]);
            m_planarPointCloudTemp.clear();
            m_planarPointCloudTemp.insert(m_planarPointCloudTemp.begin(), m_vertexData.begin(), m_vertexData.end());
            const vec3 extraPoint = N1 + m_vertexData[0];
            m_planarPointCloudTemp.push_back(extraPoint);
            maxI = m_planarPointCloudTemp.size() - 1;
            m_vertexData = VertexDataSource<T>(m_planarPointCloudTemp);
        }

        // Enforce CCW orientation (if user prefers clockwise orientation, swap two vertices in each triangle when final mesh is created)
        const Plane<T> triPlane(N, baseTriangleVertices[0]);
        if (triPlane.isPointOnPositiveSide(m_vertexData[maxI])) {
            std::swap(baseTriangle[0], baseTriangle[1]);
        }

        // Create a tetrahedron half edge mesh and compute planes defined by each triangle
        m_mesh.setup(baseTriangle[0], baseTriangle[1], baseTriangle[2], maxI);
        for (auto &f: m_mesh.m_faces) {
            auto v = m_mesh.getVertexIndicesOfFace(f);
            const Vector3<T> &va = m_vertexData[v[0]];
            const Vector3<T> &vb = m_vertexData[v[1]];
            const Vector3<T> &vc = m_vertexData[v[2]];
            const Vector3<T> N1 = mu::getTriangleNormal(va, vb, vc);
            const Plane<T> plane(N1, va);
            f.m_P = plane;
        }

        // Finally we assign a face for each vertex outside the tetrahedron (vertices inside the tetrahedron have no role anymore)
        for (size_t i = 0; i < vCount; i++) {
            for (auto &face: m_mesh.m_faces) {
                if (addPointToFace(face, i)) {
                    break;
                }
            }
        }
    }

    /*
 * Inline function definitions
 */

    template<typename T>
    std::unique_ptr<std::vector<size_t>> QuickHull<T>::getIndexVectorFromPool() {
        auto r = std::move(m_indexVectorPool.get());
        r->clear();
        return r;
    }

    template<typename T>
    void QuickHull<T>::reclaimToIndexVectorPool(std::unique_ptr<std::vector<size_t>> &ptr) {
        const size_t oldSize = ptr->size();
        if ((oldSize + 1) * 128 < ptr->capacity()) {
            // Reduce memory usage! Huge vectors are needed at the beginning of iteration when faces have many points on their positive side. Later on, smaller vectors will suffice.
            ptr.reset(nullptr);
            return;
        }
        m_indexVectorPool.reclaim(ptr);
    }

    template<typename T>
    bool QuickHull<T>::addPointToFace(typename MeshBuilder<T>::Face &f, size_t pointIndex) {
        const T D = mu::getSignedDistanceToPlane(m_vertexData[pointIndex], f.m_P);
        if (D > 0 && D * D > m_epsilonSquared * f.m_P.m_sqrNLength) {
            if (!f.m_pointsOnPositiveSide) {
                f.m_pointsOnPositiveSide = std::move(getIndexVectorFromPool());
            }
            f.m_pointsOnPositiveSide->push_back(pointIndex);
            if (D > f.m_mostDistantPointDist) {
                f.m_mostDistantPointDist = D;
                f.m_mostDistantPoint = pointIndex;
            }
            return true;
        }
        return false;
    }

    template<typename T>
    size_t MeshBuilder<T>::addFace() {
        if (m_disabledFaces.size()) {
            size_t index = m_disabledFaces.back();
            auto &f = m_faces[index];
            assert(f.isDisabled());
            assert(!f.m_pointsOnPositiveSide);
            f.m_mostDistantPointDist = 0;
            m_disabledFaces.pop_back();
            return index;
        }
        m_faces.emplace_back();
        return m_faces.size() - 1;
    }

    template<typename T>
    size_t MeshBuilder<T>::addHalfEdge() {
        if (m_disabledHalfEdges.size()) {
            const size_t index = m_disabledHalfEdges.back();
            m_disabledHalfEdges.pop_back();
            return index;
        }
        m_halfEdges.emplace_back();
        return m_halfEdges.size() - 1;
    }

    template<typename T>
    std::unique_ptr<std::vector<size_t>> MeshBuilder<T>::disableFace(size_t faceIndex) {
        auto &f = m_faces[faceIndex];
        f.disable();
        m_disabledFaces.push_back(faceIndex);
        return std::move(f.m_pointsOnPositiveSide);
    }

    template<typename T>
    void MeshBuilder<T>::disableHalfEdge(size_t heIndex) {
        auto &he = m_halfEdges[heIndex];
        he.disable();
        m_disabledHalfEdges.push_back(heIndex);
    }

    template<typename T>
    void MeshBuilder<T>::setup(size_t a, size_t b, size_t c, size_t d) {
        m_faces.clear();
        m_halfEdges.clear();
        m_disabledFaces.clear();
        m_disabledHalfEdges.clear();

        m_faces.reserve(4);
        m_halfEdges.reserve(12);

        // Create halfedges
        HalfEdge AB;
        AB.m_endVertex = b;
        AB.m_opp = 6;
        AB.m_face = 0;
        AB.m_next = 1;
        m_halfEdges.push_back(AB);

        HalfEdge BC;
        BC.m_endVertex = c;
        BC.m_opp = 9;
        BC.m_face = 0;
        BC.m_next = 2;
        m_halfEdges.push_back(BC);

        HalfEdge CA;
        CA.m_endVertex = a;
        CA.m_opp = 3;
        CA.m_face = 0;
        CA.m_next = 0;
        m_halfEdges.push_back(CA);

        HalfEdge AC;
        AC.m_endVertex = c;
        AC.m_opp = 2;
        AC.m_face = 1;
        AC.m_next = 4;
        m_halfEdges.push_back(AC);

        HalfEdge CD;
        CD.m_endVertex = d;
        CD.m_opp = 11;
        CD.m_face = 1;
        CD.m_next = 5;
        m_halfEdges.push_back(CD);

        HalfEdge DA;
        DA.m_endVertex = a;
        DA.m_opp = 7;
        DA.m_face = 1;
        DA.m_next = 3;
        m_halfEdges.push_back(DA);

        HalfEdge BA;
        BA.m_endVertex = a;
        BA.m_opp = 0;
        BA.m_face = 2;
        BA.m_next = 7;
        m_halfEdges.push_back(BA);

        HalfEdge AD;
        AD.m_endVertex = d;
        AD.m_opp = 5;
        AD.m_face = 2;
        AD.m_next = 8;
        m_halfEdges.push_back(AD);

        HalfEdge DB;
        DB.m_endVertex = b;
        DB.m_opp = 10;
        DB.m_face = 2;
        DB.m_next = 6;
        m_halfEdges.push_back(DB);

        HalfEdge CB;
        CB.m_endVertex = b;
        CB.m_opp = 1;
        CB.m_face = 3;
        CB.m_next = 10;
        m_halfEdges.push_back(CB);

        HalfEdge BD;
        BD.m_endVertex = d;
        BD.m_opp = 8;
        BD.m_face = 3;
        BD.m_next = 11;
        m_halfEdges.push_back(BD);

        HalfEdge DC;
        DC.m_endVertex = c;
        DC.m_opp = 4;
        DC.m_face = 3;
        DC.m_next = 9;
        m_halfEdges.push_back(DC);

        // Create faces
        Face ABC;
        ABC.m_he = 0;
        m_faces.push_back(std::move(ABC));

        Face ACD;
        ACD.m_he = 3;
        m_faces.push_back(std::move(ACD));

        Face BAD;
        BAD.m_he = 6;
        m_faces.push_back(std::move(BAD));

        Face CBD;
        CBD.m_he = 9;
        m_faces.push_back(std::move(CBD));
    }

    template<typename T>
    std::array<size_t, 3> MeshBuilder<T>::getVertexIndicesOfFace(const Face &f) const {
        std::array<size_t, 3> v;
        const HalfEdge *he = &m_halfEdges[f.m_he];
        v[0] = he->m_endVertex;
        he = &m_halfEdges[he->m_next];
        v[1] = he->m_endVertex;
        he = &m_halfEdges[he->m_next];
        v[2] = he->m_endVertex;
        return v;
    }

    template<typename T>
    std::array<size_t, 2> MeshBuilder<T>::getVertexIndicesOfHalfEdge(const HalfEdge &he) const {
        return {m_halfEdges[he.m_opp].m_endVertex, he.m_endVertex};
    }

    template<typename T>
    std::array<size_t, 3> MeshBuilder<T>::getHalfEdgeIndicesOfFace(const Face &f) const {
        return {f.m_he, m_halfEdges[f.m_he].m_next, m_halfEdges[m_halfEdges[f.m_he].m_next].m_next};
    }

    template<typename T>
    void MeshBuilder<T>::Face::disable() {
        m_he = std::numeric_limits<size_t>::max();
    }

    template<typename T>
    bool MeshBuilder<T>::Face::isDisabled() const {
        return m_he == std::numeric_limits<size_t>::max();
    }

    template<typename T>
    void MeshBuilder<T>::HalfEdge::disable() {
        m_endVertex = std::numeric_limits<size_t>::max();
    }

    template<typename T>
    bool MeshBuilder<T>::HalfEdge::isDisabled() const {
        return m_endVertex == std::numeric_limits<size_t>::max();
    }


/*
* Explicit template specifications for float and double
*/

template
class ConvexHull<double>;

template
class HalfEdgeMesh<double, size_t>;

template
class QuickHull<double>;

template
class QuickHull<float>;

}