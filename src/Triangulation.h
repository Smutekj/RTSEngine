#ifndef BOIDS_TRIANGULATION_H
#define BOIDS_TRIANGULATION_H

#include <unordered_set>
#include <deque>
#include <iostream>

#include "core.h"
#include "Geometry.h"

class Grid;
class DebugInfo;

typedef u_int_32_t TriInd;

// counter-clockwise ordering
//                    verts[2]
//                      /\
//    neighbours[2]    /  \      neighbours[1]
//                    /    \
//                   /      \
//                   --------
//           verts[0]          verts[1]
//                  neighbours[0]
//

//! \struct holds data relating to triangle. Ordering is counterclowise (see image)
struct Triangle {
    //    VertInd vertinds[3];                   //! indices of vertices (we do not hold coordinates here!)
    Vertex verts[3];                    //! indices of vertices (we do not hold coordinates here!)
    ::std::array<TriInd, 3> neighbours;   //! indices of neighbouring triangles
    ::std::array<bool, 3> is_constrained; //! whether corresponding edge is constrained (is this needed here?)
    explicit Triangle() {
        neighbours = {-1u, -1u, -1u};
        is_constrained = {false, false, false};
    }

    int countFreeEdges() const { return !is_constrained[0] + !is_constrained[1] + !is_constrained[2]; }
};

inline int next(const int ind_in_tri) {
    // assert(ind_in_tri <= 2 and ind_in_tri >= 0);
    if (ind_in_tri == 2) {
        return 0;
    }
    return ind_in_tri + 1;
}

inline int prev(const int ind_in_tri) {
    // assert(ind_in_tri <= 2 and ind_in_tri >= 0);
    if (ind_in_tri == 0) {
        return 2;
    }
    return ind_in_tri - 1;
}


inline int indInTriOf(const Triangle& tri, const TriInd neighbour) {
    auto neighbour_it = ::std::find(tri.neighbours.begin(), tri.neighbours.end(), neighbour);
    return neighbour_it - tri.neighbours.begin();
}

//! \brief used for finding orientation of \p p1 w.r.t. ( \p p3 - \p p2 ) 
template <typename VectorType, typename VectorType2>
inline float sign(const VectorType& p1, const VectorType2& p2, const VectorType2& p3) {
    return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
}

//! \brief checks if point \p r lies inside the triangle \p tri 
//! \tparam VectorType  
//! \param r 
//! \param tri
//! \returns true if the point lies inside the triangle 
template <typename VectorType>
inline bool isInTriangle(const VectorType& r, const Triangle& tri) {
    float d1, d2, d3;
    bool has_neg, has_pos;

    d1 = sign(r, tri.verts[0], tri.verts[1]);
    d2 = sign(r, tri.verts[1], tri.verts[2]);
    d3 = sign(r, tri.verts[2], tri.verts[0]);

    has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

    return !(has_neg && has_pos);
}

//! \struct function object used to convert pair of integers to a hash (Maybe, I did not really test it :D)
struct EdgeHash {
    std::size_t operator()(const EdgeVInd& e) const {
        return ::std::hash<VertInd>()(e.from) ^ ::std::hash<VertInd>()(e.to);
    }
};

//! \struct function object used to convert an edge into a hash
struct EdgeHash2 {
    std::size_t operator()(const EdgeI& e) const {
        return std::hash<VertInd>()(e.from.x) ^ std::hash<VertInd>()(e.t.x) ^ std::hash<VertInd>()(e.from.y) ^
               std::hash<VertInd>()(e.t.y);
    }
};

//! \class searches for triangles in triangulation
class TriangleFinder {
    Grid* p_grid;
    TriangleFinder() = default;


};


//! TODO: Refactor into two classes 1. for constructing and holding triangulation 2. for searching for triangles
//! TODO: think whether we should define edges as a pair of two triangle indices that are connected by the edge (is it
//! really necessary/better?)
class Triangulation {

  public:
    ::std::vector<std::array<VertInd, 3>> tri_ind2vert_inds_;

    std::vector<Vertex> vertices_; //! coordinates of vertices (can be only integers!)
    std::vector<Triangle> triangles_;

    std::unordered_set<EdgeI, EdgeHash2> fixed_edges2_;

    Grid* p_grid; //! underlying grid that will be used for finding triangles containing query point

    std::vector<TriInd> cell2triangle_ind_;
    TriInd last_found_tri_ind_ = 0; //! cached index of last found triangle (in a lot of cases new searched triangle is
                                    //! near previously found one)
    DebugInfo* dbg; //! for debugging
    sf::RenderWindow* window;

    sf::Vector2f boundary_; //! size of the boundary box

  public:
    explicit Triangulation(Grid& s_grid);
    Triangulation(Grid& s_grid, std::string filename);
    Triangulation(Grid& s_grid, DebugInfo& dbg);

    void reset();

    bool triangulationIsConsistent() const;

    TriInd findTriangle(Vertex query_point, bool start_from_last_found = false);
    TriInd findTriangle(sf::Vector2f query_point, bool start_from_last_found = false);

    void createBoundaryAndSuperTriangle();

    bool edgesIntersect(const EdgeVInd e1, const EdgeVInd e2) const noexcept;
    bool edgesIntersect(const EdgeI e1, const EdgeI e2) const noexcept;

    template <typename VectorType>
    bool linesIntersect(const VectorType& v11, const VectorType& v12, const VectorType& v21,
                        const VectorType& v22) const noexcept;
    bool linesIntersect(const Edgef& e1, const Edgef& e2) const noexcept;
    float linesIntersect(const sf::Vector2f& v11, const sf::Vector2f& v12, const Vertex& v21,
                         const Vertex& v22) const noexcept;


    VertInd findOverlappingVertex(const Vertex& new_vertex, const TriInd tri_ind) const;
    EdgeVInd findOverlappingEdge(const Vertex& new_vertex, const TriInd tri_ind) const;

    void insertVertex(const Vertex& v, bool = false);
    void insertVertexIntoSpace(const Vertex& v, TriInd, VertInd);
    


    void insertConstraint(const EdgeVInd edge);

    sf::Vector2f calcTriangleCenter(const Triangle& tri) const;

    void insertRectangle(const sf::Vector2i center, const sf::Vector2i sides, bool invert);

    void updateCellGrid();

    struct VertexInsertionData{
        VertInd overlapping_vertex = -1;
        EdgeVInd overlapping_edge; 
    };
    
    VertexInsertionData insertVertexAndGetData(const Vertex& v, bool = false);

  private:
 
    void insertVertexOnEdge(const Vertex& v, TriInd tri_ind_a, TriInd tri_ind_b, const EdgeI& edge);

    bool isConvex(const Vertex v1, const Vertex v2, const Vertex v3, const Vertex v4) const;
    int oppositeIndex(const TriInd np, const Triangle& tri);

    VertInd oppositeOfEdge(const Triangle& tri, const EdgeVInd& e) const;
    VertInd oppositeOfEdge(const Triangle& tri, const EdgeI& e) const;

    TriInd triangleOppositeOfEdge(const Triangle& tri, const EdgeVInd& edge);
    TriInd triangleOppositeOfEdge(const Triangle& tri, const EdgeI& edge) const;

    void swapConnectingEdge(const TriInd& tri_ind_a, const TriInd& tri_ind_b, int v_ind, Vertex v_a, bool inv = false);

    void findIntersectingEdges(const EdgeVInd& e, std::deque<EdgeI>& intersected_edges,
                               std::deque<TriInd>& intersected_tri_inds);

    bool hasGoodOrientation(const Triangle& tri) const;

    bool isCounterClockwise(const Vertex& v_query, const Vertex& v1, const Vertex& v2) const;

    int indexOf(const VertInd& v_ind, const Triangle& tri) const;
    int indexOf(const Vertex& v, const Triangle& tri) const;

    bool needSwap(const Vertex& vp, const Vertex& v1, const Vertex& v2, const Vertex& v3) const;

    bool lineIntersectsEdge(const sf::Vector2f v1, const sf::Vector2f& v2, const VertInd e_ind1,
                            const VertInd e_ind2) const;

    void dumpToFile(const std::string filename) const;
};

#endif // BOIDS_TRIANGULATION_H
