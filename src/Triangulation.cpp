
#include <fstream>
#include <iostream>
#include <string>
#include <stack>

#include "Triangulation.h"
#include "Grid.h"
#include "DebugInfo.h"


//! \brief useful for debugging triangulation 
void drawCircumcircle(sf::RenderWindow& window, sf::Vector2f p1, sf::Vector2f p2, sf::Vector2f p3) {

    sf::CircleShape circumcircle;
    circumcircle.setOutlineColor(sf::Color::Blue);
    circumcircle.setOutlineThickness(2);
    circumcircle.setFillColor(sf::Color::Transparent);

    auto p21 = p2 - p1;
    auto p31 = p3 - p1;
    auto p23 = p2 - p3;

    float denom = std::abs(p21.y * p23.x - p21.x * p23.y);

    float alpha = dot(p23, p23) * dot(-p21, -p31) / (2 * denom * denom);
    float beta = dot(p31, p31) * dot(p21, p23) / (2 * denom * denom);
    float gamma = dot(p21, p21) * dot(p31, -p23) / (2 * denom * denom);

    float r = std::sqrt(dot(p21, p21)) * std::sqrt(dot(p23, p23)) * std::sqrt(dot(p31, p31)) / (2 * denom);
    auto p_center = alpha * p1 + beta * p2 + gamma * p3;

    circumcircle.setRadius(r);
    circumcircle.setPosition(p_center);
    circumcircle.move(-r, -r);
    window.draw(circumcircle);
    //        window.display();
}

std::vector<std::string> separateLine(const std::string& str, std::string delimiter) {
    std::string::size_type pos = 0;
    std::string::size_type prev = 0;
    std::vector<std::string> strings;
    while ((pos = str.find(delimiter, prev)) != std::string::npos) {
        strings.push_back(str.substr(prev, pos - prev));
        prev = pos + delimiter.size();
    }

    // To get the last substring (or only, if delimiter is not found)
    strings.push_back(str.substr(prev));
    return strings;
};

Triangulation::Triangulation(Grid& s_grid)
    : p_grid(&s_grid)
    , boundary_(s_grid.cell_size_.x * s_grid.n_cells_.x, s_grid.cell_size_.x * s_grid.n_cells_.x) {
    cell2triangle_ind_.resize(s_grid.n_cells_.x * s_grid.n_cells_.y, -1);
}
Triangulation::Triangulation(Grid& s_grid, DebugInfo& db)
    : Triangulation(s_grid) {
    dbg = &db;
}

Triangulation::Triangulation(Grid& s_grid, std::string filename)
    : Triangulation(s_grid) {

    std::ifstream file(filename);
    if (file.is_open()) {
        std::string line;
        VertInd vx;
        VertInd vy;
        Triangle tri;
        getline(file, line);
        while (getline(file, line)) {
            if (line == "Triangles:") {
                break;
            }
            const auto separated_line = separateLine(line, " ");
            std::cout << std::stoi(separated_line[0]) << "\n";
            vertices_.emplace_back(std::stoi(separated_line.at(0)), std::stoi(separated_line.at(1)));
        }
        while (getline(file, line)) {
            const auto separated_line = separateLine(line, " ");
            for (int i = 0; i < 3; ++i) {
                //                tri.vertinds[i] = std::stoi(separated_line.at(i));
                tri.neighbours[i] = std::stoi(separated_line.at(i + 3));
            }
            triangles_.push_back(tri);
        }
        file.close();
    }
}

//! \brief Clears all vertices triangles and edges
void Triangulation::reset() {
    vertices_.clear();
    triangles_.clear();
    tri_ind2vert_inds_.clear();
    last_found_tri_ind_ = 0;
    for (auto& cached_tri_ind : cell2triangle_ind_) {
        cached_tri_ind = -1;
    }
}

//! \brief searches for triangle containing query_point
//! \param query_point point whose containing triangle we are looking for
//! \param start_from_last_found whether we start looking from previously found triangle... uses search grid if false
//!\returns index of a triangle containing query_point or -1 if no such triangle is found
TriInd Triangulation::findTriangle(Vertex query_point, bool from_last_found) {

    TriInd tri_ind = last_found_tri_ind_;
    if (!from_last_found) {
        auto cell_ind = p_grid->coordToCell(asFloat(query_point));
        tri_ind = cell2triangle_ind_.at(cell_ind);
        if (tri_ind == -1) { //! if there is no triangle yet in a cell we default to linear search
            for (TriInd i = 0; i < triangles_.size(); ++i) {
                if (isInTriangle(query_point, triangles_[i])) {
                    last_found_tri_ind_ = i;
                    return i;
                }
            }
        }
    }

    Vertex v_current;

    if (isInTriangle(query_point, triangles_[tri_ind])) {
        return tri_ind;
    }
    { //!  we look at triangle edges and find in which direction we need to go
        const auto& tri = triangles_[tri_ind];

        const auto v0 = (tri.verts[0]);
        const auto v1 = (tri.verts[1]);
        const auto v2 = (tri.verts[2]);
        const auto r_start = (v0 + v1 + v2) / 3;

        if (linesIntersect(r_start, query_point, v0, v1)) {
            tri_ind = tri.neighbours[0];
            v_current = tri.verts[0];
        } else if (linesIntersect(r_start, query_point, v1, v2)) {
            tri_ind = tri.neighbours[1];
            v_current = tri.verts[1];
        } else if (linesIntersect(r_start, query_point, v2, v0)) {
            tri_ind = tri.neighbours[2];
            v_current = tri.verts[2];
        } else {
            tri_ind = -1;
        } // throw std::runtime_error("we should never get here!");}
    }
    if (tri_ind == -1) { //! if we cannot find way (no idea why yet) we do brute force
        for (TriInd i = 0; i < triangles_.size(); ++i) {
            if (isInTriangle(query_point, triangles_[i])) {
                last_found_tri_ind_ = i;
                return i;
            }
        }
    }
    //! walk in the found diraction to triangle containing end_ind;
    while (!isInTriangle(query_point, triangles_[tri_ind])) {
        const auto& tri = triangles_[tri_ind];

        const auto index_in_tri = indexOf(v_current, tri);
        assert(index_in_tri != -1);

        const auto v1 = tri.verts[next(index_in_tri)];
        const auto v2 = tri.verts[prev(index_in_tri)];

        if (linesIntersect(v_current, query_point, v1, v2)) {
            tri_ind = tri.neighbours[next(index_in_tri)];
            v_current = tri.verts[next(index_in_tri)];
        } else {
            tri_ind = tri.neighbours[index_in_tri];
        }
    }

    last_found_tri_ind_ = tri_ind;
    return tri_ind;
}

//! \brief searches for triangle containing query_point
//! \param query_point point whose containing triangle we are looking for
//! \param start_from_last_found whether we start looking from previously found triangle... uses search grid if false
//!\returns index of a triangle containing query_point or -1 if no such triangle is found
TriInd Triangulation::findTriangle(sf::Vector2f query_point, bool from_last_found) {

    return findTriangle(static_cast<Vertex>(query_point), from_last_found);
    // TriInd tri_ind = last_found_tri_ind_;
    // if (!from_last_found) {
    //     auto cell_ind = p_grid->coordToCell(r);
    //     tri_ind = cell2triangle_ind_.at(cell_ind);
    //     if (tri_ind == -1) { //! if there is no triangle yet in a cell we default to linear search
    //         for (TriInd i = 0; i < triangles_.size(); ++i) {
    //             if (isInTriangle(r, triangles_[i], vertices_)) {
    //                 last_found_tri_ind_ = i;
    //                 return i;
    //             }
    //         }
    //     }
    // }

    // Vertex v_current;

    // if (isInTriangle(r, triangles_[tri_ind], vertices_)) {
    //     return tri_ind;
    // }
    // { //!  we look at triangle edges and find in which direction we need to go
    //     const auto& tri = triangles_[tri_ind];

    //     const auto r_end = r;
    //     const auto v0 = (tri.verts[0]);
    //     const auto v1 = (tri.verts[1]);
    //     const auto v2 = (tri.verts[2]);
    //     const auto r_start = asFloat(v0 + v1 + v2) / 3.f;

    //     if (linesIntersect(r_start, r_end, v0, v1)) {
    //         tri_ind = tri.neighbours[0];
    //         v_current = tri.verts[0];
    //     } else if (linesIntersect(r_start, r_end, v1, v2)) {
    //         tri_ind = tri.neighbours[1];
    //         v_current = tri.verts[1];
    //     } else if (linesIntersect(r_start, r_end, v2, v0)) {
    //         tri_ind = tri.neighbours[2];
    //         v_current = tri.verts[2];
    //     } else {
    //         tri_ind = -1;
    //     } // throw std::runtime_error("we should never get here!");}
    // }
    // if (tri_ind == -1) { //! if we cannot find way (no idea why yet) we do brute force
    //     for (TriInd i = 0; i < triangles_.size(); ++i) {
    //         if (isInTriangle(r, triangles_[i], vertices_)) {
    //             last_found_tri_ind_ = i;
    //             return i;
    //         }
    //     }
    // }
    // //! walk in the found diraction to triangle containing end_ind;
    // while (!isInTriangle(r, triangles_[tri_ind], vertices_)) {
    //     const auto& tri = triangles_[tri_ind];

    //     const auto index_in_tri = indexOf(v_current, tri);
    //     assert(index_in_tri != -1);

    //     const auto v1 = tri.verts[next(index_in_tri)];
    //     const auto v2 = tri.verts[prev(index_in_tri)];

    //     if (linesIntersect(asFloat(v_current), r, v1, v2)) {
    //         tri_ind = tri.neighbours[next(index_in_tri)];
    //         v_current = tri.verts[next(index_in_tri)];
    //     } else {
    //         tri_ind = tri.neighbours[index_in_tri];
    //     }
    // }

    // last_found_tri_ind_ = tri_ind;
    // return tri_ind;
}


//! \brief creates supertriangle which contains specified boundary then
//!        inserts 4 vertices corresponding to the boundary (upper left point is [0,0])
//! \param boundary dimensions of a boundary contained in supertriangle
void Triangulation::createBoundaryAndSuperTriangle() {

    const sf::Vector2i boundary = {Geometry::BOX[0], Geometry::BOX[1]};
    Triangle super_triangle;
    tri_ind2vert_inds_.push_back({0, 1, 2});

    Vertex super_tri0 = {boundary.x / 2, boundary.y * 3};
    Vertex super_tri1 = {3 * boundary.x, -boundary.y / 2};
    Vertex super_tri2 = {-3 * boundary.x, -boundary.y / 2};

    super_triangle.verts[0] = super_tri0;
    super_triangle.verts[1] = super_tri1;
    super_triangle.verts[2] = super_tri2;
    triangles_.push_back(super_triangle);

    vertices_.push_back(super_tri0);
    vertices_.push_back(super_tri1);
    vertices_.push_back(super_tri2);

    Vertex v1 = {0, 0};
    Vertex v2 = {boundary.x, 0};
    Vertex v3 = boundary;
    Vertex v4 = {0, boundary.y};
    insertVertex(v1, false);
    assert(triangulationIsConsistent());
    insertVertex(v2, true);
    assert(triangulationIsConsistent());
    insertVertex(v3, true);
    assert(triangulationIsConsistent());
    insertVertex(v4, true);
    assert(triangulationIsConsistent());

    EdgeVInd e1 = {3, 4};
    EdgeVInd e2 = {4, 5};
    EdgeVInd e3 = {5, 6};
    EdgeVInd e4 = {6, 3};

    insertConstraint(e1);
    insertConstraint(e2);
    insertConstraint(e3);
    insertConstraint(e4);

    assert(triangulationIsConsistent());
}

//! \param np index of the neighbouring triangle
//! \param tri 
//! \returns index in triangle of the vertex in tri opposite of triangle \p np

int Triangulation::oppositeIndex(const TriInd np, const Triangle& tri) {
    if (np == tri.neighbours[0]) {
        return 2;
    } else if (np == tri.neighbours[1]) {
        return 0;
    } else if (np == tri.neighbours[2]) {
        return 1;
    } else {
        throw std::runtime_error("triangles not neighbours!");
    }
}

//! \param tri  triangle
//! \param edge edge with vertex indices
//! \returns index of triangle opposite of \p tri accross \p edge
TriInd Triangulation::triangleOppositeOfEdge(const Triangle& tri, const EdgeVInd& e) {
    const auto i2 = indexOf(e.from, tri);
    const auto i1 = indexOf(e.to, tri);
    if (i1 == -1 or i2 == -1) {
        return -1;
    }
    assert(i1 != -1 and i2 != -1);
    return tri.neighbours[(oppositeOfEdge(tri, e) + 1) % 3];
}

//! \param tri  trian
//! \param e    edge containing vertex coordinates it connects
//! \returns index of triangle opposite of \p tri accross \p edge
TriInd Triangulation::triangleOppositeOfEdge(const Triangle& tri, const EdgeI& e) const {
    const auto i2 = indexOf(e.from, tri);
    const auto i1 = indexOf(e.to(), tri);
    if (i1 == -1 or i2 == -1) {
        return -1;
    }
    assert(i1 != -1 and i2 != -1);
    return tri.neighbours[(oppositeOfEdge(tri, e) + 1) % 3];
}

//! \param e1 edge containg indices of vertices forming a first edge
//! \param e2 edge containg indices of vertices forming a second edge
//! \returns true if lines representing given edges intersect
bool Triangulation::edgesIntersect(const EdgeVInd e1, const EdgeVInd e2) const noexcept {

    //    return linesIntersect(vertices_[e1.from], vertices_[e1.to], vertices_[e2.from], vertices_[e2.to]);
    const auto dr = vertices_[e2.from] - vertices_[e1.from];
    const auto t1 = vertices_[e1.to] - vertices_[e1.from];
    const auto t2 = vertices_[e2.to] - vertices_[e2.from];
    const sf::Vector2i n1 = {t1.y, -t1.x};
    const sf::Vector2i n2 = {t2.y, -t2.x};

    const auto alpha1 = +dot(dr, n2) / dot(t1, n2);
    const auto alpha2 = -dot(dr, n1) / dot(t2, n1);
    return alpha1 > 0 and alpha1 < 1 and alpha2 > 0 and alpha2 < 1;
}

//! \param e1 edge containg vertices forming a first edge
//! \param e2 edge containg vertices forming a second edge
//! \returns true if lines representing given edges intersect
bool Triangulation::edgesIntersect(const EdgeI e1, const EdgeI e2) const noexcept {

    //    return linesIntersect(vertices_[e1.from], vertices_[e1.to], vertices_[e2.from], vertices_[e2.to]);
    const auto dr = e2.from - e1.from;
    const auto t1 = e1.t;
    const auto t2 = e2.t;
    const Vertex n1 = {t1.y, -t1.x};
    const Vertex n2 = {t2.y, -t2.x};

    const auto alpha1 = +dot(dr, n2) / dot(t1, n2);
    const auto alpha2 = -dot(dr, n1) / dot(t2, n1);
    return alpha1 > 0 and alpha1 < 1 and alpha2 > 0 and alpha2 < 1;
}

//! \returns true if line connecting v11 and v12 intersects with line connecting v21 and v22
float Triangulation::linesIntersect(const sf::Vector2f& v11, const sf::Vector2f& v12, const Vertex& v21,
                                    const Vertex& v22) const noexcept {

    const auto dr = asFloat(v21) - v11;
    const auto t1 = v12 - v11;
    const auto t2 = asFloat(v22 - v21);
    const sf::Vector2f n1 = {t1.y, -t1.x};
    const sf::Vector2f n2 = {t2.y, -t2.x};

    const auto alpha1 = +dot(dr, n2) / dot(t1, n2);
    const auto alpha2 = -dot(dr, n1) / dot(t2, n1);
    if (alpha1 >= 0 and alpha1 <= 1 and alpha2 >= 0 and alpha2 <= 1) {
        return alpha1;
    }
    return -1;
}


//! \brief returns true if line connecting v11 and v12 intersects with line connecting v21 and v22
template <typename VectorType>
bool Triangulation::linesIntersect(const VectorType& v11, const VectorType& v12, const VectorType& v21,
                                   const VectorType& v22) const noexcept {

    const auto& dr = v21 - v11;
    const auto& t1 = v12 - v11;
    const auto& t2 = v22 - v21;
    const VectorType n1 = {t1.y, -t1.x};
    const VectorType n2 = {t2.y, -t2.x};

    const auto alpha1 = +dot(dr, n2) / dot(t1, n2);
    const auto alpha2 = -dot(dr, n1) / dot(t2, n1);
    return alpha1 >= 0 and alpha1 <= 1 and alpha2 >= 0 and alpha2 <= 1;
}

//! \brief
//! \returns true if edges \p e1 and \p e2 intersect
bool Triangulation::linesIntersect(const Edgef& e1, const Edgef& e2) const noexcept {
    return linesIntersect(e1.from, e1.to(), e2.from, e2.to());
}


//! \brief updates search grid used to find triangles
//! \brief should be called whenever triagulation changes
void Triangulation::updateCellGrid() {
    const auto dx = p_grid->cell_size_.x;
    const auto dy = p_grid->cell_size_.y;

    const auto n_cells_x = p_grid->n_cells_.x;
    const auto n_cells_y = p_grid->n_cells_.y;
    for (int j = 0; j < n_cells_y - 1;
         j++) { //! we walk zig-zag so that each next cell grid is close to the last one which helps findTriangle
        for (int i = 0; i < n_cells_x; i++) {
            const sf::Vector2f r_center = {i * dx + dx / 2.f, j * dy + dy / 2.f};
            const auto cell_ind = j * n_cells_x + i;

            bool from_previous_one = true;
            cell2triangle_ind_.at(cell_ind) = findTriangle(r_center, from_previous_one);
        }

        j++;
        for (int i = n_cells_x - 1; i >= 0; i--) {
            const sf::Vector2f r_center = {i * dx + dx / 2.f, j * dy + dy / 2.f};
            const auto cell_ind = j * n_cells_x + i;

            bool from_previous_one = true;
            cell2triangle_ind_.at(cell_ind) = findTriangle(r_center, from_previous_one);
        }
    }
    if (n_cells_y % 2 == 1) {
        int j = n_cells_y - 1;
        for (int i = n_cells_x - 1; i >= 0; i--) {
            const sf::Vector2f r_center = {i * dx + dx / 2.f, j * dy + dy / 2.f};
            const auto cell_ind = j * n_cells_x + i;

            bool from_previous_one = true;
            cell2triangle_ind_.at(cell_ind) = findTriangle(r_center, from_previous_one);
        }
    }
}


//! \param v1 first point of the line 
//! \param v2 second point of the line
//! \param e_ind1 first ind of vertex forming edge
//! \param e_ind2 second ind of vertex forming edge
//! \returns  true if line connecting points \p v1 and \p v2 intersects the given edge
bool Triangulation::lineIntersectsEdge(const sf::Vector2f v1, const sf::Vector2f& v2, const VertInd e_ind1,
                                       const VertInd e_ind2) const {

    const auto& v21 = asFloat(vertices_[e_ind1]);
    const auto& v22 = asFloat(vertices_[e_ind2]);

    const auto dr = v21 - v1;
    const auto t1 = v2 - v1;
    const auto t2 = v22 - v21;
    const sf::Vector2f n1 = {t1.y, -t1.x};
    const sf::Vector2f n2 = {t2.y, -t2.x};

    const auto alpha1 = +dot(dr, n2) / dot(t1, n2);
    const auto alpha2 = -dot(dr, n1) / dot(t2, n1);
    return alpha1 >= 0 and alpha1 <= 1 and alpha2 >= 0 and alpha2 <= 1;
}

   //! \brief inserts vertex given we know that it lies directly on the given edge
    //! \param v_ind index of inserted vertex
    //! \param tri_ind_a index of counterclockwise triangle
    //! \param tri_ind_b index of a clockwise triangle
    //! \param edge 
void Triangulation::insertVertexOnEdge(const Vertex& new_vertex, TriInd tri_ind_a, TriInd tri_ind_b, const EdgeI& e) {
    const auto new_vertex_ind = vertices_.size() - 1;

    const auto tri_ind_a_new = triangles_.size();
    const auto tri_ind_b_new = triangles_.size() + 1;

    auto tri_a = triangles_.at(tri_ind_a);
    auto tri_b = triangles_.at(tri_ind_b);

    const auto ind_in_tri_a = oppositeOfEdge(tri_a, e);
    const auto ind_in_tri_b = oppositeOfEdge(tri_b, e);

    Triangle tri_a_new = tri_a;
    Triangle tri_b_new = tri_b;

    tri_ind2vert_inds_.push_back(tri_ind2vert_inds_[tri_ind_a]);
    tri_ind2vert_inds_.push_back(tri_ind2vert_inds_[tri_ind_b]);

    for (int i = 0; i < 3; ++i) {
        tri_a_new.is_constrained[i] = false;
    }
    for (int i = 0; i < 3; ++i) {
        tri_b_new.is_constrained[i] = false;
    }
    if (new_vertex.x == 795 and new_vertex.y == 215) {
        std::cout << "w";
    }

    //    tri_a_new.vertinds[(ind_in_tri_a + 1) % 3] = new_vertex_ind;
    tri_ind2vert_inds_[tri_ind_a_new][(ind_in_tri_a + 1) % 3] = new_vertex_ind;
    tri_a_new.verts[(ind_in_tri_a + 1) % 3] = new_vertex;
    tri_a_new.neighbours[ind_in_tri_a] = tri_ind_a;
    tri_a_new.neighbours[(ind_in_tri_a + 1) % 3] = tri_ind_b;
    tri_a_new.is_constrained[(ind_in_tri_a + 1) % 3] = true;
    if (tri_a.is_constrained[(ind_in_tri_a + 2) % 3]) {
        tri_a_new.is_constrained[(ind_in_tri_a + 2) % 3] = true;
    }
    //    tri_a.vertinds[(ind_in_tri_a + 2) % 3] = new_vertex_ind;
    tri_ind2vert_inds_[tri_ind_a][(ind_in_tri_a + 2) % 3] = new_vertex_ind;
    tri_a.verts[(ind_in_tri_a + 2) % 3] = new_vertex;
    tri_a.neighbours[(ind_in_tri_a + 1) % 3] = tri_ind_b_new;
    tri_a.neighbours[(ind_in_tri_a + 2) % 3] = tri_ind_a_new;
    tri_a.is_constrained[(ind_in_tri_a + 2) % 3] = false;

    //    tri_b_new.vertinds[(ind_in_tri_b + 1) % 3] = new_vertex_ind;
    tri_ind2vert_inds_[tri_ind_b_new][(ind_in_tri_b + 1) % 3] = new_vertex_ind;
    tri_b_new.verts[(ind_in_tri_b + 1) % 3] = new_vertex;
    tri_b_new.neighbours[ind_in_tri_b] = tri_ind_b;
    tri_b_new.neighbours[(ind_in_tri_b + 1) % 3] = tri_ind_a;
    tri_b_new.is_constrained[(ind_in_tri_b + 1) % 3] = true;
    if (tri_b.is_constrained[(ind_in_tri_b + 2) % 3]) {
        tri_b_new.is_constrained[(ind_in_tri_b + 2) % 3] = true;
    }
    //    tri_b.vertinds[(ind_in_tri_b + 2) % 3] = new_vertex_ind;
    tri_ind2vert_inds_[tri_ind_b][(ind_in_tri_b + 2) % 3] = new_vertex_ind;
    tri_b.verts[(ind_in_tri_b + 2) % 3] = new_vertex;
    tri_b.neighbours[(ind_in_tri_b + 1) % 3] = tri_ind_a_new;
    tri_b.neighbours[(ind_in_tri_b + 2) % 3] = tri_ind_b_new;
    tri_b.is_constrained[(ind_in_tri_b + 2) % 3] = false;

    //! we tell old triangles that they have a new neighbour;
    if (tri_a_new.neighbours[(ind_in_tri_a + 2) % 3] != -1) {
        auto& tri_next = triangles_[tri_a_new.neighbours[(ind_in_tri_a + 2) % 3]];
        for (int i = 0; i < 3; ++i) {
            if (tri_next.neighbours[i] == tri_ind_a) {
                tri_next.neighbours[i] = tri_ind_a_new;
                break;
            }
        }
    }
    //! we tell old triangles that they have a new neighbour;
    if (tri_b_new.neighbours[(ind_in_tri_b + 2) % 3] != -1) {
        auto& tri_next = triangles_[tri_b_new.neighbours[(ind_in_tri_b + 2) % 3]];
        for (int i = 0; i < 3; ++i) {
            if (tri_next.neighbours[i] == tri_ind_b) {
                tri_next.neighbours[i] = tri_ind_b_new;
                break;
            }
        }
    }

    triangles_.push_back(tri_a_new);
    triangles_.push_back(tri_b_new);
    triangles_[tri_ind_a] = tri_a;
    triangles_[tri_ind_b] = tri_b;
    fixed_edges2_.erase(e);
    fixed_edges2_.insert({e.from, new_vertex});
    fixed_edges2_.insert({new_vertex, e.to()});

    //! fix delaunay property
    std::stack<std::pair<TriInd, TriInd>> triangles_to_fix;
    if (tri_a_new.neighbours[(ind_in_tri_a + 2) % 3] != -1) {
        triangles_to_fix.push({tri_ind_a_new, tri_a_new.neighbours[(ind_in_tri_a + 2) % 3]});
    }
    if (tri_a.neighbours[ind_in_tri_a] != -1) {
        triangles_to_fix.push({tri_ind_a, tri_a.neighbours[ind_in_tri_a]});
    }

    if (tri_b_new.neighbours[(ind_in_tri_b + 2) % 3] != -1) {
        triangles_to_fix.push({tri_ind_b_new, tri_b_new.neighbours[(ind_in_tri_b + 2) % 3]});
    }
    if (tri_b.neighbours[ind_in_tri_b] != -1) {
        triangles_to_fix.push({tri_ind_b, tri_b.neighbours[ind_in_tri_b]});
    }

    while (!triangles_to_fix.empty()) {
        auto next_tri_ind = triangles_to_fix.top().second;
        auto& next_tri = triangles_[next_tri_ind];
        auto old_tri_ind = triangles_to_fix.top().first;
        auto& old_tri = triangles_[old_tri_ind];
        triangles_to_fix.pop();

        auto opposite_ind_in_tri = oppositeIndex(old_tri_ind, next_tri);
        auto new_vert_ind_in_tri = indexOf(new_vertex, old_tri);
        auto v3 = next_tri.verts[opposite_ind_in_tri];
        auto v1 = next_tri.verts[(opposite_ind_in_tri + 1) % 3];
        auto v2 = next_tri.verts[(opposite_ind_in_tri + 2) % 3];
        auto vp = new_vertex;

        ;
        //        EdgeVInd flipped_edge = {next_tri.vertinds[(opposite_ind_in_tri + 1) % 3],
        //        next_tri.vertinds[(opposite_ind_in_tri + 2) % 3]};
        if (needSwap(vp, v1, v2, v3) and !next_tri.is_constrained[next(opposite_ind_in_tri)]) {

            auto& a = old_tri.verts[(new_vert_ind_in_tri + 2) % 3];
            assert(isCounterClockwise(a, v3, vp));
            swapConnectingEdge(old_tri_ind, next_tri_ind, new_vertex_ind, new_vertex);

            if (old_tri.neighbours[(new_vert_ind_in_tri + 1) % 3] != -1) {
                triangles_to_fix.emplace(old_tri_ind, old_tri.neighbours[(new_vert_ind_in_tri + 1) % 3]);
            }
            if (next_tri.neighbours[(opposite_ind_in_tri + 2) % 3] != -1) {
                triangles_to_fix.emplace(next_tri_ind, next_tri.neighbours[(opposite_ind_in_tri + 2) % 3]);
            }
        }
    }
}
 
//! \param new_vertex 
//! \param tri_ind triangle index of an existing triangle containing new_vertex 
//! \returns index of the overlapping vertex
//! \returns -1 in case there is no existing overlapping vertex
VertInd Triangulation::findOverlappingVertex(const Vertex& new_vertex, const TriInd tri_ind) const{
    assert(tri_ind != -1);
    const auto old_triangle = triangles_[tri_ind];
    const auto v0 = old_triangle.verts[0];
    const auto v1 = old_triangle.verts[1];
    const auto v2 = old_triangle.verts[2];
    if (v0 == new_vertex) {
        return tri_ind2vert_inds_[tri_ind][0];
    }
    if (v1 == new_vertex) {
        return tri_ind2vert_inds_[tri_ind][1];
    }
    if (v2 == new_vertex) {
        return tri_ind2vert_inds_[tri_ind][2];
    }
    return -1;
}

//! \brief checks whether \p new_vertex lies on some existing edge 
//! \param new_vertex
//! \param tri_ind triangle index of an existing triangle containing new_vertex 
//! \returns overlapping edge
//! \returns {-1, -1} in case there is no overlapping edge
EdgeVInd Triangulation::findOverlappingEdge(const Vertex& new_vertex, const TriInd tri_ind) const{
    const auto& old_triangle = triangles_[tri_ind];
    const auto v0 = old_triangle.verts[0];
    const auto v1 = old_triangle.verts[1];
    const auto v2 = old_triangle.verts[2];

    const auto& vert_inds = tri_ind2vert_inds_[tri_ind];

    sf::Vector2i edge_normal0 = {-(v1.y - v0.y), v1.x - v0.x};
    sf::Vector2i edge_normal1 = {-(v2.y - v1.y), v2.x - v1.x};
    sf::Vector2i edge_normal2 = {-(v0.y - v2.y), v0.x - v2.x};
    EdgeI edge0 = {old_triangle.verts[0], old_triangle.verts[1]};
    EdgeI edge1 = {old_triangle.verts[1], old_triangle.verts[2]};
    EdgeI edge2 = {old_triangle.verts[2], old_triangle.verts[0]};
    if (old_triangle.is_constrained[0] and dot(new_vertex - v0, edge_normal0) == 0) {
        return  {vert_inds[0], vert_inds[1]};
    } else if (old_triangle.is_constrained[1] and dot(new_vertex - v1, edge_normal1) == 0) {
        return  {vert_inds[1], vert_inds[2]};
    } else if (old_triangle.is_constrained[2] and dot(new_vertex - v2, edge_normal2) == 0) {
        return  {vert_inds[2], vert_inds[0]};;            
    }
    return EdgeVInd();
}

//! \brief inserts \p new_vertex into triangulation
//! \param new_vertex
//! \param search_from_last_one should be true if last added vertex is not far from \p new_vertex 
void Triangulation::insertVertex(const Vertex& new_vertex, bool search_from_last_one) {
    insertVertexAndGetData(new_vertex, search_from_last_one);
}

//! \brief inserts \p new_vertex into triangulation, the inserted vertex can either:
//! \brief already exist, or it may lie on an existing edge or it lies in free space
//! \param new_vertex
//! \param search_from_last_one should be true if last added vertex is not far from \p new_vertex 
//! \returns data relating to the actual type of insertion performed
Triangulation::VertexInsertionData Triangulation::insertVertexAndGetData(const Vertex& new_vertex, bool search_from_last_one) {

    VertexInsertionData data;

    auto tri_ind = findTriangle(new_vertex, search_from_last_one);
    const auto old_triangle = triangles_[tri_ind];

    data.overlapping_vertex = findOverlappingVertex(new_vertex, tri_ind);
    if(data.overlapping_vertex != -1){
        return data;
    }

    const auto new_vertex_ind = vertices_.size();
    vertices_.push_back(new_vertex);

    const auto overlapping_edge = findOverlappingEdge(new_vertex, tri_ind);
    if(overlapping_edge.from != -1){
        const EdgeI edge_i = {vertices_[overlapping_edge.from], vertices_[overlapping_edge.to]};
        insertVertexOnEdge(new_vertex, tri_ind, triangleOppositeOfEdge(old_triangle, edge_i), edge_i);
        data.overlapping_edge = overlapping_edge;
        return data;
    }
    
    insertVertexIntoSpace(new_vertex, tri_ind, new_vertex_ind);
    return data;
}

//! \brief inserts \p new_vertex into triangulation knowing it lies in free space (not on edge or vertex) 
//! \param new_vertex
//! \param search_from_last_one should be true if last added vertex is not far from \p new_vertex 
//! \returns data relating to the actual type of insertion performed
void Triangulation::insertVertexIntoSpace(const Vertex& new_vertex, TriInd tri_ind, VertInd new_vertex_ind){

    const auto old_triangle = triangles_[tri_ind];

    const auto first_new_triangle_ind = tri_ind;
    const auto second_new_triangle_ind = triangles_.size();
    const auto third_new_triangle_ind = triangles_.size() + 1;

    Triangle t1_new = old_triangle;
    Triangle t2_new = old_triangle;
    Triangle t3_new = old_triangle;

    tri_ind2vert_inds_.push_back(tri_ind2vert_inds_[tri_ind]);
    tri_ind2vert_inds_.push_back(tri_ind2vert_inds_[tri_ind]);

    assert(hasGoodOrientation(old_triangle));

    for (int i = 0; i < 3; ++i) {
        t1_new.is_constrained[i] = false;
    }
    for (int i = 0; i < 3; ++i) {
        t2_new.is_constrained[i] = false;
    }
    for (int i = 0; i < 3; ++i) {
        t3_new.is_constrained[i] = false;
    }
    //    tri_ind2vert_inds_[first_new_triangle_ind] = new_vertex_ind;
    tri_ind2vert_inds_[first_new_triangle_ind][2] = new_vertex_ind;
    t1_new.verts[2] = new_vertex;
    t1_new.neighbours[2] = third_new_triangle_ind;
    t1_new.neighbours[1] = second_new_triangle_ind;
    if (old_triangle.is_constrained[0]) {
        t1_new.is_constrained[0] = true;
    }

    triangles_[tri_ind] = t1_new;

    //    t2_new.vertinds[0] =  new_vertex_ind;
    tri_ind2vert_inds_[second_new_triangle_ind][0] = new_vertex_ind;
    t2_new.verts[0] = new_vertex;
    t2_new.neighbours[0] = first_new_triangle_ind;
    t2_new.neighbours[2] = third_new_triangle_ind;
    if (old_triangle.is_constrained[1]) {
        t2_new.is_constrained[1] = true;
    }

    //    t3_new.vertinds[1] = new_vertex_ind;
    tri_ind2vert_inds_[third_new_triangle_ind][1] = new_vertex_ind;
    t3_new.verts[1] = new_vertex;
    t3_new.neighbours[1] = second_new_triangle_ind;
    t3_new.neighbours[0] = first_new_triangle_ind;
    if (old_triangle.is_constrained[2]) {
        t3_new.is_constrained[2] = true;
    }

    //! fix delaunay property
    std::stack<std::pair<TriInd, TriInd>> triangles_to_fix;
    if (old_triangle.neighbours[0] != -1) {
        triangles_to_fix.push({first_new_triangle_ind, old_triangle.neighbours[0]});
    }
    if (old_triangle.neighbours[1] != -1) {
        triangles_to_fix.push({second_new_triangle_ind, old_triangle.neighbours[1]});
    }
    if (old_triangle.neighbours[2] != -1) {
        triangles_to_fix.push({third_new_triangle_ind, old_triangle.neighbours[2]});
    }

    //! we tell old triangles that they have a new neighbour;
    if (old_triangle.neighbours[1] != -1) {
        auto neighbour = old_triangle.neighbours[1];
        auto& tri_next = triangles_[neighbour];
        for (int i = 0; i < 3; ++i) {
            if (tri_next.neighbours[i] == tri_ind) {
                tri_next.neighbours[i] = second_new_triangle_ind;
                break;
            }
        }
    }
    if (old_triangle.neighbours[2] != -1) {
        auto neighbour = old_triangle.neighbours[2];
        auto& tri_next = triangles_[neighbour];
        for (int i = 0; i < 3; ++i) {
            if (tri_next.neighbours[i] == tri_ind) {
                tri_next.neighbours[i] = third_new_triangle_ind;
                break;
            }
        }
    }

    //        removeTriangle(tri_ind);

    //        triangles_.push_back(t1_new);
    triangles_.push_back(t2_new);
    triangles_.push_back(t3_new);

    while (!triangles_to_fix.empty()) {
        auto next_tri_ind = triangles_to_fix.top().second;
        auto& next_tri = triangles_[next_tri_ind];
        auto old_tri_ind = triangles_to_fix.top().first;
        auto& old_tri = triangles_[old_tri_ind];
        triangles_to_fix.pop();

        auto opposite_ind_in_tri = oppositeIndex(old_tri_ind, next_tri);
        auto newvert_ind_in_tri = indexOf(new_vertex, old_tri);
        auto v3 = next_tri.verts[opposite_ind_in_tri];
        auto v1a = next_tri.verts[(opposite_ind_in_tri + 1) % 3];
        auto v2a = next_tri.verts[(opposite_ind_in_tri + 2) % 3];
        auto vp = new_vertex;

        //        EdgeVInd flipped_edge = {next_tri.vertinds[(opposite_ind_in_tri + 1) % 3],
        //        next_tri.vertinds[(opposite_ind_in_tri + 2) % 3]};

        if (needSwap(vp, v1a, v2a, v3) and !next_tri.is_constrained[next(opposite_ind_in_tri)]) {

            auto& a = old_tri.verts[(newvert_ind_in_tri + 2) % 3];
            assert(isCounterClockwise(a, v3, vp));
            swapConnectingEdge(old_tri_ind, next_tri_ind, new_vertex_ind, new_vertex);

            if (old_tri.neighbours[(newvert_ind_in_tri + 1) % 3] != -1) {
                triangles_to_fix.emplace(old_tri_ind, old_tri.neighbours[(newvert_ind_in_tri + 1) % 3]);
            }
            if (next_tri.neighbours[(opposite_ind_in_tri + 2) % 3] != -1) {
                triangles_to_fix.emplace(next_tri_ind, next_tri.neighbours[(opposite_ind_in_tri + 2) % 3]);
            }
        }
    }

}


//! \returns center of mass of the triangle \param tri
sf::Vector2f Triangulation::calcTriangleCenter(const Triangle& tri) const {
    const auto v11 = tri.verts[0];
    const auto v12 = tri.verts[1];
    const auto v13 = tri.verts[2];
    return asFloat((v11 + v12 + v13) / 3);
}

//! \returns true if quadrilateral formed by the giver four vertices is convex
bool Triangulation::isConvex(const Vertex v1, const Vertex v2, const Vertex v3,
                             const Vertex v4) const { //! v1-v3 and v2-v4 are diagonals
    return linesIntersect(v1, v3, v2, v4);
}

//! \param edge represented by vertex indices
//! \param tri
//! \returns index in triangle of the vertex in \p tri which is opposite of the \p edge
VertInd Triangulation::oppositeOfEdge(const Triangle& tri, const EdgeVInd& edge) const {
    const auto i1 = indexOf(edge.from, tri);
    const auto i2 = indexOf(edge.to, tri);
    return 3 - (i1 + i2); //! i1 + i2 has values 1,2,3.. corresponding output should be 2,1,0
}

//! \param edge represented by vertex coordinates
//! \param tri
//! \returns index in triangle of the vertex in \p tri which is opposite of the \p edge
VertInd Triangulation::oppositeOfEdge(const Triangle& tri, const EdgeI& e) const {
    const auto i1 = indexOf(e.from, tri);
    const auto i2 = indexOf(e.to(), tri);
    return 3 - (i1 + i2); //! i1 + i2 has values 1,2,3.. corresponding output should be 2,1,0
}


//! \brief checks if neighbours of each triangles are consistent and 
//! \brief if tri_ind2vert_inds_ points to right place in vertices_ array
bool Triangulation::triangulationIsConsistent() const {
    for (int tri_ind = 0; tri_ind < triangles_.size(); ++tri_ind) {
        const auto& tri = triangles_[tri_ind];
        for (int k = 0; k < 3; ++k) {
            const auto neighbour_tri_ind = tri.neighbours[k];
            if (neighbour_tri_ind != -1) {
                const auto& neighbour_tri = triangles_[neighbour_tri_ind];
                const auto ind_in_neirhbour_tri = 0;
                const auto found_at =
                    std::find(neighbour_tri.neighbours.begin(), neighbour_tri.neighbours.end(), tri_ind) -
                    neighbour_tri.neighbours.begin();
                if (found_at < 0 or found_at > 2) {
                    return false;
                }
                if (tri.is_constrained[k] xor neighbour_tri.is_constrained[found_at]) {
                    return false;
                }
            }
            const auto v = tri.verts[k];
            if (v != vertices_[tri_ind2vert_inds_[tri_ind][k]]) {
                return false;
            }
        }
    }
    return true;
}


//! \brief forces triangulation to have a constrained edge connecting \p e.from and \p e.to
//! \param e edge representing the constraint
void Triangulation::insertConstraint(const EdgeVInd e) {
    std::deque<EdgeI> intersected_edges;
    std::deque<TriInd> intersected_tri_inds;
    findIntersectingEdges(e, intersected_edges, intersected_tri_inds);

    std::vector<EdgeI> newly_created_edges;
    std::vector<std::pair<TriInd, TriInd>> newly_created_edge_tris;

    auto vi_ind = e.from;
    auto vj_ind = e.to;
    auto vi = vertices_[vi_ind];
    auto vj = vertices_[vj_ind];

    EdgeI e_inserted = {vi, vj};

    //    auto v_current = vi;
    while (!intersected_edges.empty()) {
        auto tri_ind = intersected_tri_inds.front();
        auto& tri = triangles_[tri_ind];
        intersected_tri_inds.pop_front();

        auto e_next = intersected_edges.front();
        intersected_edges.pop_front();

        auto next_tri_ind = triangleOppositeOfEdge(tri, e_next);
        auto& next_tri = triangles_[next_tri_ind];

        auto v_current_ind = tri_ind2vert_inds_[tri_ind][oppositeOfEdge(tri, e_next)];
        auto v_current = tri.verts[oppositeOfEdge(tri, e_next)];
        auto v_opposite_ind_in_tri = oppositeIndex(tri_ind, next_tri);
        auto v_opposite_current = next_tri.verts[v_opposite_ind_in_tri];

        if (isConvex(v_current, e_next.from, v_opposite_current, e_next.to())) {

            if (!isCounterClockwise(v_opposite_current, vi, vj)) {
                swapConnectingEdge(tri_ind, next_tri_ind, v_current_ind, v_current);
            } else {
                swapConnectingEdge(tri_ind, next_tri_ind, v_current_ind, v_current, true);
            }

            //            e_next.from =  v_current_ind;
            //            e_next.t = v_opposite_current_ind;
            e_next = {v_current, v_opposite_current};

            if (edgesIntersect(e_next, e_inserted)) {
                intersected_edges.push_back(e_next);
                intersected_tri_inds.push_back(tri_ind);
            } else {
                newly_created_edges.push_back(e_next);
                newly_created_edge_tris.push_back({tri_ind, next_tri_ind});
            }
        } else {
            intersected_edges.push_back(e_next);
            intersected_tri_inds.push_back(tri_ind);
        }
    }

    fixed_edges2_.insert(e_inserted);
    //    std::unordered_set<

    //! Fix Delaunay triangulation (Steps 4.1 - 4.3)
    bool some_swap_happened = true;
    while (some_swap_happened) {
        some_swap_happened = false;

        for (int i = 0; i < newly_created_edges.size(); ++i) {
            const auto& e_new = newly_created_edges[i];
            const auto tri_ind_a = newly_created_edge_tris[i].first;
            auto& tri_a = triangles_[tri_ind_a];
            auto tri_ind_b = triangleOppositeOfEdge(tri_a, e_new);
            if (tri_ind_b == -1) {
                tri_ind_b = newly_created_edge_tris[i].second;
            }
            auto& tri_b = triangles_[tri_ind_b];

            const auto opposite_ind_in_tri_a = oppositeOfEdge(tri_a, e_new);
            const auto opposite_ind_in_tri_b = oppositeOfEdge(tri_b, e_new);

            vi = tri_a.verts[opposite_ind_in_tri_a];
            vj = tri_b.verts[opposite_ind_in_tri_b];
            vi_ind = tri_ind2vert_inds_[tri_ind_a][opposite_ind_in_tri_a];

            bool edge_is_not_fixed = fixed_edges2_.count(e_new) == 0;
            //            bool edge_is_not_fixed = .count(e_new) == 0;
            if (e_new == e_inserted) {
                tri_a.is_constrained[next(opposite_ind_in_tri_a)] = true;
                tri_b.is_constrained[next(opposite_ind_in_tri_b)] = true;
                continue;
            }
            //            if (!edge_is_not_fixed) {
            //                tri_a.is_constrained[next(opposite_ind_in_tri_a)] = true;
            //                tri_b.is_constrained[next(opposite_ind_in_tri_b)] = true;
            //                continue;
            //            }
            //
            //            const auto v1 = vertices_[tri_a.vertinds[(vi_ind + 1) % 3]];
            //            const auto v2 = vertices_[tri_a.vertinds[(vi_ind + 2) % 3]];

            const auto v1 = tri_a.verts[(vi_ind + 1) % 3];
            const auto v2 = tri_a.verts[(vi_ind + 2) % 3];

            bool edge_needs_swap = needSwap(vi, v1, v2, vj);
            bool is_convex = true; //! Convexity check should be automatically taken care of by needSwap but it doesn't
                                   //! and I don't know why yet :(
            if (!isConvex(vertices_[vi_ind], v1, v2, vertices_[vj_ind])) {
                is_convex = false;
            }
            if (edge_needs_swap and is_convex) {

                swapConnectingEdge(tri_ind_a, tri_ind_b, vi_ind, vi);
                some_swap_happened = true;
                newly_created_edges[i] = {vi, vj};
            }
        }
    }
}

//! \brief inserts rectangle with \p center
//! \param center of the rectangle
//! \param sides  lengths of rectangle edges
//! \param invert true if normals should be inverted
void Triangulation::insertRectangle(const sf::Vector2i center, const sf::Vector2i sides, bool invert) {

    const auto dx = sides.x;
    const auto dy = sides.y;

    VertInd n_vertices = vertices_.size();
    insertVertex(center + sf::Vector2i{-dx / 2, -dy / 2});
    insertVertex(center + sf::Vector2i{-dx / 2, dy / 2});
    insertVertex(center + sf::Vector2i{dx / 2, dy / 2});
    insertVertex(center + sf::Vector2i{dx / 2, -dy / 2});

    if (invert) {
        insertConstraint({n_vertices, n_vertices + 1});
        insertConstraint({n_vertices + 1, n_vertices + 2});
        insertConstraint({n_vertices + 2, n_vertices + 3});
        insertConstraint({n_vertices + 3, n_vertices});
    } else {
        insertConstraint({n_vertices, n_vertices + 3});
        insertConstraint({n_vertices + 3, n_vertices + 2});
        insertConstraint({n_vertices + 2, n_vertices + 1});
        insertConstraint({n_vertices + 1, n_vertices});
    }
}

void Triangulation::swapConnectingEdge(const TriInd& tri_ind_a, const TriInd& tri_ind_b, int v_ind_a, Vertex v_a,
                                       bool inv) {
    auto& tri_a = triangles_[tri_ind_a];
    auto& tri_b = triangles_[tri_ind_b];

    auto v_a_ind_in_tri = indexOf(v_a, tri_a);
    auto v_b_ind_in_tri = oppositeIndex(tri_ind_a, tri_b);

    if (inv) {
        //        tri_a.vertinds[(v_a_ind_in_tri + 1 + inv) % 3] = tri_b.vertinds[v_b_ind_in_tri];
        tri_ind2vert_inds_[tri_ind_a][(v_a_ind_in_tri + 1 + inv) % 3] = tri_ind2vert_inds_[tri_ind_b][v_b_ind_in_tri];
        tri_a.verts[(v_a_ind_in_tri + 1 + inv) % 3] = tri_b.verts[v_b_ind_in_tri];
        tri_a.neighbours[(v_a_ind_in_tri + 1) % 3] = tri_b.neighbours[(v_b_ind_in_tri + 1 + inv) % 3];
        tri_a.is_constrained[(v_a_ind_in_tri + 1) % 3] = tri_b.is_constrained[(v_b_ind_in_tri + 1 + inv) % 3];
        auto old_neighbour = tri_a.neighbours[(v_a_ind_in_tri + 1 + inv) % 3];
        auto old_edge_state = tri_a.is_constrained[(v_a_ind_in_tri + 1 + inv) % 3];
        tri_a.neighbours[(v_a_ind_in_tri + 1 + inv) % 3] = tri_ind_b;
        tri_a.is_constrained[(v_a_ind_in_tri + 1 + inv) % 3] = false;

        //        tri_b.vertinds[(v_b_ind_in_tri + 1 + inv) % 3] = v_a;
        tri_ind2vert_inds_[tri_ind_b][(v_b_ind_in_tri + 1 + inv) % 3] = v_ind_a;
        tri_b.verts[(v_b_ind_in_tri + 1 + inv) % 3] = v_a;
        tri_b.neighbours[(v_b_ind_in_tri + inv) % 3] = old_neighbour;
        tri_b.is_constrained[(v_b_ind_in_tri + 1 + inv) % 3] = old_edge_state;
        tri_b.neighbours[(v_b_ind_in_tri + 1 + inv) % 3] = tri_ind_a;
        tri_b.is_constrained[(v_b_ind_in_tri + 1 + inv) % 3] = false;

    } else {
        //        tri_a.vertinds[(v_a_ind_in_tri + 1 ) % 3] = tri_b.vertinds[v_b_ind_in_tri];
        tri_ind2vert_inds_[tri_ind_a][(v_a_ind_in_tri + 1) % 3] = tri_ind2vert_inds_[tri_ind_b][v_b_ind_in_tri];
        tri_a.verts[(v_a_ind_in_tri + 1) % 3] = tri_b.verts[v_b_ind_in_tri];
        tri_a.neighbours[(v_a_ind_in_tri + 1) % 3] = tri_b.neighbours[(v_b_ind_in_tri) % 3];
        tri_a.is_constrained[(v_a_ind_in_tri + 1) % 3] = tri_b.is_constrained[v_b_ind_in_tri];
        auto old_neighbour = tri_a.neighbours[v_a_ind_in_tri];
        auto old_edge_state = tri_a.is_constrained[v_a_ind_in_tri];
        tri_a.neighbours[v_a_ind_in_tri] = tri_ind_b;
        tri_a.is_constrained[(v_a_ind_in_tri) % 3] = false;

        //        tri_b.vertinds[(v_b_ind_in_tri + 1) % 3] = v_a;
        tri_ind2vert_inds_[tri_ind_b][(v_b_ind_in_tri + 1) % 3] = v_ind_a;
        tri_b.verts[(v_b_ind_in_tri + 1) % 3] = v_a;
        tri_b.neighbours[(v_b_ind_in_tri + 1) % 3] = old_neighbour;
        tri_b.is_constrained[(v_b_ind_in_tri + 1) % 3] = old_edge_state;
        tri_b.neighbours[v_b_ind_in_tri] = tri_ind_a;
        tri_b.is_constrained[v_b_ind_in_tri] = false;
    }
    //! fix neighbours

    if (tri_a.neighbours[(v_a_ind_in_tri + 1) % 3] != -1) {
        auto& tri_next_next = triangles_[tri_a.neighbours[(v_a_ind_in_tri + 1) % 3]];
        for (int i = 0; i < 3; ++i) {
            if (tri_next_next.neighbours[i] == tri_ind_b) {
                tri_next_next.neighbours[i] = tri_ind_a;
                break;
            }
            if (i == 2) {
                throw std::runtime_error("no neighbour found!");
            }
        }
    }
    if (tri_b.neighbours[(v_b_ind_in_tri + 1) % 3] != -1) {
        auto& tri_next_next = triangles_[tri_b.neighbours[(v_b_ind_in_tri + 1) % 3]];
        for (int i = 0; i < 3; ++i) {
            if (tri_next_next.neighbours[i] == tri_ind_a) {
                tri_next_next.neighbours[i] = tri_ind_b;
                break;
            }
            if (i == 2) {
                throw std::runtime_error("no neighbour found!");
            }
        }
    }
}

//! \brief finds existing edges and their corresponding triangles that would intersect with edge \p e
//! \brief writes the edges into \p intersected_edges and triangles into \p intersected_tri_inds    
//! \param e edge containing vertex indices
//! \param intersected_edges here the intersected edges are written;
//! \param intersected_tri_inds here the tri inds corresponding to \p intersected_edges are written 
void Triangulation::findIntersectingEdges(const EdgeVInd& e, std::deque<EdgeI>& intersected_edges,
                                          std::deque<TriInd>& intersected_tri_inds) {
    const auto vi_ind = e.from;
    const auto vj_ind = e.to;
    if (vi_ind == vj_ind) {
        return;
    }
    const auto vi = vertices_[vi_ind];
    const auto vj = vertices_[vj_ind];

    const auto start_tri_ind = findTriangle(vi, false);
    const auto start_tri = triangles_[start_tri_ind];
    const auto end_tri_ind = findTriangle(vj, true);
    const auto end_tri = triangles_[end_tri_ind];

    EdgeI e_inserted(vi, vj);

    auto tri_ind = start_tri_ind;
    auto tri = triangles_[tri_ind];
    auto index_in_tri = indexOf(vi, tri);
    EdgeI e_next = {tri.verts[prev(index_in_tri)], tri.verts[next(index_in_tri)]};

    //! find first direction of walk by looking at triangles that contain vi;
    while (!edgesIntersect(e_next, e_inserted)) {
        tri_ind = tri.neighbours[index_in_tri];
        tri = triangles_[tri_ind];
        index_in_tri = indexOf(vi, tri);
        assert(index_in_tri != -1);
        e_next = {tri.verts[prev(index_in_tri)], tri.verts[next(index_in_tri)]};
        if (e_next.from == vj) {
            triangles_[tri_ind].is_constrained[(index_in_tri + 2) % 3] = true;
            const auto tri_ind_opposite = triangleOppositeOfEdge(tri, e_inserted);
            const auto ind_in_opposite_tri = indexOf(vi, triangles_[tri_ind_opposite]);
            triangles_[tri_ind_opposite].is_constrained[ind_in_opposite_tri] = true;
            return; //! the end vertex of the constraint is already connected to start vertex;
        } else if (e_next.to() == vj) {
            triangles_[tri_ind].is_constrained[index_in_tri] = true;
            const auto tri_ind_opposite = triangleOppositeOfEdge(tri, e_inserted);
            const auto ind_in_opposite_tri = indexOf(vj, triangles_[tri_ind_opposite]);
            triangles_[tri_ind_opposite].is_constrained[ind_in_opposite_tri] = true;
            return;
        }
    }
    intersected_edges.push_back(e_next);
    intersected_tri_inds.push_back({tri_ind});
    auto v_current = tri.verts[(index_in_tri + 1) % 3];
    tri_ind = tri.neighbours[(index_in_tri + 1) % 3];
    EdgeI e_next1;
    EdgeI e_next2;

    //! walk in the found direction to the triangle containing end_ind;
    while (tri_ind != end_tri_ind) {
        tri = triangles_[tri_ind];
        index_in_tri = indexOf(v_current, tri);
        assert(index_in_tri != -1); //! we expect v_current to always exist in tri

        e_next1 = {tri.verts[(index_in_tri) % 3], tri.verts[(index_in_tri + 1) % 3]};
        e_next2 = {tri.verts[(index_in_tri + 1) % 3], tri.verts[(index_in_tri + 2) % 3]};
        if (e_next1.from == vj or e_next2.to() == vj or e_next1.to() == vj) {
            break; //! we found end_v_ind;
        }
        intersected_tri_inds.push_back(tri_ind);

        if (edgesIntersect(e_next1, e_inserted)) {
            tri_ind = tri.neighbours[index_in_tri];
            intersected_edges.push_back(e_next1);
        } else if (edgesIntersect(e_next2, e_inserted)) {
            intersected_edges.push_back(e_next2);
            tri_ind = tri.neighbours[(index_in_tri + 1) % 3];
            v_current = tri.verts[(index_in_tri + 1) % 3];
        }
    }
    //    intersected_tri_inds.push_back(tri_ind); //! there is one more triangle compared to intersected edges
}

bool Triangulation::hasGoodOrientation(const Triangle& tri) const {
    return isCounterClockwise(tri.verts[2], tri.verts[1], tri.verts[0]) and
           isCounterClockwise(tri.verts[0], tri.verts[2], tri.verts[1]) and
           isCounterClockwise(tri.verts[1], tri.verts[0], tri.verts[2]);
}

bool Triangulation::isCounterClockwise(const Vertex& v_query, const Vertex& v1, const Vertex& v2) const {
    Vertex v21_norm = {-v2.y + v1.y, v2.x - v1.x};
    return dot(v_query - v1, v21_norm) >= 0;
}

int Triangulation::indexOf(const VertInd& v_ind, const Triangle& tri) const {
    //    if(tri.vertinds[0] == v_ind){
    //        return 0;
    //    } else if(tri.vertinds[1] == v_ind){
    //        return 1;
    //    }else if(tri.vertinds[2] == v_ind){
    //        return 2;
    //    }
    return -1;
}

//! \param v    vertex 
//! \param tri  triangle
//! \returns index in triangle tri corresponding to vertex \p v
int Triangulation::indexOf(const Vertex& v, const Triangle& tri) const {
    if (tri.verts[0] == v) {
        return 0;
    } else if (tri.verts[1] == v) {
        return 1;
    } else if (tri.verts[2] == v) {
        return 2;
    }
    return -1;
}

//! \brief vertex \p vp and \p v3 must lie opposite to each other 
//! \param vp 
//! \param v1
//! \param v2
//! \param v3
//! \returns true if quadrilateral formed by four vertices is Delaunay
bool Triangulation::needSwap(const Vertex& vp, const Vertex& v1, const Vertex& v2, const Vertex& v3) const {

    auto v13 = sf::Vector2f(v1 - v3);
    auto v23 = sf::Vector2f(v2 - v3);
    auto v1p = sf::Vector2f(v1 - vp);
    auto v2p = sf::Vector2f(v2 - vp);

    const auto cos_a = dot(v13, v23);
    const auto cos_b = dot(v1p, v2p);

    const auto angle_132 = 180. / M_PI * std::acos(cos_a / std::sqrt(dot(v13, v13) * dot(v23, v23)));
    const auto angle_1p2 = 180. / M_PI * std::acos(cos_b / std::sqrt(dot(v1p, v1p) * dot(v2p, v2p)));

    if (cos_a >= 0 and cos_b >= 0) {
        return false;
    }
    if (cos_a < 0 and cos_b < 0) {
        return true;
    }
    const auto sin_ab = static_cast<float>(v13.x * v23.y - v23.x * v13.y) * cos_b +
                        static_cast<float>(v2p.x * v1p.y - v1p.x * v2p.y) * cos_a;

    if (-sin_ab <
        0) { //! I HAVE NO CLUE WHY THERE HAS TO BE MINUS AND IT MAKES ME SAD!!! (in the article there is plus!)
        return true;
    }

    return false;
}

void Triangulation::dumpToFile(const std::string filename) const {

    std::ofstream file(filename);
    if (file.is_open()) {
        file << "Vertices:\n";
        for (const auto vertex : vertices_) {
            file << vertex.x << " " << vertex.y << "\n";
        }
        file << "Triangles:\n";
        for (const auto& tri : triangles_) {
            file << (int)tri.neighbours[0] << " " << (int)tri.neighbours[1] << " " << (int)tri.neighbours[2] << "\n";
        }
        file.close();
    } else {
        //        "say some warning message or something";
    }
}

