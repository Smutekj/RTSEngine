#include <omp.h>
#include "Edges.h"
#include "EdgesFinder1D.h"
#include "Grid.h"

Edges::Edges(Grid& grid)
    : p_grid(&grid) {
    int n = grid.n_cells_.x;
    int m = grid.n_cells_.y;

    edge_finders_[Orientation::RIGHTLEFT] =
        std::make_unique<EdgesFinder1D>(m, Orientation::RIGHTLEFT, *p_grid, vertices_, edges2_, edges_);
    edge_finders_[Orientation::LEFTDOWNRIGHTUP] =
        std::make_unique<EdgesFinder1D>(m + n, Orientation::LEFTDOWNRIGHTUP, *p_grid, vertices_, edges2_, edges_);
    edge_finders_[Orientation::RIGHTDOWNLEFTUP] =
        std::make_unique<EdgesFinder1D>(m + n, Orientation::RIGHTDOWNLEFTUP, *p_grid, vertices_, edges2_, edges_);
    edge_finders_[Orientation::DOWNUP] =
        std::make_unique<EdgesFinder1D>(n, Orientation::DOWNUP, *p_grid, vertices_, edges2_, edges_);
}

Edges::Edges(sf::Vector2i n_cells, sf::Vector2f cell_size) {
    p_grid = std::make_unique<Grid>(n_cells, cell_size);

    int n = p_grid->n_cells_.x;
    int m = p_grid->n_cells_.y;

    edge_finders_[Orientation::RIGHTLEFT] =
        std::make_unique<EdgesFinder1D>(m, Orientation::RIGHTLEFT, *p_grid, vertices_, edges2_, edges_);
    edge_finders_[Orientation::LEFTDOWNRIGHTUP] =
        std::make_unique<EdgesFinder1D>(m + n, Orientation::LEFTDOWNRIGHTUP, *p_grid, vertices_, edges2_, edges_);
    edge_finders_[Orientation::RIGHTDOWNLEFTUP] =
        std::make_unique<EdgesFinder1D>(m + n, Orientation::RIGHTDOWNLEFTUP, *p_grid, vertices_, edges2_, edges_);
    edge_finders_[Orientation::DOWNUP] =
        std::make_unique<EdgesFinder1D>(n, Orientation::DOWNUP, *p_grid, vertices_, edges2_, edges_);
}

void Edges::reset() {
    edges_.clear();
    edges2_.clear();
    vertices_.clear();

    for (int orient = 0; orient < 4; ++orient) {
        for (auto& a : edge_finders_[orient]->edge_array_) {
            a.clear();
        }
    }
}

void Edges::addVertex(Vertex new_vertex) {
    if (vertices_.size() == 0) {
        vertices_.push_back(new_vertex);
    } else {
        bool is_far_enough = true;
        for (auto vertex : vertices_) {
            if (dist2(vertex, new_vertex) < 20) {
                is_far_enough = false;
                break;
            }
        }
        if (is_far_enough) {
            vertices_.push_back(new_vertex);
        }
    }
}

void Edges::connectVertices(int vi, int vj) {
    Edgef e;
    e.from = asFloat(vertices_[vi]);
    const auto v2 = asFloat(vertices_[vj]);
    const auto edge_length = std::sqrt(dot(v2 - e.from, v2 - e.from));
    e.t = (v2 - e.from) / edge_length;
    e.l = edge_length;
    edges_.push_back(e);

    EdgeVInd e2(std::min(vi, vj), std::max(vi, vj));
    edges2_.push_back(e2);
}

// void Edges::createSquare(sf::Vector2i center_position, sf::Vector2i sides, bool invert){
//     const float dx = sides.x;
//     const float dy = sides.y;
//
//     auto n_vertices = vertices_.size();
//     auto vertex1 = center_position - sides/2;
//     sf::Vector2i vertex2 = {vertex1.x + sides.x, vertex1.y};
//     sf::Vector2i vertex3 = {vertex1.x + sides.x, vertex1.y+ sides.y};
//     sf::Vector2i vertex4 = {vertex1.x, vertex1.y + sides.y};
//     addVertex( vertex1 );
//     addVertex( vertex2 );
//     addVertex( vertex3 );
//     addVertex( vertex4 );
//
//     if(invert) {
//         connectVertices(n_vertices+1, n_vertices);
//         connectVertices(n_vertices+2, n_vertices+1);
//         connectVertices(n_vertices+3, n_vertices+2);
//         connectVertices(n_vertices, n_vertices+3);
//     } else {
//         connectVertices(n_vertices, n_vertices + 1);
//         connectVertices(n_vertices + 1, n_vertices + 2);
//         connectVertices(n_vertices + 2, n_vertices + 3);
//         connectVertices(n_vertices + 3, n_vertices);
//     }
// }

/*
void Edges::createTriangle(sf::Vector2f center_position, sf::Vector2f sides, bool invert){
    const float dx = sides.x;
    const float dy = sides.y;

    auto n_vertices = vertices_.size();
    addVertex( center_position + sf::Vector2f {-dx/2.f, -dy/2.f});
    addVertex( center_position + sf::Vector2f {-dx/2.f,  dy/2.f});
    addVertex( center_position + sf::Vector2f { dx/2.f,  dy/2.f});

    if(invert) {
        connectVertices(n_vertices+1, n_vertices);
        connectVertices(n_vertices+2, n_vertices+1);
        connectVertices(n_vertices, n_vertices+2);
    } else {
        connectVertices(n_vertices, n_vertices + 1);
        connectVertices(n_vertices + 1, n_vertices + 2);
        connectVertices(n_vertices + 2, n_vertices);
    }
}
*/

bool Edges::isTouchingEdge(const Edgef& edge, const sf::Vector2f r, const float radius) const {
    const auto dr = edge.from - r;
    sf::Vector2f n = {-edge.t.y, edge.t.x};
    const auto norm_dr_sq = dot(dr, dr);
    const auto dot_dr_t = dot(dr, edge.t);
    const auto discriminant = dot_dr_t * dot_dr_t - norm_dr_sq + radius * radius;
    if (discriminant >= 0) {
        const auto sqrt_disc = std::sqrt(discriminant);
        auto a1 = -dot_dr_t + sqrt_disc;
        auto a2 = -dot_dr_t - sqrt_disc;
        bool point1_is_on_edge = (a1 <= edge.l and a1 > 0);
        bool point2_is_on_edge = (a2 <= edge.l and a2 > 0);
        return (point2_is_on_edge) or (point1_is_on_edge);
    }
    return false;
}

//! \brief does same thing as the first version however here we don't calculate sqrt of discriminant
//! \param edge line segment
//! \param r
//! \param radius
//! \returns true if edge intersects or lies within the circle
bool Edges::isTouchingEdge2(const Edgef& edge, const sf::Vector2f r, const float radius) const {
    const auto dr = r - edge.from;
    const auto norm_dr_sq = dot(dr, dr);
    const auto dot_dr_t = -dot(dr, edge.t);
    const auto dot_dr_t_sq = dot_dr_t * dot_dr_t;
    const auto wtf = (edge.l + dot_dr_t);
    const auto discriminant = dot_dr_t_sq - norm_dr_sq + radius * radius;

    const bool a = discriminant < wtf * wtf;
    const bool b = discriminant > dot_dr_t_sq;
    const bool c = discriminant >= 0;
    if (dot_dr_t > -edge.l / 2.f) {
        return c and a and (!(dot_dr_t >= 0) or b);
    }
    return c and !b and (!(dot_dr_t < -edge.l) or !a);
    // bool result =
    //     discriminant >= 0 and ((dot_dr_t < -edge.l and discriminant > wtf * wtf and discriminant < dot_dr_t_sq) !=
    //                            (dot_dr_t >= 0 and discriminant > dot_dr_t_sq and discriminant < wtf * wtf) !=
    //                            (dot_dr_t > -edge.l / 2.f and discriminant < wtf * wtf) !=
    //                            (dot_dr_t < -edge.l / 2.f and discriminant < dot_dr_t_sq));
    // return result;
}

std::vector<Edgef> Edges::calcContactEdges(sf::Vector2f r, float radius) const {
    std::vector<Edgef> contact_edges;

    const auto ix0 = p_grid->cellCoords(r).x;
    const auto iy0 = p_grid->cellCoords(r).y;
    const auto idiag_rd0 = ix0 - iy0 + p_grid->n_cells_.y - 1;
    const auto idiag_lu0 = ix0 + iy0;
    const auto max_delta_i = static_cast<int>(std::ceil(radius / p_grid->cell_size_.x));

    std::array<int, 4> edge_grid_inds = {iy0, idiag_rd0, ix0, idiag_lu0};
    std::array<sf::Vector2f, 4> directions = {sf::Vector2f({1.f, 0.f}), sf::Vector2f({1.f, 1.f}),
                                              sf::Vector2f({0.f, 1.f}), sf::Vector2f({-1.f, 1.f})};

    // omp_set_num_threads(4);
    // #pragma omp parallel for
    for (int orient = 0; orient < 4; ++orient) {
        for (int delta_i = -max_delta_i; delta_i <= max_delta_i; ++delta_i) {
            const auto edge_grid_ind = delta_i + edge_grid_inds.at(orient);
            const auto& edges_of_orientation = edge_finders_.at(orient)->edge_array2_;
            if (edge_grid_ind >= 0 and edge_grid_ind < edges_of_orientation.size()) {

                const auto& edges = edges_of_orientation.at(edge_grid_ind);
                if (edges.size() == 0) {
                    continue;
                }
                auto first_edge =
                    std::lower_bound(edges.begin(), edges.end(), r, [&](const Edgef& e1, const sf::Vector2f& rx) {
                        return dot(rx - e1.from, directions[orient]) > 0;
                    });

                int first = first_edge - edges.begin() - 1;
                int last = first + 1;
                if (first == -1) {
                    first = 0;
                    last = 0;
                }
                if (first == edges.size() - 1) {
                    last = edges.size() - 1;
                }
                for (int i = first; i <= last; ++i) {
                    const auto& edge = edges[i];
                    if (isTouchingEdge(edge, r, radius)) {
                        // #pragma omp critical
                        contact_edges.push_back(edge);
                    }
                }

                // for (const auto& edge : edges) {
                //     const auto dr = r - edge.from;
                //     const auto t_dist = dot(dr, edge.t);
                //     const auto norm_dist = dot(dr, {-edge.t.y, edge.t.x});

                //     if (t_dist < -radius or t_dist > edge.l + radius or std::abs(norm_dist) > radius) {
                //         continue;
                //     } else if (t_dist > 0 and t_dist < edge.l and std::abs(norm_dist) < radius) {
                //         contact_edges.push_back(edge);
                //     } else
                //     if (isTouchingEdge(edge, r, radius)) {
                //         contact_edges.push_back(edge);
                //     }
                // }
            }
        }
    }

    return contact_edges;
}

//! \brief calculates edges that intersevct a circle with center \p r and radius \p radius
//! \param r is the center of the circle
//! \param radius of the circle 
//! \returns array of dimension 4 (1 for each direction) containing a vector of intersecting edges
std::array<std::vector<Edgef>, N_DIRECTIONS> Edges::calcContactEdgesO(sf::Vector2f r, float radius) const {
    std::array<std::vector<Edgef>, N_DIRECTIONS> orientation2contact_edges;

    const auto ix0 = p_grid->cellCoords(r).x;
    const auto iy0 = p_grid->cellCoords(r).y;
    const auto idiag_rd0 = ix0 - iy0 + p_grid->n_cells_.y - 1;
    const auto idiag_lu0 = ix0 + iy0;
    const auto max_delta_i = static_cast<int>(std::ceil(radius / p_grid->cell_size_.x));

    std::array<int, 4> edge_grid_inds = {iy0, idiag_rd0, ix0, idiag_lu0};
    std::array<sf::Vector2f, 4> directions = {sf::Vector2f({1.f, 0.f}), sf::Vector2f({1.f, 1.f}),
                                              sf::Vector2f({0.f, 1.f}), sf::Vector2f({1.f, -1.f})};

    const auto n_threads_before = omp_get_num_threads();
    // omp_set_num_threads(4);
    // #pragma omp parallel for
    for (int orient = 0; orient < 4; ++orient) {
        for (int delta_i = -max_delta_i; delta_i <= max_delta_i; ++delta_i) {
            const auto edge_grid_ind = delta_i + edge_grid_inds.at(orient);
            const auto& edges_of_orientation = edge_finders_.at(orient)->edge_array2_;
            if (edge_grid_ind >= 0 and edge_grid_ind < edges_of_orientation.size()) {

                const auto& edges = edges_of_orientation.at(edge_grid_ind);
                if (edges.size() == 0) {
                    continue;
                }
                auto first_edge =
                    std::lower_bound(edges.begin(), edges.end(), r, [&](const Edgef& e1, const sf::Vector2f& rx) {
                        return dot(rx - e1.from, directions[orient]) > 0;
                    });

                int first = first_edge - edges.begin() - 1;
                int last = first + 1;
                if (first == -1) {
                    first = 0;
                    last = 0;
                }
                if (first == edges.size() - 1) {
                    last = edges.size() - 1;
                }
                for (int i = first; i <= last; ++i) {
                    const auto& edge = edges[i];
                    if (isTouchingEdge(edge, r, radius)) {
                        orientation2contact_edges[orient].push_back(edge);
                    }
                }
            }
        }
    }
    // omp_set_num_threads(n_threads_before);
    return orientation2contact_edges;
}

void Edges::draw(sf::RenderWindow& window) {
    sf::RectangleShape line;
    line.setFillColor(sf::Color::Green);
    sf::CircleShape circle;
    for (const auto& edge : edges_) {
        line.setSize({1.f, edge.l});
        line.setPosition(edge.from);
        auto angle = 180.f / (M_PIf)*std::acos(dot(edge.t, {0, 1})) * (2.f * (edge.t.x < 0.f) - 1.f);
        line.setRotation(angle);
        //            window.draw(line);
    }
    for (const auto& v : vertices_) {
        circle.setRadius(5.f);
        circle.setPosition(asFloat(v));
        window.draw(circle);
    }
}