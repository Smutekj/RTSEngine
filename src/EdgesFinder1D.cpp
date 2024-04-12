#include "EdgesFinder1D.h"
#include "Edges.h"
#include "Utils/Grid.h"

EdgesFinder1D::EdgesFinder1D(int max_n_edges, Orientation dir, Grid& grid, std::vector<Vertex>& vertices,
                             std::vector<EdgeVInd>& edge_inds, std::vector<Edgef>& edges)
    : dir_(dir)
    , grid(&grid)
    , vertices_(vertices)
    , edge_inds_(edge_inds)
    , edges_(edges) {
    edge_array_.resize(max_n_edges + 1);
    edge_array2_.resize(max_n_edges + 1);
    for (int i = 0; i < max_n_edges; ++i) {
        //            edge_array_.at(i) = std::set<int, decltype(isFurtherInDirection)>(isFurtherInDirection);
    }
}

void EdgesFinder1D::insert(int edge_index, int edge_1d_ind) {
    auto& edge_inds = edge_array_.at(edge_1d_ind);
    edge_inds.push_back(edge_index);
}

// void EdgesFinder1D::insert2(int edge_index, int edge_1d_ind) {
//     auto& edge_inds = edge_array_.at(edge_1d_ind);
//     // auto& edges = edge_array2_.at(edge_1d_ind);

//     switch (dir_) {
//     case RIGHTLEFT:
//         if (vertices_[edge_inds_[edge_index].from].x > vertices_[edge_inds_[edge_index].to].x) {
//             std::swap(edge_inds_[edge_index].from, edge_inds_[edge_index].to);
//         }
//         break;
//     case DOWNUP:
//         if (vertices_[edge_inds_[edge_index].from].y < vertices_[edge_inds_[edge_index].to].y) {
//             std::swap(edge_inds_[edge_index].from, edge_inds_[edge_index].to);
//         }
//         break;
//     default:
//         break;
//     }

//     edge_inds.push_back(edge_index);
//     // edges.push_back(edge);
// }

void EdgesFinder1D::insert2(int edge_index, int edge_1d_ind) {
    auto& edge_inds = edge_array_.at(edge_1d_ind);
    auto& edges = edge_array2_.at(edge_1d_ind);
    Edgef edge(vertices_[edge_inds_[edge_index].from], vertices_[edge_inds_[edge_index].to]);

    switch (dir_) {
    case RIGHTLEFT:
        if (edge.from.x > edge.to().x) {
            std::swap(edge_inds_[edge_index].from, edge_inds_[edge_index].to);
            // edge.from = edge.to();
            // edge.t *= -1.f;
        }
        break;
    case DOWNUP:
        if (edge.from.y > edge.to().y) {
            std::swap(edge_inds_[edge_index].from, edge_inds_[edge_index].to);
            // edge.from = edge.to();
            // edge.t *= -1.f;
        }
        break;
    default:
        break;
    }

    edge_inds.push_back(edge_index);
    edges.push_back(edge);
}

void EdgesFinder1D::insert3(const Edgef& edge, int edge_index, int edge_1d_ind) {
    auto& edge_inds = edge_array_.at(edge_1d_ind);
    auto& edges = edge_array2_.at(edge_1d_ind);

    switch (dir_) {
    case RIGHTLEFT:
        if (edge.from.x > edge.to().x) {
            std::swap(edge_inds_[edge_index].from, edge_inds_[edge_index].to);
        }
        break;
    case DOWNUP:
        if (edge.from.y > edge.to().y) {
            std::swap(edge_inds_[edge_index].from, edge_inds_[edge_index].to);
        }
        break;
    default:
        break;
    }

    edge_inds.push_back(edge_index);
    edges.push_back(edge);
}

bool segmentsIntersect(float& x1_l, float& x1_r, float& x2_l, float& x2_r) {
    if (x1_l > x1_r) {
        std::swap(x1_l, x1_r);
    } else if (x2_l > x2_r) {
        std::swap(x2_l, x2_r);
    }
    if (x1_l > x2_l) {
        return x1_l <= x2_r;
    } else if (x1_l <= x2_l) {
        return x1_r >= x2_l;
    }
    return false;
}

/*

 std::pair<int, int>  EdgesFinder1D::splitByEdge(Edge& new_edge, int cell_index, Triangulation& cdt){

    auto edge1d_ind = edge1dInd(cell_index);
    auto& edge_data = edge_array_[edge1d_ind];
    int n_inserted_vertices = 0;
    int n_inserted_edges = 0;

    if(!edge_data.empty()) {
        for (auto &ed: edge_data) {
            auto x_l = ed.position_from;
            auto x_r = ed.position_to;
            auto &old_edge = edges_.at(ed.edge_index);
            EdgeVInd old_edge_inds = edge_inds_.at(ed.edge_index);
            EdgeVInd new_edge_inds_1 = edge_inds_.at(ed.edge_index);
            EdgeVInd new_edge_inds_2 = edge_inds_.at(ed.edge_index);
            EdgeVInd new_edge_inds_3 = edge_inds_.at(ed.edge_index);

            if(dot(new_edge.t, old_edge.t)<0){ //! if they have opposite direction
                new_edge.from += new_edge.t * new_edge.l;
                new_edge.t *= -1.f;
            }
            auto position_from = dot(new_edge.from - edges_[edge_data[0].edge_index].from, t_vectors[dir_]);
            auto position_to = position_from + new_edge.l;
            EdgeData new_edge_data = {static_cast<int>(edges_.size()), position_from, position_to};
            auto xnew_l = position_from;
            auto xnew_r = position_to;


            if (xnew_l == x_l and xnew_r == x_r) {
                n_inserted_edges += 0;
                n_inserted_vertices += 0;
                continue;
            } else if (xnew_l == x_l and xnew_r != x_r) {
                vertices_.push_back(new_edge.to());
                if (xnew_r < x_r) {
                    new_edge_inds_1.to = vertices_.size() - 1;
                    new_edge_inds_2.from = vertices_.size() - 1;
                } else if (xnew_r > x_r) {
                    new_edge_inds_2.from = old_edge_inds.to;
                    new_edge_inds_2.to = vertices_.size() - 1;
                }
                edge_inds_.push_back(new_edge_inds_1);
                edge_inds_.push_back(new_edge_inds_2);
                n_inserted_edges += 2;
                n_inserted_vertices += 1;
                continue;
            } else if (xnew_r == x_r and xnew_l != x_r){
                vertices_.push_back(new_edge.from);
                if (xnew_l < x_l) {
                    new_edge_inds_1.from      = vertices_.size() - 1;
                    new_edge_inds_1.to    =  old_edge_inds.from;
                } else if (xnew_l > x_l) {
                    new_edge_inds_1.to  = vertices_.size()-1;
                    new_edge_inds_2.from = vertices_.size() - 1;
                }
                edge_inds_.push_back(new_edge_inds_1);
                edge_inds_.push_back(new_edge_inds_2);
                n_inserted_edges += 2;
                n_inserted_vertices += 1;
                continue;
            }

            vertices_.push_back(new_edge.from);
            vertices_.push_back(new_edge.to());


            if (xnew_l < x_l and xnew_r > x_l and xnew_r < x_r) {
                new_edge_inds_1.from = vertices_.size()-2;
                new_edge_inds_1.to = old_edge_inds.from;
                new_edge_inds_2.from = old_edge_inds.from;
                new_edge_inds_2.to = vertices_.size()-1;
                new_edge_inds_3.from = vertices_.size()-1;
                new_edge_inds_3.to = old_edge_inds.to;
            } else if(xnew_l < x_l and xnew_r > x_r ){
                new_edge_inds_1.from = vertices_.size()-2;
                new_edge_inds_1.to = old_edge_inds.from;
                new_edge_inds_2.from = old_edge_inds.from;
                new_edge_inds_2.to =    old_edge_inds.to;
                new_edge_inds_3.from = old_edge_inds.to;
                new_edge_inds_3.to =  vertices_.size()-1;
            } else if ( xnew_l > x_l and xnew_r < x_r){
                new_edge_inds_1.from = old_edge_inds.from;
                new_edge_inds_1.to = vertices_.size()-2;
                new_edge_inds_2.from = vertices_.size()-2;
                new_edge_inds_2.to =    vertices_.size()-1;
                new_edge_inds_3.from = vertices_.size()-1;
                new_edge_inds_3.to =old_edge_inds.to;
            } else if( xnew_l > x_l and xnew_r > x_r){
                new_edge_inds_1.from = old_edge_inds.from;
                new_edge_inds_1.to = vertices_.size()-2;
                new_edge_inds_2.from = vertices_.size()-2;
                new_edge_inds_2.to =    old_edge_inds.to;
                new_edge_inds_3.from =  old_edge_inds.to;
                new_edge_inds_3.to =    vertices_.size()-1;
            }

            edge_inds_.push_back(new_edge_inds_1);
            edge_inds_.push_back(new_edge_inds_2);
            edge_inds_.push_back(new_edge_inds_3);
            n_inserted_edges += 2;
            n_inserted_vertices += 2;
            auto old_edge_cdt =  CDT::Edge (old_edge_inds.from, old_edge_inds.to);
            cdt.fixedEdges.erase(old_edge_cdt);
            cdt.insertVertices(vertices_.end() -2, vertices_.end(),
                               [](const sf::Vector2f &v) { return v.x; },
                               [](const sf::Vector2f &v) { return v.y; });
            cdt.conformToEdges(edge_inds_.end()-3, edge_inds_.end(),
                            [](const EdgeVInd &p) { return p.from; },
                            [](const EdgeVInd &p) { return p.to; });}

    }

    return {n_inserted_vertices, n_inserted_edges};

}
*/

int EdgesFinder1D::edge1dInd(int cell_index) {
    const auto& cc = grid->cellCoords(cell_index);
    auto orient = dir_;
    return (orient == 0) * cc.y + (orient == 2) * cc.x + (orient == 1) * (cc.x - cc.y + (grid->n_cells_.y - 1)) +
           (orient == 3) * (cc.x + cc.y);
}

int EdgesFinder1D::edge1dPos(int cell_index) {

    const auto& cc = grid->cellCoords(cell_index);
    auto orient = dir_;
    return (orient == 0) * cc.x + (orient == 2) * cc.y + (orient == 1) * (cc.x + cc.y) +
           (orient == 3) * (cc.x - cc.y + +(grid->n_cells_.y - 1));
}
