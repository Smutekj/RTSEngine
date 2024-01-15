#include "core.h"
#include <set>

#ifndef BOIDS_EDGESFINDER1D_H
#define BOIDS_EDGESFINDER1D_H

class EdgeVInd;
class Grid;
class Triangulation;

struct EdgesFinder1D {

    Orientation dir_ = Orientation::RIGHTLEFT;
    Grid* grid;
    std::vector<Edgef>& edges_;
    std::vector<EdgeVInd>& edge_inds_;
    std::vector<Vertex>& vertices_;

  public:
    std::vector<std::vector<int>> edge_array_;
    std::vector<std::vector<Edgef>> edge_array2_;

  public:
    EdgesFinder1D(int max_n_edges, Orientation dir, Grid& grid, std::vector<Vertex>& vertices,
                  std::vector<EdgeVInd>& edge_inds, std::vector<Edgef>& edges);

    void insert(int edge_index, int cell_index);
    void insert2(int edge_index, int edge_index_2);
    void insert3(const Edgef& edge, int edge_index, int edge_1d_ind);
    /*
        bool edgesIntersect(float x1_l, float x1_r, float x2_l, float x2_r){
            return (x1_l<= x2_r and x1_r>=x2_l) or (x1_l>=x2_l and x2
        }
    */

  public:
    void reset() {
        for (auto& edges : edge_array_) {
            edges.clear();
        }
    }
    int edge1dInd(int cell_index);
    int edge1dPos(int cell_index);
    float positionOnEdge();
};

#endif // BOIDS_EDGESFINDER1D_H
