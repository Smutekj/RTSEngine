#ifndef BOIDS_EDGES_H
#define BOIDS_EDGES_H

#include "core.h"
#include <memory>
//
// struct EdgeVInd{
//    int from;
//    int to;
//};

constexpr static int N_DIRECTIONS = 4;

class EdgesFinder1D;
class Grid;


//! \class stores edges representing hard walls and can tell me 
class Edges {
  public:
    std::vector<Edgef> edges_;
    std::vector<EdgeVInd> edges2_;
    std::vector<Vertex> vertices_;

    std::unique_ptr<Grid> p_grid;

    typedef int GridIndex;

    std::array<std::unique_ptr<EdgesFinder1D>, 4> edge_finders_;

    explicit Edges(Grid& grid);
    Edges(sf::Vector2i n_cells, sf::Vector2f cell_size);
    explicit Edges() = default;

    ~Edges() { int a = 0; }

    void reset();

    void addVertex(Vertex vertex);
    void connectVertices(int vi, int vj);

    bool isTouchingEdge(const Edgef& edge, const sf::Vector2f r, const float radius) const;
    bool isTouchingEdge2(const Edgef& edge, const sf::Vector2f r, const float radius) const;

    std::vector<Edgef> calcContactEdges(sf::Vector2f r, float radius) const;
    std::array<std::vector<Edgef>, N_DIRECTIONS> calcContactEdgesO(sf::Vector2f r, float radius) const;

    void draw(sf::RenderWindow& window);

    void insertEdge(EdgeVInd e, Orientation orient) {}
};

#endif // BOIDS_EDGES_H
