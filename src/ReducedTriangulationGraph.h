#pragma once

#include "core.h"
#include "Triangulation.h"
#include <queue>

struct FunnelData {
    struct Portal {
        sf::Vector2f left; //! direction of point
        sf::Vector2f right;
    };
    std::vector<std::pair<sf::Vector2f, sf::Vector2f>> funnel;
};

//! \struct data  held at a vertex in a reduced triangulation graph
struct ReducedVertex {
    std::array<float, 3> widths = {MAXFLOAT, MAXFLOAT, MAXFLOAT};
    std::array<float, 3> lengths = {0, 0, 0};
    std::array<int, 3> neighbours = {-1, -1, -1};
};

//! \class reduced version of full triangulation graph where we store only edges of crossroads triangles as vertices.
//! \class distances between vertices are calculated using paths from funnel algorithm connecting midpoints of the
//! \class crossroads edges through triangle corridors. \class triangles with only one uncostrained edge (dead ends) are also
//! \class  considered vertices;
struct ReducedTriangulationGraph {

    std::vector<int> tri_ind2vertex;
    std::vector<TriInd> vertex2tri_ind;
    std::vector<ReducedVertex> reduced_vertices;
    std::vector<std::array<int, 3>> vertex2edge_inds2;

    struct TriPathData {
        TriInd current;
        int to;
    };

    struct Corridor {
        TriPathData start;
        TriPathData end;

        float length = 0;
        float width = MAXFLOAT;

        std::vector<TriInd> tri_inds;
        std::vector<int> from_start;
        std::vector<int> from_end;
        std::vector<std::pair<sf::Vector2f, sf::Vector2f>> corridor_points;

        Corridor() = default;
    };
    std::vector<Corridor> edges;

  private:
    std::vector<std::array<bool, 3>> visited;

    float sign2(const sf::Vector2f& p1, const sf::Vector2f& p2, const sf::Vector2f& p3) const;
    float calcCorridorWidth(const std::vector<std::pair<sf::Vector2f, sf::Vector2f>>& funnel) const;

  public:
    float funnelDistance(const sf::Vector2f r_start, const sf::Vector2f r_end, FunnelData& fd,
                         const Triangulation& cdt) const;
    void constructFromTriangulation(Triangulation& cdt, std::vector<TriInd>& tri_ind2component);
    void constructFromTriangulationCenters(Triangulation& cdt, std::vector<TriInd>& tri_ind2component);
};
