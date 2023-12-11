#ifndef BOIDS_MAPGRID_H
#define BOIDS_MAPGRID_H

#include <SFML/Graphics.hpp>
#include <iostream>
#include <cassert>
#include <unordered_map>
#include "Grid.h"
#include "core.h"
#include "EdgesFinder1D.h"
#include "Edges.h"

typedef int BuildingInd;

struct Building {

    std::array<Edge, 8> edges;
    std::array<int, 8> edge_grid_inds;
    int n = 3;
    int m = 3;
    int horizontal_line_length = 6;
    int vertical_line_length = 6;

    sf::ConvexShape contour;

    explicit Building() = default;
    Building(const sf::Vector2i& center_cell_coords, const sf::Vector2f& cell_size) {
        intitializeEdges(center_cell_coords, cell_size);
    }
    ~Building() = default;

    Building& operator=(const Building& b) {
        this->edges = b.edges;
        n = b.n;
        m = b.m;
        contour = b.contour;
        return *this;
    }

    sf::Vector2i calcCenter() const {
        sf::Vector2i center;
        for (int i = 0; i < 8; ++i) {
            center += edges[i].from;
        }
        return center / 8;
    }

    void intitializeEdges(sf::Vector2i center_cell_coords, sf::Vector2f cell_size) {
        const auto up_left_cell_x = center_cell_coords.x - n / 2;
        const auto up_left_cell_y = center_cell_coords.y - m / 2;

        const auto wtf_x = (n - horizontal_line_length) / 2;
        const auto wtf_y = (m - vertical_line_length) / 2;
        const float sqrt2 = M_SQRT2;
        const float diagonal_lenght = std::sqrt(wtf_x * wtf_x + wtf_y * wtf_y) * cell_size.x;

        edges[0].from = {(up_left_cell_x + wtf_x), up_left_cell_y};
        edges[0].t = {1, 0};
        edges[0].l = (n - 2 * wtf_x) * cell_size.x;

        edges[1].from = {(up_left_cell_x + n - wtf_x), up_left_cell_y};
        edges[1].t = {1 / sqrt2, 1 / sqrt2};
        edges[1].l = diagonal_lenght;

        edges[2].from = {(up_left_cell_x + n), up_left_cell_y + wtf_y};
        edges[2].t = {0, 1};
        edges[2].l = (m - 2 * wtf_y) * cell_size.y;

        edges[3].from = {up_left_cell_x + n, up_left_cell_y + wtf_y + vertical_line_length};
        edges[3].t = {-1 / sqrt2, 1 / sqrt2};
        edges[3].l = diagonal_lenght;

        edges[4].from = {up_left_cell_x + wtf_x + horizontal_line_length, up_left_cell_y + m};
        edges[4].t = {-1, 0};
        edges[4].l = (n - 2 * wtf_x) * cell_size.x;

        edges[5].from = {up_left_cell_x + wtf_x, up_left_cell_y + m};
        edges[5].t = {-1 / sqrt2, -1 / sqrt2};
        edges[5].l = diagonal_lenght;

        edges[6].from = {up_left_cell_x, up_left_cell_y + wtf_y + vertical_line_length};
        edges[6].t = {0, -1};
        edges[6].l = (m - 2 * wtf_y) * cell_size.x;

        edges[7].from = {up_left_cell_x, up_left_cell_y + wtf_y};
        edges[7].t = {1 / sqrt2, -1 / sqrt2};
        edges[7].l = diagonal_lenght;

        contour.setPointCount(8);
        contour.setFillColor(sf::Color::Red);
        for (int i = 0; i < 8; ++i) {
            edges[i].from *= static_cast<int>(cell_size.x);
            contour.setPoint(i, static_cast<sf::Vector2f>(edges[i].from));
        }
    }
};

class Buildings {
  public:
    bool load(const std::string& tileset, const std::vector<Building>& buildings) {

        const auto n_buildings = buildings.size();
        // resize the vertex array to fit the level size
        m_vertices.setPrimitiveType(sf::Triangles);
        m_vertices.resize(n_buildings * 6 * 3);

        // populate the vertex array, with two triangles per tile
        for (unsigned int i = 0; i < n_buildings; ++i) {
            sf::Vertex* triangles = &m_vertices[(i)*6 * 3]; // 6 triangles per building 3 vertices per triangle
            const auto& edges = buildings[i].edges;
            for (int k = 0; k < 18; ++k) {
                triangles[k].color = sf::Color::Red;
            }
            // get a pointer to the triangles' vertices of the current tile
            // define the 6 corners of the two triangles
            triangles[0].position = sf::Vector2f(edges[0].from);
            triangles[1].position = sf::Vector2f(edges[1].from);
            triangles[2].position = sf::Vector2f(edges[5].from);

            triangles[3].position = sf::Vector2f(edges[1].from);
            triangles[4].position = sf::Vector2f(edges[5].from);
            triangles[5].position = sf::Vector2f(edges[4].from);

            triangles[6].position = sf::Vector2f(edges[1].from);
            triangles[7].position = sf::Vector2f(edges[2].from);
            triangles[8].position = sf::Vector2f(edges[4].from);

            triangles[9].position = sf::Vector2f(edges[2].from);
            triangles[10].position = sf::Vector2f(edges[3].from);
            triangles[11].position = sf::Vector2f(edges[4].from);

            triangles[12].position = sf::Vector2f(edges[0].from);
            triangles[13].position = sf::Vector2f(edges[7].from);
            triangles[14].position = sf::Vector2f(edges[6].from);

            triangles[15].position = sf::Vector2f(edges[6].from);
            triangles[16].position = sf::Vector2f(edges[5].from);
            triangles[17].position = sf::Vector2f(edges[0].from);
        }

        return true;
    }

    virtual void draw(sf::RenderWindow& target) const {
        // apply the transform
        // states.transform *= getTransform();

        // apply the tileset texture
        // states.texture = &m_tileset;

        // draw the vertex array
        target.draw(m_vertices);
    }

  private:
    sf::VertexArray m_vertices;
    sf::Texture m_tileset;
};

namespace Mesh{
class Triangulation;
}

class MapGrid : public Grid {

  public:
    enum class TileType { GROUND, BUILDING, WALL };

    struct VertexHash {
        std::size_t operator()(const Vertex& v) const { return std::hash<int>()(v.x) ^ std::hash<int>()(v.y); }
    };

    enum WallType : u_int_16_t {
        SQUAREUP = 0,
        URTRIANGLE = 1,
        SQUARERIGHT = 2,
        DRTRIANGLE = 3,
        SQUAREDOWN = 4,
        DLTRIANGLE = 5,
        SQUARELEFT = 6,
        ULTRIANGLE = 7,
        SQUAREINSIDE,
        NONE,
    };
    const std::vector<int> delta_inds = {1,  1 + n_cells_.x,  n_cells_.x,  -1 + n_cells_.x,
                                         -1, -1 - n_cells_.x, -n_cells_.x, 1 - n_cells_.x};

    typedef int GridIndex;

    std::vector<sf::ConvexShape> walls_drawable_;

    struct Tile {
        WallType type = WallType::NONE;

        bool isBuildable = true;
        bool isBoundary = false;
        bool isSquare() const { return (type != WallType::NONE) and (type & 1u) != 1u; }
    };
    std::vector<TileType> tiles;
    std::vector<Tile> tiles2;

    sf::RectangleShape rect;
    sf::Texture grass;

    sf::ConvexShape wall_rect;
    sf::ConvexShape ur_triangle;
    sf::ConvexShape ul_triangle;
    sf::ConvexShape dl_triangle;
    sf::ConvexShape dr_triangle;

    sf::Vector2f box_size_;
    std::unordered_map<GridIndex, WallType> grid2walltype;

    sf::VertexArray walls_vertices_;
    std::vector<sf::Color> component2color_;

    std::vector<Building> buildings_;
    std::vector<int> cell2building_ind_;
    std::vector<TriInd> component2building_ind_;
    std::shared_ptr<Edges> p_edges_;

    MapGrid(sf::Vector2i n_cells, sf::Vector2f box_size, sf::Vector2f cell_size);

    TileType typeAt(const sf::Vector2f& r_coords) const;
    bool isWallIn(Direction dir, int cell_index);
    Direction countclockDirection(Direction dir);
    Direction clockDirection(Direction dir);
    Direction opposite(Direction dir);

    void setSprites();

    std::vector<sf::Vector2f> generateRandomWalls(sf::Vector2f center, sf::Vector2f max_coords, const float width);

    std::vector<sf::Vector2f> setWallsInFunction(sf::Vector2f center, sf::Vector2f max_coords,
                                                 std::vector<sf::Vector2f>& function_values, const float width);

    std::shared_ptr<Edges>& getEdges() { return p_edges_; }

    bool isBoundaryWallTile(int cell_index);
    bool isBoundaryWallTile(int cell_index, Direction dir);
    bool isBoundaryWallTileV2(int cell_index, Direction dir);
    WallType findWallType(int cell_index);

    bool isInnerBoundaryTile(int cell_index);

    void removeSharpCorners();
    void sawOffCorners();
    void updateBoundaryTypes();
    void updateBoundaryTypes2();
    void updateBoundaryTypesLocally(sf::Vector2i n_first, sf::Vector2i n_max);
    void extractEdgesFromTiles(Triangulation& cdt);
    void extractEdgesFromTilesV2(Triangulation& cdt);
    void extractVerticesForDrawing(Triangulation& cdt, std::vector<TriInd>& tri_ind2component);

    void buildWall(sf::Vector2f wall_center, sf::Vector2i square_size);
    void buildBuilding(sf::Vector2f building_center, sf::Vector2i building_size, Triangulation& cdt);

    void addAllStaticEdgesToTriangulation(Triangulation& cdt);
    void addAllBuildingsToTriangulation(Triangulation& cdt);
    Building addBuildingEdgesToTriangulation(sf::Vector2f building_center, sf::Vector2i building_size,
                                             Triangulation& cdt);
    void removeBuilding(int building_index, Triangulation& cdt);

    sf::Vector2i drawProposedBuilding(sf::RenderWindow& window, sf::Vector2f building_center,
                                      sf::Vector2i building_size);
    void drawProposedWalls(sf::Vector2f center, sf::Vector2f max_coords,
                                    std::vector<sf::Vector2f>& function_values, const float width);
    void draw(sf::RenderWindow& window);
};

#endif // BOIDS_MAPGRID_H
