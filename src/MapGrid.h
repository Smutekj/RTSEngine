#ifndef BOIDS_MAPGRID_H
#define BOIDS_MAPGRID_H

#include "Graphics/RenderTexture.hpp"
#include "Graphics/VertexArray.hpp"
#include "Graphics/RectangleShape.hpp"
#include "Graphics/ConvexShape.hpp"

#include "Utils/Grid.h"
#include "core.h"
#include "BuildingManager.h"

#include "EdgesFinder1D.h"
#include "Edges.h"

#include <iostream>
#include <cassert>
#include <unordered_map>

typedef int BuildingInd;

namespace Mesh{
class Triangulation;
}


struct Graph;

struct Building {

    std::array<Edge, 8> edges;
    std::array<int, 8> edge_grid_inds;
    int n = 4;
    int m = 4;
    int corner_size = 2;
    u_int16_t graphics_id = 0;
    Graph* p_graph = nullptr;
    int instance_id = -1;

    

    explicit Building() = default;
    Building(const sf::Vector2i& center_cell_coords, const sf::Vector2f& cell_size) {
        intitializeEdges(center_cell_coords, cell_size);
    }
    ~Building() = default;

    Building& operator=(const Building& b) ;
    sf::Vector2f calcCenter() const;
    void intitializeEdges(sf::Vector2i center_cell_coords, sf::Vector2f cell_size);
    void intitializeEdges2(sf::Vector2i center_cell_coords, sf::Vector2f cell_size);
};


class MapGrid : public Grid {

  public:
    enum class TileType { GROUND, WATER, BUILDING, WALL };

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

    // std::vector<sf::ConvexShape> walls_drawable_;

    struct Tile {
        WallType type = WallType::NONE;

        bool isBuildable = true;
        bool isBoundary = false;
        bool isSquare() const { return (type != WallType::NONE) and (type & 1u) != 1u; }
    };
    std::vector<TileType> tiles;
    std::vector<Tile> tiles2;

    sf::RectangleShape rect;
    // sf::Texture grass;

    sf::ConvexShape wall_rect;
    sf::ConvexShape ur_triangle;
    sf::ConvexShape ul_triangle;
    sf::ConvexShape dl_triangle;
    sf::ConvexShape dr_triangle;

    sf::Vector2f box_size_;
    std::unordered_map<GridIndex, WallType> grid2walltype;

    sf::VertexArray walls_vertices_;
    std::vector<sf::Color> component2color_;

    BuildingManager* p_building_manager;

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
    void updateBoundaryTypesLocally2(sf::Vector2i n_first, sf::Vector2i n_max);
    void changeToWater(sf::Vector2i n_first, sf::Vector2i n_max);
    void extractEdgesFromTiles(Triangulation& cdt);
    void extractEdgesFromTilesV2(Triangulation& cdt);
    void extractVerticesForDrawing(Triangulation& cdt, std::vector<TriInd>& tri_ind2component);
    void extractVerticesForDrawing2(const Triangulation& cdt, const Edges& edge_lord);


    void buildWall(sf::Vector2f wall_center, sf::Vector2i square_size);
    bool buildBuilding(sf::Vector2f building_center, sf::Vector2i building_size, Triangulation& cdt);
    bool buildBuilding(sf::Vector2f building_center, int building_id, Triangulation& cdt);

    void addAllStaticEdgesToTriangulation(Triangulation& cdt);
    void addAllBuildingsToTriangulation(Triangulation& cdt);
    Building addBuildingEdgesToTriangulation(sf::Vector2f building_center, sf::Vector2i building_size, int corner_size,
                                             Triangulation& cdt);
    void removeBuilding(int building_index, Triangulation& cdt);

    sf::Vector2i drawProposedBuilding(sf::RenderWindow& window, sf::Vector2f building_center,
                                      sf::Vector2i building_size);
    void drawProposedWalls(sf::Vector2f center, sf::Vector2f max_coords,
                                    std::vector<sf::Vector2f>& function_values, const float width);

    void updateTexture();
    void createRandomBlob(sf::Vector2i start_coords){
        

        const int n_max_tiles = 30;

        int i_start = start_coords.x;
        int j_start = start_coords.y;
        int i = i_start;
        int j = j_start;
        int i_dir = 1;
        int j_dir = 1;
        int step = 2;
        int radius = 6;


        const int delta_i_max = 20;
        const int delta_j_max = 20;

        int n_inserted_tiles = 0;
        while(n_inserted_tiles != n_max_tiles){
            const auto cell_index = j* n_cells_.x + i;
            
            for(int delta_i = 0; delta_i < radius; delta_i++){
                for(int delta_j = 0; delta_j < radius; delta_j++){
                    tiles[cell_index + delta_i + delta_j*n_cells_.x] = TileType::WALL;
                }
            }

            n_inserted_tiles++;

            float alpha_i = static_cast<float>(rand()) / RAND_MAX;
            float alpha_j = static_cast<float>(rand()) / RAND_MAX;
            
            i+= (alpha_i < 0.5) * step * i_dir;
            j+= (2*(alpha_j < 0.1)-1) * step * (j_dir);
            if(std::abs(i - i_start) > delta_i_max){ i_dir *= -1;}
            if(std::abs(j - j_start) > delta_j_max){ j_dir *= -1;}

        }
        updateBoundaryTypes2();
        sawOffCorners();
        updateTexture();
        //! write to texture
        map_texture.display();
    }

    void draw(sf::RenderWindow &window);

private:
    sf::RenderTexture map_texture;
    sf::RectangleShape map_rect;

};



struct TileData{

    sf::Vector2i pos;
    sf::Vector2f orient;
    std::string name;
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

    // void draw(sf::RenderWindow& target) const {
        // apply the transform
        // states.transform *= getTransform();

        // apply the tileset texture
        // states.texture = &m_tileset;

        // draw the vertex array
        // target.draw(m_vertices);
    // }

  private:
    sf::VertexArray m_vertices;
    // sf::Texture m_tileset;
};


#endif // BOIDS_MAPGRID_H