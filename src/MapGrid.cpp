#include "MapGrid.h"
#include "Triangulation.h"

MapGrid::MapGrid(sf::Vector2i n_cells, sf::Vector2f box_size, sf::Vector2f cell_size)
    : Grid(n_cells, cell_size), box_size_(box_size)
{
    tiles.resize(n_cells.x * n_cells.y, TileType::GROUND);
    tiles2.resize(n_cells.x * n_cells.y);

    // if (!grass.loadFromFile("C:\\Users\\smute\\Boids-current\\Resources\\Cartoon_green_texture_grass.png")) {
    //     throw std::runtime_error("texture not found");
    // }

    if (!map_texture.create(8*Geometry::N_CELLS[0], 8*Geometry::N_CELLS[1]))
    {
        throw std::runtime_error("texture too big");
    }
    map_texture.clear(sf::Color::Transparent);
    sf::Sprite sprite;
    auto view = map_texture.getView();
    // view.setViewport({0.1f, 0.0f, 0.9f, 1.0f});
    view.setCenter(box_size / 2.f);
    view.setSize(box_size.x, box_size.y);
    map_texture.setView(view);

    map_rect.setSize({Geometry::BOX[0], Geometry::BOX[1]});
    map_rect.setPosition(0, 0);
    map_rect.setFillColor(sf::Color::White);
    map_rect.setTexture(&map_texture.getTexture());

    const auto dx = cell_size_.x;
    const auto dy = cell_size_.y;
    rect.setSize(cell_size_);
    wall_rect.setPointCount(4);
    wall_rect.setPoint(0, {0, 0});
    wall_rect.setPoint(1, {dx, 0});
    wall_rect.setPoint(2, {dx, dy});
    wall_rect.setPoint(3, {0, dy});
    wall_rect.setFillColor(sf::Color::Blue);
    // wall_rect.setTexture(&grass);

    dl_triangle.setPointCount(3);
    dl_triangle.setPoint(0, {0, 0});
    dl_triangle.setPoint(1, {dx, 0});
    dl_triangle.setPoint(2, {dx, dy});
    dl_triangle.setFillColor(sf::Color::Red);
    // dl_triangle.setTexture(&grass);

    dr_triangle.setPointCount(3);
    dr_triangle.setPoint(0, {0, 0});
    dr_triangle.setPoint(1, {dx, 0});
    dr_triangle.setPoint(2, {0, dy});
    dr_triangle.setFillColor(sf::Color::Red);
    dr_triangle.setTexture(&grass);

    ul_triangle.setPointCount(3);
    ul_triangle.setPoint(0, {dx, 0});
    ul_triangle.setPoint(1, {dx, dy});
    ul_triangle.setPoint(2, {0, dy});
    ul_triangle.setFillColor(sf::Color::Red);
    ul_triangle.setTexture(&grass);

    ur_triangle.setPointCount(3);
    ur_triangle.setPoint(0, {0, 0});
    ur_triangle.setPoint(1, {dx, dy});
    ur_triangle.setPoint(2, {0, dy});
    ur_triangle.setFillColor(sf::Color::Red);
    ur_triangle.setTexture(&grass);

    const int n = n_cells.x;
    const int m = n_cells.y;
    cell2building_ind_.resize(n * m, -1);

    p_edges_ = std::make_shared<Edges>(n_cells, cell_size);
}

MapGrid::TileType MapGrid::typeAt(const sf::Vector2f &r_coords) const
{
    const auto cell_coord = coordToCell(r_coords);
    return tiles.at(cell_coord);
}

bool MapGrid::isWallIn(Direction dir, int cell_index)
{
    return (tiles[cell_index] == TileType::BUILDING and tiles[cell_index + delta_inds[dir]] == TileType::GROUND) or
           (tiles[cell_index] == TileType::GROUND and tiles[cell_index + delta_inds[dir]] == TileType::BUILDING);
}

Direction MapGrid::countclockDirection(Direction dir) { return Direction{((dir - 1) % 8 + (dir == 0) * 8) % 8}; }
Direction MapGrid::clockDirection(Direction dir) { return Direction{(dir + 1) % 8}; }
Direction MapGrid::opposite(Direction dir) { return Direction{(dir + 4) % 8}; }

bool MapGrid::isBoundaryWallTile(int cell_index)
{
    bool is_not_ground = tiles.at(cell_index) != TileType::GROUND;
    bool up_is_ground = tiles.at(cell_index - n_cells_.x) == TileType::GROUND;
    bool down_is_ground = tiles.at(cell_index + n_cells_.x) == TileType::GROUND;
    bool left_is_ground = tiles.at(cell_index + 1) == TileType::GROUND;
    bool right_is_ground = tiles.at(cell_index - 1) == TileType::GROUND;
    if (grid2walltype.count(cell_index) > 0)
    {
        return true;
    }
    else
    {
        return false;
    }
    //    if(is_traingle) {return true;} //! Triangles can lie only at boundary

    return is_not_ground and (up_is_ground or down_is_ground or left_is_ground or right_is_ground);
}

bool MapGrid::isBoundaryWallTile(int cell_index, Direction dir)
{
    bool is_not_ground = tiles.at(cell_index) != TileType::GROUND;
    bool next_is_ground = tiles.at(cell_index + delta_inds[dir]) == TileType::GROUND;
    bool next_is_triangle = false;
    auto a = cellCoords(cell_index);
    if (grid2walltype.count(cell_index + delta_inds[dir]) > 0)
    {
        next_is_triangle = grid2walltype[cell_index + delta_inds[dir]] % 2 == 1;
    }
    bool is_triangle = false;
    bool is_cartesian = dir % 2 == 0;
    if (grid2walltype.count(cell_index) > 0)
    {
        is_triangle = grid2walltype[cell_index] % 2 == 1;
    }
    if (is_triangle)
    {
        return true; //! is_cartesian;
    }
    return (is_not_ground and (next_is_ground));
}

bool MapGrid::isBoundaryWallTileV2(int cell_index, Direction dir)
{
    bool is_not_ground = tiles.at(cell_index) != TileType::GROUND;
    bool next_is_ground = tiles.at(cell_index + delta_inds[dir]) == TileType::GROUND;
    bool is_triangle = false;
    bool is_cartesian = dir % 2 == 0;
    if (grid2walltype.count(cell_index) > 0)
    {
        is_triangle = grid2walltype[cell_index] % 2 == 1;
    }
    if (is_triangle)
    {
        return !is_cartesian;
    }
    return (is_not_ground and (next_is_ground));
}

MapGrid::WallType MapGrid::findWallType(int cell_index)
{
    if (!isBoundaryWallTile(cell_index))
    {
        throw std::runtime_error("tile at cell_index: " + std::to_string(cell_index) + " is not at a boundary!");
    }

    return grid2walltype[cell_index];
}

bool MapGrid::isInnerBoundaryTile(int cell_index)
{
    bool is_not_ground = tiles.at(cell_index) == TileType::BUILDING;
    bool up_is_ground = tiles.at(cell_index - n_cells_.x) == TileType::GROUND;
    bool down_is_ground = tiles.at(cell_index + n_cells_.x) == TileType::GROUND;
    bool left_is_ground = tiles.at(cell_index + 1) == TileType::GROUND;
    bool right_is_ground = tiles.at(cell_index - 1) == TileType::GROUND;

    return is_not_ground and down_is_ground;
}

void MapGrid::removeSharpCorners()
{

    for (int j = 1; j < n_cells_.y - 2; ++j)
    {
        for (int i = 1; i < n_cells_.x - 2; ++i)
        {
            const auto cell_index = j * n_cells_.x + i;

            auto t11 = tiles[cell_index + delta_inds[Direction::LEFTUP]];
            auto t12 = tiles[cell_index + delta_inds[Direction::UP]];
            auto t13 = tiles[cell_index + delta_inds[Direction::RIGHTUP]];

            auto t21 = tiles[cell_index + delta_inds[Direction::LEFT]];
            auto t22 = tiles[cell_index];
            auto t23 = tiles[cell_index + delta_inds[Direction::RIGHT]];

            auto t31 = tiles[cell_index + delta_inds[Direction::LEFTDOWN]];
            auto t32 = tiles[cell_index + delta_inds[Direction::DOWN]];
            auto t33 = tiles[cell_index + delta_inds[Direction::RIGHTDOWN]];

            if (t22 == TileType::WALL)
            {
                bool ur_is_free = (t23 == TileType::GROUND and t13 == TileType::GROUND and t12 == TileType::GROUND);
                bool ul_is_free = (t21 == TileType::GROUND and t11 == TileType::GROUND and t12 == TileType::GROUND);
                bool dr_is_free = (t23 == TileType::GROUND and t33 == TileType::GROUND and t32 == TileType::GROUND);
                bool dl_is_free = (t21 == TileType::GROUND and t31 == TileType::GROUND and t32 == TileType::GROUND);
                if (ur_is_free + ul_is_free + dr_is_free + dl_is_free == 1)
                {
                    if (ur_is_free)
                    {
                        grid2walltype[cell_index] = WallType::URTRIANGLE;
                    }
                    if (ul_is_free)
                    {
                        grid2walltype[cell_index] = WallType::ULTRIANGLE;
                    }
                    if (dr_is_free)
                    {
                        grid2walltype[cell_index] = WallType::DRTRIANGLE;
                    }
                    if (dl_is_free)
                    {
                        grid2walltype[cell_index] = WallType::DLTRIANGLE;
                    }
                }
            }
        }
    }
}

void MapGrid::updateBoundaryTypesLocally(sf::Vector2i n_first, sf::Vector2i n_max)
{

    for (int dj = 0; dj < n_max.y - 1; ++dj)
    {
        for (int di = 0; di < n_max.x - 1; ++di)
        {
            const auto i = n_first.x + di;
            const auto j = n_first.y + dj;
            if (i >= n_cells_.x - 1 or j >= n_cells_.y - 1)
            {
                continue;
            };
            const auto cell_index = j * n_cells_.x + i;

            const auto t_left = tiles[cell_index + delta_inds[Direction::LEFT]];
            const auto t_right = tiles[cell_index + delta_inds[Direction::RIGHT]];
            const auto t_up = tiles[cell_index + delta_inds[Direction::UP]];
            const auto t_down = tiles[cell_index + delta_inds[Direction::DOWN]];
            auto &t = tiles[cell_index];

            if (t == TileType::WALL)
            {
                int n_walls_around = (t_left == TileType::WALL) + (t_right == TileType::WALL) +
                                     (t_down == TileType::WALL) + (t_up == TileType::WALL);

                bool has_wall_left_right = (t_left == TileType::WALL and t_right == TileType::WALL) and
                                           (t_down == TileType::GROUND and t_up == TileType::GROUND);

                bool has_wall_above_below = (t_left == TileType::GROUND and t_right == TileType::GROUND) and
                                            (t_down == TileType::WALL and t_up == TileType::WALL);

                if (n_walls_around <= 1 or has_wall_left_right or has_wall_above_below)
                {
                    t = TileType::GROUND;
                }
            }
        }
    }

    //! fill in holes
    for (int dj = 0; dj < n_max.y - 1; ++dj)
    {
        for (int di = 0; di < n_max.x - 1; ++di)
        {
            const auto i = n_first.x + di;
            const auto j = n_first.y + dj;
            if (i >= n_cells_.x - 1 or j >= n_cells_.y - 1)
            {
                continue;
            };
            const auto cell_index = j * n_cells_.x + i;

            const auto t_left = tiles[cell_index + delta_inds[Direction::LEFT]];
            const auto t_right = tiles[cell_index + delta_inds[Direction::RIGHT]];
            const auto t_up = tiles[cell_index + delta_inds[Direction::UP]];
            const auto t_down = tiles[cell_index + delta_inds[Direction::DOWN]];
            auto &t = tiles[cell_index];

            if (t == TileType::GROUND)
            {
                int n_walls_around = (t_left == TileType::WALL) + (t_right == TileType::WALL) +
                                     (t_down == TileType::WALL) + (t_up == TileType::WALL);

                bool has_wall_left_right = (t_left == TileType::WALL and t_right == TileType::WALL) and
                                           (t_down == TileType::GROUND and t_up == TileType::GROUND);

                bool has_wall_above_below = (t_left == TileType::GROUND and t_right == TileType::GROUND) and
                                            (t_down == TileType::WALL and t_up == TileType::WALL);

                if (n_walls_around >= 3)
                {
                    t = TileType::WALL;
                }
            }
        }
    }

    for (int dj = 0; dj < n_max.y - 1; ++dj)
    {
        for (int di = 0; di < n_max.x - 1; ++di)
        {
            const auto i = n_first.x + di;
            const auto j = n_first.y + dj;
            if (i >= n_cells_.x - 1 or j >= n_cells_.y - 1)
            {
                continue;
            };
            const auto cell_index = j * n_cells_.x + i;
            if (tiles[cell_index] == TileType::WALL)
            {
                wall_rect.setPosition(i * cell_size_.x, j * cell_size_.y);
                walls_drawable_.push_back(wall_rect);
            }
        }
    }
}

void MapGrid::updateBoundaryTypesLocally2(sf::Vector2i n_first, sf::Vector2i n_max)
{

    //! remove thin squares
    for (int dj = 0; dj < n_max.y - 1; ++dj)
    {
        for (int di = 0; di < n_max.x - 1; ++di)
        {
            const auto i = n_first.x + di;
            const auto j = n_first.y + dj;
            if (i >= n_cells_.x - 1 or j >= n_cells_.y - 1)
            {
                continue;
            };
            const auto cell_index = j * n_cells_.x + i;

            const auto t_left = tiles[cell_index + delta_inds[Direction::LEFT]];
            const auto t_right = tiles[cell_index + delta_inds[Direction::RIGHT]];
            const auto t_up = tiles[cell_index + delta_inds[Direction::UP]];
            const auto t_down = tiles[cell_index + delta_inds[Direction::DOWN]];
            auto &t = tiles[cell_index];

            if (t == TileType::WALL)
            {
                int n_walls_around = (t_left == TileType::WALL) + (t_right == TileType::WALL) +
                                     (t_down == TileType::WALL) + (t_up == TileType::WALL);

                bool has_wall_left_right = (t_left == TileType::WALL and t_right == TileType::WALL) and
                                           (t_down == TileType::GROUND and t_up == TileType::GROUND);

                bool has_wall_above_below = (t_left == TileType::GROUND and t_right == TileType::GROUND) and
                                            (t_down == TileType::WALL and t_up == TileType::WALL);

                if (n_walls_around <= 1 or has_wall_left_right or has_wall_above_below)
                {
                    t = TileType::GROUND;
                }
            }
        }
    }

    //! fill in holes
    for (int dj = 0; dj < n_max.y - 1; ++dj)
    {
        for (int di = 0; di < n_max.x - 1; ++di)
        {
            const auto i = n_first.x + di;
            const auto j = n_first.y + dj;
            if (i >= n_cells_.x - 1 or j >= n_cells_.y - 1)
            {
                continue;
            };
            const auto cell_index = j * n_cells_.x + i;

            const auto t_left = tiles[cell_index + delta_inds[Direction::LEFT]];
            const auto t_right = tiles[cell_index + delta_inds[Direction::RIGHT]];
            const auto t_up = tiles[cell_index + delta_inds[Direction::UP]];
            const auto t_down = tiles[cell_index + delta_inds[Direction::DOWN]];
            auto &t = tiles[cell_index];

            if (t == TileType::GROUND)
            {
                int n_walls_around = (t_left == TileType::WALL) + (t_right == TileType::WALL) +
                                     (t_down == TileType::WALL) + (t_up == TileType::WALL);

                bool has_wall_left_right = (t_left == TileType::WALL and t_right == TileType::WALL) and
                                           (t_down == TileType::GROUND and t_up == TileType::GROUND);

                bool has_wall_above_below = (t_left == TileType::GROUND and t_right == TileType::GROUND) and
                                            (t_down == TileType::WALL and t_up == TileType::WALL);

                if (n_walls_around >= 3)
                {
                    t = TileType::WALL;
                }
            }
        }
    }

    // map_texture.clear(sf::Color::Transparent);

    for (int dj = 0; dj < n_max.y - 1; ++dj)
    {
        for (int di = 0; di < n_max.x - 1; ++di)
        {
            const auto i = n_first.x + di;
            const auto j = n_first.y + dj;
            if (i >= n_cells_.x - 1 or j >= n_cells_.y - 1)
            {
                continue;
            };
            const auto cell_index = j * n_cells_.x + i;
            if (tiles[cell_index] == TileType::WALL)
            {
                wall_rect.setPosition(i * cell_size_.x, j * cell_size_.y);
                wall_rect.setFillColor(sf::Color::Red);
                wall_rect.setScale({5, 5});
                walls_drawable_.push_back(wall_rect);
                // map_texture.draw(wall_rect); // or any other drawable
            }
        }
    }

    //! draw walls to map texture
}

void MapGrid::updateBoundaryTypes2()
{

    for (int j = 1; j < n_cells_.y - 1; ++j)
    {
        for (int i = 1; i < n_cells_.x - 1; ++i)
        {
            const auto cell_index = j * n_cells_.x + i;

            const auto t_left = tiles[cell_index + delta_inds[Direction::LEFT]];
            const auto t_right = tiles[cell_index + delta_inds[Direction::RIGHT]];
            const auto t_up = tiles[cell_index + delta_inds[Direction::UP]];
            const auto t_down = tiles[cell_index + delta_inds[Direction::DOWN]];
            auto &t = tiles[cell_index];

            if (t == TileType::WALL)
            {
                int n_walls_around = (t_left == TileType::WALL) + (t_right == TileType::WALL) +
                                     (t_down == TileType::WALL) + (t_up == TileType::WALL);

                bool has_wall_left_right = (t_left == TileType::WALL and t_right == TileType::WALL) and
                                           (t_down == TileType::GROUND and t_up == TileType::GROUND);

                bool has_wall_above_below = (t_left == TileType::GROUND and t_right == TileType::GROUND) and
                                            (t_down == TileType::WALL and t_up == TileType::WALL);

                if (n_walls_around <= 1 or has_wall_left_right or has_wall_above_below)
                {
                    t = TileType::GROUND;
                }
            }
        }
    }

    //! fill in holes
    for (int j = 1; j < n_cells_.y - 1; ++j)
    {
        for (int i = 1; i < n_cells_.x - 1; ++i)
        {
            const auto cell_index = j * n_cells_.x + i;

            const auto t_left = tiles[cell_index + delta_inds[Direction::LEFT]];
            const auto t_right = tiles[cell_index + delta_inds[Direction::RIGHT]];
            const auto t_up = tiles[cell_index + delta_inds[Direction::UP]];
            const auto t_down = tiles[cell_index + delta_inds[Direction::DOWN]];
            auto &t = tiles[cell_index];

            if (t == TileType::GROUND)
            {
                int n_walls_around = (t_left == TileType::WALL) + (t_right == TileType::WALL) +
                                     (t_down == TileType::WALL) + (t_up == TileType::WALL);

                bool has_wall_left_right = (t_left == TileType::WALL and t_right == TileType::WALL) and
                                           (t_down == TileType::GROUND and t_up == TileType::GROUND);

                bool has_wall_above_below = (t_left == TileType::GROUND and t_right == TileType::GROUND) and
                                            (t_down == TileType::WALL and t_up == TileType::WALL);

                if (n_walls_around >= 3)
                {
                    t = TileType::WALL;
                }
            }
        }
    }
}

//! \brief removes sharp corners and thin walls
void MapGrid::sawOffCorners()
{

    walls_drawable_.clear();

    for (int j = 1; j < n_cells_.y - 1; ++j)
    {
        for (int i = 1; i < n_cells_.x - 1; ++i)
        {
            const auto cell_index = j * n_cells_.x + i;

            const auto t_left = tiles[cell_index + delta_inds[Direction::LEFT]];
            const auto t_right = tiles[cell_index + delta_inds[Direction::RIGHT]];
            const auto t_up = tiles[cell_index + delta_inds[Direction::UP]];
            const auto t_down = tiles[cell_index + delta_inds[Direction::DOWN]];
            auto &t = tiles[cell_index];

            if (t != TileType::GROUND)
            {
                int n_walls_around = (t_left == TileType::WALL) + (t_right == TileType::WALL) +
                                     (t_down == TileType::WALL) + (t_up == TileType::WALL);

                if (n_walls_around == 2)
                {
                    bool is_ul_corner = t_left == TileType::GROUND and t_up == TileType::GROUND;
                    bool is_ur_corner = t_right == TileType::GROUND and t_up == TileType::GROUND;
                    bool is_dl_corner = t_left == TileType::GROUND and t_down == TileType::GROUND;
                    bool is_dr_corner = t_right == TileType::GROUND and t_down == TileType::GROUND;

                    if (is_ul_corner)
                    {
                        tiles2[cell_index].type = WallType::ULTRIANGLE;
                    }
                    if (is_ur_corner)
                    {
                        tiles2[cell_index].type = WallType::URTRIANGLE;
                    }
                    if (is_dl_corner)
                    {
                        tiles2[cell_index].type = WallType::DLTRIANGLE;
                    }
                    if (is_dr_corner)
                    {
                        tiles2[cell_index].type = WallType::DRTRIANGLE;
                    }
                }

                if (n_walls_around == 3)
                {
                    bool is_up = t_up == TileType::GROUND;
                    bool is_left = t_left == TileType::GROUND;
                    bool is_down = t_down == TileType::GROUND;
                    bool is_right = t_right == TileType::GROUND;
                    if (is_up)
                    {
                        tiles2[cell_index].type = WallType::SQUAREUP;
                    }
                    if (is_left)
                    {
                        tiles2[cell_index].type = WallType::SQUARELEFT;
                    }
                    if (is_down)
                    {
                        tiles2[cell_index].type = WallType::SQUAREDOWN;
                    }
                    if (is_right)
                    {
                        tiles2[cell_index].type = WallType::SQUARERIGHT;
                    }
                }
                if (n_walls_around == 4)
                {
                    tiles2[cell_index].type = WallType::SQUAREINSIDE;
                }
            }
        }
    }

}


//! \brief reads current tiles and draws corresponding shapes on a texgture.  
void MapGrid::updateTexture(){


    for (int j = 1; j < n_cells_.y - 1; ++j)
    {
        for (int i = 1; i < n_cells_.x - 1; ++i)
        {
            const auto cell_index = j * n_cells_.x + i;
            
            
            sf::ConvexShape* shape = nullptr;

            switch (tiles2[cell_index].type)
            {
            case WallType::DLTRIANGLE:
                shape = &dl_triangle;
                break;
            case WallType::ULTRIANGLE:
                shape = &ul_triangle;
                break;
            case WallType::DRTRIANGLE:
                shape = &dr_triangle;
                break;
            case WallType::URTRIANGLE:
                shape = &ur_triangle;
                break;
            default:
                if (tiles2[cell_index].isSquare()){shape = &wall_rect;}
                break;
            }
            if (shape)
            {
                shape->setPosition(i * cell_size_.x, j * cell_size_.y);
                map_texture.draw(*shape);
            }
        
        }
    }
}

void MapGrid::updateBoundaryTypes()
{

    grid2walltype.clear();

    for (int j = 1; j < n_cells_.y - 1; ++j)
    {
        for (int i = 1; i < n_cells_.x - 1; ++i)
        {
            auto cell_index = j * n_cells_.x + i;
            auto t11 = tiles[cell_index + delta_inds[Direction::LEFTUP]];
            auto t12 = tiles[cell_index + delta_inds[Direction::UP]];
            auto t13 = tiles[cell_index + delta_inds[Direction::RIGHTUP]];

            auto t21 = tiles[cell_index + delta_inds[Direction::LEFT]];
            auto t22 = tiles[cell_index];
            auto t23 = tiles[cell_index + delta_inds[Direction::RIGHT]];

            auto t31 = tiles[cell_index + delta_inds[Direction::LEFTDOWN]];
            auto t32 = tiles[cell_index + delta_inds[Direction::DOWN]];
            auto t33 = tiles[cell_index + delta_inds[Direction::RIGHTDOWN]];

            if (t22 != TileType::GROUND)
            {
                bool ur_is_free = (t23 == TileType::GROUND and t13 == TileType::GROUND and t12 == TileType::GROUND);
                bool ul_is_free = (t21 == TileType::GROUND and t11 == TileType::GROUND and t12 == TileType::GROUND);
                bool dr_is_free = (t23 == TileType::GROUND and t33 == TileType::GROUND and t32 == TileType::GROUND);
                bool dl_is_free = (t21 == TileType::GROUND and t31 == TileType::GROUND and t32 == TileType::GROUND);
                if (ur_is_free + ul_is_free + dr_is_free + dl_is_free == 1)
                {
                    if (ur_is_free)
                    {
                        grid2walltype[cell_index] = WallType::URTRIANGLE;
                    }
                    if (ul_is_free)
                    {
                        grid2walltype[cell_index] = WallType::ULTRIANGLE;
                    }
                    if (dr_is_free)
                    {
                        grid2walltype[cell_index] = WallType::DRTRIANGLE;
                    }
                    if (dl_is_free)
                    {
                        grid2walltype[cell_index] = WallType::DLTRIANGLE;
                    }
                    continue;
                }

                if (isBoundaryWallTileV2(cell_index, Direction::RIGHT))
                {
                    grid2walltype[cell_index] = WallType::SQUARERIGHT;
                }
                else if (isBoundaryWallTileV2(cell_index, Direction::LEFT))
                {
                    grid2walltype[cell_index] = WallType::SQUARELEFT;
                }
                else if (isBoundaryWallTileV2(cell_index, Direction::UP))
                {
                    grid2walltype[cell_index] = WallType::SQUAREUP;
                }
                else if (isBoundaryWallTileV2(cell_index, Direction::DOWN))
                {
                    grid2walltype[cell_index] = WallType::SQUAREDOWN;
                }
            }
        }
    }
}

void MapGrid::extractEdgesFromTiles(Triangulation &cdt)
{
    std::vector<bool> visited(n_cells_.x * n_cells_.y, false);

    auto &edges = p_edges_->edges_;
    auto &vertices = cdt.vertices_;
    auto &edge_inds = p_edges_->edges2_;

    Edgef e;
    sf::Vector2i cell_coords;
    int last_vertex_index;
    Vertex vertex;

    int n = n_cells_.x;
    int m = n_cells_.y;

    const auto cell_size = static_cast<int>(std::floor(cell_size_.x));

    for (int j = 1; j < n_cells_.y - 1; ++j)
    { //! look for horizontal edges
        for (int i = 0; i < n_cells_.x; ++i)
        {

            auto cell_index = j * n_cells_.x + i;
            if (!visited[cell_index] and isBoundaryWallTile(cell_index, Direction::UP))
            {
                auto start_cell_index = cell_index;
                int n_iterations = 0;
                if (grid2walltype[cell_index] % 2 == 1)
                {
                    cell_index += delta_inds[Direction::RIGHT];
                    start_cell_index = cell_index;
                }
                auto direction = Direction::RIGHT;
                auto normal_direction = Direction::UP;
                last_vertex_index = vertices.size();
                do
                {
                    n_iterations++;
                    normal_direction = countclockDirection(countclockDirection(direction));
                    visited[cell_index] = true;
                    cell_coords = cellCoords(cell_index);
                    e.l = cell_size_.x;
                    if (direction <= 1)
                    {
                        vertex = {cell_coords.x, cell_coords.y};
                    }
                    else if (direction <= 3)
                    {
                        vertex = {(cell_coords.x + 1), cell_coords.y};
                    }
                    else if (direction <= 5)
                    {
                        vertex = {(cell_coords.x + 1), (cell_coords.y + 1)};
                    }
                    else
                    {
                        vertex = {cell_coords.x, (cell_coords.y + 1)};
                    }

                    e.from = asFloat(vertex);
                    e.from *= static_cast<float>(cell_size);
                    e.t = t_vectors[direction];
                    p_edges_->vertices_.push_back(vertex);
                    cdt.insertVertex(vertex, true);

                    while (isBoundaryWallTile(cell_index + delta_inds[direction]))
                    {
                        if (tiles[cell_index] != tiles[cell_index + delta_inds[direction]])
                        {
                            break;
                        }
                        else
                        {
                            if (grid2walltype[cell_index] != grid2walltype[cell_index + delta_inds[direction]])
                            {
                                break;
                            }
                        }
                        cell_index += delta_inds[direction];
                        visited[cell_index] = true;
                        visited[cell_index + delta_inds[normal_direction]] = true;
                        e.l += cell_size_.x;
                        n_iterations++;
                    }

                    auto edge_direction = direction;
                    if (direction % 2 == 1)
                    { //! the direction is diagonal thus the edge is longer
                        e.l *= std::sqrt(2.f);
                    }
                    if (direction != grid2walltype[cell_index])
                    {
                        e.from.x += static_cast<int>(std::floor(e.t.x * e.l));
                        e.from.y += static_cast<int>(std::floor(e.t.y * e.l));
                        e.t *= -1.f;
                        edge_direction = opposite(direction);
                    }

                    edges.push_back(e);
                    const VertInd last_ind = vertices.size() - 1;
                    EdgeVInd edge_ind = {last_ind, last_ind + 1};
                    edge_inds.push_back(edge_ind);

                    int ix = cellCoords(cell_index).x;
                    int iy = cellCoords(cell_index).y;
                    size_t edge_grid_index;
                    if (direction == Direction::RIGHT or direction == Direction::LEFT)
                    {
                        edge_grid_index =
                            (direction == Direction::RIGHT) * iy + (direction == Direction::LEFT) * (iy + 1);
                    }
                    else if (direction == Direction::DOWN or direction == Direction::UP)
                    {
                        edge_grid_index =
                            (direction == Direction::DOWN) * (ix + 1) + (direction == Direction::UP) * (ix);
                    }
                    else if (direction == Direction::RIGHTDOWN or direction == Direction::LEFTUP)
                    {
                        edge_grid_index = ix - iy + n_cells_.y - 1;
                    }
                    else
                    {
                        edge_grid_index = ix + iy;
                    }

                    p_edges_->edge_finders_.at(direction2Orientation(direction))
                        ->insert(edges.size() - 1, edge_grid_index);

                    if (n_iterations > n_cells_.x * n_cells_.y)
                    {
                        throw std::runtime_error("Boundary of the shape could not be found in !");
                    }
                } while (!(cell_index == start_cell_index and (direction == Direction::RIGHT or direction == RIGHTUP)));
                edge_inds.back().to = last_vertex_index; //! The last vertex is the first one since the object is closed
                                                         //! and we went around its boundary*/
                                                         //                }
            }
        }
    }

    for (int e_ind = 0; e_ind < edge_inds.size(); ++e_ind)
    {
        cdt.insertConstraint(edge_inds[e_ind]);
    }
}

Building &Building::operator=(const Building &b)
{
    this->edges = b.edges;
    n = b.n;
    m = b.m;
    contour = b.contour;
    return *this;
}

sf::Vector2i Building::calcCenter() const
{
    sf::Vector2i center;
    for (int i = 0; i < 8; ++i)
    {
        center += edges[i].from;
    }
    return center / 8;
}

void Building::intitializeEdges(sf::Vector2i center_cell_coords, sf::Vector2f cell_size)
{
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
    for (int i = 0; i < 8; ++i)
    {
        edges[i].from *= static_cast<int>(cell_size.x);
        contour.setPoint(i, static_cast<sf::Vector2f>(edges[i].from));
    }
}

void MapGrid::extractVerticesForDrawing2(const Triangulation &cdt, const Edges &edge_lord)
{

    const auto &edges = edge_lord.edges_;
    const auto n_points = edges.size();
    for (const auto &edge : edges)
    {
        // edge.from
    }
}

void MapGrid::extractVerticesForDrawing(Triangulation &cdt, std::vector<TriInd> &tri_ind2component)
{

    const auto &triangles = cdt.triangles_;
    walls_vertices_.clear();
    sf::Color component_color = sf::Color::Blue;
    walls_vertices_.setPrimitiveType(sf::Triangles);
    // component_color.a = 69;
    int tri_ind = 0;
    for (int j = 1; j < n_cells_.y - 1; ++j)
    { //! Scanning left-right
        for (int i = 1; i < n_cells_.x - 1; ++i)
        {
            const auto cell_index = j * n_cells_.x + i;
            const auto cell_coords = cell_size_.x * asFloat(cellCoords(cell_index));
            const auto &tile = tiles.at(cell_index);
            if (tiles2.at(cell_index).isSquare())
            {
                walls_vertices_.append({{cell_coords.x, cell_coords.y}, component_color});
                walls_vertices_.append({{cell_coords.x + cell_size_.x, cell_coords.y}, component_color});
                walls_vertices_.append({{cell_coords.x + cell_size_.x, cell_coords.y + cell_size_.y}, component_color});

                walls_vertices_.append({{cell_coords.x + cell_size_.x, cell_coords.y + cell_size_.y}, component_color});
                walls_vertices_.append({{cell_coords.x, cell_coords.y + cell_size_.y}, component_color});
                walls_vertices_.append({{cell_coords.x, cell_coords.y}, component_color});
            }
            if (tiles2.at(cell_index).type == WallType::DLTRIANGLE)
            {
                walls_vertices_.append({{cell_coords.x, cell_coords.y}, component_color});
                walls_vertices_.append({{cell_coords.x + cell_size_.x, cell_coords.y}, component_color});
                walls_vertices_.append({{cell_coords.x + cell_size_.x, cell_coords.y + cell_size_.y}, component_color});
            }
            if (tiles2.at(cell_index).type == WallType::DRTRIANGLE)
            {
                walls_vertices_.append({{cell_coords.x, cell_coords.y}, component_color});
                walls_vertices_.append({{cell_coords.x + cell_size_.x, cell_coords.y}, component_color});
                walls_vertices_.append({{cell_coords.x, cell_coords.y + cell_size_.y}, component_color});
            }
            if (tiles2.at(cell_index).type == WallType::URTRIANGLE)
            {
                walls_vertices_.append({{cell_coords.x, cell_coords.y}, component_color});
                walls_vertices_.append({{cell_coords.x, cell_coords.y + cell_size_.y}, component_color});
                walls_vertices_.append({{cell_coords.x + cell_size_.x, cell_coords.y + cell_size_.y}, component_color});
            }
            if (tiles2.at(cell_index).type == WallType::ULTRIANGLE)
            {
                walls_vertices_.append({{cell_coords.x + cell_size_.x, cell_coords.y}, component_color});
                walls_vertices_.append({{cell_coords.x, cell_coords.y + cell_size_.y}, component_color});
                walls_vertices_.append({{cell_coords.x + cell_size_.x, cell_coords.y + cell_size_.y}, component_color});
            }
        }
    }
    std::cout << walls_vertices_.getVertexCount() << "\n";
    walls_drawable_.clear();
};

void MapGrid::extractEdgesFromTilesV2(Triangulation &cdt)
{
    std::vector<bool> visited(n_cells_.x * n_cells_.y, false);

    auto &edges = p_edges_->edges_;
    auto &vertices = p_edges_->vertices_;
    auto &edge_inds = p_edges_->edges2_;

    cdt.reset();
    cdt.createBoundaryAndSuperTriangle();

    vertices = cdt.vertices_;
    edge_inds = {{3, 4}, {4, 5}, {5, 6}, {6, 3}};
    edges = {
        {vertices.at(3), vertices.at(4)},
        {vertices.at(4), vertices.at(5)},
        {vertices.at(5), vertices.at(6)},
        {vertices.at(6), vertices.at(3)},
    };
    p_edges_->edge_finders_.at(0)->reset();
    p_edges_->edge_finders_.at(1)->reset();
    p_edges_->edge_finders_.at(2)->reset();
    p_edges_->edge_finders_.at(3)->reset();

    std::unordered_map<Vertex, VertInd, VertexHash> vertex2vert_ind;

    Vertex vertex;
    VertInd vert_ind;

    const auto n = n_cells_.x;
    const auto m = n_cells_.y;

    const int dx = cell_size_.x;
    const int dy = cell_size_.y;

    auto get_vert_ind = [&](const Vertex &vertex)
    {
        if (vertex2vert_ind.count(vertex) == 0)
        {
            vertex2vert_ind[vertex] = p_edges_->vertices_.size();
            p_edges_->vertices_.push_back(vertex);
            return static_cast<VertInd>(p_edges_->vertices_.size() - 1);
        }
        return static_cast<VertInd>(vertex2vert_ind.at(vertex));
    };

    bool scanning_edge = false;
    bool edge_is_up;
    bool edge_is_right;
    int n_cells_forming_edge = 0;
    for (int j = 1; j < m - 1; ++j)
    { //! Scanning left-right
        for (int i = 1; i < n - 1; ++i)
        {
            const auto cell_index = j * n_cells_.x + i;
            const auto &tile = tiles2.at(cell_index).type;
            if (tile == WallType::SQUAREUP or tile == WallType::SQUAREDOWN)
            {
                if (!scanning_edge)
                {
                    edge_is_up = tile == WallType::SQUAREUP;
                    n_cells_forming_edge = 0;
                    vertex.x = i * dx;
                    vertex.y = (j + (tile == WallType::SQUAREDOWN)) * dx;
                    vert_ind = get_vert_ind(vertex);
                    scanning_edge = true;
                }
                if (scanning_edge)
                {
                    n_cells_forming_edge += 1;
                }
            }
            else
            {
                if (scanning_edge)
                {
                    Vertex end_vertex = {vertex.x + n_cells_forming_edge * dx, vertex.y};
                    const auto end_vert_ind = get_vert_ind(end_vertex);
                    if (edge_is_up)
                    {
                        edges.push_back({vertex, end_vertex});
                        edge_inds.push_back({vert_ind, end_vert_ind});
                    }
                    if (!edge_is_up)
                    {
                        edges.push_back({end_vertex, vertex});
                        edge_inds.push_back({end_vert_ind, vert_ind});
                    }
                    p_edges_->edge_finders_.at(0)->insert2(edges.size() - 1, j);

                    scanning_edge = false;
                }
            }
        }
        scanning_edge = false;
    }

    //! Scanning up-down
    for (int i = 1; i < n - 1; ++i)
    {
        for (int j = 1; j < m - 1; ++j)
        {
            const auto cell_index = j * n_cells_.x + i;
            const auto &tile = tiles2.at(cell_index).type;
            if (tile == WallType::SQUARELEFT or tile == WallType::SQUARERIGHT)
            {
                if (!scanning_edge)
                {
                    edge_is_right = tile == WallType::SQUARERIGHT;
                    n_cells_forming_edge = 0;
                    vertex.x = (i + (tile == WallType::SQUARERIGHT)) * dx;
                    vertex.y = j * dx;
                    vert_ind = get_vert_ind(vertex);
                    scanning_edge = true;
                }
                if (scanning_edge)
                {
                    n_cells_forming_edge += 1;
                }
            }
            else
            {
                if (scanning_edge)
                {
                    const Vertex end_vertex = {vertex.x, vertex.y + n_cells_forming_edge * dx};
                    const auto end_vert_ind = get_vert_ind(end_vertex);
                    if (edge_is_right)
                    {
                        edges.push_back({vertex, end_vertex});
                        edge_inds.push_back({vert_ind, end_vert_ind});
                    }
                    if (!edge_is_right)
                    {
                        edges.push_back({end_vertex, vertex});
                        edge_inds.push_back({end_vert_ind, vert_ind});
                    }
                    p_edges_->edge_finders_.at(2)->insert2(edges.size() - 1, i);

                    scanning_edge = false;
                }
            }
        }
        scanning_edge = false;
    }

    //! Scanning diagonally right down
    const auto i_diag_max = n + m - 2;
    for (int i_diag = 0; i_diag < i_diag_max; ++i_diag)
    {
        int i = std::max(0, i_diag - n);
        int j = std::max(0, m - i_diag);

        while (i < n and j < m)
        {
            const auto cell_index = j * n_cells_.x + i;
            const auto &tile = tiles2.at(cell_index).type;
            if (tile == WallType::DLTRIANGLE or tile == WallType::URTRIANGLE)
            {
                if (!scanning_edge)
                {
                    edge_is_up = tile == WallType::URTRIANGLE;
                    n_cells_forming_edge = 0;
                    vertex.x = i * dx;
                    vertex.y = j * dy;
                    vert_ind = get_vert_ind(vertex);
                    scanning_edge = true;
                }
                if (scanning_edge)
                {
                    n_cells_forming_edge += 1;
                }
            }
            else
            {
                if (scanning_edge)
                {
                    const Vertex end_vertex = {vertex.x + n_cells_forming_edge * dx,
                                               vertex.y + n_cells_forming_edge * dy};
                    const auto end_vert_ind = get_vert_ind(end_vertex);
                    if (edge_is_up)
                    {
                        edges.push_back({vertex, end_vertex});
                        edge_inds.push_back({vert_ind, end_vert_ind});
                    }
                    if (!edge_is_up)
                    {
                        edges.push_back({end_vertex, vertex});
                        edge_inds.push_back({end_vert_ind, vert_ind});
                    }
                    p_edges_->edge_finders_.at(1)->insert2(edges.size() - 1, i - j + m - 1);

                    scanning_edge = false;
                }
            }
            i++;
            j++;
        }
        scanning_edge = false;
    }

    //! Scanning diagonally right up
    for (int i_diag = 0; i_diag < i_diag_max; ++i_diag)
    {
        int i = std::max(0, i_diag - m);
        int j = std::min(i_diag, m - 1);

        while (i < n and j > 0)
        {
            const auto cell_index = j * n_cells_.x + i;
            const auto &tile = tiles2.at(cell_index).type;
            if (tile == WallType::ULTRIANGLE or tile == WallType::DRTRIANGLE)
            {
                if (!scanning_edge)
                {
                    edge_is_up = tile == WallType::ULTRIANGLE;
                    n_cells_forming_edge = 0;
                    vertex.x = i * dx;
                    vertex.y = (j + 1) * dy;
                    vert_ind = get_vert_ind(vertex);
                    scanning_edge = true;
                }
                if (scanning_edge)
                {
                    n_cells_forming_edge += 1;
                }
            }
            else
            {
                if (scanning_edge)
                {
                    const Vertex end_vertex = {vertex.x + n_cells_forming_edge * dx,
                                               vertex.y - n_cells_forming_edge * dy};
                    const auto end_vert_ind = get_vert_ind(end_vertex);

                    if (edge_is_up)
                    {
                        edges.push_back({vertex, end_vertex});
                        edge_inds.push_back({vert_ind, end_vert_ind});
                    }
                    if (!edge_is_up)
                    {
                        edges.push_back({end_vertex, vertex});
                        edge_inds.push_back({end_vert_ind, vert_ind});
                    }
                    p_edges_->edge_finders_.at(3)->insert2(edges.size() - 1, i + j + 1);

                    scanning_edge = false;
                }
            }
            i++;
            j--;
        }
        scanning_edge = false;
    }

    for (VertInd v_ind = 7; v_ind < p_edges_->vertices_.size(); ++v_ind)
    {
        cdt.insertVertex(p_edges_->vertices_[v_ind]);
    }

    // const auto& triangles = cdt.triangles_;
    // for (VertInd v_ind = 7; v_ind < p_edges_->vertices_.size(); ++v_ind) {
    //     cdt.insertVertex(p_edges_->vertices_[v_ind]);
    // }

    for (const auto &edge : p_edges_->edges2_)
    {
        cdt.insertConstraint(edge);
    }
}

void MapGrid::buildBuilding(sf::Vector2f building_center, sf::Vector2i building_size, Triangulation &cdt)
{
    auto center_cell_coords = cellCoords(building_center);
    auto lower_left_cell_coords = center_cell_coords - building_size / 2;
    bool all_cells_on_ground = true;
    for (int i = 0; i < building_size.x; ++i)
    {
        for (int j = 0; j < building_size.y; ++j)
        {
            auto i_building = center_cell_coords.x - building_size.x / 2 + i;
            auto j_building = center_cell_coords.y - building_size.y / 2 + j;
            auto cell_index = n_cells_.x * j_building + i_building;
            if (i_building >= n_cells_.x - 1 or j_building >= n_cells_.y - 1 or i_building <= 0 or j_building <= 0 or
                tiles.at(cell_index) != TileType::GROUND)
            {
                all_cells_on_ground = false;
                break;
            }
        }
    }

    if (all_cells_on_ground)
    {
        for (int j = 0; j < building_size.y; ++j)
        {
            for (int i = 0; i < building_size.x; ++i)
            {
                auto i_building = center_cell_coords.x - building_size.x / 2 + i;
                auto j_building = center_cell_coords.y - building_size.y / 2 + j;
                auto cell_index = n_cells_.x * j_building + i_building;
                tiles[cell_index] = TileType::BUILDING;
                cell2building_ind_[cell_index] = buildings_.size();

                if (j == 0)
                {
                    grid2walltype[cell_index] = WallType::SQUAREUP;
                    if (i == 0)
                    {
                        grid2walltype[cell_index] = WallType::ULTRIANGLE;
                    }
                    else if (i == building_size.x - 1)
                    {
                        grid2walltype[cell_index] = WallType::URTRIANGLE;
                    }
                }
                else if (j == building_size.y - 1)
                {
                    grid2walltype[cell_index] = WallType::SQUAREDOWN;
                    if (i == 0)
                    {
                        grid2walltype[cell_index] = WallType::DLTRIANGLE;
                    }
                    else if (i == building_size.x - 1)
                    {
                        grid2walltype[cell_index] = WallType::DRTRIANGLE;
                    }
                }
                if (i == 0 and (j > 0 and j < building_size.y - 1))
                {
                    grid2walltype[cell_index] = WallType::SQUARELEFT;
                }
                else if (i == building_size.x - 1 and (j > 0 and j < building_size.y - 1))
                {
                    grid2walltype[cell_index] = WallType::SQUARERIGHT;
                }
            }
        }

        //        updateBoundaryTypes();
        auto new_building = addBuildingEdgesToTriangulation(building_center, building_size, cdt);
        assert(cdt.triangulationIsConsistent());
        new_building.contour.setTexture(&grass);
        buildings_.push_back(new_building);
        // map_texture.draw(buildings.m_vertices);
    }
}

Building MapGrid::addBuildingEdgesToTriangulation(sf::Vector2f building_center, sf::Vector2i building_size,
                                                  Triangulation &cdt)
{
    auto center_cell_coords = cellCoords(building_center);
    auto lower_left_cell_coords = center_cell_coords - building_size / 2;

    Building b;
    b.n = building_size.x;
    b.m = building_size.y;
    b.intitializeEdges(center_cell_coords, cell_size_);
    auto edge_grid_index_up = lower_left_cell_coords.y;
    auto edge_grid_index_down = lower_left_cell_coords.y + building_size.y;
    auto edge_grid_index_left = lower_left_cell_coords.x;
    auto edge_grid_index_right = lower_left_cell_coords.x + building_size.x;
    auto edge_grid_index_ur =
        (lower_left_cell_coords.x + building_size.x - 1) - (lower_left_cell_coords.y) + n_cells_.y - 1;
    auto edge_grid_index_ul = (lower_left_cell_coords.x) + (lower_left_cell_coords.y);
    auto edge_grid_index_dr =
        (lower_left_cell_coords.x + building_size.x - 1) + (lower_left_cell_coords.y + building_size.y - 1);
    auto edge_grid_index_dl =
        (lower_left_cell_coords.x) - (lower_left_cell_coords.y + building_size.y - 1) + n_cells_.y - 1;
    std::array<int, 4> edge_grid_inds = {edge_grid_index_up, edge_grid_index_right, edge_grid_index_down,
                                         edge_grid_index_left};
    std::array<int, 4> edge_grid_inds_diag = {edge_grid_index_ur, edge_grid_index_dr, edge_grid_index_dl,
                                              edge_grid_index_ul};

    std::array<int, 8> edge_grid_inds2 = {edge_grid_index_up, edge_grid_index_ur, edge_grid_index_right,
                                          edge_grid_index_dr, edge_grid_index_down, edge_grid_index_dl,
                                          edge_grid_index_left, edge_grid_index_ul};
    std::array<VertInd, 8> building_vert_inds;

    int n_inserted_vertices = 0; //! we do not insert new vertices that overlap with existing ones
    std::vector<EdgeVInd> new_constraints(0);

    int last_vertex_ind = cdt.vertices_.size() - 1;
    int last_edge_ind = cdt.vertices_.size() - 1;

    for (int i = 0; i < 4; ++i)
    {
        auto &horizontal_edges = p_edges_->edge_finders_.at(2 * (i % 2))->edge_array_;

        VertInd ind1;
        VertInd ind2;

        VertInd last_ind = cdt.vertices_.size();
        auto &edges_on_line = horizontal_edges.at(edge_grid_inds[i]);
        auto &edges_on_line2 = p_edges_->edge_finders_.at(2 * (i % 2))->edge_array2_.at(edge_grid_inds[i]);

        if (i % 2 == 0)
        {
            std::sort(edges_on_line.begin(), edges_on_line.end(), [&](int i1, int i2)
                      { return cdt.vertices_[p_edges_->edges2_[i1].from].x < cdt.vertices_[p_edges_->edges2_[i2].from].x; });
        }
        else
        {
            std::sort(edges_on_line.begin(), edges_on_line.end(), [&](int i1, int i2)
                      { return cdt.vertices_[p_edges_->edges2_[i1].from].y < cdt.vertices_[p_edges_->edges2_[i2].from].y; });
        }

        auto vb1x = b.edges[2 * i].from.x;
        auto vb2x = b.edges[(2 * i + 1) % 8].from.x;
        auto vb1y = b.edges[2 * i].from.y;
        auto vb2y = b.edges[(2 * i + 1) % 8].from.y;

        bool inv = false;
        if (vb1y > vb2y)
        {
            std::swap(vb1y, vb2y);
            inv = true;
        }
        if (vb1x > vb2x)
        {
            std::swap(vb1x, vb2x);
            inv = true;
        }

        // auto intersected_edge1 = cdt.insertVertex(b.edges[2 * i + inv].from, true);
        // auto intersected_edge2 = cdt.insertVertex(b.edges[2 * i + !inv].from, true);

        auto intersection_data1 = cdt.insertVertexAndGetData(b.edges[2 * i + inv].from, true);
        auto intersection_data2 = cdt.insertVertexAndGetData(b.edges[2 * i + !inv].from, true);

        bool vertex_1_exitsts = intersection_data1.overlapping_vertex != -1;
        bool vertex_1_is_inside_edge = intersection_data1.overlapping_edge.from != -1;

        bool vertex_2_exitsts = intersection_data2.overlapping_vertex != -1;
        bool vertex_2_is_inside_edge = intersection_data2.overlapping_edge.from != -1;

        vertex_1_exitsts ? ind1 = intersection_data1.overlapping_vertex : ind1 = ++last_ind - 1;
        vertex_2_exitsts ? ind2 = intersection_data2.overlapping_vertex : ind2 = ++last_ind - 1;

        if (!vertex_1_exitsts)
        {
            p_edges_->vertices_.push_back(b.edges[2 * i + inv].from);
        }
        if (!vertex_2_exitsts)
        {
            p_edges_->vertices_.push_back(b.edges[2 * i + !inv].from);
        }

        building_vert_inds[2 * i + inv] = ind1;
        building_vert_inds[2 * i + !inv] = ind2;

        if (vertex_1_exitsts and vertex_2_exitsts)
        {
            continue;
        }

        std::vector<EdgeVInd> middle_edges;
        if (!vertex_1_exitsts and !vertex_1_is_inside_edge)
        {
            middle_edges.push_back({ind1, ind1});
        }
        for (const auto e_ind : edges_on_line)
        {
            auto v1 = cdt.vertices_[p_edges_->edges2_[e_ind].from];
            auto v2 = cdt.vertices_[p_edges_->edges2_[e_ind].to];
            if (i % 2 == 0)
            {
                if (v1.x > v2.x)
                {
                    std::swap(v1, v2);
                    std::swap(p_edges_->edges2_[e_ind].from, p_edges_->edges2_[e_ind].to);
                }
                if (v2.x >= vb1x and v1.x <= vb2x)
                {
                    middle_edges.push_back(p_edges_->edges2_[e_ind]);
                }
            }
            else
            {
                if (v1.y > v2.y)
                {
                    std::swap(v1, v2);
                    std::swap(p_edges_->edges2_[e_ind].from, p_edges_->edges2_[e_ind].to);
                }
                if (v2.y >= vb1y and v1.y <= vb2y)
                {
                    middle_edges.push_back(p_edges_->edges2_[e_ind]);
                }
            }
        }
        if (!vertex_2_exitsts and !vertex_2_is_inside_edge)
        {
            middle_edges.push_back({ind2, ind2});
        }

        for (int a = 1; a < middle_edges.size(); ++a)
        {
            if (middle_edges[a - 1].to != middle_edges[a].from)
            {
                new_constraints.push_back({middle_edges[a - 1].to, middle_edges[a].from});
                p_edges_->edges2_.push_back(new_constraints.back());
                if (i == 0 or i == 1)
                {
                    p_edges_->edges_.emplace_back(p_edges_->vertices_[middle_edges[a - 1].to],
                                                  p_edges_->vertices_[middle_edges[a].from]);
                }
                else
                {
                    p_edges_->edges_.emplace_back(p_edges_->vertices_[middle_edges[a].from],
                                                  p_edges_->vertices_[middle_edges[a - 1].to]);
                }

                p_edges_->edge_finders_.at(2 * (i % 2))
                    ->insert3(p_edges_->edges_.back(), p_edges_->edges2_.size() - 1, edge_grid_inds[i]);
            }
        }
        if (i % 2 == 0)
        {
            std::sort(edges_on_line2.begin(), edges_on_line2.end(),
                      [](const Edgef &e1, const Edgef &e2)
                      { return std::min({e1.from.x, e1.to().x}) < std::min({e2.from.x, e2.to().x}); });
        }
        else
        {
            std::sort(edges_on_line2.begin(), edges_on_line2.end(),
                      [](const Edgef &e1, const Edgef &e2)
                      { return e1.from.y < e2.from.y; });
        }
    }

    for (int i = 0; i < 4; ++i)
    {
        new_constraints.push_back({building_vert_inds[2 * i + 1], building_vert_inds[(2 * i + 2) % 8]});
        p_edges_->edges2_.push_back(new_constraints.back());
        p_edges_->edges_.emplace_back(asFloat(b.edges[2 * i + 1].from), b.edges[2 * i + 1].to());

        auto &edges_on_line2 = p_edges_->edge_finders_.at(2 * (i % 2) + 1)->edge_array2_[edge_grid_inds_diag[i]];
        // p_edges_->edge_finders_.at(2 * (i % 2) + 1)->insert3(p_edges_->edges_.back(),
        //                                         p_edges_->edges2_.size() - 1, edge_grid_inds_diag[i]);
        p_edges_->edge_finders_.at(2 * (i % 2) + 1)->insert2(p_edges_->edges2_.size() - 1, edge_grid_inds_diag[i]);
        std::sort(edges_on_line2.begin(), edges_on_line2.end(),
                  [](const Edgef &e1, const Edgef &e2)
                  { return e1.from.x < e2.from.x; });
    }

    const auto n_edges = p_edges_->edges2_.size();

    for (const auto new_edge : new_constraints)
    {
        cdt.insertConstraint(new_edge);
    }
    return b;
}

void MapGrid::removeBuilding(int removed_building_ind, Triangulation &cdt)
{

    const auto &removed_building = buildings_[removed_building_ind];
    auto center_cell_coords = cellCoords(removed_building.calcCenter());
    sf::Vector2i building_size = {removed_building.n, removed_building.m};
    for (int i = 0; i < building_size.x; ++i)
    {
        for (int j = 0; j < building_size.y; ++j)
        {
            auto i_building = center_cell_coords.x - building_size.x / 2 + i;
            auto j_building = center_cell_coords.y - building_size.y / 2 + j;
            auto cell_index = n_cells_.x * j_building + i_building;
            cell2building_ind_[cell_index] = -1;
            tiles.at(cell_index) = TileType::GROUND;
        }
    }

    const auto last_building_ind = buildings_.size() - 1;
    const auto last_building = buildings_[last_building_ind];
    buildings_[removed_building_ind] = last_building;

    center_cell_coords = cellCoords(last_building.calcCenter());
    building_size = {last_building.n, last_building.m};
    for (int i = 0; i < building_size.x; ++i)
    {
        for (int j = 0; j < building_size.y; ++j)
        {
            auto i_building = center_cell_coords.x - building_size.x / 2 + i;
            auto j_building = center_cell_coords.y - building_size.y / 2 + j;
            auto cell_index = n_cells_.x * j_building + i_building;
            cell2building_ind_[cell_index] = removed_building_ind;
        }
    }

    buildings_.pop_back();

    cdt.reset();
    p_edges_->reset();
    cdt.createBoundaryAndSuperTriangle();

    addAllBuildingsToTriangulation(cdt);
}

//!
void MapGrid::addAllStaticEdgesToTriangulation(Triangulation &cdt)
{

    cdt.reset();
    for (const auto &vertex : p_edges_->vertices_)
    {
    }

    for (const auto &building : buildings_)
    {
        const auto building_center = static_cast<sf::Vector2f>(building.calcCenter());
        addBuildingEdgesToTriangulation(building_center, {building.n, building.m}, cdt);
    }
}

void MapGrid::addAllBuildingsToTriangulation(Triangulation &cdt)
{
    for (const auto &building : buildings_)
    {
        const auto building_center = static_cast<sf::Vector2f>(building.calcCenter());
        addBuildingEdgesToTriangulation(building_center, {building.n, building.m}, cdt);
    }
}

void MapGrid::buildWall(sf::Vector2f wall_center, sf::Vector2i square_size)
{
    auto lower_left_cell_coords = cellCoords(wall_center);
    for (int i = 0; i < square_size.x; ++i)
    {
        for (int j = 0; j < square_size.y; ++j)
        {
            auto i_building = lower_left_cell_coords.x - square_size.x / 2 + i;
            auto j_building = lower_left_cell_coords.y - square_size.y / 2 + j;
            if (i_building <= 2 or i_building >= n_cells_.x - 1 or j_building <= 2 or j_building >= n_cells_.y - 1)
            {
                continue;
            }
            auto cell_index = n_cells_.x * j_building + i_building;
            if (tiles.at(cell_index) == TileType::GROUND)
            {
                tiles.at(cell_index) = TileType::WALL;
            }
            if (grid2walltype.count(cell_index) > 0)
            {
                grid2walltype.erase(cell_index);
            }
        }
    }
}

float fy(const float t, const std::vector<float> &cs, const std::vector<float> &ss)
{
    float result = 0;
    for (int i = 1; i < cs.size() + 1; ++i)
    {
        result += cs[i - 1] * std::cos(2.0f * M_PIf * t * i) + ss[i - 1] * std::sin(2.0f * M_PIf * t * i);
    }
    return (result);
}
float fx(const float t) { return std::sin(2.0f * M_PIf * t); }

bool is_in(sf::Vector2f r, const std::vector<float> &cs, const std::vector<float> &ss)
{

    if (r.x > 1 or r.x < -1)
    {
        return false;
    }
    auto t1 = std::asin(r.x / 1.f) / (2.0f * M_PIf);
    auto t2 = 0.5f - t1;
    if (t1 < 0)
    {
        t1 += 1;
    }
    const auto y1 = fy(t1, cs, ss);
    const auto y2 = fy(t2, cs, ss);
    return ((r.y > y1) and (r.y < y2)) xor ((r.y < y1) and (r.y > y2));
}

bool is_in(sf::Vector2f r, float x_min, float x_max, int nx, const std::vector<sf::Vector2f> &values)
{
    int ind_x = (r.x - x_min) / (x_max - x_min) * static_cast<float>(nx);
    if (ind_x < 0 or ind_x >= values.size())
    {
        return false;
    }
    if (r.y > -values[ind_x].y and r.y < -values[ind_x + values.size() / 2].y)
    {
        return true;
    }
    return false;
}

std::vector<sf::Vector2f> MapGrid::generateRandomWalls(sf::Vector2f center, sf::Vector2f max_coords,
                                                       const float width)
{

    const int NPOINTS = 2000;
    std::vector<sf::Vector2f> function_values(NPOINTS * 2);
    const float dx = cell_size_.x;

    std::vector<float> cs({1, 0.5, 0.1});
    std::vector<float> ss({0.1, 0.2, 0.0});

    for (int i = 2; i < 12; ++i)
    {
        cs.push_back((rand() % 12) / (float(5 * i)));
    }
    for (int i = 2; i < 12; ++i)
    {
        ss.push_back((rand() % 12) / (float(5 * i * i)));
    }

    float max_y = -MAXFLOAT;
    float min_y = MAXFLOAT;
    for (int i = 0; i < NPOINTS; ++i)
    {
        const auto value_x = fx(static_cast<float>(i) / NPOINTS);
        const auto value_y = fy(static_cast<float>(i) / NPOINTS, cs, ss);
        function_values[i] = {i * dx, -max_coords.y * value_y};
        function_values[i + NPOINTS] = {i * dx, -max_coords.y * value_y - width};
        min_y = std::min(-max_coords.y * value_y, min_y);
        max_y = std::max(-max_coords.y * value_y, max_y);
    }
    for (int i = 0; i < NPOINTS; ++i)
    {
        function_values[i].y *= (2 * max_coords.y) / (max_y - min_y);
        function_values[i + NPOINTS].y *= (2 * max_coords.y) / (max_y - min_y);
        function_values[i + NPOINTS] += {center.x, -center.y};
    }
    return function_values;
}

void MapGrid::drawProposedWalls(sf::Vector2f center, sf::Vector2f max_coords,
                                std::vector<sf::Vector2f> &function_values, const float width)
{

    const auto center_cell_coords = cellCoords(center);

    const auto NPOINTS = function_values.size() / 2;
    walls_drawable_.clear();

    sf::ConvexShape wall_rect;

    const auto i_max = static_cast<int>(2 * max_coords.x / cell_size_.x);
    const auto j_max = static_cast<int>(max_coords.y / cell_size_.y);
    for (int i = 0; i < i_max; ++i)
    {
        for (int j = 0; j < j_max; ++j)
        {
            auto i_building = center_cell_coords.x - i_max / 2 + i;
            auto j_building = center_cell_coords.y - j_max / 2 + j;
            if (i_building <= 2 or i_building >= n_cells_.x - 3 or j_building <= 2 or j_building >= n_cells_.y - 3)
            {
                continue;
            }
            const sf::Vector2f r = {i_building * cell_size_.x - cell_size_.x / 2.f,
                                    j_building * cell_size_.y - cell_size_.y / 2.f};
            const sf::Vector2f r_reduced = {(r.x - center.x) / (0.5f * max_coords.x),
                                            (r.y - center.y) / (0.5f * max_coords.y)};

            auto cell_index = n_cells_.x * j_building + i_building;
            if (is_in(r, function_values[0].x, function_values[NPOINTS - 1].x, NPOINTS, function_values))
            {
                wall_rect.setPosition(i * cell_size_.x, j * cell_size_.y);
                walls_drawable_.push_back(wall_rect);
            }
        }
    }
}

std::vector<sf::Vector2f> MapGrid::setWallsInFunction(sf::Vector2f center, sf::Vector2f max_coords,
                                                      std::vector<sf::Vector2f> &function_values, const float width)
{

    const auto center_cell_coords = cellCoords(center);

    const auto NPOINTS = function_values.size() / 2;

    sf::ConvexShape wall_rect;

    const auto i_max = static_cast<int>(2 * max_coords.x / cell_size_.x);
    const auto j_max = static_cast<int>(max_coords.y / cell_size_.y);
    for (int i = 0; i < i_max; ++i)
    {
        for (int j = 0; j < j_max; ++j)
        {
            auto i_building = center_cell_coords.x - i_max / 2 + i;
            auto j_building = center_cell_coords.y - j_max / 2 + j;
            if (i_building <= 2 or i_building >= n_cells_.x - 3 or j_building <= 2 or j_building >= n_cells_.y - 3)
            {
                continue;
            }
            const sf::Vector2f r = {i_building * cell_size_.x - cell_size_.x / 2.f,
                                    j_building * cell_size_.y - cell_size_.y / 2.f};
            const sf::Vector2f r_reduced = {(r.x - center.x) / (0.5f * max_coords.x),
                                            (r.y - center.y) / (0.5f * max_coords.y)};

            auto cell_index = n_cells_.x * j_building + i_building;
            if (tiles.at(cell_index) == TileType::GROUND and
                is_in(r, function_values[0].x, function_values[NPOINTS - 1].x, NPOINTS, function_values))
            {
                tiles.at(cell_index) = TileType::WALL;
            }
        }
    }
    return {};
}

// std::vector<sf::Vector2f> MapGrid::extractBoundaryFromTiles(Triangulation &cdt, Edges &edg) {
//
//     std::vector<bool> visited(n_cells_.x, n_cells_.y);
//
//     for (int j = 1; j < n_cells_.y - 1; ++j) {
//         for (int i = 1; i < n_cells_.x - 1; ++i) {
//             auto cell_index = j * n_cells_.x + i;
//             if (!visited[cell_index] and isBoundaryWallTile( cell_index, Direction::UP)) { //! we found a
//             boundary cell so we walk around the object and update edges
//                 auto start_cell_index = cell_index;
//                 int n_iterations = 0;
//                 if (grid2walltype[cell_index] % 2 == 1) {
//                     cell_index += delta_inds[Direction::RIGHT];
//                     start_cell_index = cell_index;
//                 }
//             }
//         }
//     const auto i_max = static_cast<int>(2*max_coords.x/cell_size_.x);
//     const auto j_max = static_cast<int>((max_y - min_y)/cell_size_.y);
//     for(int i = 0; i < i_max; ++i){
//         for(int j = 0; j < j_max; ++j){
//             auto i_building = center_cell_coords.x - i_max/2 + i;
//             auto j_building = center_cell_coords.y - j_max/2 + j;
//             if(i_building<=2 or i_building>=n_cells_.x-3 or j_building<=2 or j_building>=n_cells_.y-3){
//                 continue;
//             }
//             const sf::Vector2f r = {i_building*cell_size_.x - cell_size_.x/2.f, j_building*cell_size_.y -
//             cell_size_.y/2.f} ; const sf::Vector2f r_reduced = {(r.x - center.x)/(0.5f*max_coords.x), (r.y -
//             center.y)/(0.5f*max_coords.y)};
//
//             auto cell_index = n_cells_.x*j_building + i_building ;
//             if( tiles.at(cell_index) == TileType::GROUND and is_in(r_reduced, cs, ss)){
//                 tiles.at(cell_index) = TileType::WALL;
//             }
//             if(grid2walltype.count(cell_index)>0){
//                 grid2walltype.erase(cell_index);
//             }
//         }
//     }
//     return function_values;
// }

sf::Vector2i MapGrid::drawProposedBuilding(sf::RenderWindow &window, sf::Vector2f building_center,
                                           sf::Vector2i building_size)
{
    auto lower_left_cell_coords = cellCoords(building_center);
    rect.setSize(cell_size_);
    for (int i = 0; i < building_size.x; ++i)
    {
        for (int j = 0; j < building_size.y; ++j)
        {
            auto i_building = lower_left_cell_coords.x - building_size.x / 2 + i;
            auto j_building = lower_left_cell_coords.y - building_size.y / 2 + j;
            if (i_building < 0 or i_building >= n_cells_.x or j_building < 0 or j_building >= n_cells_.y)
            {
                continue;
            }
            auto cell_index = n_cells_.x * j_building + i_building;
            sf::Vector2f rect_position = {i_building * cell_size_.x, j_building * cell_size_.y};
            rect.setPosition(rect_position);
            rect.setFillColor(sf::Color::Black);
            if (tiles.at(cell_index) != TileType::GROUND)
            {
                rect.setFillColor(sf::Color::Red);
            }
            window.draw(rect);
        }
    }
    return lower_left_cell_coords - building_size / 2;
}

void MapGrid::setSprites()
{

    for (auto &building : buildings_)
    {
        building.contour.setTexture(&grass);
        building.contour.setFillColor({15, 135, 251, 120});
    }
    for (auto &wall_tile : walls_drawable_)
    {
        wall_tile.setTexture(&grass);
    }
}

void MapGrid::draw(sf::RenderWindow &window)
{

    for (const auto &building : buildings_)
    {
        // window.draw(building.contour);
    }
    // const auto& map_text = map_texture.getTexture();
    // map_rect.setTexture(&map_text);

    // map_rect.setFillColor(sf::Color::Black);

    window.draw(map_rect);
    // window.draw(walls_vertices_);
    // for (const auto &wall_tile : walls_drawable_)
    // {
    //     window.draw(wall_tile);
    // }
}