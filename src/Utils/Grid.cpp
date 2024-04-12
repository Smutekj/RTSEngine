

#include <cmath>
#include <cassert>
#include <iostream>

#include "Grid.h"

//! \brief computes cell_index of the cell containing given point
//! \param x 
//! \param y
//! \returns cell index
size_t Grid::coordToCell(const float x, const float y) const { return coordToCell({x, y}); }

//! \brief computes cell_index of the cell containing given point
//! \param r_coords given point 
//! \returns cell index
size_t Grid::coordToCell(const sf::Vector2f r_coord) const {
    const size_t ix = static_cast<size_t>(std::floor(r_coord.x / cell_size_.x)) % n_cells_.x;
    const size_t iy = static_cast<size_t>(std::floor(r_coord.y / cell_size_.y)) % n_cells_.y;

    assert(ix + iy * n_cells_.x < n_cells_.x * n_cells_.y);
    return ix + iy * n_cells_.x;
}

//! \brief computes cell_index of the cell corresponding to given 2D cell coordinates
//! \param ix
//! \param iy 
//! \returns cell index
[[nodiscard]] size_t Grid::cellIndex(const int ix, const int iy) const { return cellIndex({ix, iy}); }

//! \brief computes cell_index of the cell corresponding to given 2D cell coordinates
//! \param ixy given 2D cell coordinates
//! \returns cell index
[[nodiscard]] size_t Grid::cellIndex(sf::Vector2i ixy) const { return ixy.x + ixy.y * n_cells_.x; }

//! \brief computes the x component of 2D cell coordinates 
//! \param cell_index
//! \returns x component of 2D cell coordinates
size_t Grid::cellCoordX(const size_t cell_index) const { return cell_index % n_cells_.x; }

//! \brief computes the y component of 2D cell coordinates 
//! \param cell_index
//! \returns y component of 2D cell coordinates
size_t Grid::cellCoordY(const size_t cell_index) const { return (cell_index / n_cells_.x) % n_cells_.y; }

//! \brief computes the 2D cell coordinates of a given cell 
//! \param cell_index
//! \returns 2D cell coordinates
sf::Vector2i Grid::cellCoords(const size_t cell_index) const {
    return {static_cast<int>(cellCoordX(cell_index)), static_cast<int>(cellCoordY(cell_index))};
}

//! \brief computes the 2D cell coordinates of cell containing the given point 
//! \param r_coord given point
//! \returns x component of 2D cell coordinates
size_t Grid::cellCoordX(const sf::Vector2f r_coord) const { return static_cast<size_t>(r_coord.x / cell_size_.x); }

//! \brief computes the 2D cell coordinates of cell containing the given point 
//! \param r_coord given point
//! \returns y component of 2D cell coordinates
size_t Grid::cellCoordY(const sf::Vector2f r_coord) const { return static_cast<size_t>(r_coord.y / cell_size_.y); }

//! \brief computes the 2D cell coordinates of cell containing the given point 
//! \param r_coord given point
//! \returns 2D cell coordinates
sf::Vector2i Grid::cellCoords(const sf::Vector2f r_coord) const {
    const auto cell_coord_x = static_cast<int>(r_coord.x / cell_size_.x);
    const auto cell_coord_y = static_cast<int>(r_coord.y / cell_size_.y);
    return {cell_coord_x, cell_coord_y};
}

//! \brief computes the 2D cell coordinates of cell containing the given point 
//! \param r_coord given point
//! \returns 2D cell coordinates
sf::Vector2i Grid::cellCoords(const sf::Vector2i r_coord) const {
    return cellCoords(static_cast<sf::Vector2f>(r_coord)); 
}


//! \param cell_coords 2D cell coordinates
//! \returns true if point is within grid bounds
bool SearchGrid::isInGrid(sf::Vector2i cell_coords) const {
    return cell_coords.x < n_cells_.x and cell_coords.x > 0 and cell_coords.x < n_cells_.y and cell_coords.y > 0;
}

Grid::Grid(sf::Vector2i n_cells, sf::Vector2f cell_size)
    : n_cells_(n_cells)
    , cell_size_(cell_size) {}


SearchGrid::SearchGrid(sf::Vector2i n_cells, sf::Vector2f cell_size)
    : Grid(n_cells, cell_size) {
    sf::Vector2f box_size;
    box_size.x = cell_size_.x * n_cells_.x;
    box_size.y = cell_size_.y * n_cells_.y;
}

//! \brief calculates indices of (up to) 9 closest cells EXCLUDING CENTER CELL, takes boundary into account, 
//! \param cell_ind
//! \param nearest_cells array containing cell_indices of closest cells (right now only looking at 9 nearest cells)
//! \param n_nearest_cells number of nearest cells
void SearchGrid::calcNearestCells(const size_t cell_ind, std::array<int, 9>& nearest_cells, int& n_nearest_cells) const {

    const auto cell_coords = cellCoords(cell_ind);
    n_nearest_cells = 0;
    for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
            auto neighbour_cell_ind = cell_ind + i + j * n_cells_.x;
            if (cell_coords.x + i >= 0 and cell_coords.x + i < n_cells_.x and cell_coords.y + j >= 0 and
                cell_coords.y + j < n_cells_.y and (neighbour_cell_ind != cell_ind)) {
                nearest_cells[n_nearest_cells] = neighbour_cell_ind;
                n_nearest_cells++;
            }
        }
    }
    assert(n_nearest_cells != 0);
}

//! \brief calculates cell indices of (up to) 9 closest cells EXCLUDING CENTER CELL, 
//! \brief will add only neighbour cell indices whose value is larger than \p cell_ind  
//! \brief (this is useful to prevent double counting)
//! \param cell_ind
//! \param nearest_cells array containing cell_indices of closest cells (right now only looking at 9 nearest cells)
//! \param n_nearest_cells number of nearest cells
void SearchGrid::calcNearestCells2(const size_t cell_ind, std::array<int, 9>& nearest_cells, int& n_nearest_cells) const {

    const auto cell_coords = cellCoords(cell_ind);
    n_nearest_cells = 0;
    for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
            auto neighbour_cell_ind = cell_ind + i + j * n_cells_.x;
            if (cell_coords.x + i >= 0 and cell_coords.x + i < n_cells_.x and cell_coords.y + j >= 0 and
                cell_coords.y + j < n_cells_.y and (neighbour_cell_ind > cell_ind)) {
                nearest_cells[n_nearest_cells] = neighbour_cell_ind;
                n_nearest_cells++;
            }
        }
    }
    assert(n_nearest_cells != 0); //! this would be silly
}

