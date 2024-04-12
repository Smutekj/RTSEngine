#pragma once
#include <stdexcept>
#include <vector>
#include <unordered_set>
#include <memory>
#include <chrono>
#include "../core.h"

class Grid {

  public:
    sf::Vector2i n_cells_;
    sf::Vector2f cell_size_ = {0, 0};

  public:
    Grid(sf::Vector2i n_cells, sf::Vector2f cell_size);

    [[nodiscard]] size_t coordToCell(float x, float y) const;
    [[nodiscard]] size_t coordToCell(sf::Vector2f r) const;
    [[nodiscard]] size_t cellIndex(int ix, int iy) const;
    [[nodiscard]] size_t cellIndex(sf::Vector2i) const;

    [[nodiscard]] size_t cellCoordX(size_t cell_index) const;
    [[nodiscard]] size_t cellCoordY(size_t cell_index) const;

    [[nodiscard]] size_t cellCoordX(sf::Vector2f r_coord) const;
    [[nodiscard]] size_t cellCoordY(sf::Vector2f r_coord) const;

    [[nodiscard]] sf::Vector2i cellCoords(sf::Vector2f r_coord) const;
    [[nodiscard]] sf::Vector2i cellCoords(sf::Vector2i r_coord) const;
    [[nodiscard]] sf::Vector2i cellCoords(size_t cell_index) const;
};

//! \class represents grids that are used for searching for nearest neighbours
struct SearchGrid : Grid {

    void calcNearestCells(const size_t cell_ind, std::array<int, 9>& nearest_neighbours, int& n_nearest_cells) const;
    void calcNearestCells2(const size_t cell_ind, std::array<int, 9>& nearest_neighbours, int& n_nearest_cells) const;

    SearchGrid(sf::Vector2i n_cells, sf::Vector2f cell_size);
    bool isInGrid(sf::Vector2i cell_coords) const;
};
