#pragma once

#include "core.h"
#include "ECS.h"
#include "Grid.h"
#include <memory>
#include "NeighbourSearcherStrategy.h"

// struct InteractionData
// {
//     sf::Vector2f dr;
//     sf::Vector2f dv;
//     float r_collision;
//     float mass;
//     BoidInd second = -1;
//     MoveState state;
//     u_int8_t player_ind;
//     int n_neighbours;
// };

class SearchGrid;



// template <class CompType, int N_MAX_AGENTS_IN_CELL = 500>
// class NeighbourSearcherT
// {

//     std::vector<std::array<InteractionData, N_MAX_NEIGHBOURS>> particle2interaction_data_;
//     std::vector<std::array<CompType, N_MAX_NEIGHBOURS>> particle2components_;

//     std::shared_ptr<SearchGrid> s_grid_;

//     // static constexpr size_t N_MAX_AGENTS_IN_CELL = 500;

//     template <class CompType2>
//     struct ComponentAndInd
//     {
//         CompType2 comp;
//         int comp_ind;
//     };

//     std::vector<std::array<ComponentAndInd<CompType>, N_MAX_AGENTS_IN_CELL + 1>> cell2comp_and_comp_ind_;
//     std::vector<std::array<int, N_MAX_AGENTS_IN_CELL + 1>> cell2comp_ind_;

//     std::vector<int> active_cells_;
//     std::vector<bool> cell_visited_;

// public:
//     std::vector<int> last_i;

//     NeighbourSearcherT(sf::Vector2f box_size, float max_dist)
//     {

//         const sf::Vector2i n_cells = {static_cast<int>(box_size.x / max_dist) + 1,
//                                       static_cast<int>(box_size.y / max_dist) + 1};

//         cell2comp_ind_.resize(n_cells.x * n_cells.y);
//         cell2comp_and_comp_ind_.resize(n_cells.x * n_cells.y);
//         cell_visited_.resize(n_cells.x * n_cells.y, false);

//         for (auto &cell : cell2comp_and_comp_ind_)
//         {
//             cell.back().comp_ind = 0;
//         }

//         s_grid_ = std::make_shared<SearchGrid>(n_cells, sf::Vector2f{max_dist, max_dist});
//         particle2interaction_data_.resize(N_MAX_NAVIGABLE_BOIDS);
//         last_i.resize(N_MAX_NAVIGABLE_BOIDS, 0);
//     }

//     std::vector<int> getNeighboursIndsFull(sf::Vector2f r, const float r_max);

//     const std::array<InteractionData, N_MAX_NEIGHBOURS> &getInteractionData(const int boid_ind) const;

//     std::vector<int> &getNeighbourInds(int boid_ind, const float r_max2);

//     void update(const std::vector<CompType> &components);

// private:
//     void fillNeighbourData(const std::vector<CompType> &comps, float r_max2);

//     void addOnGrid(const std::vector<CompType> &components);
// };


// //! \brief sorts relevant data into corresponding cells and updates pair list for a current frame
// //! \param r_coords
// //! \param radii
// template <typename CompType, int N_MAX_AGENTS_IN_CELL>
// void NeighbourSearcherT<CompType, N_MAX_AGENTS_IN_CELL>::update(const std::vector<CompType> &components)
// {
//     addOnGrid(components);
//     fillNeighbourData(components, 20 * RHARD);
// }

// //! \brief sorts relevant data into corresponding cells (coord, rel_vel, radius, boid index)
// //! \param r_coords
// //! \param radii
// template <typename CompType, int N_MAX_AGENTS_IN_CELL>
// void NeighbourSearcherT<CompType, N_MAX_AGENTS_IN_CELL>::addOnGrid(const std::vector<CompType> &components)
// {

//     // visited.clear();
//     for (const auto cell_i_ind : active_cells_)
//     {
//         auto &cell_i_data = cell2comp_and_comp_ind_[cell_i_ind];
//         cell_i_data.back().comp_ind = 0;
//     }

//     const auto n_components = components.size();
//     active_cells_.resize(0);

//     for (int comp_ind = 0; comp_ind < n_components; ++comp_ind)
//     {
//         const auto &comp = components.at(comp_ind);
//         const auto &r = comp.transform.r;
//         const auto cell_ind = s_grid_->coordToCell(r);
//         auto &last_ind_in_cell = cell2comp_and_comp_ind_[cell_ind].back().comp_ind;

//         if (last_ind_in_cell < N_MAX_AGENTS_IN_CELL) [[likely]]
//         {
//             cell2comp_and_comp_ind_[cell_ind][last_ind_in_cell] = {comp, comp_ind};
//             cell2comp_ind_[cell_ind][last_ind_in_cell] = {comp_ind};
//             last_ind_in_cell++;
//         }
//         else
//         {
//             throw std::runtime_error("too many particles per cell!");
//         }

//         if (!cell_visited_[cell_ind])
//         {
//             active_cells_.push_back(cell_ind);
//             assert(last_ind_in_cell != 0);
//         }
//         cell_visited_[cell_ind] = true;
//         last_i.at(comp_ind) = 0;
//     }

//     const auto n_active_cells = active_cells_.size();
//     // #pragma omp parallel for
//     for (int active_cell_ind = 0; active_cell_ind < n_active_cells; ++active_cell_ind)
//     {
//         cell_visited_[active_cells_[active_cell_ind]] = false;
//     }
// }

// template <class CompType, int MAX_COMPONENTS>
// const std::array<InteractionData, N_MAX_NEIGHBOURS> &
// NeighbourSearcherT<CompType, MAX_COMPONENTS>::getInteractionData(const int comp_ind) const
// {
//     return particle2interaction_data_.at(comp_ind);
// }

// template <class CompType, int wtf>
// std::vector<int> NeighbourSearcherT<CompType, wtf>::getNeighboursIndsFull(sf::Vector2f r, const float r_max)
// {
//     std::vector<int> neighbour_inds;

//     const auto r_max2 = r_max * r_max;

//     int last_nearest_cell_ind = 8;
//     std::vector<int> nearest_cells;

//     const auto cell_ind = s_grid_->coordToCell(r);
//     const auto cell_coords = s_grid_->cellCoords(cell_ind);

//     int j_max = cell_coords.y + std::ceil(r_max / s_grid_->cell_size_.y);
//     int j_min = cell_coords.y - std::ceil(r_max / s_grid_->cell_size_.y);
//     j_max = std::min(j_max, s_grid_->n_cells_.y - 1);
//     j_min = std::max(j_min, 0);

//     for (int j = j_min; j <= j_max; ++j)
//     {
//         const float dy = (j - cell_coords.y) * s_grid_->cell_size_.y;
//         int i_max = cell_coords.x + std::ceil(std::sqrt(r_max2 - dy * dy));
//         int i_min = cell_coords.x - (i_max - cell_coords.x);
//         i_max = std::min(i_max, s_grid_->n_cells_.x - 1);
//         i_min = std::max(i_min, 0);

//         for (int i = i_min; i <= i_max; ++i)
//         {
//             nearest_cells.push_back(i + j * s_grid_->n_cells_.x);
//         }
//     }

//     for (const auto cell_ind_neighbour : nearest_cells)
//     {
//         const auto &cell_data = cell2comp_and_comp_ind_[cell_ind_neighbour];
//         const auto last_in_cell = cell_data.back().comp_ind;
//         for (int ind_in_cell = 0; ind_in_cell < last_in_cell; ++ind_in_cell)
//         {
//             const auto &neighbour_ind = cell_data[ind_in_cell].comp_ind;
//             const auto d2 = dist2(cell_data[ind_in_cell].comp.transform.r, r);
//             if (d2 <= r_max2)
//             {
//                 neighbour_inds.push_back(neighbour_ind);
//             }
//         }
//     }

//     return neighbour_inds;
// }