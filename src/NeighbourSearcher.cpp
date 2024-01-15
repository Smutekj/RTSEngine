#include <omp.h>
#include <memory>
#include <iostream>

#include "NeighbourSearcher.h"
#include "Grid.h"
#include "ECS.h"

// std::vector<int> NeighbourSearcher::getNeighboursIndsFull(sf::Vector2f r, const float r_max) {
//     std::vector<int> neighbour_inds;

//     const auto& r_coords = boids_->r_coords_;
//     const auto r_max2 = r_max*r_max;

//     int last_nearest_cell_ind = 8;
//     std::vector<int> nearest_cells;

//     const auto cell_ind = s_grid_->coordToCell(r);
//     const auto cell_coords = s_grid_->cellCoords(cell_ind);

//     int j_max = cell_coords.y + std::ceil(r_max/ s_grid_->cell_size_.y) ;
//     int j_min = cell_coords.y - std::ceil(r_max/ s_grid_->cell_size_.y) ;
//     j_max = std::min(j_max, s_grid_->n_cells_.y-1);
//     j_min = std::max(j_min, 0);

//     for(int j = j_min; j <= j_max; ++j){
//         const float dy = (j - cell_coords.y) * s_grid_->cell_size_.y;
//         int i_max = cell_coords.x + std::ceil(std::sqrt(r_max2 - dy*dy));
//         int i_min = cell_coords.x - (i_max - cell_coords.x);
//         i_max = std::min(i_max, s_grid_->n_cells_.x-1);
//         i_min = std::max(i_min, 0);

//         for(int i = i_min; i <= i_max; ++i){
//             nearest_cells.push_back(i + j*s_grid_->n_cells_.x);
//         }
//     }

//     for (const auto cell_ind_neighbour : nearest_cells) {
//         const auto& cell = cell2data_[cell_ind_neighbour];
//         const auto last_in_cell = cell.back().ind;
//         for (int i_cell = 0; i_cell < last_in_cell; ++i_cell) {
//             const auto& neighbour_ind = cell[i_cell].ind;
//             const auto d2 = dist2(cell[i_cell].r, r);
//             if (d2 <= r_max2) {
//                 neighbour_inds.push_back(neighbour_ind);
//             }
//         }
//     }

//     return neighbour_inds;
// }

// template <>
// void NeighbourSearcherT<PhysicsComponent>::fillNeighbourData(const std::vector<PhysicsComponent> &comps, float r_max2)
// {
//     std::vector<std::array<int, 9>> thread_id2nearest_cells(NUM_OMP_NS_THREADS);
//     const auto n_comps = comps.size();

// #pragma omp parallel num_threads(NUM_OMP_NS_THREADS)
//     {
//         const auto thread_id = omp_get_thread_num();
//         auto &nearest_cells = thread_id2nearest_cells.at(thread_id);
// #pragma omp for
//         for (int comp_ind = 0; comp_ind < n_comps; ++comp_ind)
//         {
//             const auto &comp = comps[comp_ind];
//             const auto r = comp.transform.r;
//             const auto v = comp.transform.vel;
//             const auto cell_i_ind = s_grid_->coordToCell(r);
//             int n_nearest_cells;
//             s_grid_->calcNearestCells(cell_i_ind, nearest_cells, n_nearest_cells);

//             nearest_cells[n_nearest_cells] = cell_i_ind;
//             n_nearest_cells++;
//             const auto &cell_data_i = cell2comp_and_comp_ind_[cell_i_ind];
//             const auto radius = comp.radius;

//             for (int cell_j_ind = 0; cell_j_ind < n_nearest_cells; ++cell_j_ind)
//             {
//                 const auto &cell_data_j = cell2comp_and_comp_ind_[nearest_cells[cell_j_ind]];
//                 const auto last_in_cell_j = cell2comp_and_comp_ind_[nearest_cells[cell_j_ind]].back().comp_ind;
//                 for (int j = 0; j < last_in_cell_j; ++j)
//                 {
//                     auto &[comp_j, comp_ind_j] = cell_data_j[j];
//                     const auto r_collision = comp_j.radius + radius;
//                     const auto dr = comp_j.transform.r - r;
//                     const auto v_rel = comp_j.transform.vel - v;
//                     const auto &mass = comp_j.mass;
//                     const auto &state = comp_j.state;
//                     const u_int8_t player = comp_j.player_ind;

//                     if (norm2(dr) < r_max2 && comp_ind != comp_ind_j)
//                     {
//                         particle2interaction_data_[comp_ind][last_i[comp_ind]] = {dr, v_rel, r_collision, mass, comp_ind_j, state, player, last_i[comp_ind] + 1};
//                         last_i[comp_ind] = std::min(N_MAX_NEIGHBOURS, last_i[comp_ind] + 1);
//                     }
//                 }
//             }
//             if (last_i[comp_ind] == N_MAX_NEIGHBOURS)
//             {
//                 throw std::runtime_error("too many neighbours on component: " + std::to_string(comp_ind) + " !");
//             }
//         }
//     }
// }

// template <>
// void NeighbourSearcherT<AttackComponent, 500>::fillNeighbourData(const std::vector<AttackComponent> &comps, float r_max2)
// {
//     std::vector<std::array<int, 9>> thread_id2nearest_cells(NUM_OMP_NS_THREADS);
//     const auto n_comps = comps.size();

// #pragma omp parallel num_threads(NUM_OMP_NS_THREADS)
//     {
//         const auto thread_id = omp_get_thread_num();
//         auto &nearest_cells = thread_id2nearest_cells.at(thread_id);
// #pragma omp for
//         for (int comp_ind = 0; comp_ind < n_comps; ++comp_ind)
//         {
//             const auto &comp = comps[comp_ind];
//             const auto r = comp.transform.r;
//             const auto v = comp.transform.vel;
//             const auto cell_i_ind = s_grid_->coordToCell(r);
//             int n_nearest_cells;
//             s_grid_->calcNearestCells(cell_i_ind, nearest_cells, n_nearest_cells);

//             nearest_cells[n_nearest_cells] = cell_i_ind;
//             n_nearest_cells++;
//             const auto &cell_data_i = cell2comp_and_comp_ind_[cell_i_ind];

//             float min_enemy_dist_sq = static_cast<float>(Geometry::BOX[0]);
//             for (int cell_j_ind = 0; cell_j_ind < n_nearest_cells; ++cell_j_ind)
//             {
//                 const auto &cell_data_j = cell2comp_and_comp_ind_[nearest_cells[cell_j_ind]];
//                 const auto last_in_cell_j = cell2comp_and_comp_ind_[nearest_cells[cell_j_ind]].back().comp_ind;
//                 for (int j = 0; j < last_in_cell_j; ++j)
//                 {
//                     auto &[comp_j, comp_ind_j] = cell_data_j[j];
//                     const auto dist_sq = dist2(comp_j.transform.r, r);

//                     if (dist_sq < min_enemy_dist_sq && (comp.player_ind != comp_j.player_ind))
//                     {
//                         particle2interaction_data_[comp_ind][0].second = comp_ind_j;
//                         last_i[comp_ind] = 1;
//                     }
//                 }
//             }
//             if (last_i[comp_ind] == N_MAX_NEIGHBOURS)
//             {
//                 throw std::runtime_error("too many neighbours on component: " + std::to_string(comp_ind) + " !");
//             }
//         }
//     }
// }

// template <>
// void NeighbourSearcherT<PathFinderComponent, 500>::fillNeighbourData(const std::vector<PathFinderComponent> &comps, float r_max2)
// {

//     std::vector<std::array<int, 9>> thread_id2nearest_cells(NUM_OMP_NS_THREADS);
//     const auto n_comps = comps.size();
//     std::cout << "hi";
// #pragma omp parallel num_threads(1)
//     {
//         const auto thread_id = omp_get_thread_num();
//         auto &nearest_cells = thread_id2nearest_cells.at(thread_id);
// #pragma omp for
//         for (int comp_ind = 0; comp_ind < n_comps; ++comp_ind)
//         {
//             const auto &comp = comps[comp_ind];
//             const auto r = comp.transform.r;
//             const auto v = comp.transform.vel;
//             const auto cell_i_ind = s_grid_->coordToCell(r);
//             int n_nearest_cells;
//             s_grid_->calcNearestCells(cell_i_ind, nearest_cells, n_nearest_cells);

//             nearest_cells[n_nearest_cells] = cell_i_ind;
//             n_nearest_cells++;
//             const auto &cell_data_i = cell2comp_and_comp_ind_[cell_i_ind];

//             float min_enemy_dist_sq = static_cast<float>(Geometry::BOX[0]);
//             for (int cell_j_ind = 0; cell_j_ind < n_nearest_cells; ++cell_j_ind)
//             {
//                 const auto &cell_data_j = cell2comp_and_comp_ind_[nearest_cells[cell_j_ind]];
//                 const auto last_in_cell_j = cell2comp_and_comp_ind_[nearest_cells[cell_j_ind]].back().comp_ind;
//                 for (int j = 0; j < last_in_cell_j; ++j)
//                 {
//                     auto &[comp_j, comp_ind_j] = cell_data_j[j];
//                     const auto dist_sq = dist2(comp_j.transform.r, r);

//                     if (dist_sq < r_max2 && comp_ind_j != comp_ind)
//                     {
//                         particle2interaction_data_[comp_ind][last_i[comp_ind]].second = comp_ind_j;
//                         particle2interaction_data_[comp_ind][last_i[comp_ind]].dr = -(comp.transform.r - comp_j.transform.r);
//                         particle2interaction_data_[comp_ind][last_i[comp_ind]].state = comp_j.state;
//                         last_i[comp_ind] = std::min(N_MAX_NEIGHBOURS, last_i[comp_ind] + 1);
//                     }
//                 }
//             }
//             if (last_i[comp_ind] == N_MAX_NEIGHBOURS)
//             {
//                 throw std::runtime_error("too many neighbours on component: " + std::to_string(comp_ind) + " !");
//             }
//         }
//     }
// }
