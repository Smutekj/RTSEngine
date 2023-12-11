#include <omp.h>
#include <memory>
#include <iostream>

#include "NeighbourSearcher.h"
#include "Grid.h"

NeighbourSearcher::NeighbourSearcher(sf::Vector2f box_size, float max_dist, BoidWorld& world) {
    boids_ = &world;
    const sf::Vector2i n_cells = {static_cast<int>(box_size.x / max_dist) + 1,
                                  static_cast<int>(box_size.y / max_dist) + 1};
    cell2boid_inds_.resize(n_cells.x * n_cells.y);
    cell2boid_inds2_.resize(n_cells.x * n_cells.y);
    cell_visited_.resize(n_cells.x * n_cells.y, false);
    for (auto& cell : cell2boid_inds_) {
        cell.back() = 0;
    }
    
    s_grid_ = std::make_shared<SearchGrid>(n_cells, sf::Vector2f{max_dist, max_dist});
    particle2verlet_list.resize(N_MAX_NAVIGABLE_BOIDS);
    particle2neighbour_data_.resize(N_MAX_NAVIGABLE_BOIDS);
    last_i.resize(N_MAX_NAVIGABLE_BOIDS, 0);

    omp_set_num_threads(2+0*omp_get_max_threads());

}

std::vector<int> NeighbourSearcher::getNeighboursInds(sf::Vector2f r, const float r_max2) {
    std::vector<int> neighbour_inds;
    neighbour_inds.reserve(30);

    const auto& r_coords = boids_->r_coords_;

    int last_nearest_cell_ind = 8;
    std::array<int, 9> nearest_cells;

    const auto cell_ind = s_grid_->coordToCell(r);
    s_grid_->calcNearestCells(cell_ind, nearest_cells, last_nearest_cell_ind);
    nearest_cells[last_nearest_cell_ind] = cell_ind;
    last_nearest_cell_ind++;
    for (int i = 0; i < last_nearest_cell_ind; ++i) {
        const auto cell_ind_neighbour = nearest_cells[i];
        const auto& cell = cell2boid_inds2_[cell_ind_neighbour];
        const auto last_in_cell = cell.back().ind;
        for (int i_cell = 0; i_cell < last_in_cell; ++i_cell) {
            const auto& neighbour_ind = cell[i_cell].ind;
            const auto d2 = dist2(cell[i_cell].r, r);
            if (d2 <= r_max2) {
                neighbour_inds.push_back(neighbour_ind);
            }
        }
    }

    return neighbour_inds;
}

std::vector<int>& NeighbourSearcher::getNeighbourInds(const int boid_ind, const float r_max2) {
    return particle2verlet_list.at(boid_ind);
}

const std::array<NeighbourSearcher::NeighbourData, 500>& NeighbourSearcher::getNeighbourData(const int boid_ind,
                                                                                             const float r_max2) const {
    return particle2neighbour_data_.at(boid_ind);
}

// void NeighbourSearcher::fillVerletLists(const std::vector<sf::Vector2f>& r_coords, const std::vector<float>& radii,
//                                         const float r_max2) {

//     // for (const auto boid_ind : boids_->active_inds) {
//     //     auto& verlet_list = particle2verlet_list.at(boid_ind);
//     //     verlet_list.clear();
//     //     std::vector<float> distances2;

//     //     const auto r = r_coords[boid_ind];
//     //     const int cell_index_center = s_grid_->coordToCell(r);
//     //     const auto& neighbouring_cell_inds = s_grid_->calcNearestCells(cell_index_center);
//     //     for (auto cell_ind_neighbour : neighbouring_cell_inds) {
//     //         const auto& cell = cell2boid_inds_[cell_ind_neighbour];
//     //         const auto last_in_cell = cell.back();
//     //         for (int i = 0; i < last_in_cell; ++i) {
//     //             const auto& neighbour_ind = cell[i];
//     //             const auto d2 = dist2(r_coords[neighbour_ind], r);
//     //             if (d2 <= r_max2 and boid_ind != neighbour_ind) {
//     //                 verlet_list.push_back(neighbour_ind);
//     //                 distances2.push_back(d2);
//     //             }
//     //         }
//     //     }
//     // }
//     pair_list_.clear();
//     std::array<int, 9> nearest_cells;
//     for (const auto cell_i_ind : active_cells_) {
//         const auto& cell_i = cell2boid_inds_[cell_i_ind];
//         int n_nearest_cells;
//         s_grid_->calcNearestCells(cell_i_ind, nearest_cells, n_nearest_cells);
//         for (int cell_j_ind = 0; cell_j_ind < n_nearest_cells; ++cell_j_ind) {
//             const auto& cell_j = cell2boid_inds_[nearest_cells[cell_j_ind]];

//             const auto last_in_cell_i = cell_i.back();
//             const auto last_in_cell_j = cell_j.back();
//             for (int i = 0; i < last_in_cell_i; ++i) {
//                 for (int j = 0; j < last_in_cell_j; ++j) {
//                     const auto r_collision = radii[cell_j[j]] + radii[cell_i[i]];
//                     const auto dr = r_coords[cell_j[j]] - r_coords[cell_i[i]];
//                     pair_list_.emplace_back(dr, cell_i[i], cell_j[j], r_collision);
//                 }
//             }
//         }
//     }
//     for (const auto cell_i_ind : active_cells_) {
//         cell2boid_inds_[cell_i_ind].back() = 0;
//     }

//     const auto n_pairs = pair_list_.size();
//     int last = n_pairs - 1;
//     for (int i = 0; i < n_pairs and i < last; i++) {
//         const auto& pair_data = pair_list_[i];
//         if (norm2(pair_data.dr) > r_max2 or pair_data.first == pair_data.second) {
//             pair_list_[i] = pair_list_[last];
//             i--;
//             last--;
//         }
//     }
//     for (int i = 0; i < last; ++i) {
//         particle2verlet_list[pair_list_[i].first].push_back(pair_list_[i].second);
//     }
// }

// void fillPairList(sf::Vector2f* r_coords, float* radii, la) {
//     for (int i = 0; i < last_in_cell_i; ++i) {
//         for (int j = 0; j < last_in_cell_j; ++j) {
//             const auto r_collision = radii[cell_j[j]] + radii[cell_i[i]];
//             const auto dr = r_coords[cell_j[j]] - r_coords[cell_i[i]];
//             if (norm2(dr) < r_max2) {
//                 pair_list_.emplace_back(dr, cell_i[i], cell_j[j], r_collision);
//             }
//         }
//     }
// }

void NeighbourSearcher::fillNeighbourData2(const std::vector<sf::Vector2f>& r_coords, const std::vector<float>& radii,
                                           const float r_max2) {
    std::vector<std::array<int, 9>> thread_id2nearest_cells(6);
    const auto n_boids = r_coords.size();

#pragma omp parallel num_threads(6)
{
    const auto thread_id = omp_get_thread_num();
    auto& nearest_cells = thread_id2nearest_cells.at(thread_id);
    #pragma omp for
    for (int i = 0; i < n_boids; ++i) {
        const auto r = r_coords[i];
        const auto cell_i_ind = s_grid_->coordToCell(r);
        int n_nearest_cells;
        s_grid_->calcNearestCells(cell_i_ind, nearest_cells, n_nearest_cells);
        nearest_cells[n_nearest_cells] = cell_i_ind;
        n_nearest_cells++;
        const auto& cell_data_i = cell2boid_inds2_[cell_i_ind];
        const auto radius = radii[i];

        for (int cell_j_ind = 0; cell_j_ind < n_nearest_cells; ++cell_j_ind) {
            const auto& cell_data_j = cell2boid_inds2_[nearest_cells[cell_j_ind]];
            const auto last_in_cell_j = cell_data_j.back().ind;
            for (int j = 0; j < last_in_cell_j; ++j) {
                const auto ind = cell_data_j[j].ind;
                const auto r_collision = cell_data_j[j].radius + radius;
                const auto dr = cell_data_j[j].r - r;
                if (norm2(dr) < r_max2 && i != ind) {
                    particle2neighbour_data_[i][last_i[i]] = {dr, r_collision, ind};
                    last_i[i] = std::min(300 - 1, last_i[i] + 1);
                }
            }
        }
    }
}
}

static unsigned long long n = 0;
static float avg_time1 = 0;
static float avg_time2 = 0;


//! \brief sorts relevant data into corresponding cells (coordinate, radius, boid index (later maybe move state?))
//! \param r_coords
//! \param radii 
void NeighbourSearcher::addOnGrid(const std::vector<sf::Vector2f>& r_coords, const std::vector<float>& radii) {

    // visited.clear();
    for (const auto cell_i_ind : active_cells_) {
        // auto& cell_i = cell2boid_inds_[cell_i_ind];
        auto& cell_i_data = cell2boid_inds2_[cell_i_ind];
        // cell_i.back() = 0;
        cell_i_data.back().ind = 0;
    }

    const auto n_boids = boids_->active_inds.size();
    active_cells_.reserve(n_boids);
    active_cells_.resize(0);

    for (int boid_ind = 0; boid_ind < n_boids; ++boid_ind) {
        const auto cell_ind = s_grid_->coordToCell(boids_->r_coords_[boid_ind]);
        // auto& last_ind_in_cell = cell2boid_inds_[cell_ind].back();
        auto& last_ind_in_cell = cell2boid_inds2_[cell_ind].back().ind;
        // assert(last_ind_in_cell < NMAX);
        // cell2boid_inds_[cell_ind][last_ind_in_cell] = (boid_ind);
        if (last_ind_in_cell < N_MAX_AGENTS_IN_CELL) {
            cell2boid_inds2_[cell_ind][last_ind_in_cell] = {r_coords[boid_ind], radii[boid_ind], boid_ind};
            last_ind_in_cell++;
        } else {
            // std::cout << "too many particles per cell!";
        }

        if (!cell_visited_[cell_ind]) {
            active_cells_.push_back(cell_ind);
            assert(last_ind_in_cell != 0);
        }
        cell_visited_[cell_ind] = true;
        last_i[boid_ind] = 0;
    }

    const auto n_active_cells = active_cells_.size();
    // #pragma omp parallel for
    for (int active_cell_ind = 0; active_cell_ind < n_active_cells; ++active_cell_ind) {
        cell_visited_[active_cells_[active_cell_ind]] = false;
    }
}

void NeighbourSearcher::addOnGrid(const std::vector<sf::Vector2f>& r_coords, const std::vector<float>& radii,
                                  const std::vector<int>& selection) {

    // visited.clear();
    for (const auto cell_i_ind : active_cells_) {
        // auto& cell_i = cell2boid_inds_[cell_i_ind];
        auto& cell_i_data = cell2boid_inds2_[cell_i_ind];
        // cell_i.back() = 0;
        cell_i_data.back().ind = 0;
    }

    const auto n_boids = boids_->active_inds.size();
    active_cells_.reserve(n_boids);
    active_cells_.resize(0);

    for (const auto boid_ind : selection) {
        const auto cell_ind = s_grid_->coordToCell(boids_->r_coords_[boid_ind]);
        // auto& last_ind_in_cell = cell2boid_inds_[cell_ind].back();
        auto& last_ind_in_cell = cell2boid_inds2_[cell_ind].back().ind;
        // assert(last_ind_in_cell < NMAX);
        // cell2boid_inds_[cell_ind][last_ind_in_cell] = (boid_ind);
        cell2boid_inds2_[cell_ind][last_ind_in_cell] = {r_coords[boid_ind], radii[boid_ind], boid_ind};
        last_ind_in_cell++;
        if (!cell_visited_[cell_ind] and last_ind_in_cell < N_MAX_AGENTS_IN_CELL) {
            active_cells_.push_back(cell_ind);
        }
        cell_visited_[cell_ind] = true;
        last_i[boid_ind] = 0;
    }

    const auto n_active_cells = active_cells_.size();
    for (int active_cell_ind = 0; active_cell_ind < n_active_cells; ++active_cell_ind) {
        cell_visited_[active_cells_[active_cell_ind]] = false;
    }
}

//! \brief sorts relevant data into corresponding cells and updates pair list for a current frame
//! \param r_coords
//! \param radii 
void NeighbourSearcher::update(const std::vector<sf::Vector2f>& r_coords, const std::vector<float>& radii) {


    const auto start1 = std::chrono::high_resolution_clock::now();
    addOnGrid(r_coords, radii);
    const auto time1 =
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start1);
    avg_time1 = (avg_time1 * n + time1.count()) / (n + 1);

    const auto start2 = std::chrono::high_resolution_clock::now();
    fillNeighbourData2(boids_->r_coords_, radii, RHARD * RHARD * (100));
    // fillNeighbourDataBalanced(boids_->r_coords_, radii, RHARD * RHARD * (400));

    const auto time2 =
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start2);

    avg_time2 = (avg_time2 * n + time2.count()) / (n + 1);
    n++;
    if (n % (60 * 5) == 0) {
        std::cout << "filling neighbour data takes: " << avg_time2 << " us on average\n" << std::flush;
        std::cout << "binning particles into grid takes: " << avg_time1 << " us on average\n" << std::flush;
        n = 0;
        avg_time2 = 0;
        avg_time1 = 0;
    }
}


//! \brief 
void NeighbourSearcher::fillPairListFromCells(ParticlePairData* __restrict__ pair_list, const int last_in_cell_i,
                        const int last_in_cell_j,  const CellData* __restrict__  cell_i, const CellData*  __restrict__  cell_j, const float r_max2, const int last_ind) {

    for (int i = 0; i < last_in_cell_i; ++i) {
        auto r_collision = cell_i[i].radius;
        auto r_i = cell_i[i].r;
        const int boid_ind_i = cell_i[i].ind;
        const auto last_i = last_ind + i*last_in_cell_j;
// #pragma clang loop vectorize(enable)
#pragma simd
        for (int j = 0; j < last_in_cell_j; ++j) {            
            pair_list[last_i + j].dr = cell_j[j].r - r_i;
        }
// #pragma clang loop vectorize(enable)
#pragma simd
        for (int j = 0; j < last_in_cell_j; ++j) {            
            const int boid_ind_j = cell_j[j].ind; 
            pair_list[last_i + j].first = boid_ind_i;
            pair_list[last_i + j].second = boid_ind_j;
        }
#pragma simd
        for (int j = 0; j < last_in_cell_j; ++j) {            
            pair_list[last_i + j].r_collision = r_collision + cell_j[j].radius;
        }
    }
}

void NeighbourSearcher::fillPairListFromCells2(std::pair<sf::Vector2f, u_int64_t>* __restrict__ pair_list, const int last_in_cell_i,
                        const int last_in_cell_j,  const CellData* __restrict__  cell_i, const CellData*  __restrict__  cell_j, const float r_max2, const int last_ind) {

    for (int i = 0; i < last_in_cell_i; ++i) {
        auto r_collision = cell_i[i].radius;
        auto dr = cell_i[i].r;
        const int boid_ind_i = cell_i[i].ind;
        const auto last_i = last_ind + i*last_in_cell_j;
#pragma clang loop vectorize(enable)
        for (int j = 0; j < last_in_cell_j; ++j) {            
            const int boid_ind_j = cell_j[j].ind; 
            pair_list[last_i + j].first = dr;
            pair_list[last_i + j].second = boid_ind_i;
        }
    }
}


//! \brief updates pair list for a current frame using data in cells
//! \brief each omp thread has its own pair list and we do not make any attempt at balancing them here
//! \param r_coords
//! \param radii 
//! \param r_max2 maximum distance squared
void NeighbourSearcher::fillNeighbourDataSIMD(const std::vector<sf::Vector2f>& r_coords, const std::vector<float>& radii,
                                          const float r_max2) {

    std::sort(active_cells_.begin(), active_cells_.end());

    const auto n_active_cells = active_cells_.size();

    std::array<std::array<int, 9>, 12> nearest_cells2;

//! tested it and there seems to be no improvement above 2 threads :(
//! I thi: the number of agents is too small (tested max 4000)

#pragma omp_parallel num_threads(2)
{
        const auto thread_id = omp_get_thread_num();
        pair_lists_[thread_id].clear();

    #pragma omp for schedule (dynamic, 20)
    for (int active_cell_ind = 0; active_cell_ind < n_active_cells; ++active_cell_ind) {
        const auto cell_i_ind = active_cells_[active_cell_ind];
        const auto& cell_i = cell2boid_inds2_[cell_i_ind];
        int n_nearest_cells;
        auto& nearest_cells = nearest_cells2[thread_id];
        s_grid_->calcNearestCells2(cell_i_ind, nearest_cells, n_nearest_cells);
        for (auto nearest_cell_it = nearest_cells.begin(); nearest_cell_it < nearest_cells.begin() + n_nearest_cells;
             ++nearest_cell_it) {
            const auto cell_j_ind = *nearest_cell_it;
            // if (cell_j_ind <= cell_i_ind) {
            //     continue;
            // }

            const auto& cell_j = cell2boid_inds2_[cell_j_ind];

            const auto last_in_cell_i = cell_i.back().ind;
            const auto last_in_cell_j = cell_j.back().ind;

            const auto n_pairs = pair_lists_[omp_get_thread_num()].size();
            pair_lists_[omp_get_thread_num()].resize(n_pairs + last_in_cell_i*last_in_cell_j);
            fillPairListFromCells(pair_lists_[omp_get_thread_num()].data(), last_in_cell_i, last_in_cell_j,
             cell_i.data(), cell_j.data(), r_max2, n_pairs);

        }
    }

    // #pragma omp for schedule (dynamic, 20) nowait
    // for (int active_cell_ind = 0; active_cell_ind < n_active_cells; ++active_cell_ind) {
    //     const auto cell_i_ind = active_cells_[active_cell_ind];
    //     const auto& cell_i = cell2boid_inds2_[cell_i_ind];
    //     const auto last_in_cell_i = cell_i.back().ind;
    //     for (int i = 0; i < last_in_cell_i; ++i) {
    //         for (int j = i + 1; j < last_in_cell_i; ++j) {
    //             const auto dr = cell_i[j].r - cell_i[i].r;
    //             const auto r_collision = cell_i[j].radius + cell_i[i].radius;
    //             pair_lists_[thread_id].emplace_back(dr, cell_i[i].ind, cell_i[j].ind, r_collision);
    //         }
    //     }
    // }
}

}


//! \brief updates pair list for a current frame using data in cells
//! \brief each omp thread has its own pair list and we do not make any attempt at balancing them here
//! \param r_coords
//! \param radii 
//! \param r_max2 maximum distance squared
void NeighbourSearcher::fillNeighbourData(const std::vector<sf::Vector2f>& r_coords, const std::vector<float>& radii,
                                          const float r_max2) {

    pair_list_.clear();
    std::sort(active_cells_.begin(), active_cells_.end());

    const auto n_active_cells = active_cells_.size();

    std::array<std::array<int, 9>, 12> nearest_cells2;

//! tested it and there seems to be no improvement above 2 threads :(
//! I thi: the number of agents is too small (tested max 4000)

#pragma omp_parallel num_threads(2)
{
        const auto thread_id = omp_get_thread_num();
        pair_lists_[thread_id].clear();

    #pragma omp for schedule (dynamic, 20)
    for (int active_cell_ind = 0; active_cell_ind < n_active_cells; ++active_cell_ind) {
        const auto cell_i_ind = active_cells_[active_cell_ind];
        const auto& cell_i = cell2boid_inds2_[cell_i_ind];
        int n_nearest_cells;
        auto& nearest_cells = nearest_cells2[thread_id];
        s_grid_->calcNearestCells2(cell_i_ind, nearest_cells, n_nearest_cells);
        for (auto nearest_cell_it = nearest_cells.begin(); nearest_cell_it < nearest_cells.begin() + n_nearest_cells;
             ++nearest_cell_it) {
            const auto cell_j_ind = *nearest_cell_it;
            // if (cell_j_ind <= cell_i_ind) {
            //     continue;
            // }

            const auto& cell_j = cell2boid_inds2_[cell_j_ind];

            const auto last_in_cell_i = cell_i.back().ind;
            const auto last_in_cell_j = cell_j.back().ind;

            // const auto n_pairs = pair_lists_[omp_get_thread_num()].size();
            // pair_lists_[omp_get_thread_num()].resize(n_pairs + last_in_cell_i*last_in_cell_j);
            // fillPairListFromCells(pair_lists_[omp_get_thread_num()].data(), last_in_cell_i, last_in_cell_j,
            //  cell_i.data(), cell_j.data(), r_max2, n_pairs);

            for (int i = 0; i < last_in_cell_i; ++i) {
                const auto r_collision = cell_i[i].radius;
                const auto r_i = cell_i[i].r;
                const auto boid_ind_i = cell_i[i].ind;
                for (int j = 0; j < last_in_cell_j; ++j) {            
                    const auto boid_ind_j = cell_j[j].ind;
                    const auto dr = cell_j[j].r - r_i;
                    // if (norm2(dr) < r_max2) {
                        pair_lists_[thread_id].emplace_back(dr, boid_ind_i, boid_ind_j, r_collision + cell_j[j].radius);
                    // }
                }
                //     for (int j = 0; j < last_in_cell_j; ++j) {
                //     const auto r_collision = cell_j[j].radius + cell_i[i].radius;
                //     const auto dr = cell_j[j].r - cell_i[i].r;
                //     if (norm2(dr) < r_max2) {
                //         pair_lists_[omp_get_thread_num()].emplace_back(dr, cell_i[i].ind, cell_j[j].ind, r_collision);
                //     }
                // }
            }
        }
    }

    #pragma omp for schedule (dynamic, 20) nowait
    for (int active_cell_ind = 0; active_cell_ind < n_active_cells; ++active_cell_ind) {
        const auto cell_i_ind = active_cells_[active_cell_ind];
        const auto& cell_i = cell2boid_inds2_[cell_i_ind];
        const auto last_in_cell_i = cell_i.back().ind;
        for (int i = 0; i < last_in_cell_i; ++i) {
            for (int j = i + 1; j < last_in_cell_i; ++j) {
                const auto dr = cell_i[j].r - cell_i[i].r;
                const auto r_collision = cell_i[j].radius + cell_i[i].radius;
                pair_lists_[thread_id].emplace_back(dr, cell_i[i].ind, cell_i[j].ind, r_collision);
            }
        }
    }
}

}

void NeighbourSearcher::fillNeighbourData3(const std::vector<sf::Vector2f>& r_coords, const std::vector<float>& radii,
                                          const float r_max2) {

    pair_list_.clear();
    std::sort(active_cells_.begin(), active_cells_.end());

    const auto n_active_cells = active_cells_.size();

    std::array<std::array<int, 9>, 12> nearest_cells2;

    for (int thread_id = 0; thread_id < omp_get_max_threads(); ++thread_id) {
        pair_lists_[thread_id].clear();
    }

    for (int active_cell_ind = 0; active_cell_ind < n_active_cells; ++active_cell_ind) {
        const auto cell_i_ind = active_cells_[active_cell_ind];
        const auto& cell_i = cell2boid_inds2_[cell_i_ind];
        int n_nearest_cells;
        auto& nearest_cells = nearest_cells2[omp_get_thread_num()];
        s_grid_->calcNearestCells2(cell_i_ind, nearest_cells, n_nearest_cells);
        for (auto nearest_cell_it = nearest_cells.begin(); nearest_cell_it < nearest_cells.begin() + n_nearest_cells;
             ++nearest_cell_it) {
            const auto cell_j_ind = *nearest_cell_it;
            // if (cell_j_ind <= cell_i_ind) {
            //     continue;
            // }

            const auto& cell_j = cell2boid_inds2_[cell_j_ind];

            const auto last_in_cell_i = cell_i.back().ind;
            const auto last_in_cell_j = cell_j.back().ind;

            // const auto n_pairs = pair_lists_[omp_get_thread_num()].size();
            // pair_lists_[omp_get_thread_num()].resize(n_pairs + last_in_cell_i*last_in_cell_j);
            // fillPairListFromCells(pair_lists_[omp_get_thread_num()].data(), last_in_cell_i, last_in_cell_j,
            //  cell_i.data(), cell_j.data(), r_max2, n_pairs);

#pragma omp parallel for
            for (int i = 0; i < last_in_cell_i; ++i) {
                auto r_collision = cell_i[i].radius;
                auto dr = cell_i[i].r;
                const auto boid_ind_i = cell_i[i].ind;
                for (int j = 0; j < last_in_cell_j; ++j) {            
                    const auto boid_ind_j = cell_i[j].ind;
                    if (norm2(dr) < r_max2 && (boid_ind_i != boid_ind_j)) {
                        pair_lists_[omp_get_thread_num()].emplace_back(dr, boid_ind_i, boid_ind_j, r_collision + cell_j[j].radius);
                    }
                }
           }
        }
    }

#pragma omp parallel for schedule (guided, 3)
    for (int active_cell_ind = 0; active_cell_ind < n_active_cells; ++active_cell_ind) {
        const auto cell_i_ind = active_cells_[active_cell_ind];
        const auto& cell_i = cell2boid_inds2_[cell_i_ind];
        const auto last_in_cell_i = cell_i.back().ind;
        for (int i = 0; i < last_in_cell_i; ++i) {
            for (int j = i + 1; j < last_in_cell_i; ++j) {
                const auto dr = cell_i[j].r - cell_i[i].r;
                const auto r_collision = cell_i[j].radius + cell_i[i].radius;
                pair_lists_[omp_get_thread_num()].emplace_back(dr, cell_i[i].ind, cell_i[j].ind, r_collision);
            }
        }
    }
}




//! \brief updates pair list for a current frame using data in cells
//! \brief each omp thread has its own pair list and we balance the number of pairs on each thread
//! \param r_coords
//! \param radii 
//! \param r_max2 maximum distance squared
void NeighbourSearcher::fillNeighbourDataBalanced(const std::vector<sf::Vector2f>& r_coords, const std::vector<float>& radii,
                                          const float r_max2) {

    pair_list_.clear();
    std::sort(active_cells_.begin(), active_cells_.end());

    const auto n_active_cells = active_cells_.size();

    std::array<std::array<int, 9>, 12> nearest_cells2;

    for (int thread_id = 0; thread_id < omp_get_max_threads(); ++thread_id) {
        pair_lists_[thread_id].clear();
    }

    cell_pair_list_.resize(0);
    size_t n_pairs_total = 0;

// #pragma omp parallel for schedule (guided, 3)
    for (int active_cell_ind = 0; active_cell_ind < n_active_cells; ++active_cell_ind) {
        const auto cell_i_ind = active_cells_[active_cell_ind];
        const auto& cell_i = cell2boid_inds2_[cell_i_ind];
        int n_nearest_cells;
        auto& nearest_cells = nearest_cells2[omp_get_thread_num()];
        s_grid_->calcNearestCells2(cell_i_ind, nearest_cells, n_nearest_cells);
        for (auto nearest_cell_it = nearest_cells.begin(); nearest_cell_it < nearest_cells.begin() + n_nearest_cells;
             ++nearest_cell_it) {
            const auto cell_j_ind = *nearest_cell_it;
            const auto& cell_j = cell2boid_inds2_[cell_j_ind];

            const auto last_in_cell_i = cell_i.back().ind;
            const auto last_in_cell_j = cell_j.back().ind;

            n_pairs_total += last_in_cell_i*last_in_cell_j;
            if(last_in_cell_j != 0){
                cell_pair_list_.push_back({cell_i_ind, cell_j_ind, last_in_cell_i*last_in_cell_j});
            }
        }
    }


// #pragma omp parallel for schedule (guided, 3)
    for (int active_cell_ind = 0; active_cell_ind < n_active_cells; ++active_cell_ind) {
        const auto cell_i_ind = active_cells_[active_cell_ind];
        const auto& cell_i = cell2boid_inds2_[cell_i_ind];
        const auto last_in_cell_i = cell_i.back().ind;
        cell_pair_list_.push_back({cell_i_ind, cell_i_ind, last_in_cell_i*(last_in_cell_i-1)/2});
    }

//! here we attempt to balance the number of pairs in pair_list for each thread
    thread2n_cell_pairs_.resize(omp_get_max_threads(), 0);
    thread2first_cell_pair_ind_.resize(omp_get_max_threads(), 0);
    const auto n_cell_pairs = cell_pair_list_.size();
    const auto n_pairs_per_thread = n_pairs_total/omp_get_max_threads();

    int next_thread_id = 0;
    size_t n_pairs = 0; 
    size_t n_cell_pairs_in_thread = 0; 
    for (int cell_pair_ind = 0; cell_pair_ind < n_cell_pairs; ++cell_pair_ind){
        n_pairs += cell_pair_list_[cell_pair_ind].n_pairs;
        n_cell_pairs_in_thread++;
        if(n_pairs > n_pairs_per_thread){
            thread2n_cell_pairs_[next_thread_id] = n_cell_pairs_in_thread;
            next_thread_id++;
            thread2first_cell_pair_ind_[next_thread_id] = cell_pair_ind;
            n_pairs = 0;
            n_cell_pairs_in_thread = 0;
        }
    }

#pragma omp parallel
{
    const auto first_cell_pair_ind = thread2first_cell_pair_ind_[omp_get_thread_num()];
    const auto n_cell_pairs = thread2n_cell_pairs_[omp_get_thread_num()];

    for (int cell_pair_ind = first_cell_pair_ind; cell_pair_ind < first_cell_pair_ind + n_cell_pairs; ++cell_pair_ind){
        
        const auto cell_i_ind = cell_pair_list_[cell_pair_ind].cell_i;
        const auto cell_j_ind = cell_pair_list_[cell_pair_ind].cell_j;

        const auto& cell_i = cell2boid_inds2_[cell_i_ind];
        const auto& cell_j = cell2boid_inds2_[cell_j_ind];

        const auto last_in_cell_i = cell_i.back().ind;
        const auto last_in_cell_j = cell_j.back().ind;
        for (int i = 0; i < last_in_cell_i; ++i) {
            auto r_collision = cell_i[i].radius;
            const auto r_i = cell_i[i].r;
            const auto boid_ind_i = cell_i[i].ind;
            for (int j = 0; j < last_in_cell_j; ++j) {            
                const auto boid_ind_j = cell_j[j].ind;
                const auto dr = cell_j[j].r - r_i;
                if (norm2(dr) < r_max2 && (boid_ind_j != boid_ind_i)) {
                    pair_lists_[omp_get_thread_num()].emplace_back(dr, boid_ind_i, boid_ind_j, r_collision + cell_j[j].radius);
                }
            }
        }
    }
}

}

// void NeighbourSearcher::fillNeighbourDataSerial(const std::vector<sf::Vector2f>& r_coords,
//                                                 const std::vector<float>& radii, const float r_max2) {

//     pair_list_.clear();
//     std::sort(active_cells_.begin(), active_cells_.end());
//     omp_set_num_threads(omp_get_max_threads());

//     const auto n_active_cells = active_cells_.size();

//     std::array<int, 9> nearest_cells;

//     for (int active_cell_ind = 0; active_cell_ind < n_active_cells; ++active_cell_ind) {
//         const auto cell_i_ind = active_cells_[active_cell_ind];
//         const auto& cell_i = cell2boid_inds2_[cell_i_ind];
//         int n_nearest_cells;
//         s_grid_->calcNearestCells(cell_i_ind, nearest_cells, n_nearest_cells);
//         for (auto nearest_cell_it = nearest_cells.begin(); nearest_cell_it < nearest_cells.begin() + n_nearest_cells;
//              ++nearest_cell_it) {
//             const auto cell_j_ind = *nearest_cell_it;
//             if (cell_j_ind <= cell_i_ind) {
//                 continue;
//             }

//             const auto& cell_j = cell2boid_inds2_[cell_j_ind];

//             const auto last_in_cell_i = cell_i.back().ind;
//             const auto last_in_cell_j = cell_j.back().ind;
//             for (int i = 0; i < last_in_cell_i; ++i) {
//                 for (int j = 0; j < last_in_cell_j; ++j) {
//                     const auto r_collision = cell_j[j].radius + cell_i[i].radius;
//                     const auto dr = cell_j[j].r - cell_i[i].r;
//                     if (norm2(dr) < r_max2) {
//                         pair_lists_[omp_get_thread_num()].emplace_back(dr, cell_i[i].ind, cell_j[j].ind,
//                         r_collision);
//                         // pair_list_.emplace_back(dr, cell_i[i].ind, cell_j[j].ind, r_collision);
//                     }
//                 }
//             }
//         }
//     }

// #pragma omp parallel for
//     for (int active_cell_ind = 0; active_cell_ind < n_active_cells; ++active_cell_ind) {
//         const auto cell_i_ind = active_cells_[active_cell_ind];
//         const auto& cell_i = cell2boid_inds2_[cell_i_ind];
//         const auto last_in_cell_i = cell_i.back().ind;
//         if (last_in_cell_i > 50) {
//             std::cout << "wtf";
//         }
//         for (int i = 0; i < last_in_cell_i; ++i) {
//             for (int j = i + 1; j < last_in_cell_i; ++j) {
//                 const auto dr = cell_i[j].r - cell_i[i].r;
//                 const auto r_collision = cell_i[j].radius + cell_i[i].radius;
//                 pair_lists_[omp_get_thread_num()].emplace_back(dr, cell_i[i].ind, cell_i[j].ind, r_collision);
//                 // particle2neighbour_data_[cell_i[i].ind][last_i[cell_i[i].ind]] = {dr, r_collision, cell_i[j].ind};
//                 // last_i[cell_i[i].ind] = std::min(100 - 1, last_i[cell_i[i].ind] + 1);
//             }
//         }
//     }

//     // std::cout << "n active cells: " << n_active_cells << "\n";

//     // for (int i = 0; i < n_pairs and i < last; i++) {
//     //     const auto& pair_data = pair_list_[i];
//     //     if (norm2(pair_data.dr) > r_max2 or pair_data.first == pair_data.second) {
//     //         pair_list_[i] = pair_list_[last];
//     //         i--;
//     //         last--;
//     //     }
//     // }

//     if (pair_lists_[0].size() > 100000) {
//         std::cout << "wtf2"
//                   << "\n";
//     }

//     // #pragma omp parallel
//     //     {

//     //         const auto& pair_list = pair_lists_[omp_get_thread_num()];
//     //         // #pragma omp critical
//     //         //         std::cout << "n pairs on thread " << omp_get_thread_num() << " is: " << pair_list.size() <<
//     "
//     //         \n";

//     //         const auto n_pairs = pair_list.size();
//     //         const int last = n_pairs - 1;
//     // #pragma omp for
//     //         for (int i = 0; i < last; ++i) {
//     //             const auto& ind_i = pair_list[i].first;
//     //             const auto& ind_j = pair_list[i].second;
//     //             const auto& dr = pair_list[i].dr;
//     //             const auto& r_collision = pair_list[i].r_collision;
//     // #pragma omp critical
//     //             {
//     //                 particle2neighbour_data_[ind_i][last_i[ind_i]] = {dr, r_collision, ind_j};
//     //                 particle2neighbour_data_[ind_j][last_i[ind_j]] = {-dr, r_collision, ind_i};
//     //                 last_i[ind_i] = std::min(100 - 1, last_i[ind_i] + 1);
//     //                 last_i[ind_j] = std::min(100 - 1, last_i[ind_j] + 1);
//     //             }
//     //         }
//     //     }
// }
