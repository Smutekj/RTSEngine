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
    cell2data_.resize(n_cells.x * n_cells.y);
    cell2data2_.resize(n_cells.x * n_cells.y);
    cell_visited_.resize(n_cells.x * n_cells.y, false);
    for (auto& cell : cell2boid_inds_) {
        cell.back() = 0;
    }
    
    s_grid_ = std::make_shared<SearchGrid>(n_cells, sf::Vector2f{max_dist, max_dist});
    particle2neighbour_data_.resize(N_MAX_NAVIGABLE_BOIDS);
    particle2ninteraction_data_.resize(N_MAX_NAVIGABLE_BOIDS);
    last_i.resize(N_MAX_NAVIGABLE_BOIDS, 0);
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
        const auto& cell = cell2data2_[cell_ind_neighbour];
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


std::vector<int> NeighbourSearcher::getNeighboursIndsFull(sf::Vector2f r, const float r_max) {
    std::vector<int> neighbour_inds;

    const auto& r_coords = boids_->r_coords_;
    const auto r_max2 = r_max*r_max;

    int last_nearest_cell_ind = 8;
    std::vector<int> nearest_cells;

    const auto cell_ind = s_grid_->coordToCell(r);
    const auto cell_coords = s_grid_->cellCoords(cell_ind);


    int j_max = cell_coords.y + std::ceil(r_max/ s_grid_->cell_size_.y) ;
    int j_min = cell_coords.y - std::ceil(r_max/ s_grid_->cell_size_.y) ;
    j_max = std::min(j_max, s_grid_->n_cells_.y-1);
    j_min = std::max(j_min, 0);

    for(int j = j_min; j <= j_max; ++j){
        const float dy = (j - cell_coords.y) * s_grid_->cell_size_.y; 
        int i_max = cell_coords.x + std::ceil(std::sqrt(r_max2 - dy*dy));
        int i_min = cell_coords.x - (i_max - cell_coords.x);
        i_max = std::min(i_max, s_grid_->n_cells_.x-1);
        i_min = std::max(i_min, 0);

        for(int i = i_min; i <= i_max; ++i){
            nearest_cells.push_back(i + j*s_grid_->n_cells_.x);
        }
    }

    for (const auto cell_ind_neighbour : nearest_cells) {
        const auto& cell = cell2data_[cell_ind_neighbour];
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


const std::array<NeighbourSearcher::NeighbourData, N_MAX_NEIGHBOURS>& NeighbourSearcher::getNeighbourData(const int boid_ind,
                                                                                             const float r_max2) const {
    return particle2neighbour_data_.at(boid_ind);
}

const std::array<NeighbourSearcher::InteractionData, N_MAX_NEIGHBOURS>& NeighbourSearcher::getInteractionData(const int boid_ind,
                                                                                             const float r_max2) const {
    return particle2ninteraction_data_.at(boid_ind);
}

void NeighbourSearcher::fillNeighbourData(  const std::vector<sf::Vector2f>& r_coords, const std::vector<float>& radii,
                                             const std::vector<int>& active_inds,
                                             const float r_max2) {
    std::vector<std::array<int, 9>> thread_id2nearest_cells(NUM_OMP_NS_THREADS);
    const auto n_boids = active_inds.size();

#pragma omp parallel num_threads(NUM_OMP_NS_THREADS)
{
    const auto thread_id = omp_get_thread_num();
    auto& nearest_cells = thread_id2nearest_cells.at(thread_id);
    #pragma omp for
    for (int ind = 0; ind < n_boids; ++ind) {
        const auto i = active_inds[ind]; 
        const auto r = r_coords[i];
        const auto cell_i_ind = s_grid_->coordToCell(r);
        int n_nearest_cells;

        s_grid_->calcNearestCells(cell_i_ind, nearest_cells, n_nearest_cells);
        nearest_cells[n_nearest_cells] = cell_i_ind;
        n_nearest_cells++;
        const auto& cell_data_i = cell2data_[cell_i_ind];
        const auto radius = radii[i];

        for (int cell_j_ind = 0; cell_j_ind < n_nearest_cells; ++cell_j_ind) {
            const auto& cell_data_j = cell2data_[nearest_cells[cell_j_ind]];
            const auto last_in_cell_j = cell_data_j.back().ind;
            for (int j = 0; j < last_in_cell_j; ++j) {
                const auto ind = cell_data_j[j].ind;
                const auto r_collision = cell_data_j[j].radius + radius;
                const auto dr = cell_data_j[j].r - r;
                if (norm2(dr) < r_max2 && i != ind) {
                    particle2neighbour_data_[i][last_i[i]] = {dr, r_collision, ind};
                    last_i[i] = std::min(N_MAX_NEIGHBOURS, last_i[i] + 1);
                }
            }
        }
        if(last_i[i] == N_MAX_NEIGHBOURS){
            throw std::runtime_error("too many neighbours on particle: " + std::to_string(i) + " !");
        }
    }
}
}

void NeighbourSearcher::fillNeighbourData2(  const std::vector<sf::Vector2f>& r_coords, const std::vector<float>& radii,
                                             const std::vector<int>& active_inds,
                                             const float r_max2) {
    std::vector<std::array<int, 9>> thread_id2nearest_cells(NUM_OMP_NS_THREADS);
    const auto n_boids = active_inds.size();

#pragma omp parallel num_threads(NUM_OMP_NS_THREADS)
{
    const auto thread_id = omp_get_thread_num();
    auto& nearest_cells = thread_id2nearest_cells.at(thread_id);
    #pragma omp for
    for (int ind = 0; ind < n_boids; ++ind) {
        const auto i = active_inds[ind]; 
        const auto r = r_coords[i];
        const auto v = boids_->velocities_[i];
        const auto cell_i_ind = s_grid_->coordToCell(r);
        int n_nearest_cells;
        s_grid_->calcNearestCells(cell_i_ind, nearest_cells, n_nearest_cells);

        nearest_cells[n_nearest_cells] = cell_i_ind;
        n_nearest_cells++;
        const auto& cell_data_i = cell2data_[cell_i_ind];
        const auto radius = radii[i];

        for (int cell_j_ind = 0; cell_j_ind < n_nearest_cells; ++cell_j_ind) {
            const auto& cell_data_j = cell2data2_[nearest_cells[cell_j_ind]];
            const auto last_in_cell_j = cell_data_j.back().ind;
            for (int j = 0; j < last_in_cell_j; ++j) {
                const auto ind = cell_data_j[j].ind;
                const auto r_collision = cell_data_j[j].radius + radius;
                const auto dr = cell_data_j[j].r - r;
                const auto v_rel = cell_data_j[j].v - v;
                const auto& mass = cell_data_j[j].mass;
                const auto& state = cell_data_j[j].state;
                const auto& player = cell_data_j[j].player_ind;
                
                if (norm2(dr) < r_max2 && i != ind) {
                    particle2ninteraction_data_[i][last_i[i]] = {dr, v_rel, r_collision, mass, player, ind, state};
                    last_i[i] = std::min(N_MAX_NEIGHBOURS, last_i[i] + 1);
                }
            }
        }
        if(last_i[i] == N_MAX_NEIGHBOURS){
            throw std::runtime_error("too many neighbours on particle: " + std::to_string(i) + " !");
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
        auto& cell_i_data = cell2data_[cell_i_ind];
        // cell_i.back() = 0;
        cell_i_data.back().ind = 0;
    }

    const auto n_boids = boids_->active_inds.size();
    active_cells_.reserve(n_boids);
    active_cells_.resize(0);

    for (int boid_ind = 0; boid_ind < n_boids; ++boid_ind) {
        const auto cell_ind = s_grid_->coordToCell(boids_->r_coords_[boid_ind]);
        // auto& last_ind_in_cell = cell2boid_inds_[cell_ind].back();
        auto& last_ind_in_cell = cell2data_[cell_ind].back().ind;
        // assert(last_ind_in_cell < NMAX);
        // cell2boid_inds_[cell_ind][last_ind_in_cell] = (boid_ind);
        if (last_ind_in_cell < N_MAX_AGENTS_IN_CELL) {
            cell2data_[cell_ind][last_ind_in_cell] = {r_coords[boid_ind], radii[boid_ind], boid_ind};
            last_ind_in_cell++;
        } else {
            throw std::runtime_error("too many particles per cell!");
        }

        if (!cell_visited_[cell_ind]) {
            active_cells_.push_back(cell_ind);
            assert(last_ind_in_cell != 0);
        }
        cell_visited_[cell_ind] = true;
        last_i[boid_ind] = 0;
    }

    const auto n_active_cells = active_cells_.size();
    for (int active_cell_ind = 0; active_cell_ind < n_active_cells; ++active_cell_ind)
    {
        cell_visited_[active_cells_[active_cell_ind]] = false;
    }
}


//! \brief sorts relevant data into corresponding cells (coord, rel_vel, radius, boid index)
//! \param r_coords
//! \param radii 
void NeighbourSearcher::addOnGrid(const std::vector<sf::Vector2f>& r_coords,
                                    const std::vector<sf::Vector2f>& vels, 
                                    const std::vector<float>& masses,
                                    const std::vector<float>& radii,
                                    const std::vector<int>& active_inds) {

    // visited.clear();
    for (const auto cell_i_ind : active_cells_) {
        auto& cell_i_data = cell2data2_[cell_i_ind];
        cell_i_data.back().ind = 0;
    }

    const auto n_boids = boids_->active_inds.size();
    active_cells_.reserve(n_boids);
    active_cells_.resize(0);
    const auto& states = boids_->move_states_;

    for (int boid_ind = 0; boid_ind < n_boids; ++boid_ind) {
        const auto cell_ind = s_grid_->coordToCell(boids_->r_coords_[boid_ind]);
        auto& last_ind_in_cell = cell2data2_[cell_ind].back().ind;

        if (last_ind_in_cell < N_MAX_AGENTS_IN_CELL) [[likely]] 
        { 
            cell2data2_[cell_ind][last_ind_in_cell] = {r_coords[boid_ind], vels[boid_ind], 4.f*std::pow(masses[boid_ind], 2.f), radii[boid_ind], boids_->ind2player[boid_ind], states[boid_ind], boid_ind};
            last_ind_in_cell++;
        } else {
            throw std::runtime_error("too many particles per cell!");
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

//! \brief sorts relevant data of particles in \p selection
//! \brief into corresponding cells (coordinate, radius, boid index (later maybe move state?))
//! \param r_coords
//! \param radii 
//! \param selection    selected particle inds
void NeighbourSearcher::addOnGrid(const std::vector<sf::Vector2f>& r_coords, const std::vector<float>& radii,
                                  const std::vector<int>& selection) {

    // visited.clear();
    for (const auto cell_i_ind : active_cells_) {
        auto& cell_i_data = cell2data_[cell_i_ind];
        // cell_i.back() = 0;
        cell_i_data.back().ind = 0;
    }

    const auto n_boids = boids_->active_inds.size();
    active_cells_.reserve(n_boids);
    active_cells_.resize(0);

    for (const auto boid_ind : selection) {
        const auto cell_ind = s_grid_->coordToCell(boids_->r_coords_[boid_ind]);
        // auto& last_ind_in_cell = cell2boid_inds_[cell_ind].back();
        auto& last_ind_in_cell = cell2data_[cell_ind].back().ind;
        // assert(last_ind_in_cell < NMAX);
        // cell2boid_inds_[cell_ind][last_ind_in_cell] = (boid_ind);
        cell2data_[cell_ind][last_ind_in_cell] = {r_coords[boid_ind], radii[boid_ind], boid_ind};
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
void NeighbourSearcher::update(const std::vector<sf::Vector2f>& r_coords, const std::vector<int>& active_inds, const std::vector<float>& radii) {


    const auto start1 = std::chrono::high_resolution_clock::now();
    // addOnGrid(r_coords, radii);
    addOnGrid(r_coords, boids_->velocities_, radii, radii, active_inds);


    const auto start2 = std::chrono::high_resolution_clock::now();
    // fillNeighbourData(boids_->r_coords_, radii, active_inds, RHARD * RHARD * (100));
    fillNeighbourData2(boids_->r_coords_, radii, active_inds, RHARD * RHARD * (100));


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
