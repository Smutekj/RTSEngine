#include "NeighbourSearcherStrategy.h"

#include <omp.h>

#include "../Systems/AttackSystem.h"
#include "../Systems/Components.h"

#include "Grid.h"

void NeighbourSearcherStrategyPhysics::execute(const std::vector<PhysicsComponent> &comps,
                                               const std::vector<std::array<ComponentAndInd<PhysicsComponent>, MAX_N_AGENTS_IN_PHYSICS_CELLS + 1>> &cell2comp_and_comp_ind,
                                               std::vector<std::array<InteractionData, N_MAX_NEIGHBOURS>> &component2neighbour_data,
                                               const SearchGrid &search_grid,
                                               std::vector<int> &component2last_neigbhour_index,
                                               float r_max2)
{
    std::vector<std::array<int, 9>> thread_id2nearest_cells(NUM_OMP_NS_THREADS);
    const auto n_comps = comps.size();

#pragma omp parallel num_threads(NUM_OMP_NS_THREADS)
    {
        const auto thread_id = omp_get_thread_num();
        auto &nearest_cells = thread_id2nearest_cells.at(thread_id);
#pragma omp for
        for (int comp_ind = 0; comp_ind < n_comps; ++comp_ind)
        {
            const auto &comp = comps[comp_ind];
            const auto r = comp.transform.r;
            const auto v = comp.transform.vel;
            const auto cell_i_ind = search_grid.coordToCell(r);
            int n_nearest_cells;
            search_grid.calcNearestCells(cell_i_ind, nearest_cells, n_nearest_cells);

            nearest_cells[n_nearest_cells] = cell_i_ind;
            n_nearest_cells++;
            const auto &cell_data_i = cell2comp_and_comp_ind[cell_i_ind];
            const auto radius = comp.radius;

            auto &last_neighbour_ind = component2last_neigbhour_index[comp_ind];

            for (int cell_j_ind = 0; cell_j_ind < n_nearest_cells; ++cell_j_ind)
            {
                const auto &cell_data_j = cell2comp_and_comp_ind[nearest_cells[cell_j_ind]];
                const auto last_in_cell_j = cell2comp_and_comp_ind[nearest_cells[cell_j_ind]].back().comp_ind;
                for (int j = 0; j < last_in_cell_j; ++j)
                {
                    auto &[comp_j, comp_ind_j] = cell_data_j[j];
                    const auto r_collision = comp_j.radius + radius;
                    const auto dr = comp_j.transform.r - r;
                    const auto v_rel = comp_j.transform.vel - v;
                    const auto &mass = comp_j.mass;
                    const auto &state = comp_j.state;
                    const u_int8_t player = comp_j.player_ind;

                    if (norm2(dr) < r_max2 && comp_ind != comp_ind_j)
                    {
                        component2neighbour_data[comp_ind][last_neighbour_ind] = {dr, v_rel, r_collision, mass, comp_ind_j, state, player, last_neighbour_ind + 1};
                        last_neighbour_ind = std::min(N_MAX_NEIGHBOURS, last_neighbour_ind + 1);
                    }
                }
            }
            if (last_neighbour_ind == N_MAX_NEIGHBOURS)
            {
                throw std::runtime_error("too many neighbours on component: " + std::to_string(comp_ind) + " !");
            }
        }
    }
}

void NeighbourSearcherStrategySeek::execute(
    const std::vector<PathFinderComponent> &comps,
    const std::vector<std::array<ComponentAndInd<PathFinderComponent>, MAX_N_AGENTS_IN_PHYSICS_CELLS + 1>> &cell2comp_and_comp_ind,
    std::vector<std::array<int, N_MAX_NEIGHBOURS>> &component2neighbour_data,
    const SearchGrid &search_grid,
    std::vector<int> &component2last_neigbhour_index, float r_max2)
{
    std::vector<std::array<int, 9>> thread_id2nearest_cells(NUM_OMP_NS_THREADS);
    const auto n_comps = comps.size();

#pragma omp parallel num_threads(NUM_OMP_NS_THREADS)
    {
        const auto thread_id = omp_get_thread_num();
        auto &nearest_cells = thread_id2nearest_cells.at(thread_id);
#pragma omp for
        for (int comp_ind = 0; comp_ind < n_comps; ++comp_ind)
        {
            const auto &comp = comps[comp_ind];
            const auto r = comp.transform.r;
            const auto v = comp.transform.vel;
            const auto cell_i_ind = search_grid.coordToCell(r);
            int n_nearest_cells;
            search_grid.calcNearestCells(cell_i_ind, nearest_cells, n_nearest_cells);

            nearest_cells[n_nearest_cells] = cell_i_ind;
            n_nearest_cells++;
            const auto &cell_data_i = cell2comp_and_comp_ind[cell_i_ind];
            const auto radius = comp.radius;

            auto &last_neighbour_ind = component2last_neigbhour_index[comp_ind];

            for (int cell_j_ind = 0; cell_j_ind < n_nearest_cells; ++cell_j_ind)
            {
                const auto &cell_data_j = cell2comp_and_comp_ind[nearest_cells[cell_j_ind]];
                const auto last_in_cell_j = cell2comp_and_comp_ind[nearest_cells[cell_j_ind]].back().comp_ind;
                for (int j = 0; j < last_in_cell_j; ++j)
                {
                    auto &[comp_j, comp_ind_j] = cell_data_j[j];
                    const auto dr = comp_j.transform.r - r;
                    const auto r_max_sq_of_comp = 10.f * (comp.radius + comp_j.radius) * (comp.radius + comp_j.radius);
                    if (norm2(dr) < r_max_sq_of_comp && comp_ind != comp_ind_j)
                    {
                        component2neighbour_data[comp_ind][last_neighbour_ind] = comp_ind_j;
                        last_neighbour_ind = std::min(N_MAX_NEIGHBOURS, last_neighbour_ind + 1);
                    }
                }
            }
            if (last_neighbour_ind == N_MAX_NEIGHBOURS)
            {
                throw std::runtime_error("too many neighbours on component: " + std::to_string(comp_ind) + " !");
            }
        }
    }
}

void NeighbourSearcherStrategyAttack::execute(const std::vector<AttackComponent> &comps,
                                              const std::vector<std::array<ComponentAndInd<AttackComponent>, 500 + 1>> &cell2comp_and_comp_ind,
                                              std::vector<std::array<int, N_MAX_NEIGHBOURS>> &component2neighbour_data,
                                              const SearchGrid &search_grid,
                                              std::vector<int> &component2last_neigbhour_index, float r_max2)
{
    std::vector<std::array<int, 9>> thread_id2nearest_cells(NUM_OMP_NS_THREADS);
    const auto n_comps = comps.size();

#pragma omp parallel num_threads(NUM_OMP_NS_THREADS)
    {
        const auto thread_id = omp_get_thread_num();
        auto &nearest_cells = thread_id2nearest_cells.at(thread_id);
#pragma omp for
        for (int comp_ind = 0; comp_ind < n_comps; ++comp_ind)
        {
            const auto &comp = comps[comp_ind];
            const auto r = comp.transform.r;
            const auto v = comp.transform.vel;
            const auto cell_i_ind = search_grid.coordToCell(r);
            int n_nearest_cells;
            search_grid.calcNearestCells(cell_i_ind, nearest_cells, n_nearest_cells);

            nearest_cells[n_nearest_cells] = cell_i_ind;
            n_nearest_cells++;
            const auto &cell_data_i = cell2comp_and_comp_ind[cell_i_ind];

            float min_enemy_dist_sq = static_cast<float>(Geometry::BOX[0]);
            auto &last_neighbour_ind = component2last_neigbhour_index[comp_ind];

            for (int cell_j_ind = 0; cell_j_ind < n_nearest_cells; ++cell_j_ind)
            {
                const auto &cell_data_j = cell2comp_and_comp_ind[nearest_cells[cell_j_ind]];
                const auto last_in_cell_j = cell2comp_and_comp_ind[nearest_cells[cell_j_ind]].back().comp_ind;
                for (int j = 0; j < last_in_cell_j; ++j)
                {
                    auto &[comp_j, comp_ind_j] = cell_data_j[j];
                    const auto dist_sq = dist2(comp_j.transform.r, r);

                    if (dist_sq < min_enemy_dist_sq && (comp.player_ind != comp_j.player_ind))
                    {
                        component2neighbour_data[comp_ind][0] = comp_ind_j;
                        last_neighbour_ind = 1;
                    }
                }
            }
            if (last_neighbour_ind == N_MAX_NEIGHBOURS)
            {
                throw std::runtime_error("too many neighbours on component: " + std::to_string(comp_ind) + " !");
            }
        }
    }
}

template class INeighbourSearcherStrategy<AttackComponent, int>;
template class INeighbourSearcherStrategy<PathFinderComponent, int>;
template class INeighbourSearcherStrategy<PhysicsComponent, InteractionData>;


