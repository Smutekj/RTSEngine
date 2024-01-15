#pragma once

#include <memory>

#include "core.h"
#include "ECS.h"
#include "Grid.h"

#include "NeighbourSearcherStrategy.h"

class SearchGrid;

// template <class CompType, class InteractionType, int MAX_AGENTS=500>
// class INeighbourSearcherStrategy;

static constexpr size_t N_MAX_AGENTS_IN_CELL = 500;

template <class CompType, class InteractionType>
struct NeighbourSearcherContext
{

    std::vector<std::array<InteractionType, N_MAX_NEIGHBOURS>> component2neighbour_data_;
    std::vector<std::array<CompType, N_MAX_NEIGHBOURS>> particle2components_;

    std::shared_ptr<SearchGrid> s_grid_;

    static constexpr size_t N_MAX_AGENTS_IN_CELL = 500;

    std::vector<std::array<ComponentAndInd<CompType>, N_MAX_AGENTS_IN_CELL + 1>> cell2comp_and_comp_ind_;
    std::vector<std::array<int, N_MAX_AGENTS_IN_CELL + 1>> cell2comp_ind_;

    std::vector<int> active_cells_;
    std::vector<bool> cell_visited_;

    std::unique_ptr<INeighbourSearcherStrategy<CompType, InteractionType>> p_strategy_;

public:
    std::vector<int> last_i;

    void setStrategy(NS_STRATEGY strategy_id);

    NeighbourSearcherContext(sf::Vector2f box_size, float max_dist);

    std::vector<int> getNeighboursIndsFull(sf::Vector2f r, const float r_max)
    {
        std::vector<int> neighbour_inds;

        const auto r_max2 = r_max * r_max;

        int last_nearest_cell_ind = 8;
        std::vector<int> nearest_cells;

        const auto cell_ind = s_grid_->coordToCell(r);
        const auto cell_coords = s_grid_->cellCoords(cell_ind);

        int j_max = cell_coords.y + std::ceil(r_max / s_grid_->cell_size_.y);
        int j_min = cell_coords.y - std::ceil(r_max / s_grid_->cell_size_.y);
        j_max = std::min(j_max, s_grid_->n_cells_.y - 1);
        j_min = std::max(j_min, 0);

        for (int j = j_min; j <= j_max; ++j)
        {
            const float dy = (j - cell_coords.y) * s_grid_->cell_size_.y;
            int i_max = cell_coords.x + std::ceil(std::sqrt(r_max2 - dy * dy));
            int i_min = cell_coords.x - (i_max - cell_coords.x);
            i_max = std::min(i_max, s_grid_->n_cells_.x - 1);
            i_min = std::max(i_min, 0);

            for (int i = i_min; i <= i_max; ++i)
            {
                nearest_cells.push_back(i + j * s_grid_->n_cells_.x);
            }
        }

        for (const auto cell_ind_neighbour : nearest_cells)
        {
            const auto &cells = cell2comp_and_comp_ind_[cell_ind_neighbour];
            const auto last_in_cell = cells.back().comp_ind;
            for (int i_cell = 0; i_cell < last_in_cell; ++i_cell)
            {
                const auto &neighbour_ind = cells[i_cell].comp_ind;
                const auto d2 = dist2(cells[i_cell].comp.transform.r, r);
                if (d2 <= r_max2)
                {
                    neighbour_inds.push_back(neighbour_ind);
                }
            }
        }

        return neighbour_inds;
    }

    const std::array<InteractionType, N_MAX_NEIGHBOURS> &getInteractionData(const int comp_ind) const
    {
        return component2neighbour_data_.at(comp_ind);
    }

    void update(const std::vector<CompType> &components);

private:
    void addOnGrid(const std::vector<CompType> &components);
};
