#include "NeighbourSearcherContext.h"
#include "NeighbourSearcherStrategy.h"

template <class CompType, class InteractionType>
void NeighbourSearcherContext<CompType, InteractionType>::update(const std::vector<CompType> &components)
{
    addOnGrid(components);

    p_strategy_->execute(components,
                         cell2comp_and_comp_ind_,
                         component2neighbour_data_,
                         *s_grid_,
                         last_i, 10*RHARD);
}


template <class CompType, class InteractionType>
void NeighbourSearcherContext<CompType, InteractionType>::addOnGrid(const std::vector<CompType> &components)
{

    // visited.clear();
    for (const auto cell_i_ind : active_cells_)
    {
        auto &cell_i_data = cell2comp_and_comp_ind_[cell_i_ind];
        cell_i_data.back().comp_ind = 0;
    }

    const auto n_components = components.size();
    active_cells_.resize(0);

    for (int comp_ind = 0; comp_ind < n_components; ++comp_ind)
    {
        const auto &comp = components.at(comp_ind);
        const auto &r = comp.transform.r;
        const auto cell_ind = s_grid_->coordToCell(r);
        auto &last_ind_in_cell = cell2comp_and_comp_ind_[cell_ind].back().comp_ind;

        if (last_ind_in_cell < N_MAX_AGENTS_IN_CELL) [[likely]]
        {
            cell2comp_and_comp_ind_[cell_ind][last_ind_in_cell] = {comp, comp_ind};
            cell2comp_ind_[cell_ind][last_ind_in_cell] = {comp_ind};
            last_ind_in_cell++;
        }
        else
        {
            throw std::runtime_error("too many particles per cell!");
        }

        if (!cell_visited_[cell_ind])
        {
            active_cells_.push_back(cell_ind);
            assert(last_ind_in_cell != 0);
        }
        cell_visited_[cell_ind] = true;
        last_i.at(comp_ind) = 0;
    }

    const auto n_active_cells = active_cells_.size();
    // #pragma omp parallel for
    for (int active_cell_ind = 0; active_cell_ind < n_active_cells; ++active_cell_ind)
    {
        cell_visited_[active_cells_[active_cell_ind]] = false;
    }
}

template<class CompType, class InteractionType>
NeighbourSearcherContext<CompType, InteractionType>::NeighbourSearcherContext(sf::Vector2f box_size, float max_dist) 
    {

        const sf::Vector2i n_cells = {static_cast<int>(box_size.x / max_dist) + 1,
                                      static_cast<int>(box_size.y / max_dist) + 1};

        cell2comp_ind_.resize(n_cells.x * n_cells.y);
        cell2comp_and_comp_ind_.resize(n_cells.x * n_cells.y);
        cell_visited_.resize(n_cells.x * n_cells.y, false);

        for (auto &cell : cell2comp_and_comp_ind_)
        {
            cell.back().comp_ind = 0;
        }

        s_grid_ = std::make_shared<SearchGrid>(n_cells, sf::Vector2f{max_dist, max_dist});
        component2neighbour_data_.resize(N_MAX_NAVIGABLE_BOIDS);
        last_i.resize(N_MAX_NAVIGABLE_BOIDS, 0);
    }

template<class CompType, class InteractionType>
    void NeighbourSearcherContext<CompType, InteractionType>::setStrategy(NS_STRATEGY strat_id)
    {        
        switch(strat_id){
            case NS_STRATEGY::BASIC:
                p_strategy_ = std::make_unique<NeighbourSearcherStrategySeek>();
                break;
            default:
                throw std::runtime_error("STRATEGY NOT IMPLEMENTED!");
        }
    }


template<>
    void NeighbourSearcherContext<AttackComponent, int>::setStrategy(NS_STRATEGY strat_id)
    {        
        switch(strat_id){
            case NS_STRATEGY::BASIC:
                break;
            case NS_STRATEGY::ATTACK:
                p_strategy_ = std::make_unique<NeighbourSearcherStrategyAttack>();
                break;
            default:
                throw std::runtime_error("STRATEGY NOT IMPLEMENTED!");
        }
    }

    template<>
    void NeighbourSearcherContext<PhysicsComponent, InteractionData>::setStrategy(NS_STRATEGY strat_id)
    {        
        switch(strat_id){
            case NS_STRATEGY::BASIC:
                p_strategy_ = std::make_unique<NeighbourSearcherStrategyPhysics>();
                break;
            default:
                throw std::runtime_error("STRATEGY NOT IMPLEMENTED!");
        }
    }



// template class NeighbourSearcherContext<PhysicsComponent, InteractionData>;
template class NeighbourSearcherContext<PathFinderComponent, int>;
template class NeighbourSearcherContext<AttackComponent, int>;
template class NeighbourSearcherContext<PhysicsComponent, InteractionData>;
