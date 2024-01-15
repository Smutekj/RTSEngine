#pragma once

#include "core.h"
#include "ECS.h"

enum class NS_STRATEGY{
    BASIC = 0,    
    ATTACK,
};

class SearchGrid;

template <class CompType, class InteractionType, int N_MAX_AGENTS_IN_CELL=500>
struct INeighbourSearcherStrategy{

public:
    virtual void execute(const std::vector<CompType> &comps,
                        const std::vector<std::array<ComponentAndInd<CompType>, N_MAX_AGENTS_IN_CELL + 1>> &cell2comp_and_comp_ind,
                        std::vector<std::array<InteractionType, N_MAX_NEIGHBOURS>> &component2neighbour_data,
                        const SearchGrid &search_grid,
                        std::vector<int>& component2last_neigbhour_index, float r_max2) = 0; 

};


struct NeighbourSearcherStrategySeek : INeighbourSearcherStrategy<PathFinderComponent, int, 500>
{

public:
    virtual void execute(const std::vector<PathFinderComponent> &comps,
                    const std::vector<std::array<ComponentAndInd<PathFinderComponent>, 500 + 1>> &cell2comp_and_comp_ind,
                    std::vector<std::array<int, N_MAX_NEIGHBOURS>> &component2neighbour_datalist,
                    const SearchGrid &search_grid,
                    std::vector<int>& component2last_neigbhour_index,
                    float r_max2) override;
};

struct NeighbourSearcherStrategyAttack : INeighbourSearcherStrategy<AttackComponent, int, 500>
{

public:
    virtual void execute(const std::vector<AttackComponent> &comps,
                    const std::vector<std::array<ComponentAndInd<AttackComponent>, 500 + 1>> &cell2comp_and_comp_ind,
                    std::vector<std::array<int, N_MAX_NEIGHBOURS>> &component2neighbour_datalist,
                    const SearchGrid &search_grid,
                    std::vector<int>& component2last_neigbhour_index, float r_max2) override;
};


struct NeighbourSearcherStrategyPhysics : INeighbourSearcherStrategy<PhysicsComponent, InteractionData, MAX_N_AGENTS_IN_PHYSICS_CELLS>
{

public:
    void execute(const std::vector<PhysicsComponent> &comps,
                    const std::vector<std::array<ComponentAndInd<PhysicsComponent>, MAX_N_AGENTS_IN_PHYSICS_CELLS + 1>> &cell2comp_and_comp_ind,
                    std::vector<std::array<InteractionData, N_MAX_NEIGHBOURS>> &component2neighbour_datalist,
                    const SearchGrid &search_grid,
                    std::vector<int>& component2last_neigbhour_index,
                    float r_max2);
};


