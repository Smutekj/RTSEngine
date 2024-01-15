#pragma once

#include "../ECS.h"
#include "../core.h"
#include "../Grid.h"
#include "Components.h"

constexpr float cell_size = 100.f;

struct GraphicsSystem : System2
{

    typedef ComponentArray<GraphicsComponent> CompArray;

    std::unique_ptr<SearchGrid> p_culling_grid_;
    std::vector<std::vector<int>> grid2comp_inds_;

public:
    GraphicsSystem(ComponentID id);
    virtual ~GraphicsSystem() = default;

    virtual void update() override;

    void addOnGrid();

    void draw(sf::RenderTarget &target) override;

    virtual void updateSharedData(const std::array<SharedData, N_MAX_ENTITIES> &new_data,
                                  const std::vector<Entity> &active_entity_inds ) override;
    
    virtual void communicate(std::array<SharedData, N_MAX_ENTITIES> &entity2shared_data) const override
    {}

    private:
        sf::VertexArray vertices_;
};