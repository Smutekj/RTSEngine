#pragma once

#include "../ECS.h"
#include "../NeighbourSearcherContext.h"
#include "../Edges.h"

constexpr int N_CELLS_PHYSICS_NS = 100;
constexpr float R_MAX_PHYSICS_NS = 20;


// struct WallInteractionComponent : Component<ComponentID::WALL>{
//     TransformComponent transform;
//     float radius;
// }


struct PhysicsSystem : System2{

    typedef ComponentArray<PhysicsComponent> CompArray;

    std::vector<sf::Vector2f> component2cumulative_force_;
    std::unique_ptr<NeighbourSearcherContext<PhysicsComponent, InteractionData>> p_ns_;
    Edges* p_map_;

    const float dt = 1/60.f;

    PhysicsSystem(ComponentID id);
    ~PhysicsSystem() = default;
    
    void update();
    void avoidWall();
    void draw(sf::RenderTarget& target);
    void setTransform(const int& e, const TransformComponent& trans){}
    
    void updateSharedData(const std::array<SharedData, N_MAX_ENTITIES>& new_data, const std::vector<Entity>& active_entity_inds);
    void communicateVelocities(std::array<SharedData, N_MAX_ENTITIES>& entity2shared_data)const;

    private:
    void applyForces(const std::array<InteractionData, N_MAX_NEIGHBOURS>& data, const int n_last_neighbour, PhysicsComponent& physics);
    void communicate(std::array<SharedData, N_MAX_ENTITIES>& entity2shared_data)const;
};





