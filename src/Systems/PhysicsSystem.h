#pragma once

#include "../ECS.h"
#include "../Edges.h"
#include "../Utils/NeighbourSearcherContext.h"

constexpr int N_CELLS_PHYSICS_NS = 100;
constexpr float R_MAX_PHYSICS_NS = 20;


// struct WallInteractionComponent : Component<ComponentID::WALL>{
//     TransformComponent transform;
//     float radius;
// }


struct PhysicsSystem : System2{

    typedef ComponentArray<PhysicsComponent> CompArray;

    enum class Multiplier{
        PUSH,
        REPULSE,
        SCATTER,
        ALIGN,
        SEEK,
        DECAY,
        VELOCITY
    };

    std::unordered_map<Multiplier, float> force_multipliers;

    std::vector<sf::Vector2f> component2cumulative_force_;
    std::unique_ptr<NeighbourSearcherContext<PhysicsComponent, InteractionData>> p_ns_;
    Edges* p_map_;

    float max_speed = 25.0f;

    const float dt = 1/60.f;

    PhysicsSystem(ComponentID id);
    ~PhysicsSystem() = default;
    
    void update(); 
    void avoidWall();
    // void draw(sf::RenderTarget& target);
    void setTransform(const int& e, const TransformComponent& trans){}
    
    void updateSharedData(const std::array<SharedData, N_MAX_ENTITIES>& new_data, const std::vector<Entity>& active_entity_inds);
    void communicateVelocities(std::array<SharedData, N_MAX_ENTITIES>& entity2shared_data)const;
    std::vector<PhysicsComponent>& getComponents(){
        return static_cast<CompArray&>(*p_comps_).components_;
    }

    virtual void onComponentCreation(GraphicsComponent& comp){}

    private:
    void applyForces(const std::array<InteractionData, N_MAX_NEIGHBOURS>& data, const int n_last_neighbour, PhysicsComponent& physics);
    void communicate(std::array<SharedData, N_MAX_ENTITIES>& entity2shared_data)const;
};





