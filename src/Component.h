#include "core.h"

typedef u_int_32_t ComponentID;

struct Component {

    ComponentID id;
};

struct HealthComponent : public Component {};

struct WeaponComponent : public Component {};

struct PositionComponent : public Component{
    sf::Vector2f r;
};

struct PhysicsComponent : public Component{
    sf::Vector2f r;
};



constexpr int N_MAX_COMPONENTS = 64;

enum ComponentSignature : u_int_16_t {
    HEALTH = 1 << 0,
    WEAPON = 1 << 1,
    ARMOR = 1 << 2,
    BOID = 1 << 3,
    DRAWABLE = 1 << 4,
    CONTROL = 1 << 5
};

struct System {
    std::vector<Component> components_;

    void update() {
    }
};

struct PhysicsComponent{

    sf::Vector2f vel = {0,0};
    float radius = RHARD;
    u_int16_t turn_rate = 1000;
    u_int16_t acceleration = 1000;
    

};

class PhysicsSystem{
    //! x and y are separated because they are treated differently in neighbour searching algorithm
    std::array<float, N_MAX_NAVIGABLE_BOIDS> x_;
    std::array<float, N_MAX_NAVIGABLE_BOIDS> y_;

    std::vector<PhysicsComponent> components_;
    std::array<sf::Vector2f, N_MAX_NAVIGABLE_BOIDS> velocities;
    std::array<float, N_MAX_NAVIGABLE_BOIDS> radii;
    
    void update(){
        
    }

};

struct NavigationComponent{

}

class NavigationSystem{

};



struct ComponentSystem {

    std::vector<System> systems_;
    std::vector<Entity> agents_;

    std::vector<Entity> to_remove_;
    std::vector<Entity> to_add_;

    void update() {
        for (auto& system : systems_) {
            system.update();
        }
    }

    void addEntity() {}

    void removeEntity(int entity_) {

    }
};

struct Entity {

    ComponentID id;
    int n_components = 0;
    std::array<u_int_32_t, N_MAX_COMPONENTS> ind_in_component_vec;

    bool hasComponent(ComponentSignature c) const { return (id & c) == c; }
};

Entity createBoidEntity(){
    
}
