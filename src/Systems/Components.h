#pragma once

#include "../core.h"

enum ComponentID : int
{
    TRANSFORM = 0,
    HEALTH,
    ATTACK,
    GRAPHICS,
    PHYSICS,
    PATHFINDING,
    PROJECTILE,
    VISION,
    N_COMPS
};


constexpr int MAX_COMPONENTS = ComponentID::N_COMPS;

class ProjectileFactory;
struct Weapon
{
    float time_since_shooting = 60;
    float reload_time = 60;
    float damage = 1;
    float range = 10;
    int proj_factory_ind = -1;

    Weapon(float damage, float range, float reload_time, int proj_factory_ind)
        : reload_time(reload_time), damage(damage), range(range), proj_factory_ind(proj_factory_ind) {}
    Weapon() = default;
    
    bool isReloading(){
        return time_since_shooting < reload_time;
    }

    bool shootsProjectiles() const
    {
        return proj_factory_ind != -1;
    }
};




struct BaseComponent
{
};

template <ComponentID id>
struct Component : BaseComponent
{
};

struct TransformComponent : Component<ComponentID::TRANSFORM>
{
    sf::Vector2f r;
    sf::Vector2f vel = {0, 0};
    float angle = 0.f;
    float angle_vel = 0.f;
};

struct HealthComponent : Component<ComponentID::HEALTH>{
    float health = 10.f;
    float max_health = health;
    float health_regen = 0;
    int health_type;
    int player_ind = 0;
    TransformComponent* p_shared_data_;
};

struct AttackComponent : Component<ComponentID::ATTACK>
{
    TransformComponent transform;
    int target_entity_ind = -1;
    sf::Vector2f r_target;
    MoveState state;
    int player_ind;
    Weapon weapon;
};


struct PhysicsComponent : Component<ComponentID::PHYSICS>
{
    TransformComponent transform;
    float radius = 3;
    float mass = M_PIf * radius * radius;
    float lambda = 0.169f; //! velocity slowing down parameter (nice)
    MoveState state = MoveState::STANDING;
    int player_ind;
    sf::Vector2f inertia_vel = {0.f, 0.f};
    float max_speed = 25.f;
};

struct PathFinderComponent : Component<ComponentID::PATHFINDING>
{
    TransformComponent transform;
    MoveState state = MoveState::STANDING;
    
    sf::Vector2f r_next;
    Edgef portal_next;
    sf::Vector2f r_next_next;
    Edgef portal_next_next;
    sf::Vector2f path_end;
    sf::Vector2f standing_pos;

    int start_tri_ind;
    int next_tri_ind;
    int next_next_tri_ind;

    int n_steps_since_all_in_front_standing = 0;
    float radius = RHARD;
    float max_speed = 50.f;
    float turn_rate = 5; // degrees / frame 
    float desired_angle = 0; // degrees
    bool needs_update = true;

};

struct Graph;

struct GraphicsComponent : Component<ComponentID::GRAPHICS>
{
    TransformComponent transform;
    int graphics_ind;
    int sprite_sheet_ind;
    int player_ind = 0;
    int instance_ind = -1;
    float radius = 3.f;
    HealthComponent* p_shared_data = nullptr;
    Graph* p_graph = nullptr;
};

template <class CompType>
struct ComponentAndInd
{
    CompType comp;
    int comp_ind;
};


