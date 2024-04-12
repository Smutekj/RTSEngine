#pragma once

#include "../ECS.h"

#include "VisionSystem.h"
#include "HealthSystem.h"

struct Projectile : Component<ComponentID::PROJECTILE>
{
    TransformComponent transform;

    sf::Vector2f target_ = {0, 0};
    float time_alive = 0.;
    float damage = 1.0f;
    float speed = 50.f;
    u_int8_t proj_type = 0;
    Entity target_entity_ind = -1;
    u_int8_t player_ind;


    Projectile() = default;
    ~Projectile() = default;


    bool reachedTarget() const { return dist2(transform.r, target_) < 10; }
};


class ProjectileFactory
{

    Projectile blue_print_;
public:
    ProjectileFactory(const Projectile &blue_print)
        : blue_print_(blue_print) {}
    

    Projectile createProjectile(sf::Vector2f at, sf::Vector2f target, int target_entity_ind = -1) const
    {
        Projectile p(blue_print_);
        p.transform.r = at;
        p.target_ = target;
        p.target_entity_ind = target_entity_ind;
        return p;
    }
};

struct ProjectileSystem : System2{

    typedef ComponentArray<Projectile> CompArray;

    std::vector<int> projectile_inds_to_delete_;

    VisionSystem* p_vision_system_;
    HealthSystem* p_health_system_;

    ProjectileSystem(ComponentID comp);
    virtual ~ProjectileSystem() = default;

    void attack(Entity att_entity, Entity target_entity);
    void spawnProjectile(Entity att_entity, sf::Vector2f target);

    void update();
    // void draw(sf::RenderTarget &window);
    void setTransform(const int &ind, const TransformComponent &trans){}
    
    virtual void onComponentCreation(GraphicsComponent& comp){}

    virtual void updateSharedData(const std::array<SharedData, N_MAX_ENTITIES>& new_data, const std::vector<Entity>& active_entity_inds);
    virtual void communicate(std::array<SharedData, N_MAX_ENTITIES>& entity2shared_data)const;

    private:
        sf::VertexArray vertices_;

};
