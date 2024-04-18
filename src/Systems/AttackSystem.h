#ifndef BOIDS_ATTACKSYSTEM_H
#define BOIDS_ATTACKSYSTEM_H

#include "../core.h"
#include "../ECS.h"
#include "HealthSystem.h"
#include "VisionSystem.h"
#include "ProjectileSystem.h"

#include "../Utils/NeighbourSearcherContext.h"

typedef int WeaponTypeInd;

struct AttackSystem2 : System2
{
    std::vector<Weapon> units_weapons_;
    std::vector<Projectile> projectiles_to_spawn_;
    std::vector<int> projectile2shooter_ind_;

    std::vector<ProjectileFactory> projectile_factories_;

    // std::shared_ptr<NeighbourSearcherT<AttackComponent, 500>> p_ns_;
    std::shared_ptr<NeighbourSearcherContext<AttackComponent, int>> p_neighbour_searcher_;

    HealthSystem *p_health_system_;
    VisionSystem *p_vision_system_;
    ProjectileSystem *p_projectile_system_;

    typedef ComponentArray<AttackComponent> CompArray;

public:
    AttackSystem2(ComponentID comp);
    virtual ~AttackSystem2() = default;

    void attack(Entity att_entity, Entity target_entity, float distance);
    void shootAt(Entity att_entity, sf::Vector2f target);
    Projectile spawnProjectile(Weapon &weapon, sf::Vector2f r_attacker, sf::Vector2f r_target, int target_ent_ind = -1);

    void update();

    // void draw(sf::RenderTarget &window);
    virtual void onComponentCreation(GraphicsComponent& comp){}

    void setTransform(const int &ind, const TransformComponent &trans);

    virtual void updateSharedData(const std::array<SharedData, N_MAX_ENTITIES> &new_data, const std::vector<Entity> &active_entity_inds);
    virtual void communicate(std::array<SharedData, N_MAX_ENTITIES> &entity2shared_data) const;

private:
    void findTarget(AttackComponent &comp, int compvec_ind)
    {
        auto &comps = static_cast<ComponentArray<AttackComponent> &>(*p_comps_.get()).components_;
        
        const auto closest_enemy_comp_ind = p_neighbour_searcher_->getInteractionData(compvec_ind).at(0);
        const auto n_enemy_neighbours = p_neighbour_searcher_->last_i.at(compvec_ind);

        if (comp.target_entity_ind == -1)
        {
            if (closest_enemy_comp_ind != -1 && n_enemy_neighbours != 0)
            {
                comp.target_entity_ind = compvec_ind2entity_ind_.at(closest_enemy_comp_ind);
            }
        }
        else{
            if(entity2compvec_ind_.at(comp.target_entity_ind) == -1) {return;}
            
            const auto& target_comp = comps.at(entity2compvec_ind_.at(comp.target_entity_ind));
            const float distance = dist(comp.transform.r, target_comp .transform.r);
            if(distance > comp.weapon.range && closest_enemy_comp_ind != -1 && n_enemy_neighbours != 0){
                comp.target_entity_ind = compvec_ind2entity_ind_.at(closest_enemy_comp_ind);
            }
        }
    }
    
};

#endif // BOIDS_ATTACKSYSTEM_H
