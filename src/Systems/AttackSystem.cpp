
#include "AttackSystem.h"
#include "../Utils/NeighbourSearcherStrategy.h"

// AttackSystem::AttackSystem(BoidControler& bc, BoidWorld& world, HealthSystem* hs)
//     : health_system_(hs)
//     , p_bc_(&bc)
//     , r_coords(world.r_coords_)
//     , active_inds(world.active_inds) {
    
//     Projectile p;

//     projectile_factories_.emplace_back(p);
//     units_weapons_.resize(N_MAX_NAVIGABLE_BOIDS);
//     for(int i = 0; i < N_MAX_NAVIGABLE_BOIDS; ++i){
//         units_weapons_.at(i).p_projectile_factory = &projectile_factories_.back();
//     }
// }

AttackSystem2::AttackSystem2(ComponentID comp) : System2(comp)
    {
        sf::Vector2f box = {Geometry::BOX[0], Geometry::BOX[1]};
        // p_ns_ = std::make_shared<NeighbourSearcherT<AttackComponent, int>>(box, 50 * RHARD);
        p_neighbour_searcher_ = std::make_shared<NeighbourSearcherContext<AttackComponent, int>>(box, 20 * RHARD);
        p_neighbour_searcher_->setStrategy(NS_STRATEGY::ATTACK);
    }

void AttackSystem2::attack(Entity att_entity, Entity target_entity, float distance)
    {
        const auto att_ind = att_entity.ind;
        const auto target_ind = att_entity.ind;
        const auto attacker_compvec_ind = entity2compvec_ind_.at(att_entity.ind);

        auto &comps = static_cast<ComponentArray<AttackComponent> &>(*p_comps_.get()).components_;

        auto& weapon = comps.at(attacker_compvec_ind).weapon;
        if (!weapon.isReloading() && distance < weapon.range)
        {
            weapon.time_since_shooting = 0.f;
            if (weapon.shootsProjectiles())
            {
                const auto target_compvec_ind = entity2compvec_ind_.at(att_entity.ind);
                const auto r_attacker = comps.at(attacker_compvec_ind).transform.r;
                const auto r_target = comps.at(target_compvec_ind).transform.r;
                const auto new_projectile = spawnProjectile(weapon, r_attacker, r_target, target_ind);
                projectiles_to_spawn_.push_back(std::move(new_projectile));
            }
            else
            {
                p_health_system_->changeHealth(target_entity, -weapon.damage);
            }
        }
        else
        {
            weapon.time_since_shooting += 1;
        }
    }

    Projectile AttackSystem2::spawnProjectile(Weapon& weapon, sf::Vector2f r_attacker, sf::Vector2f r_target, int target_ent_ind){
        auto& factory = projectile_factories_.at(weapon.proj_factory_ind);
        return factory.createProjectile(r_attacker, r_target, target_ent_ind);
    }

    void AttackSystem2::update()
    {
        auto &components = static_cast<CompArray&>(*p_comps_.get()).components_;

        p_neighbour_searcher_->update(components);

        //! find targets
        int compvec_ind = 0;
        for (auto &comp : components)
        {
            findTarget(comp, compvec_ind);
            compvec_ind++;
        }
        
        //! attack
        int comp_ind = 0;
        for (auto &comp : components)
        {
            auto &weapon = comp.weapon;
            Entity attack_target(0, comp.target_entity_ind);
            
            if (attack_target.ind == -1)
            {
                continue;
            }
            if(entity2compvec_ind_.at(comp.target_entity_ind) == -1){ //! target has been already destroyed
                comp.target_entity_ind = -1;
                continue;
            }

            Entity shooter_entity = {0, compvec_ind2entity_ind_.at(comp_ind)};
            const auto& target_comp = components.at(entity2compvec_ind_.at(comp.target_entity_ind));
            const float distance = dist(comp.transform.r, target_comp .transform.r);
            attack(shooter_entity, attack_target, distance);
        }
        comp_ind++;
    }

    void AttackSystem2::updateSharedData(const std::array<SharedData, N_MAX_ENTITIES>& new_data, const std::vector<Entity>& active_entity_inds)
    {
        auto &comps = static_cast<ComponentArray<AttackComponent> &>(*p_comps_.get()).components_;
        for(const auto entity : active_entity_inds){
            if(entity.hasComponent(id)){
                const auto compvec_ind = entity2compvec_ind_.at(entity.ind);
                comps.at(compvec_ind).transform = new_data.at(entity.ind).transform;
                comps.at(compvec_ind).state = new_data.at(entity.ind).state;
            }
        }
    }
    
    void AttackSystem2::communicate(std::array<SharedData, N_MAX_ENTITIES>& entity2shared_data)const{
        int comp_ind = 0;
        auto& components = static_cast<CompArray&>(*p_comps_).components_;

        for(const auto& comp : components){
            auto entity_ind = compvec_ind2entity_ind_.at(comp_ind);
            entity2shared_data.at(entity_ind).state = comp.state;
            entity2shared_data.at(entity_ind).transform.r = comp.transform.r;
            entity2shared_data.at(entity_ind).transform.vel += comp.transform.vel;
            comp_ind++;
        }
    }

    void AttackSystem2::shootAt(Entity att_entity, sf::Vector2f target)
     {
        auto& components = static_cast<CompArray&>(*p_comps_).components_;
        auto attacker_component_ind = entity2compvec_ind_.at(att_entity.ind);
        auto& comp = components.at(attacker_component_ind);
        auto &weapon = comp.weapon;
        if (weapon.time_since_shooting > weapon.reload_time)
        {
            weapon.time_since_shooting = 0.f;
            const auto &att_weapon = units_weapons_[attacker_component_ind];
            const auto r_attacker = components.at(attacker_component_ind).transform.r;
            const auto new_projectile = spawnProjectile(weapon, r_attacker, target);
            projectiles_to_spawn_.push_back(std::move(new_projectile));
        }
    }


    // void AttackSystem2::draw(sf::RenderTarget &window)
    // {
        
    // }

    void AttackSystem2::setTransform(const int &ind, const TransformComponent &trans)
    {
        if(entity2compvec_ind_.at(ind) != -1){
            auto &comps = static_cast<ComponentArray<AttackComponent> &>(*p_comps_.get()).components_;
            comps.at(entity2compvec_ind_.at(ind)).transform = trans;
        }
    }


