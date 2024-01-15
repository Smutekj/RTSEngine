#include "HealthSystem.h"

void HealthSystem::updateSharedData(const std::array<SharedData, N_MAX_ENTITIES> &new_data, const std::vector<Entity> &active_entity_inds)
{
    auto &comps = static_cast<ComponentArray<HealthComponent> &>(*p_comps_.get()).components_;
    for (const auto ent : active_entity_inds)
    {
        const auto compvec_ind = entity2compvec_ind_.at(ent.ind);
        comps.at(compvec_ind) = new_data.at(ent.ind).health;
        // comps.at(compvec_ind).state = new_data.at(ent_ind).state;
    }
}

void HealthSystem::communicate(std::array<SharedData, N_MAX_ENTITIES> &entity2shared_data) const
{
    auto &comps = static_cast<ComponentArray<HealthComponent> &>(*p_comps_.get()).components_;
    int comp_ind = 0;
    for (const auto &comp : comps)
    {
        auto entity_ind = compvec_ind2entity_ind_.at(comp_ind);
        entity2shared_data.at(entity_ind).health = comp;
        comp_ind++;
    }
}

void HealthSystem::changeHealth(Entity e, float value)
{
    auto compvec_ind = entity2compvec_ind_.at(e.ind);
    auto &comp = static_cast<CompArray &>(*p_comps_).components_.at(compvec_ind);
    comp.health = std::min({comp.max_health, comp.health + value});
    if (comp.health <= 0 && dead_entity_inds_.count(e.ind) == 0)
    {
        dead_entity_inds_.insert(e.ind);
    }
}
