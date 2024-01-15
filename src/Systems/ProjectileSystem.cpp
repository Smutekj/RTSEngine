#include "ProjectileSystem.h"


    ProjectileSystem::ProjectileSystem(ComponentID comp) : System2(comp) {
        vertices_.setPrimitiveType(sf::Triangles);
    }
    void ProjectileSystem::attack(Entity att_entity, Entity target_entity){}
    void ProjectileSystem::spawnProjectile(Entity att_entity, sf::Vector2f target){

    }

    void ProjectileSystem::update(){

        auto& comps = static_cast<CompArray&>(*p_comps_.get()).components_;
        const auto n_comps = comps.size();
        for (int comp_ind = 0; comp_ind < n_comps; ++comp_ind)
        {
            auto &projectile = comps[comp_ind];
            const auto target_ind = projectile.target_entity_ind.ind;

            auto dr = projectile.target_ - projectile.transform.r;
            dr /= norm(dr);
            projectile.transform.vel = dr * projectile.speed;
            projectile.time_alive += dt;

            if (projectile.reachedTarget())
            {
                const auto dmg = projectile.damage;
                projectile_inds_to_delete_.push_back(entity2compvec_ind_.at(comp_ind));
                if (projectile.target_entity_ind.ind != -1)
                {
                    p_health_system_->changeHealth(target_ind, -dmg);
                }else{
                    //! make some sound or whatever
                }
            }
        }
    }
    void ProjectileSystem::draw(sf::RenderTarget &window){
        const auto &comps = static_cast<ComponentArray<Projectile> &>(*p_comps_.get()).components_;
        vertices_.resize(3*comps.size());
        sf::ConvexShape tri(3);
        tri.setPoint(0, {2.f/3.f, 0.0});
        tri.setPoint(1, {-1.f/3.f,-std::sqrt(3.f)/3.f});
        tri.setPoint(2, {-1.f/3.f, std::sqrt(3.f)/3.f });
        
        for(int compvec_ind = 0; compvec_ind < comps.size(); ++compvec_ind){
            auto& comp = comps[compvec_ind];
            tri.setPosition(comp.transform.r);
            tri.rotate(comp.transform.angle);

            vertices_[compvec_ind*3 + 0] = {tri.getPoint(0), sf::Color::Black};
            vertices_[compvec_ind*3 + 1] = {tri.getPoint(1), sf::Color::Black};
            vertices_[compvec_ind*3 + 2] = {tri.getPoint(2), sf::Color::Black};
        }
        window.draw(vertices_);
    }
    void ProjectileSystem::updateSharedData(const std::array<SharedData, N_MAX_ENTITIES>& new_data, const std::vector<Entity>& active_entity_inds){
        auto &comps = static_cast<ComponentArray<Projectile> &>(*p_comps_.get()).components_;

        for(int compvec_ind = 0; compvec_ind < comps.size(); ++compvec_ind){
            auto& comp = comps.at(compvec_ind);
            comp.target_ = new_data.at(comp.target_entity_ind.ind).transform.r;
        }
    }
    void ProjectileSystem::communicate(std::array<SharedData, N_MAX_ENTITIES>& entity2shared_data)const{
        const auto &comps = static_cast<ComponentArray<Projectile> &>(*p_comps_.get()).components_;
        for(int compvec_ind = 0; compvec_ind < comps.size(); ++compvec_ind){
            entity2shared_data.at(compvec_ind2entity_ind_.at(compvec_ind)).transform.r = comps.at(compvec_ind).transform.r;
        }
    }
