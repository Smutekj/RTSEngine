#pragma once

#include "../ECS.h"


struct TransformSystem : System2{

    typedef ComponentArray<TransformComponent> CompArray;

    TransformSystem(ComponentID id) : System2(id){}
    ~TransformSystem() = default;

    void update(){
        auto& components = static_cast<CompArray&>(*p_comps_).components_;
        
        int compvec_ind = 0;
        // for(auto& comp : components){
        //     comp.r += comp.vel * dt;
        //     comp.angle += comp.angle_vel * dt; 
        //     // entity2compvec_ind_.at(compvec_ind2entity_ind_.at(compvec_ind)) = comp;
        //     compvec_ind++;
        // }
    }

    void draw(sf::RenderTarget& target){}
    void setTransform(const int& e, const TransformComponent& trans){}
    void updateSharedData(const std::array<SharedData, N_MAX_ENTITIES>& new_data, const std::vector<Entity>& active_entity_inds)
    {
        auto &comps = static_cast<ComponentArray<TransformComponent> &>(*p_comps_.get()).components_;
        for(const auto ent : active_entity_inds){
            const auto compvec_ind = entity2compvec_ind_.at(ent.ind);
            // comps.at(compvec_ind).transform = new_data.at(ent_ind).transform;
            // comps.at(compvec_ind).state = new_data.at(ent_ind).state;
        }
    }
    virtual void communicate(std::array<SharedData, N_MAX_ENTITIES>& entity2shared_data)const{
        
    }
};
