#include "UnitInitializer.h"
#include "ECS.h"
#include "Systems/VisionSystem.h"
#include "Systems/GraphicsSystem.h"


    void UnitInitializer::registerUnitType(float radius, float mass, int weapon_type_ind,
                             float r_vision, float max_vel, float rot_speed, int entity_signature, float max_range, std::string unit_name){
        
        if(unit_name2type_ind.count(unit_name) != 0){
            std::cout << "unit with called: " + unit_name + " already exists!";
            return;
        }
        unit_type2data_.push_back({radius, mass, r_vision, max_range, max_vel, rot_speed, weapon_type_ind});
        unit_name2type_ind[unit_name] = unit_type2data_.size() - 1;

    }

    void UnitInitializer::initializeUnit(sf::Vector2f pos, int player_ind, sf::Vector2f vel, float orient){
        
        int unit_type_ind = selected_unit_type_ind;
        if(unit_type_ind >= unit_type2data_.size()){
            throw std::runtime_error("unit does not exist!");
        }

        const auto& unit_data = unit_type2data_.at(unit_type_ind); 

        TransformComponent tc;
        tc.r = pos;
        tc.vel = vel;
        tc.angle = orient;
        tc.angle_vel = 0;        

        PathFinderComponent pfc;
        pfc.transform = tc;

        VisionComponent vsc;
        vsc.trans = tc;
        vsc.player_ind = player_ind;
        vsc.vision_radius_sq = unit_data.r_vision*unit_data.r_vision;

        PhysicsComponent pc;
        pc.transform = tc;
        pc.mass = unit_data.mass;
        pc.radius = unit_data.radius;
        pc.player_ind = player_ind;

        AttackComponent ac;
        ac.transform = tc;
        ac.state = MoveState::STANDING;
        ac.weapon = Weapon();
        ac.player_ind = player_ind;


        auto new_ent_ind = p_ecs_->free_entity_inds_.top();
        
        HealthComponent hc;
        hc.health = 5;
        hc.max_health = 50;
        hc.health_regen = 0.1;
        hc.p_shared_data_ = &p_ecs_->entity2shared_data.at(new_ent_ind).transform;

        GraphicsComponent gc;
        gc.transform = tc;
        gc.player_ind = player_ind;
        gc.p_shared_data = &p_ecs_->entity2shared_data.at(new_ent_ind).health;

        p_ecs_->getSystem<GraphicsSystem>(ComponentID::GRAPHICS).onComponentCreation(gc);

        // pc.v_max = unit_data.max_vel;
        p_ecs_->entity2shared_data.at(new_ent_ind) = {tc, MoveState::STANDING};
        p_ecs_->entity2shared_data.at(new_ent_ind).health.p_shared_data_ = &p_ecs_->entity2shared_data.at(new_ent_ind).transform;

        p_ecs_->initializeEntity(ac, tc, pc, vsc, pfc, hc, gc);
    }
    