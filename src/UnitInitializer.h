#pragma once

#include "core.h"
#include <unordered_map>


class ECSystem;
class UnitInitializer{

    ECSystem* p_ecs_;

    struct UnitData{
        float radius = 3.f;
        float mass = 1.f;
        float r_vision = 20*radius;
        float max_range = 17*radius;
        float max_vel = 0.1f;
        int weapon_type_ind = 0;

    };
    int unit_entity_signature;
    std::vector<UnitData> unit_type2data_;
    std::unordered_map<std::string, int> unit_name2type_ind; 

public:
    UnitInitializer(ECSystem& ecs) : p_ecs_(&ecs), unit_type2data_(1) {
        
    }

    void registerUnitType(float radius, float mass, int weapon_type_ind, float r_vision, float max_vel, 
                            float max_range = 10*RHARD, std::string unit_name = "Peter");
    void initializeUnit(sf::Vector2f pos, int player_ind, sf::Vector2f vel = {0,0}, float orient = 0, int unit_type_ind = 0);
    
};
