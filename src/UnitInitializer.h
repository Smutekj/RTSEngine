#pragma once

#include "core.h"
#include <unordered_map>
#include <vector>
#include <string>

class ECSystem;
class UnitInitializer
{

    ECSystem *p_ecs_;

    struct UnitData
    {
        float radius = 3.f;
        float mass = 1.f;
        float r_vision = 20 * radius;
        float max_range = 17 * radius;
        float max_vel = 0.1f;
        float rot_vel = 5.f;
        int weapon_type_ind = 0;
    };

    enum class Data2 :  int
    {
        MAX_VEL = 0,
        RADIUS,
        ROTATION_VEL,
        MASS,
        VISION_RADIUS,
        WEAPON_RANGE,
        COUNT
    };

    int unit_entity_signature;

public:
    std::vector<UnitData> unit_type2data_;
    std::unordered_map<std::string, int> unit_name2type_ind;

    int selected_unit_type_ind = 0;


    std::vector<std::unordered_map<Data2, float>> type_ind2data_values;
    std::vector<std::unordered_map<Data2, int>> type_ind2data_inds;
    std::unordered_map<std::string, int> type_name2type_ind;
    std::vector<std::string> type_ind2name;


    const std::string unit_types_file_name = "/home/smutekj/projects/RTSEngineWithOpenGL/UnitTypes.txt"; 

private:


public:
    UnitInitializer(ECSystem &ecs) : p_ecs_(&ecs)
    {
        unit_type2data_.resize(1);
        unit_name2type_ind["default unit"] = 0;
    }

    void registerUnitType(float radius, float mass, int weapon_type_ind,
                          float r_vision, float max_vel, float rot_speed, int entity_signature, float max_range = 10 * RHARD, std::string unit_name = "default name");
    void initializeUnit(sf::Vector2f pos, int player_ind, sf::Vector2f vel = {0, 0}, float orient = 0);
};
