#pragma once

#include <unordered_map>
#include "Utils/Vector.hpp"

struct BuildingData{
    sf::Vector2i size = {4,4};
    int corner_size = 1;
    int graphical_id = 0;
};


class BuildingManager{
    std::unordered_map<int, BuildingData> building_type2data;

public:

    BuildingManager(){
        building_type2data[0] = {{8,8}, 1, 0};
        building_type2data[1] = {{4,6}, 1, 1};
    }

    const auto& getSize(int building_type)const{
        return building_type2data.at(building_type).size;
    }
    const auto& getCornerSize(int building_type)const{
        return building_type2data.at(building_type).corner_size;
    }
    const auto& getGID(int building_type)const{
        return building_type2data.at(building_type).graphical_id;
    }
    const auto& getData()const{
        return building_type2data;
    }
};
