#pragma once

#include "core.h"

struct InteractionComponent{
    u_int_16_t entity_ind;
    sf::Vector2f dr;
    sf::Vector2f dv;
    float r_collision;

};