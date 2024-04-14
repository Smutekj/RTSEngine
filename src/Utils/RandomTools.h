#pragma once

#include "../core.h"
#include "../Game.h"
// #include "Graphics/SceneLayer.h"


inline float randf(const float min = 0, const float max = 1){
    return (rand() / static_cast<float>(RAND_MAX)) * (max - min) + min;
}

inline void generateRandomPositionsAroundBox(sf::Vector2f box_size, int n_positions, Game& game) {
    sf::Vector2f pos;
    for (int i = 0; i < n_positions / 4; ++i) {
        pos.x = rand() / static_cast<float>(RAND_MAX) * box_size.x / 10.f;
        pos.y = rand() / static_cast<float>(RAND_MAX) * box_size.y / 10.f;
        game.addUnit(0, pos, 0);
    }
    for (int i = 0; i < n_positions / 4; ++i) {
        pos.x = box_size.x - rand() / static_cast<float>(RAND_MAX) * box_size.x / 10.f;
        pos.y = rand() / static_cast<float>(RAND_MAX) * box_size.y / 10.f;
        game.addUnit(0, pos, 0);
    }
    for (int i = 0; i < n_positions / 4; ++i) {
        pos.x = rand() / static_cast<float>(RAND_MAX) * box_size.x / 10.f;
        pos.y = box_size.y - rand() / static_cast<float>(RAND_MAX) * box_size.y / 10.f;
        game.addUnit(1, pos, 0);
    }
    for (int i = 0; i < n_positions / 4; ++i) {
        pos.x = rand() / static_cast<float>(RAND_MAX) * box_size.x / 10.f;
        pos.y = rand() / static_cast<float>(RAND_MAX) * box_size.y / 10.f;
        game.addUnit(1, pos, 0);
    }

    // for (int i = 0; i < n_positions; ++i) {
    //     pos.x = rand() / static_cast<float>(RAND_MAX) * box_size.x / 5.f;
    //     pos.y = rand() / static_cast<float>(RAND_MAX) * box_size.y / 5.f;
    //     game.addUnit(0, pos, 0);
    // }
}


inline  sf::Vector2f randomPosInBox(sf::Vector2f ul_corner = {0,0},
                            sf::Vector2f box_size = {Geometry::BOX[0], Geometry::BOX[1]}
                            ){
    return {ul_corner.x + rand() / static_cast<float>(RAND_MAX) * box_size.x,
            ul_corner.y + rand() / static_cast<float>(RAND_MAX) * box_size.y };
}



struct Circle{
    sf::Vector2f r = {0,0}; //! center 
    float radius_sq = 1;
};

inline Circle randomCircle(const float r_max){
     return {randomPosInBox(), randf(0, r_max*r_max)}; 
}
inline  void generateRandomPositionsInCircles(const float density,
                                      sf::Vector2f box_size,
                                      int n_positions,
                                      Game& game
                                      ) {
    const int n_to_insert = n_positions;
    while(n_positions > 0){
        const auto circle = randomCircle(100);
        int n_in_circle = std::min({static_cast<int>(std::floor(M_PI*circle.radius_sq * density)), n_positions}); 
        const auto range_x = 2*std::sqrt(circle.radius_sq);
        const sf::Vector2f r0 = {circle.r.x - range_x/2.0f, circle.r.y - range_x/2.0f};
        sf::Vector2f r = r0;
        int player_ind =0;
        const float dx = std::sqrt(1.f/density); 
        while(n_in_circle > 0 and r.y < r0.y + range_x){
            n_positions > n_to_insert/2 ? player_ind = 0 : player_ind = 1;
            if(dist2(r, circle.r) < circle.radius_sq){
                game.addUnit(player_ind, r, 0);
                n_positions--;
                n_in_circle--;
            }
            r.x += dx;
            if(r.x > (r0.x + range_x) || r.x > Geometry::BOX[0]){
                r.x -= range_x;
                r.y += dx;
            }
        }
    }
}

inline  void generateRandomPositionsInCircles2(const float density,
                                      sf::Vector2f box_size,
                                      int n_positions,
                                      Game& game, SquareScene& scene
                                      ) {
    const int n_inserted = n_positions;
    while(n_positions > 0){
        const auto circle = randomCircle(100);
        int n_in_circle = std::min({static_cast<int>(std::floor(M_PI*circle.radius_sq * density)), n_positions}); 
        const auto range_x = 2*std::sqrt(circle.radius_sq);
        const sf::Vector2f r0 = {circle.r.x - range_x/2.0f, circle.r.y - range_x/2.0f};
        sf::Vector2f r = r0;
        int player_ind =0;
        n_positions > n_inserted/2 ? player_ind = 0 : player_ind = 1;
        const float dx = std::sqrt(1.f/density); 
        while(n_in_circle > 0 and r.y < r0.y + range_x){
            if(dist2(r, circle.r) < circle.radius_sq){
                game.addUnit(player_ind, r, 0);
                scene.createInstanceOf(0);
                n_positions--;
                n_in_circle--;
            }
            r.x += dx;
            if(r.x > (r0.x + range_x)){
                r.x -= range_x;
                r.y += dx;
            }
        }
    }
}

