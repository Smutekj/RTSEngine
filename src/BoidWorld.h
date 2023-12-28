#include "core.h"
#pragma once

class BoidControler;

enum class MoveState { MOVING, STANDING, HOLDING };


struct BoidWorld {

    std::vector<int> boid_inds2cellind_;
    std::vector<int> boid_inds2ind_in_cell_;
    std::vector<MoveState> move_states_;

    std::vector<float> orientation_;
    std::vector<float> radii;

    std::vector<std::vector<int>> player2active_inds;
    std::vector<std::vector<int>> player2inactive_inds;

    std::vector<int> active_inds;
    std::vector<int> inactive_inds;

    std::vector<int> ind_in_active_inds;
    std::vector<int> ind2player;

    std::vector<sf::Vector2f> r_coords_;
    std::vector<sf::Vector2f> velocities_;
    std::vector<sf::Vector2f> intertia_velocities_;

    size_t n_active_ = 0;
    size_t last_active_ind = -1;
    std::vector<int> n_alive_of_players_;

    sf::VertexArray vertices_;

    void update(float dt);

    int activate(int player_ind, sf::Vector2f r_coord, sf::Vector2f vel);

    void draw(sf::RenderWindow& window);
    void draw(sf::RenderWindow& window, BoidControler& bc);

    void deactivate(int u_ind);

    BoidWorld() {
        const auto max_boid_ind = N_MAX_NAVIGABLE_BOIDS; // MAX_UNITS_PER_PLAYER*N_PLAYERS;
        r_coords_.resize(max_boid_ind);
        velocities_.resize(max_boid_ind, {0, 0});
        intertia_velocities_.resize(max_boid_ind, {0, 0});
        boid_inds2cellind_.resize(max_boid_ind);
        boid_inds2ind_in_cell_.resize(max_boid_ind);
        move_states_.resize(max_boid_ind);

        ind2player.resize(max_boid_ind, 0);
        ind_in_active_inds.resize(max_boid_ind);
        radii.resize(N_MAX_NAVIGABLE_BOIDS);

        n_alive_of_players_.resize(N_PLAYERS, 0);
        player2active_inds.resize(N_PLAYERS);
        player2inactive_inds.resize(N_PLAYERS);
    }
};
