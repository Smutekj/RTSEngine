
#ifndef BOIDS_HEALTHSYSTEM_H
#define BOIDS_HEALTHSYSTEM_H

#include "core.h"
#include <deque>

class HealthSystem {

    std::vector<float> healths_;

  public:
    std::deque<BoidInd> to_kill_; //! contains indices of units that died. is read by game

  public:
    HealthSystem() { healths_.resize(N_MAX_NAVIGABLE_BOIDS); }

    void changeHealth(int boid_ind, float value) {
        healths_.at(boid_ind) += value;
        if (healths_.at(boid_ind) <= 0) {
            to_kill_.push_back(boid_ind);
        }
    }

    void deactivate(BoidInd u_ind) {}

    void activate(const int boid_ind, const float health) {
        healths_.at(boid_ind) = health;
        //        healths_.push_back(player_ind, health);
    }
};
//
//
// class HealthComponent : Component{
//
//};

#endif // BOIDS_HEALTHSYSTEM_H
