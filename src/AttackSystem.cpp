
#include "AttackSystem.h"
#include "Grid.h"
#include "BoidControler.h"

AttackSystem::AttackSystem(BoidWorld& world, HealthSystem* hs)
    : health_system_(hs)
    , r_coords(world.r_coords_)
    , active_inds(world.active_inds) {
    units_weapons_.resize(N_MAX_NAVIGABLE_BOIDS);
}
