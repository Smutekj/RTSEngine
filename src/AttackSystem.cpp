
#include "AttackSystem.h"
#include "Grid.h"
#include "BoidControler.h"

AttackSystem::AttackSystem(BoidControler& bc, BoidWorld& world, HealthSystem* hs)
    : health_system_(hs)
    , p_bc_(&bc)
    , r_coords(world.r_coords_)
    , active_inds(world.active_inds) {
    
    Projectile p;

    projectile_factories_.emplace_back(p);
    units_weapons_.resize(N_MAX_NAVIGABLE_BOIDS);
    for(int i = 0; i < N_MAX_NAVIGABLE_BOIDS; ++i){
        units_weapons_.at(i).p_projectile_factory = &projectile_factories_.back();
    }
}
