#include "ECS.h"

#include "Systems/PhysicsSystem.h"
#include "Systems/TransformSystem.h"
#include "Systems/SeekSystem.h"
#include "Systems/GraphicsSystem.h"
#include "Systems/VisionSystem.h"
#include "Systems/HealthSystem.h"
#include "Systems/AttackSystem.h"

#include "Edges.h"

ECSystem::ECSystem(Edges &map_grid) : free_entity_inds_(static_cast<size_t>(N_MAX_ENTITIES))
{

    systems2_.at(ComponentID::PHYSICS) = std::make_unique<PhysicsSystem>(ComponentID::PHYSICS);
    systems2_.at(ComponentID::PATHFINDING) = std::make_unique<SeekSystem>(ComponentID::PATHFINDING);
    systems2_.at(ComponentID::GRAPHICS) = std::make_unique<GraphicsSystem>(ComponentID::GRAPHICS);
    systems2_.at(ComponentID::VISION) = std::make_unique<VisionSystem>(ComponentID::VISION);
    systems2_.at(ComponentID::TRANSFORM) = std::make_unique<TransformSystem>(ComponentID::TRANSFORM);
    systems2_.at(ComponentID::HEALTH) = std::make_unique<HealthSystem>(ComponentID::HEALTH);
    systems2_.at(ComponentID::ATTACK) = std::make_unique<AttackSystem2>(ComponentID::ATTACK);
    systems2_.at(ComponentID::PROJECTILE) = std::make_unique<AttackSystem2>(ComponentID::PROJECTILE);

    systems2_.at(ComponentID::TRANSFORM)->p_comps_ = std::make_shared<ComponentArray<TransformComponent>>();
    systems2_.at(ComponentID::PHYSICS)->p_comps_ = std::make_shared<ComponentArray<PhysicsComponent>>();
    systems2_.at(ComponentID::VISION)->p_comps_ = std::make_shared<ComponentArray<VisionComponent>>();
    systems2_.at(ComponentID::PATHFINDING)->p_comps_ = std::make_shared<ComponentArray<PathFinderComponent>>();
    systems2_.at(ComponentID::GRAPHICS)->p_comps_ = std::make_shared<ComponentArray<GraphicsComponent>>();
    systems2_.at(ComponentID::HEALTH)->p_comps_ = std::make_shared<ComponentArray<HealthComponent>>();
    systems2_.at(ComponentID::ATTACK)->p_comps_ = std::make_shared<ComponentArray<AttackComponent>>();
    systems2_.at(ComponentID::PROJECTILE)->p_comps_ = std::make_shared<ComponentArray<Projectile>>();

    //! connect systems here
    auto &as = getSystem<AttackSystem2>(ComponentID::ATTACK);
    as.p_health_system_ = static_cast<HealthSystem *>(systems2_.at(ComponentID::HEALTH).get());
    as.p_vision_system_ = static_cast<VisionSystem *>(systems2_.at(ComponentID::VISION).get());

    auto &ps = getSystem<PhysicsSystem>(ComponentID::PHYSICS);
    ps.p_map_ = &map_grid;

    auto &seek_s = getSystem<SeekSystem>(ComponentID::PATHFINDING);

    auto &projs = getSystem<ProjectileSystem>(ComponentID::PROJECTILE);
    projs.p_health_system_ = static_cast<HealthSystem *>(systems2_.at(ComponentID::HEALTH).get());
    projs.p_vision_system_ = static_cast<VisionSystem *>(systems2_.at(ComponentID::VISION).get());

    for (auto &system : systems2_)
    {
        system_time_statistics_.at(system->id) = std::make_unique<TimeData2>();
    }

    system2name_.at(ComponentID::TRANSFORM) = "transform";
    system2name_.at(ComponentID::VISION) = "vision";
    system2name_.at(ComponentID::PATHFINDING) = "pathfinder";
    system2name_.at(ComponentID::GRAPHICS) = "graphics";
    system2name_.at(ComponentID::HEALTH) = "health";
    system2name_.at(ComponentID::PHYSICS) = "physics";
    system2name_.at(ComponentID::ATTACK) = "attack";

    entity2ind_in_active_inds_.fill(-1);
}

int frame_ind = 0;
void ECSystem::update()
{

    std::chrono::high_resolution_clock clock;

    //! update systems with new data
    for (auto &system : systems2_)
    {
        system->updateSharedData(entity2shared_data, active_entities_);
    }

    for (auto &system : systems2_)
    {

        auto t_start = clock.now();
        system->update();
        auto delta_t = std::chrono::duration_cast<std::chrono::microseconds>(clock.now() - t_start);
        system_time_statistics_.at(system->id)->addTime(delta_t.count());
    }

    auto t_start = clock.now();
    //! communicate transforms from master vector to systems
    for (auto &system : systems2_)
    {
        system->communicate(entity2shared_data);
    }

    auto &ps = getSystem<PhysicsSystem &>(ComponentID::PHYSICS);
    ps.updateSharedData(entity2shared_data, active_entities_);
    ps.avoidWall();
    ps.communicateVelocities(entity2shared_data);

    //! integrate
    for (auto ent : active_entities_)
    {
        auto &data = entity2shared_data.at(ent.ind);
        auto &tc = data.transform;
        auto seek_dir = data.target - tc.r;
        // seek_dir /= norm(seek_dir);
        // tc.vel += seek_dir * 5.f * dt;
        // if(norm(tc.vel) > 25){
        //     tc.vel = tc.vel / norm(tc.vel)* 25.f;
        // }
        // if(data.state != MoveState::MOVING){
        // }
        tc.r += tc.vel * dt;
        tc.angle += tc.angle_vel;
        tc.vel *= 0.f;
    }

    auto delta_t = std::chrono::duration_cast<std::chrono::microseconds>(clock.now() - t_start);
    system_time_statistics_.at(ComponentID::TRANSFORM)->addTime(delta_t.count());

    if ((frame_ind) % 500 == 0)
    {
        for (auto &system : systems2_)
        {
            std::cout << "system: " << system2name_.at(system->id) << " took on avg: "
                      << system_time_statistics_.at(system->id)->avg_time << " us\n";
        }
        frame_ind = 0;
    }
    frame_ind++;

    auto &as = getSystem<AttackSystem2 &>(ComponentID::ATTACK);
    auto &proj_s = getSystem<ProjectileSystem &>(ComponentID::PROJECTILE);

    while (!as.projectiles_to_spawn_.empty())
    {
        initializeEntity(as.projectiles_to_spawn_.back());
        as.projectiles_to_spawn_.pop_back();
    }
    while (!proj_s.projectile_inds_to_delete_.empty())
    {
        removeEntity(proj_s.projectile_inds_to_delete_.back());
        proj_s.projectile_inds_to_delete_.pop_back();
    }
}
