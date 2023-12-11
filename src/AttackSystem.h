//
// Created by smutekj on 21.08.23.
//

#ifndef BOIDS_ATTACKSYSTEM_H
#define BOIDS_ATTACKSYSTEM_H

#include "core.h"

// struct ProjectileType {
//
//
// };

typedef int WeaponTypeInd;

struct Projectile {

    sf::CircleShape shape;
    sf::Vector2f target_;
    sf::Vector2f position_;
    float speed_ = 50;
    float time_alive = 0.;
    float damage = 1.0f;
    BoidInd target_ind = -1;
    //    BoidInd shooting_unit_ind;

    Projectile() {
        //        shape.setPointCount(3);
        shape.setRadius(1.f);
        //        shape.setPoint(0, {0,0});
        //        shape.setPoint(1, {1,0});
        //        shape.setPoint(2, {0.5, 2.0});
        shape.setFillColor(sf::Color::Black);
    }

    void update(float dt) {}

    bool reachedTarget() const { return dist2(position_, target_) < 10; }
};

class ProjectileFactory {

    Projectile blue_print_;

  public:
    ProjectileFactory(const Projectile& blue_print)
        : blue_print_(blue_print) {}

    Projectile createProjectile(sf::Vector2f at, sf::Vector2f target, int target_ind = -1) const {
        Projectile p(blue_print_);
        p.position_ = at;
        p.target_ = target;
        p.target_ind = target_ind;
        p.shape.setPosition(at);
        return p;
    }
};

struct Weapon {

    float time_since_shooting = 0;
    float reload_time = 16;
    float damage = 1;
    float range = RHARD;
    ProjectileFactory* p_projectile_factory = nullptr;

    Weapon(float damage, float range, float reload_time, ProjectileFactory* proj_producer)
        : reload_time(reload_time)
        , damage(damage)
        , range(range)
        , p_projectile_factory(proj_producer) {}
    Weapon() = default;

    Projectile shootAt(sf::Vector2f shooter_position, sf::Vector2f target, int target_ind = -1) const {
        return p_projectile_factory->createProjectile(shooter_position, target, target_ind);
    }

    bool shootsProjectiles() const {
         return p_projectile_factory != nullptr; }
};
//
// enum ComponentID {
//    Attack,
//    Draw,
//
//};
//
//
// class Component{
//
//    u_int32_t value = 0;
//
//    virtual void removeComponent(ComponentID c){
//
//    }
//};

class BoidWorld;
#include "HealthSystem.h"
#include "BoidControler.h"

class AttackSystem {

    std::vector<Weapon> units_weapons_;

    std::vector<ProjectileFactory> projectile_factories_;
    std::vector<Projectile> projectiles_;

    //    std::deque<HitInfo> hits_to_resolve_;

    std::vector<sf::Vector2f>& r_coords;
    const std::vector<int>& active_inds;
    HealthSystem* health_system_;

  public:
    AttackSystem(BoidWorld& world, HealthSystem* hs);

    void attack(BoidInd att_ind, BoidInd tar_ind) {
        auto& weapon = units_weapons_.at(att_ind);
        if (weapon.time_since_shooting > weapon.reload_time) {
            weapon.time_since_shooting = 0.f;
            if (weapon.shootsProjectiles()) {
                const auto new_projectile = weapon.shootAt(r_coords[att_ind], r_coords[tar_ind], tar_ind);
                projectiles_.push_back(std::move(new_projectile));
            } else {
                health_system_->changeHealth(tar_ind, -weapon.damage);
            }
        } else {
            weapon.time_since_shooting += 1;
        }
    }

    void shootAt(BoidInd att_ind, sf::Vector2f target) {
        auto& weapon = units_weapons_.at(att_ind);
        if (weapon.time_since_shooting > weapon.reload_time) {
            weapon.time_since_shooting = 0.f;
            const auto& att_weapon = units_weapons_[att_ind];
            const auto new_projectile = att_weapon.shootAt(r_coords[att_ind], target);
            projectiles_.push_back(std::move(new_projectile));
        }
    }

    void update(float dt) {

        std::vector<int> projectiles_to_delete;

        for (int proj_ind = 0; proj_ind < projectiles_.size(); ++proj_ind) {
            auto& projectile = projectiles_[proj_ind];
            if (projectile.target_ind != -1) {
                projectile.target_ = r_coords[projectile.target_ind];
            }
            const auto target_ind = projectile.target_ind;

            //            if(boid2unit(projectile.target_ind).offset >=
            //            units_weapons_.n_alive_[boid2unit(projectile.target_ind).player_ind]){
            //                throw std::runtime_error("");
            //            }

            auto dr = projectile.target_ - projectile.position_;
            dr /= norm(dr);
            projectile.speed_ = 50;
            projectile.position_ += dr * projectile.speed_ * dt;
            projectile.time_alive += dt;
            projectile.shape.setPosition(projectile.position_);

            if (projectile.reachedTarget()) {
                const auto dmg = projectile.damage;
                projectiles_to_delete.push_back(proj_ind);
                if (projectile.target_ind != -1) {
                    health_system_->changeHealth(target_ind, -dmg);

                    if (!health_system_->to_kill_.empty()) {
                        if (health_system_->to_kill_.back() == target_ind) {
                            for (auto& proj2 : projectiles_) {
                                if (target_ind == proj2.target_ind) {
                                    proj2.target_ind = -1;
                                    proj2.target_ = projectile.target_;
                                }
                            }
                        }
                    }
                } else {
                    //                    throw std::runtime_error("not implemented yet!");
                }
            }
        }

        for (int proj_ind : projectiles_to_delete) {
            projectiles_.erase(projectiles_.begin() + proj_ind);
        }

        for (int active_ind : active_inds) {
            units_weapons_[active_ind].time_since_shooting += dt;
        }
    }

    void activate(int player_ind, int weapon_type_ind) {
            //    units_weapons_.push_back(Weapon());
            //    unit_weapon_types_.push_back(player_ind, 0);
    }
    void deactivate(BoidInd boid_ind) {
            //    units_weapons_.remove(u_ind);
            //    unit_weapon_types_.remove(u_ind);
    }

    void draw(sf::RenderWindow& window) {
        for (auto& projectile : projectiles_) {
            window.draw(projectile.shape);
        }
    }
};

#endif // BOIDS_ATTACKSYSTEM_H
