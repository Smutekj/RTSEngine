#include <omp.h>
#include "BoidWorld.h"
#include "BoidControler.h"
#include "core.h"
#include "Grid.h"
#include "MapGrid.h"

BoidControler::BoidControler(BoidWorld* world, NeighbourSearcher* ns, std::shared_ptr<Edges> map, sf::Vector2f box_size)
    : orientation_(world->orientation_)
    , world_(*world)
    , ns_(*ns)
    {

    map_ = map;

    forces_.resize(N_MAX_NAVIGABLE_BOIDS);
    path_data_.resize(N_MAX_NAVIGABLE_BOIDS);

    is_near_wall.resize(N_MAX_NAVIGABLE_BOIDS);

    radii_.resize(N_MAX_NAVIGABLE_BOIDS);
    max_speeds_.resize(N_MAX_NAVIGABLE_BOIDS);

    attack_targets_.resize(N_MAX_NAVIGABLE_BOIDS, -1);

    orientation_.resize(N_MAX_NAVIGABLE_BOIDS, 0);
    is_turning.resize(N_MAX_NAVIGABLE_BOIDS, false);
    turn_rates_.resize(N_MAX_NAVIGABLE_BOIDS, 8);

    is_returning_.resize(N_MAX_NAVIGABLE_BOIDS, false);
    n_steps_since_moved_away_.resize(N_MAX_NAVIGABLE_BOIDS, 0);

    boidind2hold_data_ind_.resize(N_MAX_NAVIGABLE_BOIDS, -1);
    boidind2cluster_ind_.resize(N_MAX_NAVIGABLE_BOIDS, -1);
    standing_pos_.resize(N_MAX_NAVIGABLE_BOIDS);
    n_steps_since_all_in_front_standing_.resize(N_MAX_NAVIGABLE_BOIDS, 0);
    is_being_pushed_.resize(N_MAX_NAVIGABLE_BOIDS, false);

    align_data_.resize(N_MAX_NAVIGABLE_BOIDS);

    p_ns_holding_ = std::make_unique<NeighbourSearcher>(box_size, 20, world_);
}

void BoidControler::activate(int ind, const float max_speed, const float radius, const float turn_rate) {

    standing_pos_[ind] = world_.r_coords_[ind];
    radii_[ind] = radius;
    max_speeds_[ind] = max_speed;
    this->turn_rates_[ind] = turn_rate;
}

void BoidControler::deactivate(BoidInd deactivated_ind) {

    for (int ind : world_.active_inds) {
        if (attack_targets_[ind] == deactivated_ind) {
            attack_targets_[ind] = -1;
        }
    }
}

void BoidControler::setPathData(int boid_ind, sf::Vector2f target, const Edgef& portal) {
    path_data_[boid_ind].move_target = target;
    path_data_[boid_ind].portal = portal;
    world_.move_states_[boid_ind] = MoveState::MOVING;
}

static int frame_i = 0;

void BoidControler::updateState(const float dt, sf::RenderWindow& win) {
    sf::Clock clock;
    
    ns_.update(world_.r_coords_, radii_);
    const auto neighbour_search_time = clock.restart().asMicroseconds();

    //    if(steps_since_last_turn >  4) {
    turn();
    const auto turn_time = clock.restart().asMicroseconds();
    //        steps_since_last_turn = 0;
    //    }
    //    steps_since_last_turn++;

    attack(dt);
    avoidHoldingAgents(dt, win);
    // push(dt);
    // flock(dt);
    scatter(dt);
    const auto scatter_time = clock.restart().asMicroseconds();
    seek();

    repulseBoidsNeighbourList(dt);
    const auto repulsion_time = clock.restart().asMicroseconds();

    allign(dt);
    const auto seek_time = clock.restart().asMicroseconds();

    const float lambda = 0.00005f;
    //! apply forces
    for (const auto i : world_.active_inds) {
        world_.velocities_[i] = forces_[i].accumulate(dt / 0.016f, settings_.values_);
        // world_.velocities_[i] -= lambda * world_.velocities_[i];
        // forces_[i].reset();

        if (world_.move_states_[i] != MoveState::MOVING) {
            world_.velocities_[i] *= 0.5f;
        }

        if (world_.move_states_[i] == MoveState::STANDING and norm2(forces_[i].push) > 0) {
            n_steps_since_moved_away_[i]++;
        }

        if (world_.move_states_[i] == MoveState::STANDING and norm2(forces_[i].push) == 0.f and
                dist(standing_pos_[i], world_.r_coords_[i]) > 5 * RHARD and n_steps_since_moved_away_[i] > 60 or
            (n_steps_since_moved_away_[i] > 180)) {
            n_steps_since_moved_away_[i] = 0;
            is_returning_[i] = true;
            world_.move_states_[i] = MoveState::MOVING;
            path_data_[i].move_target = standing_pos_[i];
            path_data_[i].next_move_target = standing_pos_[i];
            // next_move_targets_[i]       = standing_pos_[i];
            path_data_[i].path_end = standing_pos_[i];
        }
    }
    const auto update_time = clock.restart().asMicroseconds();

    //! apply constraints
    avoidWall(dt);
    const auto wall_calc_time = clock.restart().asMicroseconds();

    // wallConstraint(dt);
    applyExternalForces(dt);
    
    truncateVels();
    const auto truc_vels_time = clock.restart().asMicroseconds();

    updateInactiveGroupInd();
    const auto wtf_time = clock.restart().asMicroseconds();

    if (++frame_i == 100) {
        frame_i = 0;
        std::cout << "\n---------- ::BOID CONTROL CLOCKS:: ----------- \n";

        std::cout << "neighbour search took: " << neighbour_search_time << " us\n";

        std::cout << "seek took: " << seek_time << " us\n";
        std::cout << "wall interaction took: " << wall_calc_time << " us\n";

        std::cout << "repulsion took: " << repulsion_time << " us\n";
        std::cout << "scatter took: " << scatter_time << " us\n";
        // std::cout << "all took: " << penis.restart().asMicroseconds() << " us\n";
        // std::cout << "wtf took: " << wtf_time << " us\n";
    }
}

//! this is stupid and will probably not use it anyway 
void BoidControler::formFormation(const sf::Vector2f center, const sf::Vector2f orientation,
                                  const std::vector<int>& selection, const uint8_t n_collumns) {

    const auto& r_coords = world_.r_coords_;
    const auto& v_coords = world_.velocities_;

    const auto n_rows = selection.size() / n_collumns + 1;
    const auto n_rest = selection.size() - (n_rows - 1) * n_collumns;

    std::vector<sf::Vector2f> end_positions(selection.size());

    const float dx = 8;
    const float dy = 8;

    const sf::Vector2f upper_left = center - sf::Vector2f(n_collumns * dx / 2.f, n_rows * dy / 2.f);
    for (int i_row = 0; i_row < n_rows - 1; ++i_row) {
        for (int i_col = 0; i_col < n_collumns; ++i_col) {
            const auto index = i_col + i_row * n_collumns;
            end_positions[index] = upper_left + sf::Vector2f(i_col * dx, i_row * dy);
        }
    }

    for (int i_col = 0; i_col < n_rest; ++i_col) {
        const auto index = i_col + (n_rows - 1) * n_collumns;
        end_positions[index] = upper_left + sf::Vector2f(i_col * dx, n_rows * dy);
    }

    int index = 0;
    for (const auto selected : selection) {
        path_data_[selected].path_end = end_positions[index];
        need_path_update_.push(selected);
        path_data_[selected].need_path_update = true;
        world_.move_states_[selected] = MoveState::MOVING;
        index++;
    }
}

//! this is stupid and will probably not use it anyway 
void BoidControler::addExplosion(const sf::Vector2f center) {
    auto n_explosions = explosions_.size();
    ExplosionData e_data;
    e_data.r_center = center;
    explosions_[n_explosions] = e_data;
}


struct ThreadWork {
    std::vector<int> data_id;
    std::vector<std::pair<int, int>> from_to;
};

std::array<ThreadWork, 12> thread_work;

void BoidControler::repulseBoids(float dt) {

    const auto& r_coords = world_.r_coords_;
    const auto& vs = world_.velocities_;

    // std::vector<int> thread_n_pairs(omp_get_max_threads(), 0);
    // std::vector<int> free_threads;
    // int n_pairs_total = 0;
    // for (int thread_id = 0; thread_id < omp_get_num_threads(); ++thread_id) {
    //     const auto n_pairs = ns_.getPairList(thread_id).size();
    //     thread_work[thread_id].data_id = {thread_id};
    //     thread_work[thread_id].from_to = {{0, n_pairs}};
    //     thread_n_pairs[thread_id] = n_pairs;
    //     n_pairs_total += n_pairs;
    //     free_threads.push_back(thread_id);
    // }

    // std::sort(thread_work.begin(), thread_work.end(), [](const ThreadWork& tw1, const ThreadWork& tw2) {
    //     return tw1.from_to[0].second > tw2.from_to[0].second;
    // });

    // int next_free_thread_id = thread_work.back().data_id[0];
    // int a = omp_get_max_threads() - 1;

    // const auto n_pairs_per_thread = n_pairs_total / omp_get_max_threads();
    // for (int i = 0; i < omp_get_max_threads(); ++i) {
    //     const auto thread_id = thread_work[i].data_id[0];
    //     const auto n_pairs = thread_n_pairs.at(thread_id);
    //     while (thread_n_pairs.at(thread_id) > n_pairs_per_thread) {
    //         const auto n_extra_pairs = thread_n_pairs.at(thread_id) - n_pairs_per_thread;
    //         const auto n_free_pairs = n_pairs_per_thread - thread_n_pairs[next_free_thread_id];
    //         thread_n_pairs[next_free_thread_id] += std::min(n_free_pairs, n_extra_pairs);
    //         thread_work[next_free_thread_id].from_to.push_back(
    //             {thread_work[next_free_thread_id].from_to.back().second, thread_n_pairs[next_free_thread_id]});
    //         thread_work[next_free_thread_id].data_id.push_back(thread_id);

    //         if (n_extra_pairs > n_free_pairs) {
    //             a--;
    //             next_free_thread_id = thread_work.at(a).data_id[0];
    //             if (thread_n_pairs[next_free_thread_id] >= n_pairs_per_thread) {
    //                 break;
    //             }
    //         }
    //         thread_n_pairs[thread_id] -= std::min(n_free_pairs, n_extra_pairs);
    //     }
    // }


//! Force calculation is separated into two for cycles to avoid data race
#pragma omp parallel num_threads(4)
{
    for(int thread_id = 0; thread_id < 2; ++thread_id){
        const auto& pair_list = ns_.getPairList(thread_id);
        const auto n_pairs = pair_list.size();
        #pragma omp for 
        for (int pair_ind = 0; pair_ind < n_pairs; ++pair_ind) {
            const auto& pair = pair_list[pair_ind];
            const auto ind_i = pair.first;
            const auto ind_j = pair.second;

            // if (world_.move_states_[ind_i] == MoveState::MOVING and
            //                    world_.move_states_[ind_j] == MoveState::MOVING) {
            //     a = dot(world_.velocities_[ind_i], pair.dr) > 0;
            // }
            const auto r_collision_sq = pair.r_collision * pair.r_collision;
            // const auto dv = world_.velocities_[ind_i] - world_.velocities_[ind_j];
            const auto d_surfs = norm2(pair.dr);
            const auto x = r_collision_sq / d_surfs;
            const auto alpha = 0.5f; // radii_[ind_i] / pair.r_collision;

            //! no overlap
            if(x > 1.0f)  [[unlikely]] 
            {
                const auto x_prime = (x - 0.00000000001f);
                const auto force =
                    -pair.dr / norm(pair.dr) * std::min(500.f, 3.f * x_prime * x_prime*x_prime*x_prime); //* ( 1 - a *2.f/3.f);
                forces_[ind_i].repulse += force * (1 - alpha);//*agents_are_overlapping;// * static_cast<float>(!is_near_wall[ind_i]);
                forces_[ind_j].repulse += -force * (alpha);//*agents_are_overlapping;// * static_cast<float>(!is_near_wall[ind_j]);
            }
            

            if (world_.move_states_[ind_i] == MoveState::MOVING and world_.move_states_[ind_j] == MoveState::MOVING and
                commander_groups_.isSameGroup(ind_i, ind_j) and
                x > r_collision_sq / (RALLIGN * RALLIGN)) {
                align_data_[ind_i].direction += forces_[ind_j].seek;
                align_data_[ind_j].direction += forces_[ind_i].seek;
                align_data_[ind_i].count++;
                align_data_[ind_j].count++;
            }

            bool moving_pushes_standing_a =
                world_.move_states_[ind_i] == MoveState::MOVING and world_.move_states_[ind_j] == MoveState::STANDING;
            bool moving_pushes_standing_b =
                world_.move_states_[ind_j] == MoveState::MOVING and world_.move_states_[ind_i] == MoveState::STANDING;
                if (moving_pushes_standing_a and x > 0.75f) {
                    forces_[ind_j].push += (pair.dr) * 1000.f * (x/0.75f);
                } else if (moving_pushes_standing_b and x > 0.75f) {
                    forces_[ind_i].push += -(pair.dr) * 1000.f * (x/0.75f) ;
                }
        }

    }
}

}


void BoidControler::repulseBoidsNeighbourList(float dt) {

    const auto& r_coords = world_.r_coords_;
    const auto& vs = world_.velocities_;
//! Force calculation is separated into two for cycles to avoid data race
#pragma omp parallel num_threads(6)
{
        #pragma omp for 
        for (int ind_i = 0; ind_i < N_MAX_NAVIGABLE_BOIDS; ++ind_i) {
            const auto& n_data = ns_.getNeighbourData(ind_i, 0);
            const auto n_last_neighbour = ns_.last_i[ind_i];

            for (int ind_j = 0; ind_j < n_last_neighbour; ++ind_j) {
                const auto& dr = n_data[ind_j].dr;
                const auto boid_ind_j = n_data[ind_j].second;
                const auto r_collision_sq = n_data[ind_j].r_collision*n_data[ind_j].r_collision;
                const auto d_surfs = norm2(dr);
                const auto x = r_collision_sq / d_surfs;
                const auto alpha = 0.5f; // radii_[ind_i] / pair.r_collision;
                            //! no overlap
                if(x > 1.0f)  [[unlikely]] 
                {
                    const auto x_prime = (x - 0.00000000001f);
                    const auto force =
                        -dr / norm(dr) * std::min(500.f, 3.f * x_prime * x_prime*x_prime*x_prime); //* ( 1 - a *2.f/3.f);
                    forces_[ind_i].repulse += force * (1 - alpha);//*agents_are_overlapping;// * static_cast<float>(!is_near_wall[ind_i]);
                }

                            

                // if (world_.move_states_[ind_i] == MoveState::MOVING and world_.move_states_[boid_ind_j] == MoveState::MOVING and
                //     commander_groups_.isSameGroup(ind_i, boid_ind_j) and
                //     x > r_collision_sq / (RALLIGN * RALLIGN)) {
                //     align_data_[ind_i].direction += forces_[ind_i].seek;
                //     align_data_[ind_i].count++;
                // }

                 bool moving_pushes_standing_a =
                world_.move_states_[ind_i] == MoveState::MOVING and world_.move_states_[boid_ind_j] == MoveState::STANDING;
                bool moving_pushes_standing_b =
                world_.move_states_[boid_ind_j] == MoveState::MOVING and world_.move_states_[ind_i] == MoveState::STANDING;
                if (moving_pushes_standing_b and x > 0.75f) {
                    forces_[ind_i].push += -(dr) * 1000.f * (x/0.75f) ;
                }
            }

        }
    }
}

// void BoidControler::avoid(float dt){
//     auto factor = 0.0025f*800.f*dt/2.f;

//     auto& r_coords = world_.r_coords_;
//     auto& v_coords = world_.velocities_;

//     sf::Vector2f ws = {1846, 1016};
//     auto ll = ws/2.f - ws/6.f;
//     auto ur = ws/2.f + ws/6.f;

//     for(const auto selected : world_.active_inds){
//         sf::Vector2f delta_v(0,0);
//         auto& r_selected = r_coords[selected];
//         auto& v_selected = v_coords[selected];

//         auto r_search = (radii_[selected] + 4*RHARD);
//         const auto& verlet_list = ns_.getNeighbourInds(selected, r_search*r_search);
//         if(world_.move_states_[selected] != MoveState::HOLDING and is_near_wall[selected]) {
//             for(auto neighbour : verlet_list){
//                 auto d2 = dist2(r_coords[neighbour] + v_coords[neighbour]*dt, r_selected + v_selected*dt);
//                 auto r_collision = (radii_[selected] + radii_[neighbour]);
//                 if(d2 < r_collision*r_collision and is_near_wall[neighbour]){
//                     r_selected += (r_selected - r_coords[neighbour])*factor;

//                 } else if(d2 < r_collision*r_collision and !is_near_wall[neighbour]){

//                 }
//             }
//         }else if( world_.move_states_[selected] != MoveState::HOLDING and !is_near_wall[selected]){
//             for(auto neighbour : verlet_list){
//                 auto d2 = dist2(r_coords[neighbour], r_selected);
//                 auto r_collision = (radii_[selected] + radii_[neighbour]);
//                 if(d2 < r_collision*r_collision){
//                     r_selected += (r_selected - r_coords[neighbour])*factor;
//                 }
//             }
//         }
//     }
// }

void BoidControler::attack(float dt) {
    auto& r_coords = world_.r_coords_;

    for(auto ind : world_.active_inds){
        const auto player_ind = world_.ind2player[ind];
        auto& move_state = world_.move_states_[ind];

        if (attack_targets_[ind] == -1) {
            auto r_selected = r_coords[ind];
            int nearest_neighbour = -1;
            float min_dist2 = MAXFLOAT;
            const auto& n_data = ns_.getNeighbourData(ind, 0);
            const auto last_ind = ns_.last_i[ind]; 
            for (int j = 0; j < last_ind; ++j) {
                const auto& data = n_data[j];
                const auto neighbour_ind = data.second;

                const auto d2 = dist2(r_coords[neighbour_ind], r_selected);
                if (d2 < min_dist2 and player_ind != world_.ind2player[neighbour_ind]) {
                    min_dist2 = d2;
                    nearest_neighbour = neighbour_ind;
                }
            }
            if (nearest_neighbour != -1) {
                setAttackTarget(ind, nearest_neighbour);
            }
        }
    }
}

//! \brief moving agents push away standing agents
void BoidControler::push(float dt) {
    auto& r_coords = world_.r_coords_;
    for (const auto selected : world_.active_inds) {
        auto move_state = world_.move_states_[selected];

        auto r_selected = r_coords[selected];
        auto r_search = (radii_[selected] + 10 * RHARD);
        const auto& verlet_list = ns_.getNeighbourData(selected, r_search * r_search);
        for (const auto& neighbour_data : verlet_list) {
            auto d2 = norm2(neighbour_data.dr);
            auto dr = -neighbour_data.dr;
            const auto neighbour = neighbour_data.second;
            auto move_state_neighbour = world_.move_states_[neighbour];
            auto r_avoid = neighbour_data.r_collision;

            if (move_state == MoveState::MOVING and move_state_neighbour == MoveState::STANDING and
                !is_near_wall[neighbour] and d2 < 2 * r_avoid * r_avoid) {

                forces_[neighbour].push += (dr)*(0.1f / d2 / d2 + 5.0f*std::sqrt(d2));
                //                if(!is_being_pushed_[neighbour] and !is_returning_[neighbour]){
                ////                    return_pos_info_.push_back({neighbour, r_coords[neighbour]});
                //                    is_being_pushed_[neighbour] = true;
                //                }
            }
        }
    }
}

//! \brief alligns velocities of agents that share a command group
void BoidControler::allign(const float dt) {
    auto& r_coords = world_.r_coords_;
    auto& vs = world_.velocities_;
    for (auto selected : world_.active_inds) {
        auto v = vs[selected];
        auto new_direction = align_data_[selected].direction;
        forces_[selected].align *= 0.f;
        if (world_.move_states_[selected] == MoveState::MOVING and norm2(new_direction) != 0.f and norm2(v) != 0) {
            new_direction /= norm(new_direction) / static_cast<float>(align_data_[selected].count);
            auto v_norm = v / norm(v);
            const auto desired_v = new_direction;
            auto dot_prod = dot(new_direction, v_norm);
            dot_prod = std::min(1.0f, dot_prod);
            dot_prod = std::max(-1.0f, dot_prod);

            auto angle = std::acos(dot_prod) * (2.f * (new_direction.x < 0.f) - 1.f);
//            world_.velocities_[selected].x = v.x*std::cos(angle) + v.y*std::sin(angle);
//            world_.velocities_[selected].y = -v.x*std::sin(angle) + v.y*std::cos(angle);
            forces_[selected].align = max_speeds_[selected] * (new_direction) / 5.f;
        }
        align_data_[selected].count = 0;
        align_data_[selected].direction = {0, 0};
    }
}

//! \brief makes agents go around other agents, that are HOLDING
void BoidControler::avoidHoldingAgents(const float dt, sf::RenderWindow& window) {
    auto& r_coords = world_.r_coords_;

    // for (const auto cluster_ind : active_cluster_inds_) {
    //     auto cluster_radius2 = clusters_[cluster_ind].radius2;
    //     auto& center_of_cluster = clusters_[cluster_ind].center;
    //     center_of_cluster *= 0.f;
    //     const float n_in_cluster = clusters_[cluster_ind].inds.size();
    //     for (const auto ind_in_cluster : clusters_[cluster_ind].inds) {
    //         center_of_cluster += r_coords[ind_in_cluster] / n_in_cluster;
    //     }
    //     for (const auto ind_in_cluster : clusters_[cluster_ind].inds) {
    //         cluster_radius2 =
    //             std::max({dist2(center_of_cluster, r_coords[ind_in_cluster]) + RHARD * RHARD, cluster_radius2});
    //     }
    //     clusters_[cluster_ind].radius2 = cluster_radius2;
    // }

    for (const auto selected : world_.active_inds) {
        sf::Vector2f delta_v(0, 0);
        const auto& move_target = path_data_[selected].move_target;
        const auto r_selected = r_coords[selected];
        auto dr_to_target = move_target - r_selected;
        dr_to_target /= norm(dr_to_target);

        const auto selected_is_moving = world_.move_states_[selected] == MoveState::MOVING;

        if (selected_is_moving) {
            if (norm2(dr_to_target) == 0.f) {
                continue;
            }
            for (const auto cluster_ind : active_cluster_inds_) {
                const auto& cluster = clusters_.at(cluster_ind);
                if (cluster.is_in(move_target)) { //! we do not move around when we want to get inside cluster!
                    break;
                }
                const auto dr_to_cluster = cluster.center - r_selected;
                if (norm2(dr_to_cluster) == 0) {
                    continue;
                }
                const auto angle = angle_calculator_.angleBetween(dr_to_cluster, dr_to_target);
                //                bool is_above = sign(r_selected, cluster.center, move_targets_[selected]);
                const float sign =
                    2 * (angle < 0) - 1; // 2*(dot(dr_to_target, dr_to_target - dr_to_cluster*dot(dr_to_target,
                                         // dr_to_cluster)/dot(dr_to_cluster, dr_to_cluster))<0) -1;

                dr_to_target /= norm(dr_to_target);
                sf::Vector2f dr_norm = {dr_to_target.y, -dr_to_target.x};

                //                if (dot(dr_norm, dr_to_cluster) < 0.f) { dr_norm *= -1.f;}

                const auto dist2_to_cluster = dist2(cluster.center, r_selected);
                if (dist2_to_cluster < cluster.radius2 * 4 and
                    std::abs(angle) < 70) { //! if the cluster is close enough in my direction
                    const auto next_value =
                        sign * dr_norm * 1.5f * cluster.radius2 / (dist2_to_cluster + cluster.radius2);
                    delta_v += next_value * settings_.values_[BoidControlerSettings2::AVOIDCLUSTERS];
                }
            }

            sf::Vector2f steepest_growth_dir = {0.f, 0.f};
            const auto radius = radii_[selected];
            const auto& verlet_list = p_ns_holding_->getNeighboursInds(r_selected, 6.0f * radius * radius);

            for (const auto& neighbour : verlet_list) {
                const auto neighbour_is_holding = world_.move_states_[neighbour] == MoveState::HOLDING;

                if (neighbour_is_holding) {
                    const auto dr = r_coords[neighbour] - r_selected;
                    const sf::Vector2f dr_norm = {dr.y, -dr.x};
                    const auto r_collision = radius + radii_[neighbour];
                    const auto d2 = norm2(dr);
                    const auto cluster_center = clusters_.at(boidind2cluster_ind_[neighbour]).center;
                    //                    bool neighbour_in_front = dot(dr, dr_to_target) > 0;
                    bool distance_close_enough = d2 < 1.2 * r_collision * r_collision;
                    bool angle_condition = std::abs(angle_calculator_.angleBetween(dr, dr_to_target)) < 120;
                    if (distance_close_enough)
                        if (angle_condition) {
                            float angle;
                            if (r_coords[neighbour] != cluster_center) {
                                angle = angle_calculator_.angleBetween(move_target - cluster_center,
                                                                       r_coords[neighbour] - cluster_center);
                            } else {
                                angle = 0;
                            }
                            const float sign = 2 * (angle < 0) - 1;
                            steepest_growth_dir += sign * settings_.values_[BoidControlerSettings2::AVOID] /
                                                   std::pow((1 + d2 / RHARD / RHARD), -1.f) * dr_norm / (RHARD * RHARD);
                        }
                }

                // 2*(dot(dr_to_target, dr_to_target - dr_to_cluster*dot(dr_to_target, dr_to_cluster)/dot(dr_to_cluster,
                // dr_to_cluster))<0) -1;

                //                steepest_growth_dir = {steepest_growth_dir.y, -steepest_growth_dir.x};
                delta_v += steepest_growth_dir;
                if (norm2(delta_v) > max_speeds_[selected] * max_speeds_[selected]) {
                    delta_v *= max_speeds_[selected] / norm(delta_v);
                }
            }
            forces_[selected].avoid = delta_v;
        }
    }
}

//! \brief adds force to push away particles from walls and makes the slide when moving next to them
//! \brief (the sliding part is probably not needed)
void BoidControler::avoidWall(const float dt) {
    auto& r_coords = world_.r_coords_;
    auto& velocities = world_.velocities_;
    typedef BoidControlerSettings2::Values Behaviour;

    const auto n_active = world_.active_inds.size();
#pragma omp parallel for num_threads(6)
    for (int i = 0; i < n_active; ++i) {
        const auto active_ind = world_.active_inds[i];
        auto& r_selected = r_coords[active_ind];
        auto& v_selected = velocities[active_ind];
        const auto radius = radii_[active_ind];
        const auto walls = map_->calcContactEdgesO(r_selected, radius);

        auto dr_to_target = path_data_[active_ind].move_target - r_selected;
        bool is_touching_wall = false;
        for (int orient = 0; orient < N_DIRECTIONS; orient++) {
            for (const auto& wall : walls[orient]) {
                is_touching_wall = true;
                sf::Vector2f n_wall = {wall.t.y, -wall.t.x};

                // auto dr_to_wall_start = r_selected - wall.from;
                // auto dist_to_wall = std::abs(dot(dr_to_wall_start, n_wall));
                // bool wall_is_opposite = dot(dr_to_target, dr_to_wall_start) > 0;
                // if (dot(dr_to_wall_start, wall.t) > -radius and dot(dr_to_wall_start, wall.t) < wall.l + radius and
                //     dot(dr_to_wall_start, n_wall) < 1.5 * radius) {
                //     v_selected += settings_.values_[Behaviour::WALLREPULSE] * max_speeds_[selected] * n_wall *
                //                       std::pow(dist_to_wall, -4.f) +
                //                   settings_.values_[Behaviour::WALLSLIDE] * max_speeds_[selected] * wall.t *
                //                       float(2 * wall_is_opposite - 1);
                // }

                auto v_away_from_surface = n_wall * dot(v_selected, n_wall);
                if (dot(v_selected, n_wall) < 0) {
                    v_selected -= v_away_from_surface;
                }
            }
        }
        is_near_wall[active_ind] = is_touching_wall;
    }

}

//! \brief removes velocity component normal to the walls, agents are touching
void BoidControler::wallConstraint(const float dt) {
    auto& r_coords = world_.r_coords_;
    auto& velocities = world_.velocities_;
    for (const auto selected : world_.active_inds) {

        auto& r_selected = r_coords[selected];
        auto& v_selected = velocities[selected];
        const auto radius = radii_[selected];
        auto r_new = r_selected + v_selected * dt;
        const auto& walls = map_->calcContactEdges(r_new, radius);

        for (auto wall : walls) {
            sf::Vector2f n_wall = {wall.t.y, -wall.t.x};
            auto v_away_from_surface = n_wall * dot(v_selected, n_wall);
            if (dot(v_selected, n_wall) < 0) {
                v_selected -= v_away_from_surface;
            }
        }
    }
}

// //! \brief adds force to make agents scatter when moving or standing
// void BoidControler::flock(const float dt){
//     auto& r_coords = world_.r_coords_;
//     auto& v_coords = world_.velocities_;
//     for(const auto selected : world_.active_inds){
//         sf::Vector2f delta_v(0,0);
//         auto r_selected = r_coords[selected];
//         sf::Vector2f r_com(r_selected);
//         const auto& verlet_list = ns_.getNeighbourInds(selected, RFLOCK*RFLOCK);
//         float n_neighbours = 1;

//         for(auto neighbour : verlet_list){
//             if(world_.move_states_[neighbour] == MoveState::MOVING){
//                 auto d2 = dist2(r_coords[neighbour], r_selected);
//                 r_com += r_coords[neighbour];
//                 n_neighbours++;
//             }
//         }
//         r_com /= n_neighbours;
//         delta_v += (r_com - r_selected)*0.01f;

//         if(world_.move_states_[selected] == MoveState::MOVING){
//             v_coords[selected] += delta_v;
//         }
//     }
// }

//! \brief adds force to make agents scatter when moving or standing
void BoidControler::scatter(const float dt) {
    auto& r_coords = world_.r_coords_;
    auto& v_coords = world_.velocities_;

    #pragma omp parallel for num_threads(6)
    for (const auto selected : world_.active_inds) {
        const auto radius = radii_[selected];
        auto r_selected = r_coords[selected];
        sf::Vector2f dr_nearest_neighbours(0, 0);
        const auto& verlet_list = ns_.getNeighbourData(selected, radius * radius);
        float n_neighbours = 1;

        const auto move_state = world_.move_states_[selected];
        if (move_state == MoveState::MOVING) {
            for (int i = 0; i < ns_.last_i[selected]; ++i) {
                const auto& neighbour_data = verlet_list[i];
                const auto neighbour = neighbour_data.second;
                auto d2 = dist2(r_coords[neighbour], r_selected);
                if (world_.move_states_[neighbour] == MoveState::MOVING and d2 < 6 * radius * radius) {
                    dr_nearest_neighbours += neighbour_data.dr;
                    n_neighbours++;
                }
            }
            dr_nearest_neighbours /= n_neighbours;

            if (n_neighbours > 1) {
                forces_[selected].scatter =
                    -dr_nearest_neighbours / norm(dr_nearest_neighbours) * max_speeds_[selected];
            }
        }
    }
}

//! \brief manages agents movement to their target positions (together with stopping conditions)
void BoidControler::seek() {
    auto& r_coords = world_.r_coords_;

    for (const auto selected : world_.active_inds) {
        const auto& verlet_list = ns_.getNeighbourData(selected, 1.1 * RHARD * RHARD);

        if (world_.move_states_[selected] == MoveState::MOVING) {
            const auto& r = r_coords[selected];
            auto& v = world_.velocities_[selected];

            auto& move_target = path_data_[selected].move_target;
            auto& next_move_target = path_data_[selected].next_move_target;
            const auto& path_end = path_data_[selected].path_end;

            auto dr_to_target = move_target - r_coords[selected];
            float norm_dr = norm(dr_to_target);
            forces_[selected].seek = dr_to_target / norm_dr * max_speeds_[selected];

            const auto dr_to_next_target = next_move_target - r_coords[selected];
            const auto norm_dr_next = norm(dr_to_next_target);

            const auto current_vel = world_.velocities_[selected];

            auto t = next_move_target - move_target;
            if (norm2(t) != 0) {
                t /= norm(t);
            };
            bool can_move_to_next_target = dot(-dr_to_target, t) > 0;
            bool update_next_target = norm_dr < RHARD and can_move_to_next_target or move_target == next_move_target;
            const auto closest_point_on_next_portal = closestPointOnEdge(r, path_data_[selected].portal);
            const auto a = 0;

            if (is_turning[selected]) {
                const auto desired_vel = dr_to_target / norm_dr;
                const auto desired_angle = angle_calculator_.angle(desired_vel);
                auto d_angle = desired_angle - orientation_[selected];
                d_angle > 180 ? d_angle = -360 + d_angle : d_angle = d_angle;
                d_angle < -180 ? d_angle = 360 - d_angle : d_angle = d_angle;
                auto angle_change = std::min({turn_rates_[selected], d_angle});
                if (d_angle < 0) {
                    angle_change = std::max({-turn_rates_[selected], d_angle});
                }
                if (std::abs(d_angle) > 0.3f) {
                    orientation_[selected] += angle_change;
                    if (orientation_[selected] > 180) {
                        orientation_[selected] -= 360;
                    }
                    if (orientation_[selected] < -180) {
                        orientation_[selected] += 360;
                    }
                } else {
                    is_turning[selected] = false;
                }
            } else {

                auto& portal = path_data_[selected].portal;
                const sf::Vector2f n = {-portal.t.y, portal.t.x};

                if (dot(-dr_to_target, t) > 0) {
                    update_next_target = true;
                } else {
                    //                if (t_dist > -RHARD and t_dist < portal.l and norm_dist > -RHARD / 2.0f and
                    //                norm_dist < RHARD / 2.0f) {
                    //                    update_next_target = true;
                    if (dist2(closest_point_on_next_portal, r) < dist2(r, move_target) and portal.l > 0 and
                        dist2(closest_point_on_next_portal, r) < 16 * RHARD * RHARD and norm2(t) != 0) {
                        move_target = closest_point_on_next_portal;
                        portal.l = 0;
                        dr_to_target = move_target - r_coords[selected];
                        update_next_target = false;
                    } else if (vequal(move_target, next_move_target) and move_target != path_end) {
                        update_next_target = true;
                    }
                }

                if (update_next_target) {
                    move_target = next_move_target;
                    portal = path_data_[selected].next_portal;
                    forces_[selected].seek = dr_to_next_target / norm_dr_next * max_speeds_[selected];
                    //+ dr_to_target / (norm_dr + 1000.f * FLT_EPSILON) / (norm_dr + 1000.f * FLT_EPSILON);

                    if (norm_dr_next == 0) {
                        forces_[selected].seek = {0, 0};
                    };
                    if (next_move_target != path_end) {
                        path_data_[selected].need_path_update = true;
                        need_path_update_.push(selected);
                    } else {
                        path_data_[selected].need_path_update = false;
                    }
                } else if (!is_turning[selected]) {
                    forces_[selected].seek = dr_to_target / norm_dr * max_speeds_[selected];
                    //+ dr_to_target / (norm_dr + 1000.f * FLT_EPSILON) / (norm_dr + 1000.f * FLT_EPSILON);
                    if (norm_dr == 0) {
                        forces_[selected].seek = {0, 0};
                    };
                }
            }

            const auto radius = radii_[selected];
            auto dist_to_target = dist(path_end, r_coords[selected]);
            if (dist_to_target < 2 * radius) {
                world_.move_states_[selected] = MoveState::STANDING;
                standing_pos_[selected] = r_coords[selected];
                commander_groups_.removeFromGroup(selected);


                const auto& neighbour_inds = ns_.getNeighboursInds(r, 2.69 * 2.69 * radius * radius);
                for (const auto neighbour : neighbour_inds) {
                    if (dist(r_coords[neighbour], r_coords[selected]) < 3 * radius and
                        commander_groups_.isSameGroup(selected, neighbour)) {
                        world_.move_states_[neighbour] = MoveState::STANDING;
                        standing_pos_[neighbour] = r_coords[neighbour];
                        commander_groups_.removeFromGroup(selected);
                    }
                }
            }

            if (v.x == 0 && v.y == 0)[[unlikely]] { //! this would break 
                continue;
            }

            int n_in_front = 0;
            int n_in_front_standing = 0;
            if (vequal(path_end, move_target)) {
                const auto& neighbour_inds = ns_.getNeighboursInds(r, radius * radius * 12.f);
                for (const auto neighbour : neighbour_inds) {
                    const auto d_r = r_coords[neighbour] - r;
                    if (norm2(d_r) < 9 * radius * radius and commander_groups_.isSameGroup(selected, neighbour)) {
                        if (std::abs(angle_calculator_.angleBetween(d_r, v)) < 15) {
                            n_in_front_standing += world_.move_states_[neighbour] == MoveState::STANDING;
                            n_in_front++;
                        }
                    }
                }
            }

            if (n_in_front != 0 and n_in_front_standing == n_in_front) {
                n_steps_since_all_in_front_standing_[selected]++;
                if (n_steps_since_all_in_front_standing_[selected] > 5) {
                    world_.move_states_[selected] = MoveState::STANDING;
                    standing_pos_[selected] = r_coords[selected];
                    n_steps_since_all_in_front_standing_[selected] = 0;
                    commander_groups_.removeFromGroup(selected);
                }
            }
        } else {
            forces_[selected].seek *= 0.f;
        }
    }
}

void BoidControler::turn() {
    auto& r_coords = world_.r_coords_;

    for (const auto selected : world_.active_inds) {

        if (world_.move_states_[selected] != MoveState::HOLDING) {
            auto& v = world_.velocities_[selected];
            auto dr = path_data_[selected].move_target - r_coords[selected];
            float norm_dr = norm(dr);

            float desired_angle;
            if (norm2(v) == 0) {
                desired_angle = orientation_[selected];
            } else {
                desired_angle = angle_calculator_.angle(v);
            }
            auto d_angle = desired_angle - orientation_[selected];
            d_angle > 180 ? d_angle = -360 + d_angle : d_angle = d_angle;
            d_angle < -180 ? d_angle = 360 + d_angle : d_angle = d_angle;
            auto angle_change = std::min({turn_rates_[selected], d_angle});
            if (d_angle < 0) {
                angle_change = std::max({-turn_rates_[selected], d_angle});
            }
            if (std::abs(d_angle) > 0.1f and !is_turning[selected]) {
                orientation_[selected] += angle_change / 3.f;
                if (orientation_[selected] > 180) {
                    orientation_[selected] -= 360;
                }
                if (orientation_[selected] < -180) {
                    orientation_[selected] += 360;
                }
            }
        }
    }
}

void BoidControler::truncateVels() {
    for (const auto ind : world_.active_inds) {
        const auto velocity = world_.velocities_[ind];
        const auto speed = norm(world_.velocities_[ind]);
        // if (isnan(velocity.x) or isnan(velocity.y)) {
        //     throw std::runtime_error("velocity not a number!");
        // }
        if (speed > max_speeds_[ind]) {
            world_.velocities_[ind] = (velocity / speed * max_speeds_[ind]);
        }
    }
}

void BoidControler::removeFromClusters(const std::vector<int>& selection) {
    for (const auto ind : selection) {
        const auto cluster_ind = boidind2cluster_ind_[ind];
        if (cluster_ind != -1) {
            const auto cluster_size = clusters_.at(cluster_ind).removeInd(ind, world_.r_coords_[ind]);
            if (cluster_size <= 0) {
                assert(cluster_size == 0);
                int removed_ind_in_active = clusters_.at(cluster_ind).ind_in_active;
                active_cluster_inds_.at(removed_ind_in_active) = active_cluster_inds_.back();
                clusters_.at(active_cluster_inds_.back()).ind_in_active = removed_ind_in_active;
                active_cluster_inds_.pop_back();
                if (cluster_ind == clusters_.size() - 1) { //! remove if newly the vacant cluster is the last one
                    clusters_.pop_back();
                } else { //! otherwise put it on a stack of inactive ones, the vector of active_inds is made contigious
                    clusters_[cluster_ind].radius2 = 0;
                    inactive_cluster_inds_.push_back(cluster_ind);
                }
            }
            boidind2cluster_ind_[ind] = -1;
        }
    }
}

void BoidControler::addToClusters( std::vector<int>& selection) {

    std::sort(selection.begin(), selection.end());
    auto is_in_selection = [&](const int& ind) { return std::binary_search(selection.begin(), selection.end(), ind); };

    const auto& r_coords = world_.r_coords_;

    //! we set cluster_inds of all selections to default value
    //!
    for (const auto selected_ind : selection) {
        //        if(
        //        boidind2cluster_ind_[selected_ind] = -1;
    }

    std::vector<int> encountered_old_cluster_inds;
    for (const auto selected_ind : selection) {

        std::queue<int> to_check;
        to_check.push(selected_ind);
        if (boidind2cluster_ind_[selected_ind] != -1) {
            //! to avoid visiting one boid_ind multiple times since we already assigned cluster_ind to it
            continue;
        }

        const auto radius_sq = radii_[selected_ind] * radii_[selected_ind];

        Cluster new_cluster;
        int current_cluster_ind;
        if (inactive_cluster_inds_.empty()) {
            current_cluster_ind = clusters_.size();
            clusters_.push_back(new_cluster);
        } else {
            current_cluster_ind = inactive_cluster_inds_.back();
            inactive_cluster_inds_.pop_back();
        }
        auto& cluster = clusters_.at(current_cluster_ind);
        cluster.ind_in_active = active_cluster_inds_.size();
        active_cluster_inds_.push_back(current_cluster_ind);

        std::vector<bool> cluster2already_encountered(clusters_.size(), false);

        while (!to_check.empty()) {
            const auto ind = to_check.front();
            to_check.pop();
            if (boidind2cluster_ind_[ind] !=
                -1) { //! to avoid visiting one boid_ind multiple times since we already assigned cluster_ind to it
                continue;
            }

            const auto r = r_coords[ind];
            clusters_[current_cluster_ind].add(ind, r);
            boidind2cluster_ind_[ind] = current_cluster_ind;

            const auto& neighbours = p_ns_holding_->getNeighboursInds(r, 6.0f * radius_sq);
            for (const auto neighbour : neighbours) {
                const auto& r_neighbour = r_coords[neighbour];
                const auto existing_cluster_ind = boidind2cluster_ind_[neighbour];
                const auto r_collision = radii_[selected_ind] + radii_[neighbour];
                if (dist(r, r_neighbour) < 2.1 * r_collision) {
                    if (is_in_selection(neighbour)) {
                        to_check.push(neighbour);
                    } else if (world_.move_states_[neighbour] == MoveState::HOLDING and
                               !cluster2already_encountered[existing_cluster_ind]) {
                        encountered_old_cluster_inds.push_back(
                            existing_cluster_ind); //! we will have to update its radius2
                        cluster2already_encountered[existing_cluster_ind] = true;
                    }
                }
            }
        }

        //! We merge all encountered clusters into one (maybe one with smallest index?)

        if (encountered_old_cluster_inds.size() > 0) {
            const auto cluster_ind_to_keep = current_cluster_ind;
            auto& cluster_to_keep = clusters_[cluster_ind_to_keep];

            for (int i = 0; i < encountered_old_cluster_inds.size(); ++i) {
                const auto cluster_ind_to_remove = encountered_old_cluster_inds[i];
                auto& cluster_to_remove = clusters_[cluster_ind_to_remove];
                for (int ind : cluster_to_remove.inds) {
                    boidind2cluster_ind_[ind] = cluster_ind_to_keep;
                }
                cluster_to_keep.merge(cluster_to_remove);
                cluster_to_remove.inds.clear();
                inactive_cluster_inds_.push_back(cluster_ind_to_remove);

                int removed_ind_in_active = clusters_.at(cluster_ind_to_remove).ind_in_active;
                active_cluster_inds_.at(removed_ind_in_active) = active_cluster_inds_.back();
                clusters_.at(active_cluster_inds_.back()).ind_in_active = removed_ind_in_active;
                active_cluster_inds_.pop_back();
            }

            current_cluster_ind = cluster_ind_to_keep;
        }

        auto& cluster_radius2 = clusters_[current_cluster_ind].radius2;
        auto& center_of_cluster = clusters_[current_cluster_ind].center;
        for (const auto ind_in_cluster : clusters_[current_cluster_ind].inds) {
            cluster_radius2 =
                std::max({dist2(center_of_cluster, r_coords[ind_in_cluster]) + radius_sq, cluster_radius2});
        }
    }
}

void BoidControler::addHoldingAgents( std::vector<int>& selection) {

    const auto& r_coords = world_.r_coords_;

    std::vector<int> encountered_old_cluster_inds;
    for (const auto ind : selection) {
        if (world_.move_states_[ind] != MoveState::HOLDING) {
            boidind2hold_data_ind_[ind] = hold_data_.size();
            hold_data_.push_back({r_coords[ind], radii_[ind], ind});
            hold_inds_.push_back(ind);
            world_.move_states_[ind] = MoveState::HOLDING;
        }
    }
    p_ns_holding_->addOnGrid(r_coords, radii_, hold_inds_);

    addToClusters(selection);
}

void BoidControler::removeHoldingAgents( const std::vector<int>& selection) {
    removeFromClusters(selection);

    const auto& r_coords = world_.r_coords_;

    for (const auto ind : selection) {
        const auto hold_data_ind = boidind2hold_data_ind_[ind];
        if (hold_data_ind < 0 || hold_data_ind >= hold_data_.size()) {
            world_.move_states_[ind] = MoveState::STANDING;
            continue;
        }

        hold_data_[hold_data_ind] = hold_data_.back();
        hold_data_.pop_back();
        hold_inds_[hold_data_ind] = hold_inds_.back();
        hold_inds_.pop_back();
        if (hold_data_ind < hold_inds_.size()) {
            boidind2hold_data_ind_.at(hold_inds_[hold_data_ind]) = hold_data_ind;
        }

        world_.move_states_[ind] = MoveState::STANDING;
        boidind2hold_data_ind_[ind] = -1;
    }
    p_ns_holding_->addOnGrid(r_coords, radii_, hold_inds_);
}