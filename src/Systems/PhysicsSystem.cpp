#include "PhysicsSystem.h"

#include "../Utils/NeighbourSearcherStrategy.h"

PhysicsSystem::PhysicsSystem(ComponentID id) : System2(id){
        sf::Vector2f box = {(float)Geometry::BOX[0], (float)Geometry::BOX[1]};
        p_ns_ = std::make_unique<NeighbourSearcherContext<PhysicsComponent, InteractionData>>(box, RHARD * 20);
        p_ns_->setStrategy(NS_STRATEGY::BASIC);
    }


//! \brief calculates "forces" of each component and clamps velocities. Velocities are of two types: inertial and normal. 
//! \brief Inertial velocities do not get zeroes at the end of frame and thus can accumulate.
//! \brief  However, they are subject to exponential decay.
//! \brief afterwards we apply wall and other constraints to avoid passing through obstacles  
void PhysicsSystem::update() {
        auto& components = static_cast<CompArray&>(*p_comps_).components_;
        p_ns_->update(components);

        std::chrono::high_resolution_clock clock;
        auto t_start = clock.now();
        for(int comp_ind = 0; comp_ind < n_components; ++comp_ind){
            const auto& interaction_data = p_ns_->getInteractionData(comp_ind);
            auto& phys_comp = components.at(comp_ind);

            applyForces(interaction_data, p_ns_->last_i.at(comp_ind), phys_comp);
            auto speed = norm(phys_comp.transform.vel);
            if( speed > max_speed ){
                phys_comp.transform.vel /= speed * max_speed ;
            }
            speed = norm(phys_comp.inertia_vel);
            if( speed > 3*phys_comp.max_speed ){
                phys_comp.inertia_vel *=3*phys_comp.max_speed / speed ;
            }

            phys_comp.transform.vel += phys_comp.inertia_vel;
            phys_comp.inertia_vel -= phys_comp.inertia_vel*phys_comp.lambda; //! velocity decay              
        }
        auto delta_t =  std::chrono::duration_cast<std::chrono::microseconds>(clock.now() - t_start);
        // std::cout << delta_t << "\n";
        avoidWall();
    }


//! \brief calculates forces of a given \param phys_comp 
//! \brief The forces are: 
//! \brief   
    void PhysicsSystem::applyForces(const std::array<InteractionData, N_MAX_NEIGHBOURS>& data, const int n_last_neighbour, PhysicsComponent& phys_comp){
            const auto mass_i = phys_comp.mass;
            const auto state_i = phys_comp.state;
            const auto player_ind = phys_comp.player_ind;

            const auto scatter_multiplier = force_multipliers[Multiplier::SCATTER];
            const auto repulse_multiplier = force_multipliers[Multiplier::REPULSE];
            const auto push_multiplier = force_multipliers[Multiplier::PUSH];
            const auto align_multiplier = force_multipliers[Multiplier::ALIGN];

            sf::Vector2f repulsion_force(0,0);
            sf::Vector2f push_force(0,0);
            sf::Vector2f scatter_force(0,0);
            float n_neighbours = 0;
            sf::Vector2f dr_nearest_neighbours(0,0);
            for (int ind_j = 0; ind_j < n_last_neighbour; ++ind_j) {
                const auto& n_data = data[ind_j];
                const auto& dr = n_data.dr;
                const auto mass_j = n_data.mass;
                const auto boid_ind_j = n_data.second;
                const auto r_collision_sq = n_data.r_collision*n_data.r_collision;
                const auto d_surfs = norm2(dr);
                const auto x = r_collision_sq / (d_surfs);
                const auto state_j = n_data.state;
                // const auto alpha = radii_[ind_i] / pair.r_collision;
                // const bool is_same_group = commander_groups_.isSameGroup(ind_i, boid_ind_j);
                const bool is_same_player =  player_ind != n_data.player_ind;
                if(x > 0.8f)  [[unlikely]] 
                {
                    const auto x_prime = (x);
                    repulsion_force += -dr / norm(dr) * std::min(50000.0f, 3.f * x_prime*x_prime * x_prime)* mass_j/(mass_i+mass_j); 
                }

                // if (state_i == MoveState::MOVING and state_j == MoveState::MOVING 
                //      and is_same_group and x > r_collision_sq / (RALLIGN * RALLIGN)) {
                //     align_data_[ind_i].direction += forces_[ind_i].seek;
                //     align_data_[ind_i].count++;
                // }

                 bool moving_pushes_standing_a = state_j == MoveState::MOVING and state_i == MoveState::STANDING;
                if (moving_pushes_standing_a and x > 0.75f) [[unlikely]] 
                {
                    push_force += -dr * x * mass_j/(mass_i+mass_j) ;
                }

                if (state_j == MoveState::MOVING and x > r_collision_sq / (RSCATTER * RSCATTER))
                {
                    dr_nearest_neighbours += n_data.dr;
                    n_neighbours++;
                }
            }

             dr_nearest_neighbours /= n_neighbours;

            if (n_neighbours > 1) {
                scatter_force = dr_nearest_neighbours / norm(dr_nearest_neighbours) * phys_comp.max_speed;
            }

        
            phys_comp.inertia_vel += (repulse_multiplier*repulsion_force +
                                      push_multiplier*push_force +
                                      scatter_multiplier*scatter_force);

    }
    void PhysicsSystem::communicate(std::array<SharedData, N_MAX_ENTITIES>& entity2shared_data)const{
        auto& components = static_cast<CompArray&>(*p_comps_).components_;
        
        int comp_ind = 0;
        for(const auto& comp : components){
            auto entity_ind = compvec_ind2entity_ind_.at(comp_ind);
            entity2shared_data.at(entity_ind).transform.vel += comp.transform.vel;
            comp_ind++;
        }
    }


void PhysicsSystem::avoidWall() {
    auto& components = static_cast<CompArray&>(*p_comps_).components_;
    p_ns_->update(components);
    const auto n_comps = components.size();    
    
#pragma omp parallel for num_threads(NUM_OMP_WALLS_THREADS)
    for (int comp_ind = 0; comp_ind < n_comps; ++comp_ind) {
        auto& comp = components.at(comp_ind);
        const auto& r_selected = comp.transform.r;
        auto& vel = comp.transform.vel; 
        auto& v_inertial = comp.inertia_vel;
        const auto radius = comp.radius;
        const auto walls = p_map_->calcContactEdgesO(r_selected, radius);

        bool is_touching_wall = false;
        for (int orient = 0; orient < N_DIRECTIONS; orient++) {
            for (const auto& wall : walls[orient]) {
                is_touching_wall = true;
                sf::Vector2f n_wall = {wall.t.y, -wall.t.x};

                auto v_away_from_surface = n_wall * dot(vel, n_wall);
                if (dot(vel, n_wall) < 0) {
                    vel -= 1.0f*v_away_from_surface;
                }
                v_away_from_surface = n_wall * dot(v_inertial, n_wall);
                if (dot(v_inertial, n_wall) < 0) {
                    v_inertial -= v_away_from_surface;
                }

            }
        }
    }
}


void PhysicsSystem::updateSharedData(const std::array<SharedData, N_MAX_ENTITIES>& new_data, const std::vector<Entity>& active_entity_inds)
{
    auto &comps = static_cast<ComponentArray<PhysicsComponent> &>(*p_comps_.get()).components_;
    for(const auto ent : active_entity_inds){
        const auto compvec_ind = entity2compvec_ind_.at(ent.ind);
        comps.at(compvec_ind).transform = new_data.at(ent.ind).transform;
        comps.at(compvec_ind).state = new_data.at(ent.ind).state;
    }
}

void PhysicsSystem::communicateVelocities(std::array<SharedData, N_MAX_ENTITIES>& entity2shared_data)const{
    auto& components = static_cast<CompArray&>(*p_comps_).components_;
    int comp_ind = 0;
    for(const auto& comp : components){
        auto entity_ind = compvec_ind2entity_ind_.at(comp_ind);
        entity2shared_data.at(entity_ind).transform.vel = comp.transform.vel;
        comp_ind++;
    }
}



// //! \brief makes agents go around other agents, that are HOLDING
// void BoidControler::avoidHoldingAgents(const float dt, sf::RenderWindow& window) {
//     auto& r_coords = world_.r_coords_;

//     // for (const auto cluster_ind : active_cluster_inds_) {
//     //     auto cluster_radius2 = clusters_[cluster_ind].radius2;
//     //     auto& center_of_cluster = clusters_[cluster_ind].center;
//     //     center_of_cluster *= 0.f;
//     //     const float n_in_cluster = clusters_[cluster_ind].inds.size();
//     //     for (const auto ind_in_cluster : clusters_[cluster_ind].inds) {
//     //         center_of_cluster += r_coords[ind_in_cluster] / n_in_cluster;
//     //     }
//     //     for (const auto ind_in_cluster : clusters_[cluster_ind].inds) {
//     //         cluster_radius2 =
//     //             std::max({dist2(center_of_cluster, r_coords[ind_in_cluster]) + RHARD * RHARD, cluster_radius2});
//     //     }
//     //     clusters_[cluster_ind].radius2 = cluster_radius2;
//     // }

//     for (const auto selected : world_.active_inds) {
//         sf::Vector2f delta_v(0, 0);
//         const auto& move_target = path_data_[selected].move_target;
//         const auto r_selected = r_coords[selected];
//         auto dr_to_target = move_target - r_selected;
//         dr_to_target /= norm(dr_to_target);

//         const auto selected_is_moving = world_.move_states_[selected] == MoveState::MOVING;

//         if (selected_is_moving) {
//             if (norm2(dr_to_target) == 0.f) {
//                 continue;
//             }
//             for (const auto cluster_ind : active_cluster_inds_) {
//                 const auto& cluster = clusters_.at(cluster_ind);
//                 if (cluster.is_in(move_target)) { //! we do not move around when we want to get inside cluster!
//                     break;
//                 }
//                 const auto dr_to_cluster = cluster.center - r_selected;
//                 if (norm2(dr_to_cluster) == 0) {
//                     continue;
//                 }
//                 const auto angle = angle_calculator_.angleBetween(dr_to_cluster, dr_to_target);
//                 //                bool is_above = sign(r_selected, cluster.center, move_targets_[selected]);
//                 const float sign =
//                     2 * (angle < 0) - 1; // 2*(dot(dr_to_target, dr_to_target - dr_to_cluster*dot(dr_to_target,
//                                          // dr_to_cluster)/dot(dr_to_cluster, dr_to_cluster))<0) -1;

//                 dr_to_target /= norm(dr_to_target);
//                 sf::Vector2f dr_norm = {dr_to_target.y, -dr_to_target.x};

//                 //                if (dot(dr_norm, dr_to_cluster) < 0.f) { dr_norm *= -1.f;}

//                 const auto dist2_to_cluster = dist2(cluster.center, r_selected);
//                 if (dist2_to_cluster < cluster.radius2 * 4 and
//                     std::abs(angle) < 70) { //! if the cluster is close enough in my direction
//                     const auto next_value =
//                         sign * dr_norm * 1.5f * cluster.radius2 / (dist2_to_cluster + cluster.radius2);
//                     delta_v += next_value * settings_.values_[BoidControlerSettings2::AVOIDCLUSTERS];
//                 }
//             }

//             sf::Vector2f steepest_growth_dir = {0.f, 0.f};
//             const auto radius = radii_[selected];
//             const auto& verlet_list = p_ns_holding_->getNeighboursInds(r_selected, 6.0f * radius * radius);

//             for (const auto& neighbour : verlet_list) {
//                 const auto neighbour_is_holding = world_.move_states_[neighbour] == MoveState::HOLDING;

//                 if (neighbour_is_holding) {
//                     const auto dr = r_coords[neighbour] - r_selected;
//                     const sf::Vector2f dr_norm = {dr.y, -dr.x};
//                     const auto r_collision = radius + radii_[neighbour];
//                     const auto d2 = norm2(dr);
//                     const auto cluster_center = clusters_.at(boidind2cluster_ind_[neighbour]).center;
//                     //                    bool neighbour_in_front = dot(dr, dr_to_target) > 0;
//                     bool distance_close_enough = d2 < 1.2 * r_collision * r_collision;
//                     bool angle_condition = std::abs(angle_calculator_.angleBetween(dr, dr_to_target)) < 120;
//                     if (distance_close_enough)
//                         if (angle_condition) {
//                             float angle;
//                             if (r_coords[neighbour] != cluster_center) {
//                                 angle = angle_calculator_.angleBetween(move_target - cluster_center,
//                                                                        r_coords[neighbour] - cluster_center);
//                             } else {
//                                 angle = 0;
//                             }
//                             const float sign = 2 * (angle < 0) - 1;
//                             steepest_growth_dir += sign * settings_.values_[BoidControlerSettings2::AVOID] /
//                                                    std::pow((1 + d2 / RHARD / RHARD), -1.f) * dr_norm / (RHARD * RHARD);
//                         }
//                 }

//                 // 2*(dot(dr_to_target, dr_to_target - dr_to_cluster*dot(dr_to_target, dr_to_cluster)/dot(dr_to_cluster,
//                 // dr_to_cluster))<0) -1;

//                 //                steepest_growth_dir = {steepest_growth_dir.y, -steepest_growth_dir.x};
//                 delta_v += steepest_growth_dir;
//                 if (norm2(delta_v) > max_speeds_[selected] * max_speeds_[selected]) {
//                     delta_v *= max_speeds_[selected] / norm(delta_v);
//                 }
//             }
//             forces_[selected].avoid = delta_v;
//         }
//     }
// }




// void BoidControler::removeFromClusters(const std::vector<int>& selection) {
//     for (const auto ind : selection) {
//         const auto cluster_ind = boidind2cluster_ind_[ind];
//         if (cluster_ind != -1) {
//             const auto cluster_size = clusters_.at(cluster_ind).removeInd(ind, world_.r_coords_[ind]);
//             if (cluster_size <= 0) {
//                 assert(cluster_size == 0);
//                 int removed_ind_in_active = clusters_.at(cluster_ind).ind_in_active;
//                 active_cluster_inds_.at(removed_ind_in_active) = active_cluster_inds_.back();
//                 clusters_.at(active_cluster_inds_.back()).ind_in_active = removed_ind_in_active;
//                 active_cluster_inds_.pop_back();
//                 if (cluster_ind == clusters_.size() - 1) { //! remove if newly the vacant cluster is the last one
//                     clusters_.pop_back();
//                 } else { //! otherwise put it on a stack of inactive ones, the vector of active_inds is made contigious
//                     clusters_[cluster_ind].radius2 = 0;
//                     inactive_cluster_inds_.push_back(cluster_ind);
//                 }
//             }
//             boidind2cluster_ind_[ind] = -1;
//         }
//     }
// }

// void BoidControler::addToClusters( std::vector<int>& selection) {

//     std::sort(selection.begin(), selection.end());
//     auto is_in_selection = [&](const int& ind) { return std::binary_search(selection.begin(), selection.end(), ind); };

//     const auto& r_coords = world_.r_coords_;

//     //! we set cluster_inds of all selections to default value
//     //!
//     for (const auto selected_ind : selection) {
//         //        if(
//         //        boidind2cluster_ind_[selected_ind] = -1;
//     }

//     std::vector<int> encountered_old_cluster_inds;
//     for (const auto selected_ind : selection) {

//         std::queue<int> to_check;
//         to_check.push(selected_ind);
//         if (boidind2cluster_ind_[selected_ind] != -1) {
//             //! to avoid visiting one boid_ind multiple times since we already assigned cluster_ind to it
//             continue;
//         }

//         const auto radius_sq = radii_[selected_ind] * radii_[selected_ind];

//         Cluster new_cluster;
//         int current_cluster_ind;
//         if (inactive_cluster_inds_.empty()) {
//             current_cluster_ind = clusters_.size();
//             clusters_.push_back(new_cluster);
//         } else {
//             current_cluster_ind = inactive_cluster_inds_.back();
//             inactive_cluster_inds_.pop_back();
//         }
//         auto& cluster = clusters_.at(current_cluster_ind);
//         cluster.ind_in_active = active_cluster_inds_.size();
//         active_cluster_inds_.push_back(current_cluster_ind);

//         std::vector<bool> cluster2already_encountered(clusters_.size(), false);

//         while (!to_check.empty()) {
//             const auto ind = to_check.front();
//             to_check.pop();
//             if (boidind2cluster_ind_[ind] !=
//                 -1) { //! to avoid visiting one boid_ind multiple times since we already assigned cluster_ind to it
//                 continue;
//             }

//             const auto r = r_coords[ind];
//             clusters_[current_cluster_ind].add(ind, r);
//             boidind2cluster_ind_[ind] = current_cluster_ind;

//             const auto& neighbours = p_ns_holding_->getNeighboursInds(r, 6.0f * radius_sq);
//             for (const auto neighbour : neighbours) {
//                 const auto& r_neighbour = r_coords[neighbour];
//                 const auto existing_cluster_ind = boidind2cluster_ind_[neighbour];
//                 const auto r_collision = radii_[selected_ind] + radii_[neighbour];
//                 if (dist(r, r_neighbour) < 2.1 * r_collision) {
//                     if (is_in_selection(neighbour)) {
//                         to_check.push(neighbour);
//                     } else if (world_.move_states_[neighbour] == MoveState::HOLDING and
//                                !cluster2already_encountered[existing_cluster_ind]) {
//                         encountered_old_cluster_inds.push_back(
//                             existing_cluster_ind); //! we will have to update its radius2
//                         cluster2already_encountered[existing_cluster_ind] = true;
//                     }
//                 }
//             }
//         }

//         //! We merge all encountered clusters into one (maybe one with smallest index?)

//         if (encountered_old_cluster_inds.size() > 0) {
//             const auto cluster_ind_to_keep = current_cluster_ind;
//             auto& cluster_to_keep = clusters_[cluster_ind_to_keep];

//             for (int i = 0; i < encountered_old_cluster_inds.size(); ++i) {
//                 const auto cluster_ind_to_remove = encountered_old_cluster_inds[i];
//                 auto& cluster_to_remove = clusters_[cluster_ind_to_remove];
//                 for (int ind : cluster_to_remove.inds) {
//                     boidind2cluster_ind_[ind] = cluster_ind_to_keep;
//                 }
//                 cluster_to_keep.merge(cluster_to_remove);
//                 cluster_to_remove.inds.clear();
//                 inactive_cluster_inds_.push_back(cluster_ind_to_remove);

//                 int removed_ind_in_active = clusters_.at(cluster_ind_to_remove).ind_in_active;
//                 active_cluster_inds_.at(removed_ind_in_active) = active_cluster_inds_.back();
//                 clusters_.at(active_cluster_inds_.back()).ind_in_active = removed_ind_in_active;
//                 active_cluster_inds_.pop_back();
//             }

//             current_cluster_ind = cluster_ind_to_keep;
//         }

//         auto& cluster_radius2 = clusters_[current_cluster_ind].radius2;
//         auto& center_of_cluster = clusters_[current_cluster_ind].center;
//         for (const auto ind_in_cluster : clusters_[current_cluster_ind].inds) {
//             cluster_radius2 =
//                 std::max({dist2(center_of_cluster, r_coords[ind_in_cluster]) + radius_sq, cluster_radius2});
//         }
//     }
// }

// void BoidControler::addHoldingAgents( std::vector<int>& selection) {

//     const auto& r_coords = world_.r_coords_;

//     std::vector<int> encountered_old_cluster_inds;
//     for (const auto ind : selection) {
//         if (world_.move_states_[ind] != MoveState::HOLDING) {
//             boidind2hold_data_ind_[ind] = hold_data_.size();
//             hold_data_.push_back({r_coords[ind], radii_[ind], ind});
//             hold_inds_.push_back(ind);
//             world_.move_states_[ind] = MoveState::HOLDING;
//         }
//     }
//     p_ns_holding_->addOnGrid(r_coords, radii_, hold_inds_);

//     addToClusters(selection);
// }

// void BoidControler::removeHoldingAgents( const std::vector<int>& selection) {
//     removeFromClusters(selection);

//     const auto& r_coords = world_.r_coords_;

//     for (const auto ind : selection) {
//         const auto hold_data_ind = boidind2hold_data_ind_[ind];
//         if (hold_data_ind < 0 || hold_data_ind >= hold_data_.size()) {
//             world_.move_states_[ind] = MoveState::STANDING;
//             continue;
//         }

//         hold_data_[hold_data_ind] = hold_data_.back();
//         hold_data_.pop_back();
//         hold_inds_[hold_data_ind] = hold_inds_.back();
//         hold_inds_.pop_back();
//         if (hold_data_ind < hold_inds_.size()) {
//             boidind2hold_data_ind_.at(hold_inds_[hold_data_ind]) = hold_data_ind;
//         }

//         world_.move_states_[ind] = MoveState::STANDING;
//         boidind2hold_data_ind_[ind] = -1;
//     }
//     p_ns_holding_->addOnGrid(r_coords, radii_, hold_inds_);
// }
