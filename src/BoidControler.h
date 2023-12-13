#ifndef BOIDS_BOIDCONTROLER_H
#define BOIDS_BOIDCONTROLER_H

#include <vector>
#include <map>
#include <cmath>
#include <set>
#include <memory>
#include <deque>
#include <unordered_set>
#include <unordered_map>


#include "SFML/Graphics.hpp"
#include "core.h"
#include "queue"
#include "Settings.h"
#include "NeighbourSearcher.h"



class DebugInfo;
class BoidWorld;
class NeighbourSearcher;
class Edges;

    //! \class knows which agents belong to which command group 
    //! \class 
    struct CommandGroupManager{
        //! Stuff for differnt groups sharing same command
        std::vector<int> boidind2_command_group_;
        std::vector<int> group2count_;
        std::unordered_set<int> inactive_groups = {0};
        std::vector<std::unordered_set<int>> group2_indices_;

        CommandGroupManager() :
        boidind2_command_group_(N_MAX_NAVIGABLE_BOIDS, -1) {}

        void removeFromGroup(const BoidInd ind){
            const auto command_group_ind = boidind2_command_group_[ind];
            if (command_group_ind != -1) {
                group2count_.at(command_group_ind)--;
            }
        }

        bool isSameGroup(const BoidInd i1, const BoidInd i2){
            return boidind2_command_group_[i1] == boidind2_command_group_[i2];
        }

        void addGroup(const std::vector<BoidInd>& selection){
                    int count = 0;
            int next_inactive_ind;
            if (inactive_groups.size() == 0) {
                next_inactive_ind = group2count_.size();
            } else {
                next_inactive_ind = *(inactive_groups.begin());
                inactive_groups.erase(next_inactive_ind);
            }

            for (const auto selected : selection) {
                boidind2_command_group_[selected] = next_inactive_ind;
                count++;
            }

            if (next_inactive_ind == group2_indices_.size()) {
                group2_indices_.push_back(std::unordered_set<int>(selection.begin(), selection.end()));
                group2count_.push_back(count);
                inactive_groups.insert(group2count_.size() - 1);
            } else {
                group2count_.at(next_inactive_ind) = count;

                group2_indices_[next_inactive_ind].insert(selection.begin(), selection.end());
            }
        }

        void updateInactives(){
            for (int group_ind = 0; group_ind < group2count_.size(); group_ind++) {
                if (group2count_[group_ind] == 0) {
                    inactive_groups.insert(group_ind);
                    for (const auto ind : group2_indices_.at(group_ind)) {
                        boidind2_command_group_[ind] = -1;
                    }
                    group2_indices_.at(group_ind).clear();
                }
            }
        }
    };



struct BoidControlerSettings2 : Settings {
    enum Options {};

    enum Values {
        AVOID = 0,
        AVOIDCLUSTERS = 1,
        PUSH = 2,
        SEEK = 3,
        SCATTER = 4,
        REPULSE = 5,
        ALIGN = 6,
        WALLSLIDE = 7,
        WALLREPULSE = 8,
        COUNT,
    };

    std::array<bool, 0> options_;
    std::array<float, COUNT> values_;
    std::array<std::string, 0> option_names_;
    std::array<std::string, COUNT> value_names_;

    BoidControlerSettings2() {
        value_names_[Values::AVOID] = "avoid";
        value_names_[Values::PUSH] = "push";
        value_names_[Values::AVOIDCLUSTERS] = "avoid clusters";
        value_names_[Values::SEEK] = "seek";
        value_names_[Values::SCATTER] = "scatter";
        value_names_[Values::REPULSE] = "repulse";
        value_names_[Values::ALIGN] = "align";
        value_names_[Values::WALLSLIDE] = "slide_wall";
        value_names_[Values::WALLREPULSE] = "repulse_wall";

        values_.fill(1);
        values_[Values::SEEK] = 0.5f;
        values_[Values::ALIGN] = 0.5f;
    }

    virtual void toggleOption(int o) override {}

    virtual void setValue(int v, float new_value) override {
        assert(v < COUNT);
        values_[v] = new_value;
    }
    virtual float getValue(int v) override {
        assert(v < COUNT);
        return values_[v];
    }

    bool hasAttribute(Options o) const { return options_[o]; }

    virtual const std::string& getNameOf(int o) const override {
        assert(o < COUNT);
        return value_names_.at(o);
    }

    virtual const int getCount() const override { return COUNT; }
};


//! \struct holds forces to simulate agents behaviour
struct ControlForces {
    sf::Vector2f avoid = {0, 0}; //! forces MOVING agents to move around HOLDING agents
    sf::Vector2f seek = {0, 0};  //! pulls MOVING agents towards their targets
    sf::Vector2f push = {0, 0};  //! pushes STANDING agents when MOVING
    sf::Vector2f align = {
        0, 0}; //! alligns MOVING agents velocities with neighbouring agents in the same command group
    sf::Vector2f scatter = {0, 0}; //! scatters agents so that they are not too close (maybe causes jerkiness)
    sf::Vector2f repulse = {0, 0}; //! physical repulsion of agents hard cores; (maybe causes jerkiness))

    typedef BoidControlerSettings2::Values Behaviour;

    sf::Vector2f accumulate(float dt, const std::array<float, Behaviour::COUNT>& proportions) const {
        return (avoid * proportions[Behaviour::AVOID] + seek * proportions[Behaviour::SEEK] +
                push * proportions[Behaviour::PUSH] + align * proportions[Behaviour::ALIGN] +
                scatter * proportions[Behaviour::SCATTER] + repulse * proportions[Behaviour::REPULSE]) *
                dt;
    }

    void reset() {
        avoid *= 0.f;
        seek *= 0.f;
        push *= 0.f;
        align *= 0.f;
        scatter *= 0.f;
        repulse *= 0.f;
    }
};


//! TODO: I should refactor it into physics and navigation part (physics does agents interaction + walls
//! TODO:  and navigation does checking whether agents passed point on a path...)  
//! \class is responsible for physics and neighbour searching of collidable agents with themselves + with walls
//! \note everything is on one place right now because it is easier to tweek this way and I don't know yet what type of forces I exactly want
class BoidControler {

    BoidControlerSettings2 settings_; //! contains data that I want to through UI at runtime (mainly proportions of
                                      //! different forces for tweeking)

    typedef BoidControlerSettings2::Values Behaviour;


    std::vector<sf::Vector2f> dr_nearest_neighbours_;
    std::vector<ControlForces> forces_;

    struct PathData {
        sf::Vector2f move_target;      //! position of currently sought point (boids move towards this point!)
        sf::Vector2f next_move_target; //! position of point sought after the current ont
        Edgef portal;                  //! when this line is passed the boid starts moving towards next_move_target
        Edgef next_portal;             //!
        sf::Vector2f path_end;         //! final point on entire path
        bool need_path_update = false; //! wtf?
    };

    std::vector<PathData> path_data_;

    BoidWorld& world_; //! reference to actual positions and velocities
    NeighbourSearcher&
        ns_; //! object that manages verlet lists and can give me nearest neigbhours (I think raw pointer is ok because
             //! lifetime of Boidcontroler is bound by lifetime of Game which owns BoidControler)

    struct HoldingData {
        sf::Vector2f r;
        float radius;
        int boid_ind;
    };
    std::vector<int> boidind2hold_data_ind_;
    std::vector<HoldingData> hold_data_;
    std::vector<int> hold_inds_;

    std::unique_ptr<NeighbourSearcher> p_ns_holding_; //! used for finding closest holding agents does not get updated
                                                      //! each frame because holding agents do not move
    
    //! \struct holds data needed to calculate force direction to align velocities of neighbouring agents
    struct AlignData {
        sf::Vector2f direction;
        int count = 0; //! number of neighbouring agents from the same command group
    }; 
    std::vector<AlignData> align_data_;

    std::shared_ptr<Edges> map_; //! reference to hard walls which agents avoid

    std::vector<int> n_steps_since_all_in_front_standing_;

    //! stuff to for experimenting with collective motion (will clean once I decide what will be kept)
    std::vector<bool> is_near_wall;
    std::vector<bool> is_being_pushed_;
    std::vector<bool> is_returning_;
    std::vector<int> n_steps_since_moved_away_;
    std::deque<std::pair<int, sf::Vector2f>> return_pos_info_;
    int steps_since_last_turn = 0;

    std::vector<float> max_speeds_; //! holds maximum speed of each agent

  public:
    const AcosTable<1000> angle_calculator_; //! table for calculating angles (not sure if I need it anywhere else right
                                             //! now but I should probably make that static?)

    std::queue<int> need_path_update_;
    std::vector<float> radii_;

    friend DebugInfo;

    BoidControlerSettings2& getSettings() { return settings_; }

    std::vector<sf::Vector2f> standing_pos_;    //! position where the agent should return after being pushed

    CommandGroupManager commander_groups_;    

    //! stuff related to turning and orientation
    std::vector<bool> is_turning;               
    std::vector<float> turn_rates_;
    std::vector<float>& orientation_;
    //!
    std::vector<BoidInd> attack_targets_;

    //! Holds data relating to clusters of HOLDING particles
    struct Cluster {
        sf::Vector2f center;   //! center of mass of cluster
        float radius2 = 0;     //! radius squared
        std::vector<int> inds; //! boid_inds that form the cluster
        int ind_in_active;     //! index in vector of active clusters
        int ind_in_inactive;   //! index in vector of inactive clusters (wtf?)
        bool is_in(const sf::Vector2f& r) const { return dist2(center, r) < radius2; }
        void merge(const Cluster& cluster) {
            const float old_size = inds.size();
            inds.insert(inds.end(), cluster.inds.begin(), cluster.inds.end());
            const float new_size = inds.size();
            center = (center * old_size + cluster.center * (new_size - old_size)) / new_size;
        }
        void add(const int ind, const sf::Vector2f& r) {
            inds.push_back(ind);
            const float n_in_cluster = inds.size();
            center = (center * (n_in_cluster - 1) + r) / n_in_cluster;
        }
        int removeInd(const int ind, const sf::Vector2f& r) {
            const float old_size = inds.size();
            inds.erase(
                std::find(inds.begin(), inds.end(), ind)); //! we could make this constant at a cost of some memory
            center = (center * old_size - r) / (old_size - 1);
            return inds.size();
        }
    };

    std::vector<int> boidind2cluster_ind_;
    std::vector<Cluster> clusters_;
    std::vector<int> inactive_cluster_inds_;
    std::vector<int> active_cluster_inds_;

    void addHoldingAgents(std::vector<int>& selection);
    void addToClusters( std::vector<int>& selection);
    void removeFromClusters( const std::vector<int>& selection);
    void removeHoldingAgents( const std::vector<int>& selection);

    BoidControler(BoidWorld* world, NeighbourSearcher* ns, std::shared_ptr<Edges> map, sf::Vector2f box_size);

    void activate(int player_ind, const float max_speed, const float radius, const float turn_rate);
    void deactivate(BoidInd u_ind);
    void updateState(float dt, sf::RenderWindow& window);

    void setMoveTarget(int boid_ind, sf::Vector2f target) {
        path_data_[boid_ind].move_target = target;
        path_data_[boid_ind].next_move_target = target;
        path_data_[boid_ind].portal = Edgef{};
        path_data_[boid_ind].next_portal = Edgef{};
    }

    std::vector<ControlForces>& getForces(){
        return forces_;
    } 

    void setPathEnd(int boid_ind, sf::Vector2f end) { path_data_[boid_ind].path_end = end; }
    sf::Vector2f getPathEnd(int boid_ind) const { return path_data_[boid_ind].path_end; }

    void setPathData(int boid_ind, sf::Vector2f target, const Edgef& portal);
    void setPathDataNext(int boid_ind, sf::Vector2f target, const Edgef& portal) {
        path_data_[boid_ind].next_move_target = target;
        path_data_[boid_ind].next_portal = portal;
    }
    void setAttackTarget(BoidInd boid_ind, BoidInd target_ind) { attack_targets_[boid_ind] = target_ind; }
    void addCommandGroup(const std::vector<BoidInd>& selection) {
        commander_groups_.addGroup(selection);
    }
    void updateInactiveGroupInd() {
        commander_groups_.updateInactives();
    }

    void setPathEnds(const std::vector<BoidInd>& selection, const sf::Vector2f path_end){
        for (const auto selected : selection) {
            setPathEnd(selected, path_end);
            commander_groups_.removeFromGroup(selected);
        }
        if (selection.size() > 0) {
            addCommandGroup(selection);
        }
    }

    void setStanding( const std::vector<BoidInd>& selection){
        for (int sel : selection) {
            world_.move_states_[sel] = MoveState::STANDING;
            standing_pos_[sel] = world_.r_coords_[sel];
            commander_groups_.removeFromGroup(sel);
        }
        removeHoldingAgents(selection);
    }

    void formFormation(const sf::Vector2f center, const sf::Vector2f orientation, const std::vector<int>& selection,
                       const uint8_t n_collumns = 5);
    void attack(float dt);


    struct ExplosionData{
        sf::Vector2f r_center; //! center of explosion
        float vel = 100.f;
        float radius = 0.f;
        float max_radius = 300.f;
        float delta_t = 0; //! time since explosion
        // ExplosionData(sf::Vector2f r_center) : r_center(r_center) {}
        // ~ExplosionData() = default;
    };
    std::unordered_map<int, ExplosionData> explosions_; 
    void addExplosion(const sf::Vector2f center);

  private:
    void applyExternalForces(float dt);

    void avoid(float dt);
    void repulseBoidsPairList(float dt);
    void repulseBoidsNeighbourList(float dt);

    void avoidHoldingAgents(float dt, sf::RenderWindow& window);
    void allign(float dt);
    void avoidWall(float dt);
    void flock(float dt);
    void scatter(float dt);
    void push(float dt);
    void wallConstraint(float dt);
    void seek();
    void turn();
    void truncateVels();
    bool isStuck(int boid_ind);
};

#endif // BOIDS_BOIDCONTROLER_H
