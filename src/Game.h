#ifndef BOIDS_GAME_H
#define BOIDS_GAME_H

#include <unordered_set>
#include <numeric>

#include "core.h"
#include "Grid.h"
#include "HealthSystem.h"
#include "AttackSystem.h"
#include "MapGrid.h"
#include "BoidWorld.h"
#include "FogOfWar.h"

class BoidControler;
class UI;
class PathFinder;
// class HealthSystem;
// class AttackSystem;

inline void selectInRectangle(BoidWorld& world, const sf::Vector2f lower_left, const sf::Vector2f upper_right,
                              std::vector<int>& selection, int player_ind) {

    selection.clear();
    for (const auto ind : world.active_inds) {
        if (player_ind != world.ind2player[ind]) {
            continue;
        }
        if (isInRect(world.r_coords_[ind], lower_left, upper_right)) {
            selection.push_back(ind);
        }
    }
}

typedef u_int_16_t UnitTypeInd;
struct UnitType {
    float radius = RHARD;
    float max_speed = 0.3;
    float acceleration = MAXFLOAT;
    float turn_speed = 4;
    float hit_points = 10;
    float vision_range = 30 * RHARD;
    float armour = 0;
};

// class UnitCreator {
//
//     UnitType type_;
//
// public:
//     void setRadius(float radius){
//         type_.radius = radius;
//     }
//     void setMaxSpeed(float max_speed){
//         type_.max_speed = max_speed;
//     }
//     void
//
// };

struct UnitCreatorSettings : public Settings {
    enum Values { RADIUS = 0, MAX_SPEED, TURNRATE, ACCELERATION, COUNT };

    std::array<float, COUNT> values_;
    std::array<std::string, COUNT> value_names_;

    UnitCreatorSettings() {
        value_names_[RADIUS] = "radius";
        value_names_[MAX_SPEED] = "max speed";
        value_names_[TURNRATE] = "turn rate";
        value_names_[ACCELERATION] = "accel.";
        values_[RADIUS] = RHARD;
        values_[MAX_SPEED] = 0.75f;
        values_[TURNRATE] = 24;
        values_[ACCELERATION] = 1;
    }

    virtual void toggleOption(int option_ind) override {}

    virtual void setValue(int value_ind, float new_value) override { values_[value_ind] = new_value; };
    virtual float getValue(int value_ind) override { return values_[value_ind]; };
    virtual const std::string& getNameOf(int value_ind) const override { return value_names_[value_ind]; };
    virtual const int getCount() const override { return COUNT; }
};

struct TimeData {

    float max_time = 0;
    float avg_time = 0;
    const int n_frames = 5000;

    std::vector<float> times;
    int first = 0;

    TimeData()
        : times(n_frames, 0) {}

    void addTime(float t) {
        avg_time = (avg_time * n_frames + t - times[first]) / n_frames;
        times[first] = t;
        first++;
        if (first >= n_frames) {
            first = 0;
        }
        max_time = std::max(t, max_time);
    }
};

struct TimeData2 {

    float max_time = 0;
    float avg_time = 0;
    float avg_time_sq = 0;
    float error_est_avg_time = 0;
    unsigned long long n_frames = 0;
    u_int_32_t block_size_;
    std::vector<float> times;
    int first = 0;
    unsigned long long n_blocks = 0;

    TimeData2(int block_size = 200)
        : block_size_(block_size)
        , times(block_size, 0) {}

    void addTime(float t) {
        n_frames++;
        times[first] = t;
        first++;
        if (n_frames % block_size_ == 0) {
            const auto new_block_avg_time = std::accumulate(times.begin(), times.end(), 0) / block_size_;
            avg_time = (avg_time * n_blocks + new_block_avg_time) / (n_blocks + 1);
            avg_time_sq = (avg_time_sq * n_blocks + (new_block_avg_time * new_block_avg_time)) / (n_blocks + 1);
            std::fill(times.begin(), times.end(), 0);
            first = 0;
            n_blocks++;
        }
        max_time = std::max(t, max_time);
    }
};

class Triangulation;

class Game {

    struct Clocks {

        sf::Clock clock;

        TimeData pathfinding;
        TimeData2 neighbour_searching;
        TimeData fog_of_war;
        TimeData drawing;
    };

    Clocks clocks;
    sf::Clock wall_clock;

    std::unique_ptr<HealthSystem> health_system_;

    std::unique_ptr<AttackSystem> attack_system_;
    std::vector<UnitType> unit_types_;

    std::vector<UnitTypeInd> boid_ind2unit_type_;
    sf::Vector2f click_position = {0, 0};

    sf::Vector2f start_view_move_position;
    bool view_is_moving = false;
    std::vector<int> selection;

    sf::Vector2f start_position = {0, 0};
    sf::Vector2f end_position = {0, 0};
    bool selection_pending = false;
    bool drawing = false;
    bool bulding = false;
    int selected_building_index = 0;
    int selected_player = 0;
    sf::RectangleShape mouse_selection;
    sf::Vector2f box_size;

    sf::Clock building_clock;

    sf::Clock building_clock2;
    sf::Clock frame_clock;
    int frame_i = 0;

    sf::Clock explosion_time;
    float e_time;

  public:
    UnitCreatorSettings uc_settings_;

    std::unique_ptr<MapGrid> p_map_grid;
    std::unique_ptr<BoidControler> bc_; //! contains all steering information and pathfinding
    BoidWorld world_;                   //! world contains static coordinates and velocities
    std::unique_ptr<NeighbourSearcher> searcher;

    PathFinder& pf;
    Triangulation& cdt;

    bool game_is_stopped_ = false;
    bool last_pressed_was_space = false;

    std::unique_ptr<FogOfWar> p_fow_;

    Buildings buildings;

    Game(PathFinder& pf, Triangulation& cdt, sf::Vector2i n_cells, sf::Vector2f box_size);

    void addUnit(int player_ind, sf::Vector2f r, int unit_type_ind);

    void removeUnit(BoidInd u_ind);

    void update(const float dt, sf::RenderWindow& win);

    void parseInput(sf::RenderWindow& window, UI& ui);

    void draw(sf::RenderWindow& window);

    void createUnitType(const float max_speed, const float radius);

  private:

};

#endif // BOIDS_GAME_H