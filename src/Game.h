#ifndef BOIDS_GAME_H
#define BOIDS_GAME_H

#include <unordered_set>
#include <numeric>


#include "Utils/Vector.hpp"
#include "Utils/Grid.h"

#include "Selection.h"

#include "core.h"
#include "MapGrid.h"
#include "ECS.h"
#include "Settings.h"
#include "PathFinding/PathFinder2.h"
#include "SoundModule.h"
#include "BuildingManager.h"

#include "Graphics/RenderWindow.hpp"
#include "Graphics/SceneLayer.h"

// #include "Clock.hpp"

class UI;



inline void selectInRectangle(ECSystem& god, const sf::Vector2f lower_left, const sf::Vector2f upper_right,
                              std::vector<int>& selection, int player_ind) {

    const auto& data = god.entity2shared_data;

    selection.clear();
    for (const auto ent : god.active_entities_) {
        // if (player_ind != data.at(ind).transform) {
        //     continue;
        // }
        if (isInRect(data.at(ent.ind).transform.r, lower_left, upper_right)) {
            selection.push_back(ent.ind);
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

struct UnitCreatorSettings : public Settings {
    enum Values { RADIUS = 0, MAX_SPEED, TURNRATE, ACCELERATION, COUNT_VALS };

    std::array<float, COUNT_VALS> values_;
    std::array<std::string, COUNT_VALS> value_names_;

    UnitCreatorSettings() {
        value_names_[RADIUS] = "radius";
        value_names_[MAX_SPEED] = "max speed";
        value_names_[TURNRATE] = "turn rate";
        value_names_[ACCELERATION] = "accel.";
        values_[RADIUS] = RHARD;
        values_[MAX_SPEED] = 0.75f;
        values_[TURNRATE] = 5;
        values_[ACCELERATION] = 1;
    }

    virtual void toggleOption(int option_ind) override {}

    virtual void setValue(int value_ind, float new_value) override { values_[value_ind] = new_value; };
    virtual float getValue(int value_ind) override { return values_[value_ind]; };
    virtual const std::string& getNameOfOption(int o) const override { return value_names_.at(0);}
    virtual const std::string& getNameOfValue(int o) const override { return value_names_.at(o);} 
    virtual const int getCountValues() const override { return COUNT_VALS; }
    virtual const int getCountOptions() const override { return 0; }
};


class Triangulation;
class UnitInitializer ;

class Game {

public:



    struct Clocks {
        // sf::Clock clock;
        TimeData pathfinding;
        TimeData2 neighbour_searching;
        TimeData fog_of_war;
        TimeData drawing;
    };

    Clocks clocks;
    // sf::Clock wall_clock;
    std::vector<UnitType> unit_types_;
    std::vector<UnitTypeInd> boid_ind2unit_type_;
    sf::Vector2f click_position = {0, 0};


    sf::Vector2f start_view_move_position;
    bool view_is_moving = false;
    Selection selection_;

    sf::Vector2f start_position = {0, 0};
    sf::Vector2f end_position = {0, 0};
    bool selection_pending = false;
    bool drawing = false;
    bool bulding = false;
    int selected_building_index = 0;
    int selected_player = 0;
    sf::RectangleShape mouse_selection;
    sf::Vector2f box_size;

    // sf::Clock building_clock;

    // sf::Clock building_clock2;
    // sf::Clock frame_clock;
    // int frame_i = 0;

    // sf::Clock explosion_time;
    // float e_time;

    // SoundModule sound_module_;

  public:

    UnitCreatorSettings uc_settings_;


    std::unique_ptr<MapGrid> p_map_grid;
    BuildingManager building_manager;

    std::shared_ptr<PathFinder2> p_pathfinder_;
    Triangulation& cdt;

    bool game_is_stopped_ = false;
    bool last_pressed_was_space = false;

    std::unique_ptr<ECSystem> p_the_god_;
    std::shared_ptr<UnitInitializer> unit_creator_;

    Buildings buildings;
    std::deque<sf::Vector2f> path;
    std::deque<Edgef> portals;

    UnitLayer unit_scene;
    BuildingLayer building_scene;
    VisionLayer vision_layer;
    MapGridLayer map_layer;

    Game(Triangulation& cdt, sf::Vector2i n_cells, sf::Vector2f box_size);

    void addUnit(int player_ind, sf::Vector2f r, int unit_type_ind);
    void removeUnit(Entity e);

    void update(const float dt, sf::RenderWindow& win);

    void parseInput(sf::RenderWindow& window, UI& ui);

    void draw(sf::RenderWindow& window);

    void createUnitType(const float max_speed, const float radius);

  private:
    void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
    void moveView(sf::RenderWindow& window);
    void parseEvents(sf::RenderWindow& window, UI& ui);
    void updateTriangulation();
};

#endif // BOIDS_GAME_H