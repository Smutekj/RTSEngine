#ifndef BOIDS_UI_H
#define BOIDS_UI_H

#include <vector>
#include <memory>
#include <unordered_map>
#include <array>

#include "Utils/Vector.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/matrix_transform_2d.hpp>
#include <glm/gtc/type_ptr.hpp>

class Game;
class VisionSystem;
class DebugInfo;
namespace sf
{
    class RenderWindow;
}

enum class UIWindowType
{
    PHYSICS = 0,
    PATHFINDING,
    DEBUG,
    SHADERS,
    GRAPHICS,
    UNIT_MAKER,
    WALLS,
    COUNT
};

constexpr int N_UI_WINDOWS = static_cast<int>(UIWindowType::COUNT);

class UIWindow
{

protected:
    std::string name;
    bool is_active = false;
    std::vector<std::unique_ptr<UIWindow>> children;

    std::array<void *, N_UI_WINDOWS> controled_data;

public:
    virtual void draw() = 0;
    virtual ~UIWindow() = 0;

    UIWindow(std::string name);

    const std::string& getName()const{
        return name;
    } 
};

class PhysicsSystem;
class SeekSystem;

class PhysicsWindow : public UIWindow
{

    float *gravity = nullptr;

    enum Data
    {
        GRAVITY = 0,
        MAX_SPEED,
    };

public:
    PhysicsWindow(PhysicsSystem &ps);

    ~PhysicsWindow();

    virtual void draw() override;
};

class PathfindingWindow : public UIWindow
{

    // float* gravity = nullptr;
    enum Data
    {
        SHOW_PATH
    };

public:
    PathfindingWindow(SeekSystem &ss, DebugInfo& dbg);

    virtual ~PathfindingWindow();

    virtual void draw() override;
};

class Shader;

struct ColorData{
    std::string uniform_name;
    glm::vec4 value = glm::vec4(0.5, 0.1, 0.5, 1.0);
};

struct ValueData{
    std::string uniform_name;
    float value;
};

struct ShaderUIData{
    Shader* p_program;
    std::string filename;
    std::vector<ColorData> colors;
    std::vector<ValueData> values;
};

struct BuildingLayer;

class ShadersWindow : public UIWindow
{

    std::vector<ShaderUIData> shaders;
    enum Data
    {
        COLOR1,
        COLOR2,
    };

public:
    ShadersWindow(BuildingLayer& bl);

    virtual ~ShadersWindow();

    virtual void draw() override;
};


class DeubgInfoWindow : public UIWindow
{

    // float* gravity = nullptr;
    enum Data
    {
        SHOW_GRID = 0,
        SHOW_TRI_INDS,
        SHOW_TRIANGLES,
        SHOW_N_TRIANGLES,
        SHOW_FPS,
        SHOW_SUPER_SECRET_WINDOW
    };

public:
    DeubgInfoWindow(DebugInfo &di);

    virtual ~DeubgInfoWindow();

    virtual void draw() override;
};

class UnitInitializer;
class UnitMakerWindow : public UIWindow
{
        
    enum class Data2 :  int
    {
        MAX_VEL = 0,
        RADIUS,
        ROTATION_VEL,
        MASS,
        VISION_RADIUS,
        WEAPON_RANGE,
        COUNT
    };
    const std::size_t unit_data_count = static_cast<std::size_t>(Data2::COUNT);

    std::vector<std::unordered_map<Data2, float>> type_ind2data_values;
    std::vector<std::unordered_map<Data2, int>> type_ind2data_inds;
    std::unordered_map<std::string, int> type_name2type_ind;
    std::vector<std::string> type_ind2name;
    

struct UnitData{
        float radius = 3.f;
        float mass = 1.f;
        float r_vision = 20*radius;
        float max_range = 17*radius;
        float max_vel = 0.1f;
        float rot_vel = 5.f;
        int weapon_type_ind = 0;
    };

    std::vector<UnitData> type_ind2unit_data;

    const std::string unit_types_file_name = "/home/smutekj/projects/RTSEngineWithOpenGL/UnitTypes.txt"; 

    int selected_unit_ind = 0;
    std::string new_unit_name = "new unit";
    
    std::unordered_map<Data2, float> new_unit_data2;
    UnitData new_unit_data;
    UnitData selected_unit_data;
    UnitInitializer* p_unit_maker = nullptr;
public:
    UnitMakerWindow(UnitInitializer &unit_initializer);

    virtual ~UnitMakerWindow();

    virtual void draw() override;

private:
    void changeUnitTypeDatum(std::string unit_type_name,
                             UnitMakerWindow::Data2 datum_type,
                             float datum_value);
    void readUnitTypesFile();
    void appendToUnitTypesFile();
};


class UI
{

    struct UIWindowData
    {
        std::unique_ptr<UIWindow> p_window;
        bool is_active = false;
        std::string name;
    };

    friend UIWindowType;

    sf::Vector2f mouse_coords_on_click_;

    std::unordered_map<UIWindowType, std::unique_ptr<UIWindow>> windows;
    // std::unordered_map<UIWindowType, >
    std::unordered_map<UIWindowType, bool> is_active;
    std::unordered_map<UIWindowType, std::string> names;
    std::unordered_map<UIWindowType, UIWindowData> window_data;

    bool show_demo_window = true;

public:
    UI(sf::RenderWindow &window, Game &game, VisionSystem &fow, DebugInfo &dbg);

    void showWindow();
    void draw(sf::RenderWindow &window);
};

#endif // BOIDS_UI_H
