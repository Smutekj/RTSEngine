#include "UI.h"

#include "Systems/VisionSystem.h"
#include "Systems/SeekSystem.h"
#include "Systems/PhysicsSystem.h"

#include "Game.h"
#include "UnitInitializer.h"

#include "Graphics/RenderWindow.hpp"
#include "Graphics/SceneLayer.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "Utils/DebugInfo.h"
#include "Utils/magic_enum.hpp"
#include "Utils/magic_enum_utility.hpp"

std::vector<std::string> separateLine(std::string line, char delimiter = ' ')
{
    std::vector<std::string> result;
    int start = 0;
    int end = 0;

    while ((start = line.find_first_not_of(' ', end)) != std::string::npos)
    {
        end = line.find(' ', start);
        result.push_back(line.substr(start, end - start));
    }
    return result;
}

UIWindow::UIWindow(std::string name) : name(name)
{
}

UI::UI(sf::RenderWindow &window, Game &game, VisionSystem &fow, DebugInfo &dbg)
{

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;  // Enable Gamepad Controls
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;         // IF using Docking Branch
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable; 

    // // // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window.handle, true); // Second param install_callback=true will install GLFW callbacks and chain to existing ones.
    ImGui_ImplOpenGL3_Init();

    auto &ps = game.p_the_god_->getSystem<PhysicsSystem>(ComponentID::PHYSICS);
    auto &ss = game.p_the_god_->getSystem<SeekSystem>(ComponentID::PATHFINDING);
    auto physics_window = std::make_unique<PhysicsWindow>(ps);
    auto pf_window = std::make_unique<PathfindingWindow>(ss, dbg);
    auto debuginfo_window = std::make_unique<DeubgInfoWindow>(dbg);
    auto shaders_window = std::make_unique<ShadersWindow>(game.building_scene);
    auto unit_window = std::make_unique<UnitMakerWindow>(*game.unit_creator_);

    windows[UIWindowType::PHYSICS] = std::move(physics_window);
    windows[UIWindowType::PATHFINDING] = std::move(pf_window);
    windows[UIWindowType::DEBUG] = std::move(debuginfo_window);
    windows[UIWindowType::SHADERS] = std::move(shaders_window);
    windows[UIWindowType::UNIT_MAKER] = std::move(unit_window);

    is_active[UIWindowType::PHYSICS] = true;
    is_active[UIWindowType::PATHFINDING] = true;
    is_active[UIWindowType::DEBUG] = true;
    is_active[UIWindowType::SHADERS] = true;
    is_active[UIWindowType::UNIT_MAKER] = true;

    names[UIWindowType::PHYSICS] = "Physics";
    names[UIWindowType::PATHFINDING] = "Pathfinding";
    names[UIWindowType::DEBUG] = "DebugInfo";
    names[UIWindowType::SHADERS] = "Shaders";
    names[UIWindowType::UNIT_MAKER] = "Units";
}

// void UI::update(){

// }

UIWindow::~UIWindow()
{
}

void UI::showWindow()
{
}

void UI::draw(sf::RenderWindow &window)
{

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::Begin("Control Panel"); // Create a window called "Hello, world!" and append into it.
    for (auto &[window_type, p_window] : windows)
    {
        if (ImGui::Button(p_window->getName().c_str()))
            is_active[window_type] = !is_active[window_type];
    }

    ImGui::End();

    for (auto &[window_type, p_window] : windows)
    {
        if (is_active[window_type]){
            p_window->draw();
        }
    }
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        
}

PhysicsWindow::PhysicsWindow(PhysicsSystem &ps) : force_multipliers(ps.force_multipliers), UIWindow("Physics")
{
    controled_data.at(Data::MAX_SPEED) = &ps.max_speed;
    force_multipliers[PhysicsSystem::Multiplier::ALIGN] = 1.0f;
    force_multipliers[PhysicsSystem::Multiplier::SEEK] = 1.0f;
    force_multipliers[PhysicsSystem::Multiplier::SCATTER] = 0.1f;
    force_multipliers[PhysicsSystem::Multiplier::DECAY] = 1.0f;
    force_multipliers[PhysicsSystem::Multiplier::SEEK] = 1.0f;
    force_multipliers[PhysicsSystem::Multiplier::REPULSE] = 1.f;
    force_multipliers[PhysicsSystem::Multiplier::PUSH] = 0.1f;
    force_multipliers[PhysicsSystem::Multiplier::VELOCITY] = 2.0;

    for (auto &[multiplier_type, value] : force_multipliers)
    {
        mulitplier2slider_min_max[multiplier_type] = {0.0f, 5.f};
    }
}

PhysicsWindow::~PhysicsWindow() {}

void PhysicsWindow::draw()
{

    ImGui::Begin(name.c_str());

    // if (ImGui::BeginListBox("Force Multipliers"))
    {
        for (auto &[multiplier_type, value] : force_multipliers)
        {
            auto multiplier_name = static_cast<std::string>(magic_enum::enum_name(multiplier_type));
            auto &min_value = mulitplier2slider_min_max[multiplier_type].first;
            auto &max_value = mulitplier2slider_min_max[multiplier_type].second;

            ImGui::PushItemWidth(ImGui::GetWindowWidth()*0.66);
            ImGui::SliderFloat(multiplier_name.c_str(), &value, min_value, max_value);
            
            ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.33f);
            ImGui::InputFloat(("min##" + multiplier_name).c_str(), &min_value);
            ImGui::SameLine();
            ImGui::InputFloat(("max##" + multiplier_name).c_str(), &max_value);

        }
        // ImGui::EndListBox();
    }

    ImGui::End();
}

DeubgInfoWindow::DeubgInfoWindow(DebugInfo &di) : UIWindow("Debug info")
{
    auto &settings = di.getSettings();
    controled_data.at(Data::SHOW_GRID) = &settings.options_.at(DebugInfoSettings2::Options::DRAW_BUILDING_GRID);
    controled_data.at(Data::SHOW_TRI_INDS) = &settings.options_.at(DebugInfoSettings2::Options::DRAW_TRI_INDS);
    controled_data.at(Data::SHOW_TRIANGLES) = &settings.options_.at(DebugInfoSettings2::Options::DRAW_TRIANGLES);
}

DeubgInfoWindow::~DeubgInfoWindow() {}

void DeubgInfoWindow::draw()
{

    ImGui::Begin(name.c_str());               // Create a window called "Hello, world!" and append into it.
    ImGui::Text("This is some useful text."); // Display some text (you can use a format strings too)

    if (ImGui::Button("Show Grid"))
    {
        bool &value = *static_cast<bool *>(controled_data.at(Data::SHOW_GRID));
        value = !(value);
    }
    if (ImGui::Button("Show Triangle indices"))
    {
        bool &value = *static_cast<bool *>(controled_data.at(Data::SHOW_TRI_INDS));
        value = !(value);
    }
    if (ImGui::Button("Show triangles"))
    {
        bool &value = *static_cast<bool *>(controled_data.at(Data::SHOW_TRIANGLES));
        value = !(value);
    }

    ImGui::End();
}

UnitMakerWindow::UnitMakerWindow(UnitInitializer &unit_init) : p_unit_maker(&unit_init), UIWindow("Unit maker")
{
    // controled_data.at(Data2::RADIUS) = &new_unit_data.radius;
    // controled_data.at(Data2::MAX_VEL) = &new_unit_data.max_vel;
    // controled_data.at(Data2::WEAPON_RANGE) = &new_unit_data.max_range;
    // controled_data.at(Data2::VISION_RADIUS) = &new_unit_data.r_vision;
    // controled_data.at(Data2::ROTATION_VEL) = &new_unit_data.rot_vel;
    // controled_data.at(Data2::MASS) = &new_unit_data.mass;

    readUnitTypesFile();

    for (auto [unit_type_name, type_ind] : type_name2type_ind)
    {

        const auto &values = type_ind2data_values.at(type_ind);
        new_unit_data.radius = values.at(Data2::RADIUS);
        new_unit_data.mass = values.at(Data2::MASS);
        new_unit_data.max_vel = values.at(Data2::MAX_VEL);
        new_unit_data.rot_vel = values.at(Data2::ROTATION_VEL);
        new_unit_data.r_vision = values.at(Data2::VISION_RADIUS);
        new_unit_data.max_range = values.at(Data2::WEAPON_RANGE);
        p_unit_maker->registerUnitType(new_unit_data.radius, new_unit_data.mass, new_unit_data.weapon_type_ind,
                                       new_unit_data.r_vision, new_unit_data.max_vel, new_unit_data.rot_vel, 0,
                                       30.f, new_unit_name);
    }
}

void UnitMakerWindow::readUnitTypesFile()
{

    const auto filename = unit_types_file_name;
    std::ifstream file(filename);

    std::string line;

    int data_type_ind = 0;
    while (std::getline(file, line))
    { //! iterate over [UnitType] blocks
        if (line != "[UnitType]")
        {
            throw std::runtime_error("congrats, you fucked up file format yet again, lol");
        }
        std::getline(file, line);
        auto words_on_line = separateLine(line);

        type_ind2data_values.resize(data_type_ind + 1);
        type_ind2data_inds.resize(data_type_ind + 1);
        type_ind2name.resize(data_type_ind + 1);

        auto &data_values = type_ind2data_values.at(data_type_ind);
        std::string unit_type_name = words_on_line.at(1);
        type_ind2name.at(data_type_ind) = unit_type_name;
        type_name2type_ind[unit_type_name] = data_type_ind;

        while (std::getline(file, line))
        {
            if (line == "END")
            {
                break;
            }
            auto words_on_line = separateLine(line);
            assert(words_on_line.size() == 2);

            const auto datum_name = words_on_line.at(0);
            const auto datum_value = std::stof(words_on_line.at(1));

            const auto datum_type = magic_enum::enum_cast<Data2>(datum_name).value();
            data_values[datum_type] = datum_value;
        }
        data_type_ind++;
    }
    file.close();
}

void UnitMakerWindow::appendToUnitTypesFile()
{
    std::ofstream file;
    file.open(unit_types_file_name, std::ios_base::app); // append instead of overwrite
    file << "[UnitType]\n";
    file << "name " + new_unit_name + "\n";

    for (int data_ind = 0; data_ind < unit_data_count; data_ind++)
    {

        const auto data_type = magic_enum::enum_value<Data2>(data_ind);
        const auto data_name = std::string(magic_enum::enum_name(data_type));

        file << data_name + " " + std::to_string(new_unit_data2[data_type]) + "\n";
    }
    file << "END\n";
    file.close();
}

void UnitMakerWindow::changeUnitTypeDatum(std::string unit_type_name, UnitMakerWindow::Data2 datum_type, float datum_value)
{

    const auto filename = unit_types_file_name;
    const auto tmp_filename = filename + ".tmp";
    const auto backup_filename = filename + ".backup";
    std::ifstream file(filename);
    std::ofstream backup_file(backup_filename);
    std::ofstream new_file(tmp_filename);
    std::string line;

    const auto datum_name = std::string(magic_enum::enum_name(datum_type));

    //! backup the file
    while (std::getline(file, line))
    {
        backup_file << line << "\n";
    }
    backup_file.close();
    file.close();

    file.open(filename);
    bool found_right_unit = false;
    //! read the file and write changed version to tmp_file
    while (std::getline(file, line))
    {
        auto words_on_line = separateLine(line);
        if (line.empty())
        {
            continue;
        }
        if (words_on_line.at(0) == "name" && line.substr(5, line.size()) == new_unit_name)
        {
            found_right_unit = true;
        }
        if (found_right_unit && words_on_line.at(0) == datum_name)
        {
            line = datum_name + " " + std::to_string(datum_value);
            found_right_unit = false;
        }
        new_file << line << "\n";
    }
    new_file.close();
    file.close();

    std::remove(filename.c_str());
    std::rename(tmp_filename.c_str(), filename.c_str());
}

void UnitMakerWindow::draw()
{
    ImGui::Begin("Unit Maker");
    auto &unit_data = p_unit_maker->unit_type2data_;
    if (ImGui::TreeNode("Unit Maker"))
    {
        if (ImGui::BeginListBox("Unit Types"))
        {
            for (auto &[unit_type_name, unit_type_ind] : p_unit_maker->unit_name2type_ind)
            {
                auto &type_data = unit_data.at(unit_type_ind);
                const bool is_selected = (selected_unit_ind == unit_type_ind);
                if (ImGui::Selectable(unit_type_name.c_str(), is_selected))
                {
                    selected_unit_ind = unit_type_ind;
                }

                // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
                if (is_selected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndListBox();
        }
        ImGui::TreePop();
    }
    if (ImGui::TreeNode("New Unit Properties"))
    {
        if (ImGui::BeginListBox("Unit Data"))
        {

            constexpr std::size_t unit_data_count = magic_enum::enum_count<Data2>();
            for (int data_ind = 0; data_ind < unit_data_count; ++data_ind)
            {
                auto data_type = magic_enum::enum_value<Data2>(data_ind);
                const auto data_name = std::string(magic_enum::enum_name(data_type));
                bool is_selected = false;
                if (ImGui::InputFloat(data_name.c_str(), &(new_unit_data2[data_type])))
                {
                    changeUnitTypeDatum(new_unit_name, data_type, new_unit_data2[data_type]);
                }
            }
            ImGui::EndListBox();
        }
        ImGui::TreePop();
    }
    // if (ImGui::TreeNode("Selected Unit Properties"))
    // {
    //     if (ImGui::BeginListBox("Unit Data"))
    //     {
    //         constexpr std::size_t unit_data_count = magic_enum::enum_count<Data2>();
    //         const auto& selected_data = p_unit_maker->unit_type2data_.at(selected_unit_ind);
    //         for(int data_ind = 0; data_ind < unit_data_count; ++data_ind){
    //             const auto data_name = std::string(magic_enum::enum_name(magic_enum::enum_value<Data2>(data_ind)));
    //             bool is_selected = false;
    //             ImGui::Text(data_name.c_str(), selected_data.mass);
    //         }
    //         ImGui::EndListBox();
    //     }
    //     ImGui::TreePop();
    // }

    if (ImGui::Button("Register new unit"))
    {
        p_unit_maker->registerUnitType(new_unit_data.radius, new_unit_data.mass, new_unit_data.weapon_type_ind,
                                       new_unit_data.r_vision, new_unit_data.max_vel, new_unit_data.rot_vel, 0,
                                       30.f, new_unit_name);
        appendToUnitTypesFile();
    }

    ImGui::InputText("new unit name", new_unit_name.data(), 500);
    p_unit_maker->selected_unit_type_ind = selected_unit_ind;

    ImGui::End();
}

UnitMakerWindow::~UnitMakerWindow() {}

PathfindingWindow::PathfindingWindow(SeekSystem &ss, DebugInfo &dbg) : UIWindow("Pathfinding")
{
    controled_data.at(Data::SHOW_PATH) = &dbg.draw_path;
    dbg.p_pathfinder = ss.p_pathfinder_;
}

PathfindingWindow::~PathfindingWindow() {}

void PathfindingWindow::draw()
{
    {
        ImGui::Begin("Pathfinding!");             // Create a window called "Hello, world!" and append into it.
        ImGui::Text("This is some useful text."); // Display some text (you can use a format strings too)
        if (ImGui::Button("Show triangles"))
        {
            bool &value = *static_cast<bool *>(controled_data.at(Data::SHOW_PATH));
            value = !(value);
        }
        ImGui::End();
    }
}

ShadersWindow::ShadersWindow(BuildingLayer &bl) : UIWindow("Shaders")
{
    ShaderUIData d;
    d.p_program = &bl.id2shader.at(BuildingLayer::GraphicsID::BUILDING1);
    ColorData data;
    data.uniform_name = "base_color";
    d.colors.push_back(data);

    data.uniform_name = "center_color";
    d.colors.push_back(data);

    ValueData data_v;
    data_v.uniform_name = "wandering_radius";
    data_v.value = 0.2f;
    d.values.push_back(data_v);
    data_v.uniform_name = "scale";
    data_v.value = 10;
    d.values.push_back(data_v);

    shaders.push_back(d);

    d.colors.clear();
    d.values.clear();
    d.p_program = &bl.id2shader.at(BuildingLayer::GraphicsID::BUILDING2);
    data.uniform_name = "base_color";
    d.colors.push_back(data);
    data.uniform_name = "spots_color";
    d.colors.push_back(data);
    shaders.push_back(d);
}

ShadersWindow::~ShadersWindow() {}

static void drawStuff(glm::vec4 &color)
{
    // ImGui::SeparatorText("Color picker");
    static bool alpha = true;
    static bool alpha_bar = true;
    static bool side_preview = true;
    static bool ref_color = true;
    static ImVec4 ref_color_v(1.0f, 0.0f, 1.0f, 0.5f);
    static int display_mode = 0;
    static int picker_mode = 0;

    ImGui::Combo("Display Mode", &display_mode, "Auto/Current\0None\0RGB Only\0HSV Only\0Hex Only\0");
    ImGuiColorEditFlags flags;
    if (!alpha)
        flags |= ImGuiColorEditFlags_NoAlpha; // This is by default if you call ColorPicker3() instead of ColorPicker4()
    if (alpha_bar)
        flags |= ImGuiColorEditFlags_AlphaBar;
    if (!side_preview)
        flags |= ImGuiColorEditFlags_NoSidePreview;
    if (picker_mode == 1)
        flags |= ImGuiColorEditFlags_PickerHueBar;
    if (picker_mode == 2)
        flags |= ImGuiColorEditFlags_PickerHueWheel;
    if (display_mode == 1)
        flags |= ImGuiColorEditFlags_NoInputs; // Disable all RGB/HSV/Hex displays
    if (display_mode == 2)
        flags |= ImGuiColorEditFlags_DisplayRGB; // Override display mode
    if (display_mode == 3)
        flags |= ImGuiColorEditFlags_DisplayHSV;
    if (display_mode == 4)
        flags |= ImGuiColorEditFlags_DisplayHex;
    ImGui::ColorPicker4("MyColor##4", (float *)&color, flags, ref_color ? &ref_color_v.x : NULL);
}

void setShaderVariableValue(Shader &shader, std::string var_name, glm::vec4 color)
{

    const auto filename = shader.fragment_path;
    const auto tmp_filename = filename + ".tmp";
    std::ifstream file(filename);
    std::ofstream new_file(tmp_filename);

    std::string line;
    while (std::getline(file, line))
    {
        std::stringstream iss(line);

        auto words_on_line = separateLine(line);
        if (words_on_line.size() >= 3 && words_on_line.at(0) == "const")
        {
            if (words_on_line.at(2) == var_name)
            {

                std::string new_value = "vec4(" + std::to_string(color.x) + "," +
                                        std::to_string(color.y) + "," +
                                        std::to_string(color.z) + "," +
                                        std::to_string(color.w) + ");";

                words_on_line.at(4) = new_value;

                line = "";
                for (const auto &word : words_on_line)
                {
                    line += word;
                    line += " ";
                }
            }
        }

        new_file << line << "\n";
    }
    new_file.close();
    file.close();

    std::remove(filename.c_str());
    std::rename(tmp_filename.c_str(), filename.c_str());
    shader.recompile();
}

void setShaderVariableValue(Shader &shader, std::string var_name, float value)
{

    const auto filename = shader.fragment_path;
    const auto tmp_filename = filename + ".tmp";
    std::ifstream file(filename);
    std::ofstream new_file(tmp_filename);

    std::string line;
    while (std::getline(file, line))
    {
        std::stringstream iss(line);

        auto words_on_line = separateLine(line);
        if (words_on_line.size() >= 3 && words_on_line.at(0) == "const")
        {
            if (words_on_line.at(2) == var_name)
            {

                std::string new_value = std::to_string(value);

                words_on_line.at(4) = new_value + ";";

                line = "";
                for (const auto &word : words_on_line)
                {
                    line += word;
                    line += " ";
                }
            }
        }

        new_file << line << "\n";
    }
    new_file.close();
    file.close();

    std::remove(filename.c_str());
    std::rename(tmp_filename.c_str(), filename.c_str());
    shader.recompile();
}

void ShadersWindow::draw()
{
    {
        ImGui::Begin("Shader!");

        static int chosen_shader_ind = 0; // Here we store our selection data as an index.
        static int chosen_field_ind = 0;  // Here we store our selection data as an index.
        if (ImGui::TreeNode("Shaders"))
        {
            if (ImGui::BeginListBox("Shaders"))
            {
                for (int n = 0; n < shaders.size(); n++)
                {
                    const bool is_selected = (chosen_shader_ind == n);
                    if (ImGui::Selectable(shaders[n].p_program->shader_name.c_str(), is_selected))
                    {
                        chosen_shader_ind = n;
                        chosen_field_ind = 0;
                    }

                    // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
                    if (is_selected)
                        ImGui::SetItemDefaultFocus();
                }
                ImGui::EndListBox();
            }
            ImGui::TreePop();
        }

        auto &shader_data = shaders.at(chosen_shader_ind);

        if (ImGui::TreeNode("Field Picker"))
        {
            if (ImGui::BeginListBox("Color Fields"))
            {
                for (int n = 0; n < shader_data.colors.size(); n++)
                {
                    const bool is_selected = (chosen_field_ind == n);
                    if (ImGui::Selectable(shader_data.colors.at(n).uniform_name.c_str(), is_selected))
                        chosen_field_ind = n;
                    if (is_selected)
                        ImGui::SetItemDefaultFocus();
                }
                ImGui::EndListBox();
            }

            for (int n = 0; n < shader_data.values.size(); n++)
            {
                const bool is_selected = (chosen_field_ind == n);
                if (ImGui::InputFloat(shader_data.values.at(n).uniform_name.c_str(), &shader_data.values.at(n).value))
                    setShaderVariableValue(*shader_data.p_program,
                                           shader_data.values.at(n).uniform_name.c_str(),
                                           shader_data.values.at(n).value);
            }

            ImGui::TreePop();
        }

        drawStuff(shader_data.colors.at(chosen_field_ind).value);
        setShaderVariableValue(*shader_data.p_program,
                               shader_data.colors.at(chosen_field_ind).uniform_name.c_str(),
                               shader_data.colors.at(chosen_field_ind).value);

        ImGui::End();
    }
}