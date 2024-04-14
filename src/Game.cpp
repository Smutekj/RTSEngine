#include <omp.h>
#include <iostream>
#include <fstream>
#include <unordered_set>

#include "ECS.h"
#include "Game.h"
#include "MapGrid.h"
#include "UI.h"
#include "Triangulation.h"
#include "UnitInitializer.h"
#include "SoundModule.h"

#include "Systems/SeekSystem.h"
#include "Systems/PhysicsSystem.h"
#include "Systems/HealthSystem.h"
#include "Systems/AttackSystem.h"
#include "Systems/GraphicsSystem.h"

#include "MouseKeyboard.hpp"

void Game::updateTriangulation()
{
    p_map_grid->sawOffCorners();
    p_map_grid->extractEdgesFromTilesV2(cdt);
    p_map_grid->addAllBuildingsToTriangulation(cdt);
    p_pathfinder_->update();
    p_map_grid->extractVerticesForDrawing(cdt, p_pathfinder_->tri_ind2component_);
    map_layer.updateFromMap();
    auto &vs = p_the_god_->getSystem<VisionSystem>(ComponentID::VISION);
    vs.updateWallFromMap(*p_map_grid);
}

void readMazeFile(std::string maze_file_name, const int n, const int m)
{
    std::ifstream file(maze_file_name);
    std::string text;
    // Use a while loop together with the getline() function to read the file line by line
    while (getline(file, text))
    {
    }
}

bool isKeyPressed(GLFWwindow *window, int key)
{
    int state = glfwGetKey(window, key);
    return (state == GLFW_PRESS);
}
void Game::removeUnit(BoidInd u_ind)
{
}

void Game::removeUnit(Entity e)
{
    selection_.selected_agents.removeEnt(e.ind);
    auto &gs = p_the_god_->getSystem<GraphicsSystem>(ComponentID::GRAPHICS);
    auto &g_comp = gs.getComponent<ComponentID::GRAPHICS, GraphicsComponent>(e);
    // unit_scene.destroyInstanceOf(g_comp.graphics_ind, g_comp.instance_ind);
    p_the_god_->removeEntity(e);
}

// void Game::removeUnit(int entity_ind) {
//     p_the_god_->removeEntity({0, entity_ind});
// }

void Game::addUnit(int player_ind, sf::Vector2f r, int unit_type_ind)
{
    unit_creator_->initializeUnit(r, player_ind);
}

void Game::createUnitType(const float max_speed, const float radius)
{

    UnitType u_type;
    u_type.max_speed = max_speed;
    u_type.radius = radius;
    unit_types_.push_back(u_type);
}

Game::Game(Triangulation &cdt, sf::Vector2i n_cells, sf::Vector2f box_size)
    : box_size(box_size), cdt(cdt), building_scene(building_manager), building_manager()
{


    mouse_selection.setFillColor({0, 0, 255, 69});
    mouse_selection.setOutlineThickness(5);

    const sf::Vector2f cell_size = {box_size.x / asFloat(n_cells).x, box_size.y / asFloat(n_cells).y};

    p_map_grid = std::make_unique<MapGrid>(n_cells, box_size, cell_size);
    p_map_grid->getEdges()->vertices_ = cdt.vertices_;
    p_map_grid->p_building_manager = &building_manager;
    building_scene.p_building_manager = &building_manager;

    map_layer.p_map_grid = p_map_grid.get();

    p_the_god_ = std::make_unique<ECSystem>(*p_map_grid->getEdges());
    unit_creator_ = std::make_shared<UnitInitializer>(*p_the_god_);

    p_pathfinder_ = std::make_shared<PathFinder2>(&cdt);

    auto &ss = p_the_god_->getSystem<SeekSystem>(ComponentID::PATHFINDING);
    ss.p_pathfinder_ = p_pathfinder_.get();
    ss.p_cdt_ = &cdt;

    createUnitType(0.3, 1 * RHARD);

    auto &gs = p_the_god_->getSystem<GraphicsSystem>(ComponentID::GRAPHICS);
    gs.p_graphics_layer = &unit_scene;

    auto &vs = p_the_god_->getSystem<VisionSystem>(ComponentID::VISION);
    vision_layer.p_vs = &vs;

    // makeOrbitingSquare4(scene);
}
int frame = 0;

void Game::moveView(sf::RenderWindow &window)
{
    auto &view = window.view;
    auto vp1 = window.getViewport(view);
    // auto vp2 = view.getViewport();
    const sf::Vector2f view_size = {view.width, view.height}; // * (view.getViewport().width);
    const auto left_border = window.getViewport(view).left;
    const auto right_border = left_border + window.getViewport(view).width;
    const auto top_border = window.getViewport(view).top;
    const auto bottom_border = top_border + window.getViewport(view).height;

    const auto &mouse_pos = sf::Mouse::getPosition(window);
    const auto &mouse_coords = window.mapPixelToCoords(sf::Mouse::getPosition(window));

    bool mouse_in_left_border = mouse_pos.x < 0 + 5;
    bool mouse_in_top_border = mouse_pos.y < top_border + 5;
    bool mouse_in_right_border = mouse_pos.x > right_border - 5;
    bool mouse_in_bottom_border = mouse_pos.y > bottom_border - 5;

    bool right_arrow_pushed = isKeyPressed(window.handle, GLFW_KEY_RIGHT);
    bool left_arrow_pushed = isKeyPressed(window.handle, GLFW_KEY_LEFT);
    bool up_arrow_pushed = isKeyPressed(window.handle, GLFW_KEY_UP);
    bool down_arrow_pushed = isKeyPressed(window.handle, GLFW_KEY_DOWN);
    if ((mouse_in_right_border || right_arrow_pushed) && mouse_coords.x < Geometry::BOX[0] * 1.05f)
    {
        view.move(-view_size.x / 100.f, 0);
    }
    if ((mouse_in_left_border || left_arrow_pushed) && mouse_coords.x > -Geometry::BOX[0] * 0.05f)
    {
        view.move(view_size.x / 100.f, 0);
    }
    if ((mouse_in_bottom_border || down_arrow_pushed) && mouse_coords.y < Geometry::BOX[1] * 1.05f)
    {
        view.move(0, -view_size.x / 100.f);
    }
    if ((mouse_in_top_border || up_arrow_pushed) && mouse_coords.y > -Geometry::BOX[1] * 0.05f)
    {
        view.move(0, view_size.x / 100.f);
    }
    // window.setView(view);
}

void Game::parseEvents(sf::RenderWindow &window, UI &ui)
{

    sf::Event event;
    auto mouse_position = window.mapPixelToCoords(sf::Mouse::getPosition(window));

    while (window.pollEvent(event))
    {

        if (event.type == sf::Event::KeyPressed)
        {
            if (isKeyPressed(window.handle, GLFW_KEY_SPACE))
            {
                if (game_is_stopped_)
                {
                    last_pressed_was_space = true;
                }
                else
                {
                    game_is_stopped_ = true;
                }
            }
            if (isKeyPressed(window.handle, GLFW_KEY_DELETE))
            {
                auto &to_kill = p_the_god_->getSystem<HealthSystem>(ComponentID::HEALTH).dead_entity_inds_;
                to_kill.insert(selection_.getSelectedInds().begin(), selection_.getSelectedInds().end());
            }
        }
        if (event.type == sf::Event::KeyReleased)
        {
            if (last_pressed_was_space)
            {
                game_is_stopped_ = false;
                last_pressed_was_space = false;
            }
        }
        if (event.type == sf::Event::MouseButtonPressed)
        {
            // ui.onClick(window);

            if (event.mouseButton.button == sf::Mouse::Left)
            {
                const auto mouse_coords = window.mapPixelToCoords(sf::Mouse::getPosition(window));
                const auto map_cell_index = p_map_grid->coordToCell(mouse_coords);
                if (!selection_pending)
                {
                    start_position = mouse_coords;
                    mouse_selection.setPosition(start_position);

                    end_position = start_position;
                    selection_pending = true;
                }
                if (p_map_grid->cell2building_ind_.at(map_cell_index) != -1)
                {
                    selected_building_index = p_map_grid->cell2building_ind_.at(map_cell_index);
                }
            }
            else if (event.mouseButton.button == sf::Mouse::Middle)
            {
                view_is_moving = true;
                start_view_move_position = window.mapPixelToCoords(sf::Mouse::getPosition());
            }
            else if (event.mouseButton.button == GLFW_MOUSE_BUTTON_RIGHT)
            {
                click_position = window.mapPixelToCoords(sf::Mouse::getPosition(window));

                if (isKeyPressed(window.handle, GLFW_KEY_W))
                {
                    auto &tris = cdt.triangles_;
                    auto clicked_tri_ind = cdt.findTriangle(click_position, false);
                    auto &tri2comp = p_pathfinder_->tri_ind2component_;
                    auto clicked_component = tri2comp.at(clicked_tri_ind);
                    for (int tri_ind = 0; tri_ind < tris.size(); ++tri_ind)
                    {
                        if (clicked_component != 0 && clicked_component == tri2comp.at(tri_ind))
                        {
                            tris.at(tri_ind).type == TriangleType::YELLOW;
                        }
                    }
                }
                p_the_god_->setMoveState(MoveState::MOVING, selection_.getSelectedInds());
                p_the_god_->getSystem<SeekSystem>(ComponentID::PATHFINDING).issuePaths2(selection_.getSelectedInds(), click_position);
            }
        }
        else if (event.type == sf::Event::MouseMoved)
        {
            // ui.onMouseHold(window);
            if (selection_pending)
            {
                end_position = window.mapPixelToCoords(sf::Mouse::getPosition(window));
            }
        }
        else if (event.type == sf::Event::MouseButtonReleased)
        {
            // ui.onRelease(window);
            if (selection_pending)
            {
                end_position = window.mapPixelToCoords(sf::Mouse::getPosition(window));
            }
            view_is_moving = false;
            if (event.mouseButton.button == sf::Mouse::Left)
            {
                end_position = window.mapPixelToCoords(sf::Mouse::getPosition(window));
                selection_pending = false;
                selection_.selectInRectangle(*p_the_god_, start_position, end_position, selected_player);
            }
            if (event.mouseButton.button == GLFW_MOUSE_BUTTON_RIGHT)
            {
                if (isKeyPressed(window.handle, GLFW_KEY_F))
                {
                    p_the_god_->getSystem<GraphicsSystem>(ComponentID::GRAPHICS).createSpawner(mouse_position);
                }
            }
        }
        if (event.type == sf::Event::MouseWheelMoved || isKeyPressed(window.handle, GLFW_KEY_Z))
        {
            auto &view2 = window.view;
            if (event.mouseWheelScroll.delta > 0 || isKeyPressed(window.handle, GLFW_KEY_LEFT_ALT))
            {
                view2.zoom(0.9f);
            }
            else
            {
                view2.zoom(1. / 0.9f);
            }
            // window.setView(view2);
        }

        if (bulding)
        {
            auto lower_left_cell_coord = p_map_grid->drawProposedBuilding(window, mouse_position, {8, 8});
            sf::Vector2f building_size = {2.f * p_map_grid->cell_size_.x, 2.f * p_map_grid->cell_size_.y};
            sf::Vector2f lower_left_coord = {lower_left_cell_coord.x * p_map_grid->cell_size_.x,
                                             lower_left_cell_coord.y * p_map_grid->cell_size_.y};
            sf::Vector2f center_coord = lower_left_coord + building_size / 2.f;

            if (event.type == sf::Event::MouseButtonReleased)
            {
                p_map_grid->buildBuilding(mouse_position, 1, cdt);
                auto &gs = p_the_god_->getSystem<GraphicsSystem>(ComponentID::GRAPHICS);
                p_map_grid->buildings_.back().graphics_id = rand() % 2;
                building_scene.addBuilding(p_map_grid->buildings_.back());

                bulding = false;
                p_pathfinder_->update();
                buildings.load("", p_map_grid->buildings_);
            }
        }
        if (event.type == sf::Event::KeyPressed)
        {
            if (isKeyPressed(window.handle, GLFW_KEY_C))
            {
                const auto clicked_grid_point = p_map_grid->cellCoords(p_map_grid->coordToCell(mouse_position));
                p_map_grid->createRandomBlob(clicked_grid_point);
                updateTriangulation();
            }
        }
    }
}

void mouse_button_callback(GLFWwindow *window, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS)
    {
    }
}

bool isButtonPressed(GLFWwindow *window, int button)
{
    int state = glfwGetMouseButton(window, button);
    return state == GLFW_PRESS;
}

//! \brief parse events and normal input
//! \note  right now this is just a placeholder code until I make a nice OOP solution with bindings and stuff
void Game::parseInput(sf::RenderWindow &window, UI &ui)
{
    // glfwSetKeyCallback(window.handle, key_callback);
    // bool clicked = false;
    parseEvents(window, ui);
    moveView(window);
    if (game_is_stopped_)
    {
        return;
    }

    if (isButtonPressed(window.handle, GLFW_MOUSE_BUTTON_RIGHT) and isKeyPressed(window.handle, GLFW_KEY_LEFT_CONTROL))
    {
        auto mouse_position = window.mapPixelToCoords(sf::Mouse::getPosition(window));

        auto nearest_neigbhours = p_the_god_->getSystem<PhysicsSystem>(ComponentID::PHYSICS).p_ns_->getNeighboursIndsFull(click_position, 5 * RHARD);

        float radius = RHARD;
        if (isKeyPressed(window.handle, GLFW_KEY_A))
        {
            radius = 3 * RHARD;
        }
        if (nearest_neigbhours.empty())
        {
            //                UnitInd u_ind = {0, static_cast<unsigned  char> (selected_player)} ;
            addUnit(selected_player, static_cast<sf::Vector2f>(mouse_position), 0);
        }
    }
    if (isKeyPressed(window.handle, GLFW_KEY_V))
    {
        drawing = true;
    }
    if (isKeyPressed(window.handle, GLFW_KEY_ESCAPE))
    {
        drawing = false;
        bulding = false;
        selected_building_index = -1;
        selection_.clear();
    }
    if (drawing)
    {
        auto mouse_position = window.mapPixelToCoords(sf::Mouse::getPosition(window));
        const sf::Vector2i building_size = {5, 5};
        auto lower_left_cell_coord = p_map_grid->drawProposedBuilding(window, mouse_position, building_size);

        if (isButtonPressed(window.handle, GLFW_MOUSE_BUTTON_RIGHT))
        {
            if (isKeyPressed(window.handle, GLFW_KEY_LEFT_SHIFT))
            {
                // wall_clock.restart();
                // p_map_grid->generateRandomWalls(mouse_position, {3000, 400}, 200);
                // p_map_grid->updateBoundaryTypesLocally(lower_left_cell_coord, {3000, 400});
            }
            else
            {
                p_map_grid->buildWall(mouse_position, building_size);
                p_map_grid->updateBoundaryTypesLocally(lower_left_cell_coord, building_size);
                // if (p_map_grid->walls_drawable_.size() > 10000)
                {
                    updateTriangulation();
                    // p_map_grid->walls_drawable_.clear();
                }
            }
        }
        // if (event.type == sf::Event::MouseButtonReleased) {
        //     p_map_grid->updateBoundaryTypes2();
        //     p_map_grid->sawOffCorners();
        //     drawing = false;
        // }
    }
    if (isKeyPressed(window.handle, GLFW_KEY_B))
    {
        bulding = true;
        // building_clock.restart();
    }
    if (bulding and isKeyPressed(window.handle, GLFW_KEY_ESCAPE) or
        (bulding and isKeyPressed(window.handle, GLFW_KEY_B)))
    {
        bulding = false;
    }

    //! generate random buildings (useful for testing stuff)
    if (isKeyPressed(window.handle, GLFW_KEY_LEFT_SHIFT) and isKeyPressed(window.handle, GLFW_KEY_B))
    {
        for (int i = 0; i < 1 * 600; ++i)
        {
            auto [x, y] = randPoint(0, Geometry::BOX[0], 0, Geometry::BOX[1]);
            // auto lower_left_cell_coord = p_map_grid->drawProposedBuilding(window, {x, y}, {4, 4});
            int building_id = rand() % 2;
            if (p_map_grid->buildBuilding({x, y}, building_id, cdt))
            {
                auto &gs = p_the_god_->getSystem<GraphicsSystem>(ComponentID::GRAPHICS);
                p_map_grid->buildings_.back().graphics_id = building_manager.getGID(building_id);
                building_scene.addBuilding(p_map_grid->buildings_.back());
            }
        }
        buildings.load("", p_map_grid->buildings_);
        p_pathfinder_->update();
        if (!cdt.triangulationIsConsistent())
        {
            throw std::runtime_error("triangulation not consistent!");
        }
        bulding = false;
    }

    if (isKeyPressed(window.handle, GLFW_KEY_LEFT_SHIFT) and isKeyPressed(window.handle, GLFW_KEY_T) )
    //  and building_clock.restart().asSeconds() > 0.3f)
    {
        updateTriangulation();
    }

    //! delete a building
    if (isKeyPressed(window.handle, GLFW_KEY_D))
    {
        //    cdt = Triangulation(*p_grid); //! this is just retarded; ...
        //    edges = Edges();
        //    cdt.createBoundaryAndSuperTriangle();
        //    p_pathfinder_->update();

        cdt.dumpToFile("CDT.dat");

        // if (selected_building_index != -1)
        // {
        //     p_map_grid->removeBuilding(selected_building_index, cdt);
        //     p_pathfinder_->update();
        //     selected_building_index = -1;
        // }
    }

    //! player switching (useful for testing)
    if (isKeyPressed(window.handle, GLFW_KEY_1))
    {
        selected_player = 0;
        p_the_god_->getSystem<VisionSystem>(ComponentID::VISION).selected_player_ind = 0;
    }
    if (isKeyPressed(window.handle, GLFW_KEY_2))
    {
        selected_player = 1;
        p_the_god_->getSystem<VisionSystem>(ComponentID::VISION).selected_player_ind = 1;
    }

    //! removed the HOLDING state for now because I may not use it in the end :(
    if (isKeyPressed(window.handle, GLFW_KEY_H) and !selection_.empty())
    {
        for (int sel : selection_.getSelectedInds())
        {
            // world_.move_states_[sel] = MoveState::HOLDING;
        }
        // bc_->addHoldingAgents(selection);
    }
    else if (isKeyPressed(window.handle, GLFW_KEY_S) and !selection_.empty())
    {
        p_the_god_->setMoveState(MoveState::STANDING, selection_.getSelectedInds());
    }

    mouse_selection.setSize(
        {static_cast<float>(end_position.x - start_position.x), static_cast<float>(end_position.y - start_position.y)});
}

void Game::update(const float dt, sf::RenderWindow &window)
{
    if (game_is_stopped_)
    {
        return;
    }

    auto &to_kill = p_the_god_->getSystem<HealthSystem>(ComponentID::HEALTH).dead_entity_inds_;
    for (auto entity_ind : to_kill)
    {
        removeUnit({0, entity_ind});
        p_pathfinder_->onUnitRemoval({0, entity_ind});
        // sound_module_.playSound(Sounds::ID::Pop);
    }
    to_kill.clear();

    p_the_god_->update();
}

void Game::draw(sf::RenderWindow &window)
{
    // attack_system_->draw(window);
    // p_map_grid->draw(window);
    // // world_.draw(window, *bc_);
    // if (selection_pending)
    // {
    //     window.draw(mouse_selection);
    // }

    // p_the_god_->draw(window);

    // buildings.draw(window);
    // p_fow_->draw(this->selected_player, window);
    // drawPath(window, path, portals);

    // unit_scene.update();
    unit_scene.initialize();
    unit_scene.draw(0, window.view);

    building_scene.update();
    building_scene.initialize();
    building_scene.draw(window);

    p_map_grid->draw(window);
    // map_layer.draw(window);

    // vision_layer.setup();
    // vision_layer.draw2(window);
}
