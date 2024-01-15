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

void Game::updateTriangulation()
{
    p_map_grid->extractEdgesFromTilesV2(cdt);
    p_map_grid->addAllBuildingsToTriangulation(cdt);
    p_pathfinder_->update();
    p_map_grid->sawOffCorners();
    p_map_grid->extractVerticesForDrawing(cdt, p_pathfinder_->tri_ind2component_);
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

void Game::removeUnit(BoidInd u_ind)
{
}

void Game::removeUnit(Entity e)
{
    p_the_god_->removeEntity(e);
}

// void Game::removeUnit(int entity_ind) {
//     p_the_god_->removeEntity({0, entity_ind});
// }

void Game::addUnit(int player_ind, sf::Vector2f r, int unit_type_ind)
{

    typedef UnitCreatorSettings::Values Values;

    const auto &u_type = unit_types_.at(unit_type_ind);
    const auto radius = uc_settings_.values_[Values::RADIUS];
    const auto max_speed = 1.69f * uc_settings_.values_[Values::MAX_SPEED];
    const auto turn_rate = uc_settings_.values_[Values::TURNRATE];

    unit_creator_->initializeUnit(r, player_ind);
    //                world_.activate(u_ind,  {0, 0}, radius2);
}

void Game::createUnitType(const float max_speed, const float radius)
{

    UnitType u_type;
    u_type.max_speed = max_speed;
    u_type.radius = radius;
    unit_types_.push_back(u_type);
}

Game::Game(Triangulation &cdt, sf::Vector2i n_cells, sf::Vector2f box_size)
    : box_size(box_size), cdt(cdt)
{

    mouse_selection.setFillColor({0, 0, 255, 69});
    mouse_selection.setOutlineThickness(5);

    const sf::Vector2f cell_size = {box_size.x / asFloat(n_cells).x, box_size.y / asFloat(n_cells).y};

    p_map_grid = std::make_unique<MapGrid>(n_cells, box_size, cell_size);
    p_map_grid->getEdges()->vertices_ = cdt.vertices_;

    p_the_god_ = std::make_unique<ECSystem>(*p_map_grid->getEdges());
    unit_creator_ = std::make_shared<UnitInitializer>(*p_the_god_);

    p_pathfinder_ = std::make_shared<PathFinder2>(&cdt);

    auto &ss = p_the_god_->getSystem<SeekSystem>(ComponentID::PATHFINDING);
    ss.p_pathfinder_ = p_pathfinder_.get();
    ss.p_cdt_ = &cdt;

    createUnitType(0.3, 1 * RHARD);
}
int frame = 0;

void Game::moveView(sf::RenderWindow &window)
{
    auto view = window.getView();
    auto vp1 = window.getViewport(view);
    auto vp2 = view.getViewport();
    const auto view_size = view.getSize(); // * (view.getViewport().width);
    const auto left_border = window.getViewport(view).left;
    const auto right_border = left_border + window.getViewport(view).width;
    const auto top_border = window.getViewport(view).top;
    const auto bottom_border = top_border + window.getViewport(view).height;

    const auto &mouse_pos = sf::Mouse::getPosition();
    const auto &mouse_coords = window.mapPixelToCoords(sf::Mouse::getPosition());

    bool mouse_in_left_border = mouse_pos.x < 0 + 5;
    bool mouse_in_top_border = mouse_pos.y < top_border + 5;
    bool mouse_in_right_border = mouse_pos.x > right_border - 5;
    bool mouse_in_bottom_border = mouse_pos.y > bottom_border - 5;

    bool right_arrow_pushed = sf::Keyboard::isKeyPressed(sf::Keyboard::Right);
    bool left_arrow_pushed = sf::Keyboard::isKeyPressed(sf::Keyboard::Left);
    bool up_arrow_pushed = sf::Keyboard::isKeyPressed(sf::Keyboard::Up);
    bool down_arrow_pushed = sf::Keyboard::isKeyPressed(sf::Keyboard::Down);
    if ((mouse_in_right_border || right_arrow_pushed) && mouse_coords.x < Geometry::BOX[0] * 1.05f)
    {
        view.move(view_size.x / 100.f, 0);
    }
    if ((mouse_in_left_border || left_arrow_pushed) && mouse_coords.x > -Geometry::BOX[0] * 0.05f)
    {
        view.move(-view_size.x / 100.f, 0);
    }
    if ((mouse_in_bottom_border || down_arrow_pushed) && mouse_coords.y < Geometry::BOX[1] * 1.05f)
    {
        view.move(0, view_size.y / 100.f);
    }
    if ((mouse_in_top_border || up_arrow_pushed) && mouse_coords.y > -Geometry::BOX[1] * 0.05f)
    {
        view.move(0, -view_size.y / 100.f);
    }
    window.setView(view);
}

void Game::parseEvents(sf::RenderWindow &window, UI &ui)
{

    sf::Event event;

    auto mouse_position = window.mapPixelToCoords(sf::Mouse::getPosition(window));

    while (window.pollEvent(event))
    {

        if (event.type == sf::Event::Closed)
        {
            window.close();
        }
        if (event.type == sf::Event::KeyPressed)
        {
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
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
            else
            {
                ui.onKeyPress(event.key.code);
            }
        }

        if (event.type == sf::Event::KeyReleased)
        {
            if (last_pressed_was_space)
            {
                game_is_stopped_ = false;
                last_pressed_was_space = false;
            }
            else
            {
                ui.onKeyRelease(window);
            }
        }
        if (event.type == sf::Event::MouseButtonPressed)
        {
            ui.onClick(window);

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
            else if (event.mouseButton.button == sf::Mouse::Right)
            {
                // if (!clicked) {
                click_position = window.mapPixelToCoords(sf::Mouse::getPosition(window));
                // };
                p_the_god_->setMoveState(MoveState::MOVING, selection);
                p_the_god_->getSystem<SeekSystem>(ComponentID::PATHFINDING).issuePaths(selection, click_position);
            }
        }
        else if (event.type == sf::Event::MouseMoved)
        {
            ui.onMouseHold(window);
            if (selection_pending)
            {
                end_position = window.mapPixelToCoords(sf::Mouse::getPosition(window));
            }
        }
        else if (event.type == sf::Event::MouseButtonReleased)
        {
            ui.onRelease(window);

            view_is_moving = false;
            if (event.mouseButton.button == sf::Mouse::Left)
            {
                end_position = window.mapPixelToCoords(sf::Mouse::getPosition(window));
                selection_pending = false;
                const auto previous_selection = selection;
                selectInRectangle(*p_the_god_, start_position, end_position, selection, selected_player);
            }
        }
        if (event.type == sf::Event::MouseWheelMoved)
        {
            auto view2 = window.getView();
            if (event.mouseWheel.delta > 0)
            {
                view2.zoom(0.9f);
            }
            else
            {
                view2.zoom(1. / 0.9f);
            }
            window.setView(view2);
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
                p_map_grid->buildBuilding(mouse_position, {8, 8}, cdt);
                bulding = false;
                sf::Clock time;
                p_pathfinder_->update();
                buildings.load("", p_map_grid->buildings_);
                // std::cout << "pathfinder updating took: " << time.getElapsedTime().asMilliseconds() << " ms\n";
            }
        }
        if (event.type == sf::Event::KeyPressed)
        {
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::C))
            {
                const auto clicked_grid_point = p_map_grid->cellCoords(p_map_grid->coordToCell(mouse_position));
                p_map_grid->createRandomBlob(clicked_grid_point);
                updateTriangulation();
            }
        }
    }
}

//! \brief parse events and normal input
//! \note  right now this is just a placeholder code until I make a nice OOP solution with bindings and stuff
void Game::parseInput(sf::RenderWindow &window, UI &ui)
{

    // bool clicked = false;
    parseEvents(window, ui);
    moveView(window);

    if (sf::Mouse::isButtonPressed(sf::Mouse::Right) and sf::Keyboard::isKeyPressed(sf::Keyboard::LControl))
    {
        auto mouse_position = window.mapPixelToCoords(sf::Mouse::getPosition(window));

        auto nearest_neigbhours = p_the_god_->getSystem<PhysicsSystem>(ComponentID::PHYSICS).p_ns_->getNeighboursIndsFull(click_position, 5 * RHARD);

        float radius = RHARD;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::A))
        {
            radius = 3 * RHARD;
        }
        if (nearest_neigbhours.empty())
        {
            //                UnitInd u_ind = {0, static_cast<unsigned  char> (selected_player)} ;
            addUnit(selected_player, static_cast<sf::Vector2f>(mouse_position), 0);
        }
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::V))
    {
        drawing = true;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Escape))
    {
        drawing = false;
        bulding = false;
        selected_building_index = -1;
        selection.clear();
    }
    if (drawing)
    {
        auto mouse_position = window.mapPixelToCoords(sf::Mouse::getPosition(window));
        const sf::Vector2i building_size = {100, 100};
        auto lower_left_cell_coord = p_map_grid->drawProposedBuilding(window, mouse_position, building_size);

        if (sf::Mouse::isButtonPressed(sf::Mouse::Right))
        {
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) and wall_clock.getElapsedTime().asSeconds() > 0.4)
            {
                wall_clock.restart();
                p_map_grid->generateRandomWalls(mouse_position, {3000, 400}, 200);
                p_map_grid->updateBoundaryTypesLocally(lower_left_cell_coord, {3000, 400});
            }
            else
            {
                p_map_grid->buildWall(mouse_position, building_size);
                p_map_grid->updateBoundaryTypesLocally(lower_left_cell_coord, building_size);
                if (p_map_grid->walls_drawable_.size() > 10000)
                {
                    updateTriangulation();
                    p_map_grid->walls_drawable_.clear();
                }
            }
        }
        // if (event.type == sf::Event::MouseButtonReleased) {
        //     p_map_grid->updateBoundaryTypes2();
        //     p_map_grid->sawOffCorners();
        //     drawing = false;
        // }
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::B))
    {
        bulding = true;
        building_clock.restart();
    }
    if (bulding and sf::Keyboard::isKeyPressed(sf::Keyboard::Escape) or
        (bulding and sf::Keyboard::isKeyPressed(sf::Keyboard::B) and
         building_clock.getElapsedTime().asSeconds() > 0.5))
    {
        bulding = false;
    }

    //! generate random buildings (useful for testing stuff)
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) and sf::Keyboard::isKeyPressed(sf::Keyboard::B) and
        building_clock2.restart().asSeconds() > 0.2)
    {
        for (int i = 0; i < 1 * 200; ++i)
        {
            auto [x, y] = randPoint(0, Geometry::BOX[0], 0, Geometry::BOX[1]);
            auto lower_left_cell_coord = p_map_grid->drawProposedBuilding(window, {x, y}, {4, 4});
            p_map_grid->buildBuilding({x, y}, {8, 8}, cdt);
        }
        buildings.load("", p_map_grid->buildings_);
        p_pathfinder_->update();
        if (!cdt.triangulationIsConsistent())
        {
            throw std::runtime_error("triangulation not consistent!");
        }
        bulding = false;
    }

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) and sf::Keyboard::isKeyPressed(sf::Keyboard::T) and
        building_clock.restart().asSeconds() > 0.3f)
    {
        updateTriangulation();
    }

    //! delete a building
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::D) and building_clock.restart().asSeconds() > 0.5)
    {
        //    cdt = Triangulation(*p_grid); //! this is just retarded; ...
        //    edges = Edges();
        //    cdt.createBoundaryAndSuperTriangle();
        //    p_pathfinder_->update();

        if (selected_building_index != -1)
        {
            p_map_grid->removeBuilding(selected_building_index, cdt);
            p_pathfinder_->update();
            selected_building_index = -1;
        }
    }

    //! player switching (useful for testing)
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Num1))
    {
        selected_player = 0;
        p_the_god_->getSystem<VisionSystem>(ComponentID::VISION).selected_player_ind = 0;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Num2))
    {
        selected_player = 1;
        p_the_god_->getSystem<VisionSystem>(ComponentID::VISION).selected_player_ind = 1;
    }

    //! removed the HOLDING state for now because I may not use it in the end :(
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::H) and !selection.empty())
    {
        for (int sel : selection)
        {
            // world_.move_states_[sel] = MoveState::HOLDING;
        }
        // bc_->addHoldingAgents(selection);
    }
    else if (sf::Keyboard::isKeyPressed(sf::Keyboard::S) and !selection.empty())
    {
        p_the_god_->setMoveState(MoveState::STANDING, selection);
    }

    mouse_selection.setSize(
        {static_cast<float>(end_position.x - start_position.x), static_cast<float>(end_position.y - start_position.y)});
}

void Game::update(const float dt, sf::RenderWindow &window)
{

    p_the_god_->update();

    auto &to_kill = p_the_god_->getSystem<HealthSystem>(ComponentID::HEALTH).dead_entity_inds_;
    for (auto entity_ind : to_kill)
    {
        removeUnit({0, entity_ind});
        sound_module_.playSound(Sounds::ID::Pop);
    }
    to_kill.clear();
}

void drawPath(sf::RenderWindow &window, const std::deque<sf::Vector2f> &path, const std::deque<Edgef> &portals)
{
    sf::RectangleShape line;
    line.setFillColor(sf::Color::Cyan);
    sf::CircleShape node;
    node.setRadius(1.f);
    node.setFillColor(sf::Color::Cyan);

    sf::RectangleShape portal_line;
    portal_line.setFillColor(sf::Color::Black);

    for (int i = 1; i < path.size(); ++i)
    {
        const auto dr = path.at(i) - path.at(i - 1);
        const auto dr_norm = norm(dr);
        const auto dr2 = portals[i].t;
        const auto angle = 180.f / (M_PIf)*std::acos(dot(dr / dr_norm, {0, 1})) * (2.f * (dr.x < 0.f) - 1.f);
        const auto angle2 = 180.f / (M_PIf)*std::acos(dot(dr2, {0, 1})) * (2.f * (dr2.x < 0.f) - 1.f);
        line.setPosition(path.at(i - 1));
        line.setSize({1, dr_norm});
        line.setRotation(angle);

        portal_line.setPosition(portals[i].from);
        portal_line.setSize({1, portals[i].l});
        portal_line.setRotation(angle2);

        node.setPosition(path.at(i) - sf::Vector2f{1.f, 1.f});

        window.draw(portal_line);
        window.draw(line);
        window.draw(node);
    }
}

void drawFunction(const std::vector<sf::Vector2f> &points, sf::RenderWindow &window)
{

    sf::RectangleShape line;
    line.setFillColor(sf::Color::Black);

    for (int i = 1; i < points.size(); ++i)
    {
        sf::Vector2f p2 = points[i];
        sf::Vector2f p1 = points[i - 1];
        p1.y *= -1;
        p2.y *= -1;
        const sf::Vector2f v = p2 - p1;
        if (norm2(v) != 0)
        {
            auto angle = 180.f / (M_PIf)*std::acos(dot(v / norm(v), {0, 1})) * (2.f * (v.x < 0.f) - 1.f);
            line.setSize({5.0, dist(p1, p2)});
            line.setRotation(angle);
            line.setPosition(p1);
            window.draw(line);
        }
    }
}

void Game::draw(sf::RenderWindow &window)
{
    // attack_system_->draw(window);
    p_map_grid->draw(window);
    // world_.draw(window, *bc_);
    if (selection_pending)
    {
        window.draw(mouse_selection);
    }

    p_the_god_->draw(window);

    buildings.draw(window);
    // p_fow_->draw(this->selected_player, window);
    drawPath(window, path, portals);
}
