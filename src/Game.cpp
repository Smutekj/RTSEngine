#include <omp.h>
#include <iostream>
#include <fstream>

#include "Game.h"
#include "MapGrid.h"
#include "BoidControler.h"
#include "HealthSystem.h"
#include "AttackSystem.h"
#include "UI.h"
#include "PathFinder.h"
#include "BoidWorld.h"
#include "Triangulation.h"


void readMazeFile(std::string maze_file_name, const int n, const int m){
    std::ifstream file(maze_file_name);
    std::string text;
    // Use a while loop together with the getline() function to read the file line by line
    while (getline (file, text)) {
      
    }
}

void Game::removeUnit(BoidInd u_ind) {
    world_.deactivate(u_ind);
    health_system_->deactivate(u_ind);
    attack_system_->deactivate(u_ind);
    bc_->deactivate(u_ind);
}

void Game::addUnit(int player_ind, sf::Vector2f r, int unit_type_ind) {

    typedef UnitCreatorSettings::Values Values;

    if (world_.active_inds.size() >= N_MAX_NAVIGABLE_BOIDS) {
        return;
    }
    const auto new_boid_ind = world_.activate(player_ind, r, {0, 0});
    health_system_->activate(new_boid_ind, 5);
    attack_system_->activate(new_boid_ind, 0);

    const auto& u_type = unit_types_.at(unit_type_ind);
    const auto radius = uc_settings_.values_[Values::RADIUS];
    const auto max_speed = uc_settings_.values_[Values::MAX_SPEED];
    const auto turn_rate = uc_settings_.values_[Values::TURNRATE];
    bc_->activate(new_boid_ind, max_speed, radius, turn_rate);

    auto& unit_circle = world_.boid_inds2draw_data_[new_boid_ind].circle;
    unit_circle.setRadius(radius);
    if (player_ind == 0) {
        unit_circle.setFillColor(sf::Color::Green);
    } else {
        unit_circle.setFillColor(sf::Color::Red);
    }
}

void Game::createUnitType(const float max_speed, const float radius) {

    UnitType u_type;
    u_type.max_speed = max_speed;
    u_type.radius = radius;
    unit_types_.push_back(u_type);
}

Game::Game(PathFinder& pf, Triangulation& cdt, sf::Vector2i n_cells, sf::Vector2f box_size)
    : box_size(box_size)
    , cdt(cdt)
    , pf(pf) {
    health_system_ = std::make_unique<HealthSystem>();
    attack_system_ = std::make_unique<AttackSystem>(world_, health_system_.get());

    mouse_selection.setFillColor({0, 0, 255, 69});
    mouse_selection.setOutlineThickness(5);

    const sf::Vector2f cell_size = {box_size.x / asFloat(n_cells).x, box_size.y / asFloat(n_cells).y};
    p_fow_ = std::make_unique<FogOfWar>(static_cast<sf::Vector2i>(box_size), cell_size);

    searcher = std::make_unique<NeighbourSearcher>(box_size, RHARD * 10, world_);

    p_map_grid = std::make_unique<MapGrid>(n_cells, box_size, cell_size);
    p_map_grid->getEdges()->vertices_ = cdt.vertices_;
    bc_ = std::make_unique<BoidControler>(&world_, searcher.get(), p_map_grid->getEdges(), box_size);

    boid_ind2unit_type_.resize(N_MAX_NAVIGABLE_BOIDS, 0);
    unit_types_.resize(1);
    createUnitType(0.3, 3 * RHARD);
}
int frame = 0;
bool first_frame = true;

void Game::parseInput(sf::RenderWindow& window, UI& ui) {

    sf::Event event;
    bool clicked = false;

    if (false and ++frame > 10) {
        frame = 0;
        event.type = sf::Event::EventType::MouseButtonPressed;
        event.mouseButton.button = sf::Mouse::Right;
        sf::Vector2f rand_pos = {(rand() / static_cast<float>(RAND_MAX)) * box_size.x,
                                 (rand() / static_cast<float>(RAND_MAX)) * box_size.y};
        // sf::Mouse::setPosition(window.mapCoordsToPixel(rand_pos), window);
        click_position = rand_pos;
        selectInRectangle(world_, {0, 0}, box_size, selection, selected_player);
        clicked = true;
    }
    if (first_frame) {
        first_frame = false;
    }
    while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
            window.close();
        }
        if (event.type == sf::Event::KeyPressed) {
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Space)) {
                if (game_is_stopped_) {
                    last_pressed_was_space = true;
                } else {
                    game_is_stopped_ = true;
                }
            } else {
                ui.onKeyPress(event.key.code);
            }
        }

        if (event.type == sf::Event::KeyReleased) {
            if (last_pressed_was_space) {
                game_is_stopped_ = false;
                last_pressed_was_space = false;
            } else {
                ui.onKeyRelease(window);
            }
        }
        if (event.type == sf::Event::MouseButtonPressed) {
            ui.onClick(window);

            if (event.mouseButton.button == sf::Mouse::Left) {
                const auto mouse_coords = window.mapPixelToCoords(sf::Mouse::getPosition(window));
                const auto map_cell_index = p_map_grid->coordToCell(mouse_coords);
                if (!selection_pending) {
                    start_position = mouse_coords;
                    mouse_selection.setPosition(start_position);

                    end_position = start_position;
                    selection_pending = true;
                }
                if (p_map_grid->cell2building_ind_.at(map_cell_index) != -1) {
                    selected_building_index = p_map_grid->cell2building_ind_.at(map_cell_index);
                }
            } else if (event.mouseButton.button == sf::Mouse::Middle) {
                view_is_moving = true;
                start_view_move_position = window.mapPixelToCoords(sf::Mouse::getPosition());
            } else if (event.mouseButton.button == sf::Mouse::Right) {
                if (!clicked) {
                    click_position = window.mapPixelToCoords(sf::Mouse::getPosition(window));
                };
                const auto neighbours = searcher->getNeighboursInds(click_position, RHARD * RHARD);

                BoidInd clicked_unit_ind = -1;
                for (const auto neighbour : neighbours) {
                    if (world_.ind2player[neighbour] != selected_player) {
                        clicked_unit_ind = neighbour;
                        break;
                    }
                }
                bc_->setPathEnds(selection, click_position);
                bc_->removeHoldingAgents(selection);

                std::chrono::high_resolution_clock path_finding_clock;
                const auto start = path_finding_clock.now();
                pf.issuePaths(*bc_, world_.r_coords_, bc_->radii_, selection, click_position);
                // auto path_and_portals = pf.calcPathOfSelection(*bc_, world_.r_coords_, bc_->radii_, selection, click_position);
                // path = path_and_portals.path;
                // portals = path_and_portals.portals;
                const auto time =
                    std::chrono::duration_cast<std::chrono::microseconds>(path_finding_clock.now() - start);
                std::cout << "path finding of selection took: " << time.count() << " us\n" << std::flush;
            }
        } else if (event.type == sf::Event::MouseMoved) {
            ui.onMouseHold(window);
            if (selection_pending) {
                end_position = window.mapPixelToCoords(sf::Mouse::getPosition(window));
            }
        } else if (event.type == sf::Event::MouseButtonReleased) {
            ui.onRelease(window);

            view_is_moving = false;

            if (event.mouseButton.button == sf::Mouse::Left) {
                end_position = window.mapPixelToCoords(sf::Mouse::getPosition(window));
                selection_pending = false;
                for (auto selected : selection) {
                    // world_.boid_inds2draw_data_[selected].deselect();
                }
                const auto previous_selection = selection;
                selectInRectangle(world_, start_position, end_position, selection, selected_player);
                int i = 0;
                for (auto selected : selection) {
                    // world_.boid_inds2draw_data_[selected].select();
                    // if(selected != previous_selection[i]){
                    //     selection_changed
                    // }
                    // i++;
                }
            }
        }
        if (event.type == sf::Event::MouseWheelMoved) {
            auto view2 = window.getView();
            if (event.mouseWheel.delta > 0) {
                view2.zoom(0.9f);
            } else {
                view2.zoom(1. / 0.9f);
            }
            window.setView(view2);
        }
    }

    auto view = window.getView();
    const auto view_size = view.getSize();
    const auto lx = view.getCenter() - view.getSize() / 2.f;
    const auto left_border = window.getViewport(view).left;
    const auto right_border = left_border + window.getViewport(view).width;
    const auto top_border = window.getViewport(view).top;
    const auto bottom_border = top_border + window.getViewport(view).height;
    const auto left_top_coord = window.mapPixelToCoords({left_border, top_border});
    const auto right_bottom_coord = window.mapPixelToCoords({right_border, bottom_border});

    const auto& mouse_pos = sf::Mouse::getPosition();
    const auto& mouse_coords = window.mapPixelToCoords(sf::Mouse::getPosition());
    if (view_is_moving) {
        const auto dr = mouse_coords - start_view_move_position;
        view.move(-dr * 0.01f);
    }
    if (mouse_pos.x + left_border * 0.2 > window.getViewport(view).width - 10 and
        left_top_coord.x < box_size.x - view_size.x / 2.f) {
        view.move(view_size.x / 100.f, 0);
    }

    if (mouse_pos.x > left_border - 50 and mouse_pos.x <= left_border and
        right_bottom_coord.x > window.getView().getSize().x) {
        view.move(-view_size.x / 100.f, 0);
    }
    if (mouse_pos.y >= bottom_border - 5 and left_top_coord.y < box_size.y - view_size.y / 2.) {
        view.move(0, view_size.y / 100.f);
    }
    if (mouse_pos.y <= top_border + 5 and right_bottom_coord.y > view_size.y / 2.) {
        view.move(0, -view_size.y / 100.f);
    }
    window.setView(view);

    if (sf::Mouse::isButtonPressed(sf::Mouse::Right) and sf::Keyboard::isKeyPressed(sf::Keyboard::LControl)) {
        auto mouse_position = window.mapPixelToCoords(sf::Mouse::getPosition(window));

        auto nearest_neigbhours = searcher->getNeighboursInds(mouse_position, RHARD * RHARD * 30);
        float radius = RHARD;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::A)) {
            radius = 3 * RHARD;
        }
        if (nearest_neigbhours.empty()) {
            //                UnitInd u_ind = {0, static_cast<unsigned  char> (selected_player)} ;
            addUnit(selected_player, static_cast<sf::Vector2f>(mouse_position), 0);
            //                world_.activate(u_ind,  {0, 0}, radius2);
        }
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::V)) {
        drawing = true;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Escape)) {
        drawing = false;
        bulding = false;
        selected_building_index = -1;
        selection.clear();
    }
    if (drawing) {
        auto mouse_position = window.mapPixelToCoords(sf::Mouse::getPosition(window));
        auto lower_left_cell_coord = p_map_grid->drawProposedBuilding(window, mouse_position, {5, 5});

        if (sf::Mouse::isButtonPressed(sf::Mouse::Right)) {
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) and wall_clock.getElapsedTime().asSeconds() > 0.4) {
                wall_clock.restart();
                p_map_grid->generateRandomWalls(mouse_position, {3000, 400}, 200);
                p_map_grid->updateBoundaryTypesLocally(lower_left_cell_coord, {3000, 400});
            } else {
                p_map_grid->buildWall(mouse_position, {5, 5});
                p_map_grid->updateBoundaryTypesLocally(lower_left_cell_coord, {10, 10});
            }
        }
        if (event.type == sf::Event::MouseButtonReleased) {
            p_map_grid->updateBoundaryTypes2();
            p_map_grid->sawOffCorners();
            drawing = false;
        }
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::F)) {
        bc_->formFormation(window.mapPixelToCoords(sf::Mouse::getPosition(window)), {0, 0}, selection);
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::E) &&  explosion_time.getElapsedTime().asSeconds()>0.05f) {
        bc_->addExplosion(window.mapPixelToCoords(sf::Mouse::getPosition(window)));
        explosion_time.restart();
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::B)) {
        bulding = true;
        building_clock.restart();
    }
    if (bulding and sf::Keyboard::isKeyPressed(sf::Keyboard::Escape) or
        (bulding and sf::Keyboard::isKeyPressed(sf::Keyboard::B) and
         building_clock.getElapsedTime().asSeconds() > 0.5)) {
        bulding = false;
    }
    if (bulding) {
        auto mouse_position = window.mapPixelToCoords(sf::Mouse::getPosition(window));
        auto lower_left_cell_coord = p_map_grid->drawProposedBuilding(window, mouse_position, {8, 8});
        sf::Vector2f building_size = {2.f * p_map_grid->cell_size_.x, 2.f * p_map_grid->cell_size_.y};
        sf::Vector2f lower_left_coord = {lower_left_cell_coord.x * p_map_grid->cell_size_.x,
                                         lower_left_cell_coord.y * p_map_grid->cell_size_.y};
        sf::Vector2f center_coord = lower_left_coord + building_size / 2.f;

        if (event.type == sf::Event::MouseButtonReleased) {
            p_map_grid->buildBuilding(mouse_position, {8, 8}, cdt);
            bulding = false;
            sf::Clock time;
            pf.update();
            buildings.load("", p_map_grid->buildings_);
            // std::cout << "pathfinder updating took: " << time.getElapsedTime().asMilliseconds() << " ms\n";
        }
    }

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) and sf::Keyboard::isKeyPressed(sf::Keyboard::B) and
        building_clock2.restart().asSeconds() > 0.2) {
        for (int i = 0; i < 1 * 200; ++i) {
            const float x = static_cast<float>(rand()) / RAND_MAX * box_size.x;
            const float y = static_cast<float>(rand()) / RAND_MAX * box_size.y;
            auto lower_left_cell_coord = p_map_grid->drawProposedBuilding(window, {x, y}, {4, 4});
            p_map_grid->buildBuilding({x, y}, {8, 8}, cdt);
        }
        buildings.load("", p_map_grid->buildings_);
        pf.update();
        if (!cdt.triangulationIsConsistent()) {
            throw std::runtime_error("triangulation not consistent!");
        }
        bulding = false;
    }

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) and sf::Keyboard::isKeyPressed(sf::Keyboard::T) and
        building_clock.restart().asSeconds() > 0.3f) {
        p_map_grid->extractEdgesFromTilesV2(cdt);
        p_map_grid->addAllBuildingsToTriangulation(cdt);
        pf.update();
        p_map_grid->sawOffCorners();
        p_map_grid->extractVerticesForDrawing(cdt, pf.tri_ind2component_);
    }

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::D) and building_clock.restart().asSeconds() > 0.5) {
        //            cdt = Triangulation(*p_grid); //! this is just retarded; ...
        //            edges = Edges();
        //            cdt.createBoundaryAndSuperTriangle();
        //            pf.update();
        //
        if (selected_building_index != -1) {
            p_map_grid->removeBuilding(selected_building_index, cdt);
            pf.update();
            selected_building_index = -1;
        }
    }

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::A) and !selection.empty()) {
        for (int sel : selection) {}
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Num1)) {
        selected_player = 0;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Num2)) {
        selected_player = 1;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::H) and !selection.empty()) {
        for (int sel : selection) {
            // world_.move_states_[sel] = MoveState::HOLDING;
        }
        bc_->addHoldingAgents(selection);
    } else if (sf::Keyboard::isKeyPressed(sf::Keyboard::S) and !selection.empty()) {
        bc_->setStanding(selection);
    }

    mouse_selection.setSize(
        {static_cast<float>(end_position.x - start_position.x), static_cast<float>(end_position.y - start_position.y)});

    for (int i : selection) {
        if (world_.move_states_[i] == MoveState::MOVING and dist2(world_.r_coords_[i], click_position) < 20) {
            world_.velocities_[i] = {0, 0};
            world_.move_states_[i] = MoveState::STANDING;
        }
    }
}
void Game::update(const float dt, sf::RenderWindow& window) {

    auto& bc_clock = clocks.clock;
    bc_clock.restart();
    bc_->updateState(dt, window);
    // // std::cout << " update state finished \n" << std::flush;
    clocks.neighbour_searching.addTime(bc_clock.restart().asMicroseconds());

    auto& path_finding_clock = clocks.clock;
    path_finding_clock.restart();
    if (!bc_->need_path_update_.empty()) {
        int n_agents_to_update = bc_->need_path_update_.size();
        while (!bc_->need_path_update_.empty()) {
            const auto ind = bc_->need_path_update_.front(); 
            // pf.updatePathOf(ind, world_.r_coords_[ind], *bc_, bc_->radii_[ind]);
            pf.issuePaths(*bc_, world_.r_coords_, bc_->radii_, {ind}, bc_->getPathEnd(ind));
            bc_->need_path_update_.pop();
        }
    }

    attack_system_->update(dt);

    for (const auto ind : world_.active_inds) {
        const auto player_ind = world_.ind2player.at(ind);
        const auto target_ind = bc_->attack_targets_[ind];
        std::vector<int> a({ind});

        if (target_ind != -1 and ind != target_ind) {
            const auto player_of_target_unit = world_.ind2player[target_ind];
            const auto r = world_.r_coords_[ind];
            const auto r_target = world_.r_coords_[target_ind];
            bool in_range = dist(r, r_target) < 10;

            if (in_range and player_ind != player_of_target_unit and world_.move_states_[ind] != MoveState::MOVING) {
                if (world_.move_states_[ind] == MoveState::STANDING) {
                    bc_->addHoldingAgents(a);
                }
                attack_system_->attack(ind, target_ind);
                world_.move_states_[ind] = MoveState::HOLDING;
            }
        } else {
            //            if(world_.move_states_[ind] == MoveState::HOLDING){
            //                bc_->removeFromClusters(a);
            //                world_.move_states_[ind] = MoveState::STANDING;
            //            }
        }
    }

    auto& to_kill = health_system_->to_kill_;
    //    std::unordered_set<int> wtf;
    //    for(const auto a : to_kill){
    //        wtf.insert(unit2boid(a));
    //    }
    //    for(const auto a : to_kill) {
    //        if(wtf.count(unit2boid(a))>1){
    //            throw std::runtime_error("");
    //        }
    //    }
    while (!to_kill.empty()) {
        removeUnit(to_kill.back());
        to_kill.pop_back();
    }

    world_.update(dt * 100.f);
    auto& fow_clock = clocks.clock;
    p_fow_->update(world_.r_coords_, world_.active_inds);
    clocks.fog_of_war.addTime(fow_clock.getElapsedTime().asMicroseconds());
    // std::cout << "fog of war finished \n" << std::flush;
    const auto fow_time = fow_clock.restart().asMicroseconds();

    pf.updatePaths(world_.r_coords_, *bc_, 5000);
    clocks.pathfinding.addTime(path_finding_clock.restart().asMilliseconds());


    frame_i++;
    if (frame_i == 100) {
        frame_i = 0;
        std::cout << "\n---------- ::CLOCKS:: ----------- \n";

        std::cout << "neighbour search and control took on avg: " << clocks.neighbour_searching.avg_time << " us\n";
        std::cout << "neighbour search and control took at most: " << clocks.neighbour_searching.max_time << " us\n";

        std::cout << "pathfinding took on avg: " << clocks.pathfinding.avg_time << " us\n";
        std::cout << "pathfinding took at most: " << clocks.pathfinding.max_time << " us\n";

        std::cout << "fog of war took on avg: " << fow_time << " us\n";
        std::cout << "fog of war took at most: " << clocks.fog_of_war.max_time << " us\n";
    }
}

void drawPath(sf::RenderWindow& window, const std::vector<sf::Vector2f>& path, const std::vector<Edgef>& portals) {
    sf::RectangleShape line;
    line.setFillColor(sf::Color::Cyan);
    sf::CircleShape node;
    node.setRadius(1.f);
    node.setFillColor(sf::Color::Cyan);

    sf::RectangleShape portal_line;
    portal_line.setFillColor(sf::Color::Black);

    for (int i = 1; i < path.size(); ++i) {
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

void drawFunction(const std::vector<sf::Vector2f>& points, sf::RenderWindow& window) {

    sf::RectangleShape line;
    line.setFillColor(sf::Color::Black);

    for (int i = 1; i < points.size(); ++i) {
        sf::Vector2f p2 = points[i];
        sf::Vector2f p1 = points[i - 1];
        p1.y *= -1;
        p2.y *= -1;
        const sf::Vector2f v = p2 - p1;
        if (norm2(v) != 0) {
            auto angle = 180.f / (M_PIf)*std::acos(dot(v / norm(v), {0, 1})) * (2.f * (v.x < 0.f) - 1.f);
            line.setSize({5.0, dist(p1, p2)});
            line.setRotation(angle);
            line.setPosition(p1);
            window.draw(line);
        }
    }
}

void Game::draw(sf::RenderWindow& window) {
    attack_system_->draw(window);
    p_map_grid->draw(window);
    world_.draw(window, *bc_);
    if (selection_pending) {
        window.draw(mouse_selection);
    }

    buildings.draw(window);
    p_fow_->draw(window);
    drawPath(window, path, portals);
}
