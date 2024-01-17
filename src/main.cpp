#include <iostream>
#include <memory>
#include "Grid.h"
#include "Triangulation.h"
#include "DebugInfo.h"
#include "Game.h"
#include "UI.h"
#include "Geometry.h"
#include "RandomTools.h"

constexpr float FRAME_RATE = 60;
#define RES_X 1920
#define RES_Y 1080

int main() {

    const auto& BOX = Geometry::BOX;
    const auto& N_CELLS = Geometry::N_CELLS;

    sf::ContextSettings settings;

    sf::RenderWindow window(sf::VideoMode(RES_X, RES_Y), "",  sf::Style::Fullscreen, settings); //,
    window.setFramerateLimit(FRAME_RATE);

    sf::Vector2f box_size = {static_cast<float>(BOX[0]), static_cast<float>(BOX[1])};
    sf::Vector2i n_cells = {N_CELLS[0], N_CELLS[1]};
    sf::Vector2f cell_size = {static_cast<float>(BOX[0] / n_cells.x), static_cast<float>(BOX[1] / n_cells.y)};

    auto view = window.getView();
    view.setViewport({0.1f, 0.0f, 0.9f, 1.0f});
    view.setCenter(box_size / 2.f);
    view.setSize(box_size.x, box_size.x * (1080.f / 1920.f));
    window.setView(view);

    //! time and clocks stuff
    const auto fps = 60; //! fixed fps
    const size_t fps_calc_period = 369; //! how often we update fps
    const auto time_of_frame = 1. / fps;
    const auto maximum_frame_time = time_of_frame * 1e6;
    sf::Clock frame_clock;
    sf::Clock game_clock;
   
    //! create game world and some helper stuff
    DebugInfo dbg; 
    auto triangles_s_grid = std::make_shared<SearchGrid>(n_cells / 8, cell_size * 8.f);
    auto p_cdt = std::make_unique<Triangulation>(*triangles_s_grid, dbg);
    p_cdt->createBoundaryAndSuperTriangle();
    Game game(*p_cdt, n_cells, box_size);
    UI ui(game, (game.p_the_god_->getSystem<VisionSystem>(ComponentID::VISION)), dbg);

    //! generate random positions in circles around the map (used for testing only)
    generateRandomPositionsInCircles(0.01f, box_size, 100, game);

    int frame_i = 0;
    unsigned long long time_of_n_frames = 0;
    float real_fps = fps;

    while (window.isOpen()) {
        window.clear(sf::Color::White);
        frame_clock.restart();

        game_clock.restart();
        game.parseInput(window, ui);
        game.update(time_of_frame, window);
        const auto game_step_time = game_clock.restart().asMicroseconds();

        sf::Clock drawing_clock;
        game.draw(window);
        dbg.draw(window, real_fps, *p_cdt, *game.p_map_grid);
        ui.draw(window);
        const auto drawing_time = drawing_clock.restart().asMicroseconds();
        time_of_n_frames += frame_clock.getElapsedTime().asMicroseconds();
        if (frame_i++ == fps_calc_period) {
            real_fps = static_cast<float>(fps_calc_period) / time_of_n_frames * 1e6f;
            time_of_n_frames = 0;
            frame_i = 0;
            std::cout << "drawing took: " << drawing_time << " us\n";
            std::cout << "game step took: " << game_step_time << " us\n";
        }

        window.display();
    }
    return 0;
}

