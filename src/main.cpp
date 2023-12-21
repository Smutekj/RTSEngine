#include <iostream>
#include <memory>
#include "Grid.h"
#include "BoidControler.h"
#include "Triangulation.h"
#include "PathFinder.h"
#include "DebugInfo.h"
#include "Game.h"
#include "UI.h"
#include "Geometry.h"

constexpr float FRAME_RATE = 60;

inline float randf(const float min = 0, const float max = 1){
    return (rand() / static_cast<float>(RAND_MAX)) * (max - min) + min;
}

void generateRandomPositionsAroundBox(sf::Vector2f box_size, int n_positions, Game& game) {
    sf::Vector2f pos;
    for (int i = 0; i < n_positions / 4; ++i) {
        pos.x = rand() / static_cast<float>(RAND_MAX) * box_size.x / 10.f;
        pos.y = rand() / static_cast<float>(RAND_MAX) * box_size.y / 10.f;
        game.addUnit(0, pos, 0);
    }
    for (int i = 0; i < n_positions / 4; ++i) {
        pos.x = box_size.x - rand() / static_cast<float>(RAND_MAX) * box_size.x / 10.f;
        pos.y = rand() / static_cast<float>(RAND_MAX) * box_size.y / 10.f;
        game.addUnit(0, pos, 0);
    }
    for (int i = 0; i < n_positions / 4; ++i) {
        pos.x = rand() / static_cast<float>(RAND_MAX) * box_size.x / 10.f;
        pos.y = box_size.y - rand() / static_cast<float>(RAND_MAX) * box_size.y / 10.f;
        game.addUnit(0, pos, 0);
    }
    for (int i = 0; i < n_positions / 4; ++i) {
        pos.x = rand() / static_cast<float>(RAND_MAX) * box_size.x / 10.f;
        pos.y = rand() / static_cast<float>(RAND_MAX) * box_size.y / 10.f;
        game.addUnit(0, pos, 0);
    }

    // for (int i = 0; i < n_positions; ++i) {
    //     pos.x = rand() / static_cast<float>(RAND_MAX) * box_size.x / 5.f;
    //     pos.y = rand() / static_cast<float>(RAND_MAX) * box_size.y / 5.f;
    //     game.addUnit(0, pos, 0);
    // }
}


sf::Vector2f randomPosInBox(sf::Vector2f ul_corner = {0,0},
                            sf::Vector2f box_size = {Geometry::BOX[0], Geometry::BOX[1]}
                            ){
    return {ul_corner.x + rand() / static_cast<float>(RAND_MAX) * box_size.x,
            ul_corner.y + rand() / static_cast<float>(RAND_MAX) * box_size.y };
}



struct Circle{
    sf::Vector2f r = {0,0}; //! center 
    float radius_sq = 1;
};

Circle randomCircle(const float r_max){
     return {randomPosInBox(), randf(0, r_max*r_max)}; 
}
void generateRandomPositionsInCircles(const float density,
                                      sf::Vector2f box_size,
                                      int n_positions,
                                      Game& game
                                      ) {

    while(n_positions > 0){
        const auto circle = randomCircle(100);
        int n_in_circle = std::min({static_cast<int>(std::floor(M_PI*circle.radius_sq * density)), n_positions}); 
        const auto range_x = 2*std::sqrt(circle.radius_sq);
        const sf::Vector2f r0 = {circle.r.x - range_x/2.0f, circle.r.y - range_x/2.0f};
        sf::Vector2f r = r0;
        const float dx = std::sqrt(1.f/density); 
        while(n_in_circle > 0 and r.y < r0.y + range_x){
            if(dist2(r, circle.r) < circle.radius_sq){
                game.addUnit(0, r, 0);
                n_positions--;
                n_in_circle--;
            }
            r.x += dx;
            if(r.x > (r0.x + range_x)){
                r.x -= range_x;
                r.y += dx;
            }
        }
    }
}

int main() {

    const auto& BOX = Geometry::BOX;
    const auto& N_CELLS = Geometry::N_CELLS;

    sf::ContextSettings settings;

    sf::RenderWindow window(sf::VideoMode(1920, 1080), "", sf::Style::Default, settings); //, sf::Style::Fullscreen
    window.setFramerateLimit(FRAME_RATE);

    sf::Vector2f box_size = asFloat({BOX[0], BOX[1]});

    auto view = window.getView();
    view.setViewport({0.1f, 0.0f, 1.0f, 1.0f});
    view.setCenter(box_size / 2.f);
    view.setSize(box_size.x, box_size.x * (1080.f / 1920.f));
    window.setView(view);

    const auto fps = 60; //! fixed fps
    const auto time_of_frame = 1. / fps;
    const auto maximum_frame_time = time_of_frame * 1e6;
    sf::Clock frame_clock;
    sf::Clock game_clock;

    float frame_time = maximum_frame_time;
    sf::Vector2i n_cells = {N_CELLS[0], N_CELLS[1]};
    sf::Vector2f cell_size = asFloat({BOX[0] / n_cells.x, BOX[1] / n_cells.y});

    auto triangles_s_grid = std::make_shared<SearchGrid>(n_cells / 10, cell_size * 10.f);
    auto p_cdt = std::make_unique<Triangulation>(*triangles_s_grid);
    auto p_pf = std::make_unique<PathFinder>(p_cdt.get());

    DebugInfo dbg; 

    p_cdt->window = &window;
    p_cdt->dbg = &dbg;
    p_cdt->createBoundaryAndSuperTriangle();

    Game game(*p_pf, *p_cdt, n_cells, box_size);

    auto* world = &game.world_;

    UI ui(game, *game.p_fow_, dbg, *game.bc_);

    // generateRandomPositionsAroundBox(box_size, 1, game);
    generateRandomPositionsInCircles(0.005f, box_size, 4000, game);

    int i = 0;
    unsigned long long time_of_n_frames = 0;
    float real_fps = fps;
    while (window.isOpen()) {
        window.clear(sf::Color::White);

        frame_clock.restart();
        game_clock.restart();
        // if (!game.game_is_stopped_) {
        game.parseInput(window, ui);
        game.update(time_of_frame, window);
        // }
        const auto game_step_time = game_clock.restart().asMicroseconds();

        sf::Clock drawing_clock;
        game.draw(window);
        dbg.draw(window, real_fps, world->n_active_, *p_cdt, *game.p_map_grid, *game.bc_);
        ui.draw(window);
        const auto drawing_time = drawing_clock.restart().asMicroseconds();

        const size_t n_frames_fps = 369;
        time_of_n_frames += frame_clock.getElapsedTime().asMicroseconds();
        if (i++ == n_frames_fps) {
            real_fps = static_cast<float>(n_frames_fps) / time_of_n_frames * 1e6f;
            time_of_n_frames = 0;
            i = 0;
            std::cout << "drawing took: " << drawing_time << " us\n";
            // std::cout << "real fps is: " << static_cast<float>(n_frames_fps) / time_of_n_frames * 1e6 << " fps\n";
            std::cout << "game step took: " << game_step_time << " us\n";
        }

        window.display();

        // if (frame_time < maximum_frame_time) {
        //     sf::Time sleep_time = sf::microseconds(maximum_frame_time - frame_time);
        //     sf::sleep(sleep_time);
        // }
    }
    return 0;
}