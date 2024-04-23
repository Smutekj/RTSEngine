#include <iostream>
#include <memory>
#include "Game.h"
#include "UI.h"
#include "Geometry.h"
#include "Triangulation.h"
#include "Utils/DebugInfo.h"
#include "Utils/RandomTools.h"
#include "Utils/Grid.h"


#include "include/glad/glad.h"
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

constexpr float FRAME_RATE = 60;
#define RES_X 1920
#define RES_Y 1080

#include "Systems/GraphicsSystem.h"
#include "Systems/VisionSystem.h"

#include "Graphics/SceneLayer.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"




int main() {
    const auto& BOX = Geometry::BOX;
    const auto& N_CELLS = Geometry::N_CELLS;

    sf::RenderWindow window({RES_X, RES_Y}); //,

    sf::Vector2f box_size = {static_cast<float>(BOX[0]), static_cast<float>(BOX[1])};
    sf::Vector2i n_cells = {N_CELLS[0], N_CELLS[1]};
    sf::Vector2f cell_size = {static_cast<float>(BOX[0] / n_cells.x), static_cast<float>(BOX[1] / n_cells.y)};

    //! time and clocks stuff
    const auto fps = 60; //! fixed fps
    const size_t fps_calc_period = 369; //! how often we update fps
    const auto time_of_frame = 1. / fps;
    const auto maximum_frame_time = time_of_frame * 1e6;
   
    //! create game world and some helper stuff
    DebugInfo dbg; 
    auto triangles_s_grid = std::make_shared<SearchGrid>(n_cells / 8, cell_size * 8.f);
    auto p_cdt = std::make_unique<Triangulation>(*triangles_s_grid);
    p_cdt->createBoundaryAndSuperTriangle();
    Game game(*p_cdt, n_cells, box_size);
    UI ui(window, game, (game.p_the_god_->getSystem<VisionSystem>(ComponentID::VISION)), dbg);
    
    generateRandomPositionsInCircles(0.01f, box_size, 200, game);

    int frame_i = 0;
    unsigned long long time_of_n_frames = 0;
    float real_fps = fps;
    double last_update_time = 0;

    glViewport(0.0, 0.0, window.size.x, window.size.y);
    while(!glfwWindowShouldClose(window.handle)){
        
        double now = glfwGetTime();
        double delta_time = now - last_update_time;
        
        glfwPollEvents();
        if (delta_time >= time_of_frame)
        {

            // draw your frame here
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
            
            ui.draw(window);

            game.parseInput(window, ui);
            game.update(time_of_frame, window);

            game.draw(window);

            dbg.draw(window, real_fps, *p_cdt, *game.p_map_grid);

            GLFWwindow* backup_current_context = glfwGetCurrentContext();
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            glfwMakeContextCurrent(backup_current_context);

            glfwSwapBuffers(window.handle);


            // only set lastFrameTime when you actually draw something
            if(frame_i++ == 120){
                auto t = 1000.*(glfwGetTime() - now);
                dbg.time_of_frame = t;
                frame_i = 0;
                std::cout << "frame took: " << t << " ms\n";
            }
            last_update_time = now;

        }
    }
    //! cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window.handle); 
    glfwTerminate();

    return 0;
}