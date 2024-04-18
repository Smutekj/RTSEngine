add_definitions(-D_USE_MATH_DEFINES)

# ========== place libraries and executables in outermost dir ==========

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})


include(FetchContent)
FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        release-1.10.0
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)


# ========== cpp and header files ==========

# TODO: Create a Pathfinding Library which will include triangulation and pathfinding
set(objs ECS.cpp
        UI.cpp  Settings.cpp )
set(hdrs ECS.h core.h 
        UI.h Settings.h )

###### GAME ###### 
set(game_objs Selection.cpp UnitInitializer.cpp MapGrid.cpp EdgesFinder1D.cpp Edges.cpp   Game.cpp SoundModule.cpp )
set(game_hdrs Selection.h UnitInitializer.h MapGrid.h EdgesFinder1D.h Edges.h   Game.h  SoundModule.h )

###### SYSTEMS ###### 
set(systems_objs  VisionSystem.cpp PhysicsSystem.cpp SeekSystem.cpp AttackSystem.cpp ProjectileSystem.cpp GraphicsSystem.cpp )
set(systems_hdrs  VisionSystem.h PhysicsSystem.h SeekSystem.h AttackSystem.h ProjectileSystem.h GraphicsSystem.h )
list(TRANSFORM systems_objs PREPEND ${CMAKE_CURRENT_SOURCE_DIR}/Systems/ )
list(TRANSFORM systems_hdrs PREPEND ${CMAKE_CURRENT_SOURCE_DIR}/Systems/ )

###### PATHFINDING ###### 
set(pathfinding_objs Triangulation.cpp PathFinder2.cpp ReducedTriangulationGraph.cpp )
set(pathfinding_hdrs Triangulation.h PathFinder2.h  ReducedTriangulationGraph.h )
list(TRANSFORM pathfinding_objs PREPEND ${CMAKE_CURRENT_SOURCE_DIR}/PathFinding/ )
list(TRANSFORM pathfinding_hdrs PREPEND ${CMAKE_CURRENT_SOURCE_DIR}/PathFinding/ )

###### GUI ###### 
set(gui_hdrs  imgui.h backends/imgui_impl_glfw.h backends/imgui_impl_opengl3.h )
set(gui_objs imgui.cpp backends/imgui_impl_glfw.cpp backends/imgui_impl_opengl3.cpp imgui_demo.cpp imgui_draw.cpp imgui_tables.cpp imgui_widgets.cpp) 
list(TRANSFORM gui_hdrs PREPEND ${CMAKE_CURRENT_SOURCE_DIR}/../imgui/ )
list(TRANSFORM gui_objs PREPEND ${CMAKE_CURRENT_SOURCE_DIR}/../imgui/ )


set(graphic_hdrs Utils.hpp Text.hpp Font.hpp SceneLayer.h Image.h
    stb_image.h stb_image_write.h ShaderNames.h Shader.h Texture.hpp RenderTarget.hpp RenderTexture.hpp
     RenderWindow.hpp VertexArray.hpp RectangleShape.hpp ConvexShape.hpp Shape.hpp Vertex.hpp 
     Time.hpp Color.hpp Rect.hpp)

set(graphics_objs Utils.cpp Text.cpp Font.cpp  SceneLayer.cpp Image.cpp
 Shader.cpp Texture.cpp RenderTarget.cpp RenderTexture.cpp RenderWindow.cpp VertexArray.cpp RectangleShape.cpp
  ConvexShape.cpp Shape.cpp Vertex.cpp  Color.cpp) 
list(TRANSFORM graphic_hdrs PREPEND ${CMAKE_CURRENT_SOURCE_DIR}/Graphics/ )
list(TRANSFORM graphics_objs PREPEND ${CMAKE_CURRENT_SOURCE_DIR}/Graphics/ )

set(utils_objs DebugInfo.cpp Grid.h Vector.hpp GayVector.h  RandomTools.h NeighbourSearcherStrategy.h  NeighbourSearcherContext.h  ) 
set(utils_hdrs DebugInfo.h  Grid.cpp GayVector.cpp NeighbourSearcherStrategy.cpp  NeighbourSearcherContext.cpp ) 
list(TRANSFORM utils_objs PREPEND ${CMAKE_CURRENT_SOURCE_DIR}/Utils/ )
list(TRANSFORM utils_hdrs PREPEND ${CMAKE_CURRENT_SOURCE_DIR}/Utils/ )

add_executable(Boids main.cpp glad.c MouseKeyboard.cpp MouseKeyboard.hpp ${utils_hdrs} ${utils_objs}
 ${objs} ${game_objs}   ${systems_objs} ${pathfinding_objs} ${gui_objs}
                              ${hdrs} ${game_hdrs}  ${systems_hdrs} ${pathfinding_hdrs}
                               ${gui_hdrs} ${graphics_hdrs} ${graphics_objs}
                )


#### THIS IS FOR GOOGLE TEST
enable_testing()

add_executable(
        test_all
        Tests/main.cpp
        glad.c MouseKeyboard.cpp MouseKeyboard.hpp ${utils_hdrs} ${utils_objs}
        ${objs} ${game_objs}   ${systems_objs} ${pathfinding_objs} ${gui_objs}
                              ${hdrs} ${game_hdrs}  ${systems_hdrs} ${pathfinding_hdrs}
                               ${gui_hdrs} ${graphics_hdrs} ${graphics_objs}
                )
include_directories( ${OPENGL_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS} ${FREETYPE_INCLUDE_DIRS} lib ../imgui ../imgui/backends)

if(OpenMP_CXX_FOUND)
target_link_libraries(Boids PRIVATE  ${GLEW_LIBRARIES} glfw OpenMP::OpenMP_CXX ${FREETYPE_LIBRARIES})
target_link_libraries(test_all PRIVATE  ${GLEW_LIBRARIES} glfw OpenMP::OpenMP_CXX ${FREETYPE_LIBRARIES} 
                        GTest::gtest_main)
endif()


include(GoogleTest)
gtest_discover_tests(test_all)

