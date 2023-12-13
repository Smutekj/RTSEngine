add_definitions(-D_USE_MATH_DEFINES)

# ========== place libraries and executables in outermost dir ==========

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})


# ========== cpp and header files ==========

# TODO: Create a Pathfinding Library which will include triangulation and pathfinding
set(objs Grid.cpp BoidWorld.cpp BoidControler.cpp  PathFinder.cpp
        Triangulation.cpp DebugInfo.cpp ReducedTriangulationGraph.cpp
        UI.cpp  Settings.cpp NeighbourSearcher.cpp)

set(hdrs core.h Grid.h BoidWorld.h  BoidControler.h PathFinder.h 
        Triangulation.h DebugInfo.h ReducedTriangulationGraph.h
        UI.h Settings.h Component.h NeighbourSearcher.h)

set(game_objs MapGrid.cpp EdgesFinder1D.cpp
Edges.cpp Selection.cpp  HealthSystem.cpp  Game.cpp 
AttackSystem.cpp BloomEffect.cpp FogOfWar.cpp)

set(game_hdrs MapGrid.h EdgesFinder1D.h
Edges.h  Selection.h  HealthSystem.h  Game.h 
AttackSystem.h BloomEffect.hpp FogOfWar.h)

set(graphics_hdrs PostEffect.cpp)
set(graphics_cpp PostEffect.hpp ResourceHolder.hpp ResourceIdentifiers.hpp)

add_executable(Boids main.cpp ${objs} ${game_objs}  ${graphics_cpp} ${hdrs} ${game_hdrs} ${graphics_hdrs} )


# set(libraries sfml-graphics OpenMP::OpenMP_CXX)

if(OpenMP_CXX_FOUND)
target_link_libraries(Boids PRIVATE sfml-graphics OpenMP::OpenMP_CXX)
endif()

if(!OpenMP_CXX_FOUND)
target_link_libraries(Boids PRIVATE sfml-graphics)
endif()



#     target_link_libraries(Boids PRIVATE sfml-graphics)