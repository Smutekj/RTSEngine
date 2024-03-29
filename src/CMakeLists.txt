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
set(objs ECS.cpp Grid.cpp Triangulation.cpp
        Triangulation.cpp DebugInfo.cpp 
        UI.cpp  Settings.cpp NeighbourSearcherStrategy.cpp  NeighbourSearcherContext.cpp )
set(hdrs ECS.h core.h Grid.h Triangulation.h
        Triangulation.h DebugInfo.h 
        UI.h Settings.h NeighbourSearcherStrategy.h  NeighbourSearcherContext.h )

set(game_objs UnitInitializer.cpp MapGrid.cpp EdgesFinder1D.cpp Edges.cpp   Game.cpp SoundModule.cpp )
set(game_hdrs UnitInitializer.h MapGrid.h EdgesFinder1D.h Edges.h   Game.h  SoundModule.h RandomTools.h  )


set(systems_objs  VisionSystem.cpp PhysicsSystem.cpp SeekSystem.cpp AttackSystem.cpp ProjectileSystem.cpp GraphicsSystem.cpp )
set(systems_hdrs  VisionSystem.h PhysicsSystem.h SeekSystem.h AttackSystem.h ProjectileSystem.h GraphicsSystem.h )
list(TRANSFORM systems_objs PREPEND ${CMAKE_CURRENT_SOURCE_DIR}/Systems/ )
list(TRANSFORM systems_hdrs PREPEND ${CMAKE_CURRENT_SOURCE_DIR}/Systems/ )


set(pathfinding_objs PathFinder2.cpp ReducedTriangulationGraph.cpp )
set(pathfinding_hdrs PathFinder2.h  ReducedTriangulationGraph.h )
list(TRANSFORM pathfinding_objs PREPEND ${CMAKE_CURRENT_SOURCE_DIR}/PathFinding/ )
list(TRANSFORM pathfinding_hdrs PREPEND ${CMAKE_CURRENT_SOURCE_DIR}/PathFinding/ )

set(graphics_hdrs  BloomEffect.cpp PostEffect.cpp)
set(graphics_objs BloomEffect.hpp PostEffect.hpp ResourceHolder.hpp ResourceIdentifiers.hpp)
list(TRANSFORM graphics_hdrs PREPEND ${CMAKE_CURRENT_SOURCE_DIR}/Graphics/ )
list(TRANSFORM graphics_objs PREPEND ${CMAKE_CURRENT_SOURCE_DIR}/Graphics/ )

add_executable(Boids main.cpp ${objs} ${game_objs}  ${graphics_objs}  ${systems_objs} ${pathfinding_objs}
                              ${hdrs} ${game_hdrs} ${graphics_hdrs}  ${systems_hdrs} ${pathfinding_hdrs})


# set(libraries sfml-graphics OpenMP::OpenMP_CXX
##### THIS IS FOR GOOGLE TEST
enable_testing()

add_executable(
        test_all
        Tests/main.cpp
        ${objs} ${game_objs}  ${graphics_objs}  ${systems_objs} ${pathfinding_objs}
                              ${hdrs} ${game_hdrs} ${graphics_hdrs}  ${systems_hdrs} ${pathfinding_hdrs}
                          )

if(OpenMP_CXX_FOUND)
target_link_libraries(Boids PRIVATE sfml-graphics sfml-audio OpenMP::OpenMP_CXX)
target_link_libraries(test_all PRIVATE sfml-graphics sfml-audio OpenMP::OpenMP_CXX GTest::gtest_main)
endif()


include(GoogleTest)
gtest_discover_tests(test_all)


#     target_link_libraries(Boids PRIVATE sfml-graphics)