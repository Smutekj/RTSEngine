#pragma once
#include <gtest/gtest.h>
#include "../Utils/Grid.h"
#include "../PathFinding/Triangulation.h"
#include "../PathFinding/ReducedTriangulationGraph.h"
#include "../PathFinding/PathFinder2.h"



//! when we insert vertex on an existing one nothing should happen
TEST(TestReducedGraph, BasicAssertions) {

    sf::Vector2f box = {100, 100};
    sf::Vector2i n_cells = {5, 5};

    auto triangles_s_grid = std::make_shared<SearchGrid>(n_cells, box/4.f);
    auto p_cdt = std::make_unique<Triangulation>(*triangles_s_grid);

    p_cdt->createBoundaryAndSuperTriangle({100, 100});
    sf::Vector2i r_vertex0 = {50, 50};
    sf::Vector2i r_vertex1 = {55, 50};
    sf::Vector2i r_vertex2 = {55, 55};
    sf::Vector2i r_vertex3 = {50, 55};
    auto data_before = p_cdt->insertVertexAndGetData(r_vertex0 , false);
    data_before = p_cdt->insertVertexAndGetData(r_vertex1 , false);
    data_before = p_cdt->insertVertexAndGetData(r_vertex2 , false);
    data_before = p_cdt->insertVertexAndGetData(r_vertex3 , false);

    auto last_vert_ind = p_cdt->vertices_.size()-1;
    EdgeVInd e;
    e.from = last_vert_ind;
    e.to = last_vert_ind  - 1;
    p_cdt->insertConstraint(e);
    e.from = last_vert_ind -1;
    e.to = last_vert_ind - 2;
    p_cdt->insertConstraint(e);
    e.from = last_vert_ind - 2;
    e.to = last_vert_ind  - 3;
    p_cdt->insertConstraint(e);
    e.from = last_vert_ind - 3;
    e.to = last_vert_ind ;
    p_cdt->insertConstraint(e);

    r_vertex0 = {30, 30};
    r_vertex1 = {35, 30};
    r_vertex2 = {35, 35};
    r_vertex3 = {30, 35};
    data_before = p_cdt->insertVertexAndGetData(r_vertex0 , false);
    data_before = p_cdt->insertVertexAndGetData(r_vertex1 , false);
    data_before = p_cdt->insertVertexAndGetData(r_vertex2 , false);
    data_before = p_cdt->insertVertexAndGetData(r_vertex3 , false);

    last_vert_ind = p_cdt->vertices_.size()-1;
    e.from = last_vert_ind;
    e.to = last_vert_ind  - 1;
    p_cdt->insertConstraint(e);
    e.from = last_vert_ind -1;
    e.to = last_vert_ind - 2;
    p_cdt->insertConstraint(e);
    e.from = last_vert_ind - 2;
    e.to = last_vert_ind  - 3;
    p_cdt->insertConstraint(e);
    e.from = last_vert_ind - 3;
    e.to = last_vert_ind ;
    p_cdt->insertConstraint(e);
    
    EXPECT_TRUE(p_cdt->triangulationIsConsistent());

    std::vector<TriInd> tri_ind2components(p_cdt->triangles_.size());

    ReducedTriangulationGraph rtg;
    rtg.constructFromTriangulation(*p_cdt, tri_ind2components);
    EXPECT_TRUE(rtg.edges.size() == 5);
    for(int vert_ind = 0; vert_ind < rtg.vertex2tri_ind.size(); ++vert_ind){
        auto tri_ind =  rtg.vertex2tri_ind[vert_ind];
        auto vert_ind_after = rtg.tri_ind2vertex.at(tri_ind);
        EXPECT_EQ(vert_ind, vert_ind_after);

        // auto vertex = rtg.vertices_.at(vert_ind);
    }

}

