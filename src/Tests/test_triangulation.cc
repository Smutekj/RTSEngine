#pragma once
#include <gtest/gtest.h>
#include "../Grid.h"
#include "../Triangulation.h"

bool containsPoint(const Triangle &tri, Vertex point)
{
    return tri.verts[0] == point || tri.verts[1] == point || tri.verts[2] == point;
}

TEST(TestVertexInsertions, BasicAssertions)
{

    sf::Vector2f box = {100, 100};
    sf::Vector2i n_cells = {5, 5};

    auto triangles_s_grid = std::make_shared<SearchGrid>(n_cells, box / 20.f);
    auto p_cdt = std::make_unique<Triangulation>(*triangles_s_grid);

    p_cdt->createBoundaryAndSuperTriangle();
    sf::Vector2i r_vertex = {50, 50};
    p_cdt->insertVertex(r_vertex, false);

    const auto &tris = p_cdt->triangles_;

    EXPECT_TRUE(containsPoint(tris.back(), r_vertex));

    EXPECT_EQ(11, tris.size());
}

//! when we insert vertex on an existing one nothing should happen
TEST(TestVertexOnVertexInsertion, BasicAssertions)
{

    sf::Vector2f box = {100, 100};
    sf::Vector2i n_cells = {5, 5};

    auto triangles_s_grid = std::make_shared<SearchGrid>(n_cells, box / 4.f);
    auto p_cdt = std::make_unique<Triangulation>(*triangles_s_grid);

    p_cdt->createBoundaryAndSuperTriangle();
    sf::Vector2i r_vertex = {50, 50};
    auto data_before = p_cdt->insertVertexAndGetData(r_vertex, false);
    auto data_after = p_cdt->insertVertexAndGetData(r_vertex, false);
    EXPECT_TRUE(data_before.overlapping_vertex == -1);
    EXPECT_TRUE(data_after.overlapping_vertex == p_cdt->vertices_.size() - 1);
}

//! when we insert vertex on an existing one nothing should happen
TEST(TestVertexOnEdgeInsertion, BasicAssertions)
{

    sf::Vector2f box = {100, 100};
    sf::Vector2i n_cells = {5, 5};

    auto triangles_s_grid = std::make_shared<SearchGrid>(n_cells, box / 20.f);
    auto p_cdt = std::make_unique<Triangulation>(*triangles_s_grid);

    p_cdt->createBoundaryAndSuperTriangle();
    sf::Vector2i r_vertex0 = {50, 50};
    sf::Vector2i r_vertex1 = {55, 50};
    sf::Vector2i r_vertex2 = {55, 55};
    sf::Vector2i r_vertex3 = {50, 55};
    auto data_before = p_cdt->insertVertexAndGetData(r_vertex0, false);
    data_before = p_cdt->insertVertexAndGetData(r_vertex1, false);
    data_before = p_cdt->insertVertexAndGetData(r_vertex2, false);
    data_before = p_cdt->insertVertexAndGetData(r_vertex3, false);

    auto last_vert_ind = p_cdt->vertices_.size() - 1;
    EdgeVInd e;
    e.from = last_vert_ind;
    e.to = last_vert_ind - 1;
    p_cdt->insertConstraint(e);
    e.from = last_vert_ind - 1;
    e.to = last_vert_ind - 2;
    p_cdt->insertConstraint(e);
    e.from = last_vert_ind - 2;
    e.to = last_vert_ind - 3;
    p_cdt->insertConstraint(e);
    e.from = last_vert_ind - 3;
    e.to = last_vert_ind;
    p_cdt->insertConstraint(e);

    EXPECT_TRUE(p_cdt->triangulationIsConsistent());

    sf::Vector2i r_vertex_on_edge = {50, 52};
    auto n_tris_before = p_cdt->triangles_.size();
    data_before = p_cdt->insertVertexAndGetData(r_vertex_on_edge, false);
    auto n_tris_after = p_cdt->triangles_.size();
    EXPECT_TRUE(data_before.overlapping_edge.to == last_vert_ind - 3 &&
                data_before.overlapping_edge.from == last_vert_ind);
    EXPECT_EQ(data_before.overlapping_vertex, -1);
    EXPECT_EQ(n_tris_after, n_tris_before + 2);
}

//! when we insert vertex on an existing one nothing should happen
TEST(TestTriangleFinder, BasicAssertions)
{

    sf::Vector2f box = {100, 100};
    sf::Vector2i n_cells = {5, 5};

    auto triangles_s_grid = std::make_shared<SearchGrid>(n_cells, box / 20.f);
    auto p_cdt = std::make_unique<Triangulation>(*triangles_s_grid);

    p_cdt->createBoundaryAndSuperTriangle();
    sf::Vector2i r_vertex = {50, 50};
    p_cdt->insertVertex(r_vertex, false);

    sf::Vector2f r_search = {54, 54};
    auto sought_tri_ind = p_cdt->findTriangle(r_search, false);
    const auto &tris = p_cdt->triangles_;

    EXPECT_EQ(sought_tri_ind, 4);
}
