
#include <gtest/gtest.h>

#include "test_neighbour_search.cc"
#include "test_triangulation.cc"
#include "test_reduced_graph.cc"

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}