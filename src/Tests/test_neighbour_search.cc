#pragma once
#include <gtest/gtest.h>
#include "../Grid.h"
#include "../NeighbourSearcherContext.h"
#include "../NeighbourSearcherStrategy.h"


TEST(TestSmallestDistance, BasicAssertions) {

    sf::Vector2f box = {100, 100};

    std::unique_ptr<NeighbourSearcherContext<PathFinderComponent, int>> p_ns;
    p_ns = std::make_unique<NeighbourSearcherContext<PathFinderComponent, int>>(box, 10); //! make 10x10 grid
    p_ns->setStrategy(NS_STRATEGY::BASIC);    
    
    std::vector<PathFinderComponent> comps;
    
    PathFinderComponent pfc;
    pfc.radius = 5;
    pfc.transform.r = {25, 25};
    comps.push_back(pfc);
    pfc.transform.r = {40, 40};
    comps.push_back(pfc);
    pfc.transform.r = {24, 24};
    comps.push_back(pfc);

    p_ns->update(comps);
    const auto& i_data1 = p_ns->getInteractionData(0);
    const auto n_neighbours1 = p_ns->last_i.at(0);

    const auto& i_data2 = p_ns->getInteractionData(1);
    const auto n_neighbours2 = p_ns->last_i.at(1);

    const auto& i_data3 = p_ns->getInteractionData(2);
    const auto n_neighbours3 = p_ns->last_i.at(2);
    EXPECT_EQ(1 , n_neighbours1);
    EXPECT_EQ(2, i_data1.at(0));
    EXPECT_EQ(0, i_data2.at(0));
    EXPECT_EQ(0 , n_neighbours2);
}

TEST(TestFullSearch, BasicAssertions) {
    sf::Vector2f box = {100, 100};

    std::unique_ptr<NeighbourSearcherContext<PathFinderComponent, int>> p_ns;
    p_ns = std::make_unique<NeighbourSearcherContext<PathFinderComponent, int>>(box, 10); //! make 10x10 grid
    p_ns->setStrategy(NS_STRATEGY::BASIC);    
    
    std::vector<PathFinderComponent> comps;
    
    PathFinderComponent pfc;
    pfc.radius = 5;
    pfc.transform.r = {25, 25};
    comps.push_back(pfc);
    pfc.transform.r = {40, 40};
    comps.push_back(pfc);
    pfc.transform.r = {24, 24};
    comps.push_back(pfc);

    p_ns->update(comps);

    auto neighbour_inds =p_ns->getNeighboursIndsFull({39, 39}, 10);
    EXPECT_EQ(1, neighbour_inds.size());
    neighbour_inds = p_ns->getNeighboursIndsFull({39, 39}, 1);
    EXPECT_EQ(0, neighbour_inds.size());
}

TEST(TestAttackNSStrategy, BasicAssertions) {

    sf::Vector2f box = {100, 100};

    std::unique_ptr<NeighbourSearcherContext<AttackComponent, int>> p_ns;
    p_ns = std::make_unique<NeighbourSearcherContext<AttackComponent, int>>(box, 10); //! make 10x10 grid
    p_ns->setStrategy(NS_STRATEGY::ATTACK);    
    
    std::vector<AttackComponent> comps;
    
    AttackComponent pfc;
    pfc.weapon.range = 5;
    pfc.transform.r = {25, 25};
    comps.push_back(pfc);
    pfc.transform.r = {40, 40};
    comps.push_back(pfc);
    pfc.transform.r = {24, 24};
    comps.push_back(pfc);

    p_ns->update(comps);

    auto neighbour_inds =p_ns->getNeighboursIndsFull({39, 39}, 10);
    EXPECT_EQ(1, neighbour_inds.size());
    neighbour_inds = p_ns->getNeighboursIndsFull({39, 39}, 1);
    EXPECT_EQ(0, neighbour_inds.size());
    
}