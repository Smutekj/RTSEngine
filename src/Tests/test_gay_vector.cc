#pragma once
#include <gtest/gtest.h>
#include "../Utils/GayVector.h"



TEST(TestGayVectorI, BasicAssertions) {

    GayVectorI<10> selected_inds;
    
    auto& inds = selected_inds.data;
    EXPECT(inds.empty());
    
    vector<int> to_insert = {0,1,2, 7,5,4,3};
    for(auto ind : to_insert){
        selected_inds.insert(to_insert);
        EXPECT(selected_inds.contains(ind));
    }

    selected_inds.removeEnt(0);
    auto old_inds = selected_inds.data;
    EXPECT(!selected_inds.contains(0));
    
    //! nothing should change when removing non-existing index
    selected_inds.removeEnt(0);
    auto new_inds = selected_inds.data;
    for(int i = 0; i < old_inds.size(); ++i){
        EXPECT(old_inds[i] == new_inds[i]);
    }

    
    selected_inds.insert(0);
    selected_inds.insert(0);
    EXPECT(selected_inds.noDuplicates());
    
    
}

