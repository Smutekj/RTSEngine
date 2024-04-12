#include "core.h"
#include "ECS.h"

#include "Utils/GayVector.h"

struct Selection{

    GayVectorI<N_MAX_NAVIGABLE_BOIDS> selected_agents;

    void selectInRectangle(ECSystem& god, const sf::Vector2f lower_left, const sf::Vector2f upper_right,
                              int player_ind) {

    const auto& data = god.entity2shared_data;

    selected_agents.clear();
    for (const auto ent : god.active_entities_) {
        // if (player_ind != data.at(ind).transform) {
        //     continue;
        // }
        if (isInRect(data.at(ent.ind).transform.r, lower_left, upper_right)) {
            selected_agents.insert(ent.ind, ent.ind);
        }
    }
}

    const std::vector<int>& getSelectedInds() const{
        return selected_agents.data;
    }

    bool empty()const{
        return selected_agents.data.empty();
    }

    void clear(){
        selected_agents.clear();
    }

};
