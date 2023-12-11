//
// Created by smutekj on 15.08.23.
//

#ifndef BOIDS_SELECTION_H
#define BOIDS_SELECTION_H

#include "core.h"
#include "BoidControler.h"
#include "Grid.h"

class BoidsSelection {
    std::vector<int> selected_inds_;

    void select(const BoidWorld& world, sf::RectangleShape selection_rect) {
        const auto& r_coords = world.r_coords_;
        for (int i = 0; i < world.n_active_; ++i) {
            if (isInRect(r_coords[i], selection_rect.getPoint(0), selection_rect.getPoint(2))) {
                selected_inds_.push_back(i);
            }
        }
    }

    void cancel() { selected_inds_.clear(); }
};

class BuildingSelection {};

#endif // BOIDS_SELECTION_H
