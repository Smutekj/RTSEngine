#ifndef BOIDS_FOGOFWAR_H
#define BOIDS_FOGOFWAR_H


#include "core.h"
#include "BloomEffect.hpp"
#include "Geometry.h"
#include "Settings.h"
#include <iostream>
#include <deque>

namespace FOW {

#include <array>
#include <limits>

//! constexpr signel iteration of newton Raphson
float constexpr sqrtNewtonRaphson(float x, float curr, float prev) {
    return curr == prev ? curr : sqrtNewtonRaphson(x, 0.5f * (curr + x / curr), curr);
}

/*
 * Constexpr version of the square root
 * Return value:
 *   - For a finite and non-negative value of "x", returns an approximation for the square root of "x"
 *   - Otherwise, returns NaN
 *   - I stole this from stackoverflow! :D
 */ 
float constexpr sqrt_static(float x) {
    return x >= 0 && x < std::numeric_limits<float>::infinity() ? sqrtNewtonRaphson(x, x, 0)
                                                                : std::numeric_limits<float>::quiet_NaN();
}

static constexpr float DY_VISION = 2*Geometry::CELL_SIZE;
static constexpr int N_STRIPES = Geometry::BOX[1] / DY_VISION;
static constexpr float R_MAX_VISION = 150;
static constexpr int N_MAX_DELTA_STRIPEIND = static_cast<int>(R_MAX_VISION / DY_VISION);

template <int N> struct DeltaY {
    constexpr DeltaY()
        : arr() {
        constexpr auto dy_sq = FOW::DY_VISION * FOW::DY_VISION;
        constexpr auto r_sq = FOW::R_MAX_VISION * FOW::R_MAX_VISION;
        for (auto i = 0; i != N; ++i) {
            arr[i] = sqrt_static(r_sq - i * i * dy_sq);
        }
    }
    float arr[N];
};

static constexpr auto DELTAS = DeltaY<N_MAX_DELTA_STRIPEIND>();
} //! Namespace FOW


struct VisionData {
    sf::Vector2f r;
    float radius_sq;
    int player_ind = 0;
};


struct VisionField {

    //! \struct left and right coordinates of the interval that the stripe 
    struct Interval {
        float x_left;
        float x_right;
    };

    typedef std::deque<Interval> StripeVec;
    std::array<StripeVec, FOW::N_STRIPES> stripes_;
    std::array<StripeVec, FOW::N_STRIPES> revealed_stripes_;

    sf::VertexArray fow_vertices_;  //! this holds vertices for drawing
    sf::VertexArray revealed_fow_vertices_;  //! this holds vertices for drawing
};

class FogOfWarV2 {

  public:
    FogOfWarSettings settings_;

    std::vector<VisionField> player2vision_field_;

    std::array<std::vector<VisionData>, FOW::N_STRIPES> stripe2particle_data_;

    FogOfWarV2(sf::Vector2i box_size, sf::Vector2f cell_size);

    void update(const std::vector<sf::Vector2f>& r_coords,
                const std::vector<float>& radius,
                const std::vector<int>& active_inds,
                const std::vector<int>& player_inds);
    void update2(const std::vector<sf::Vector2f>& r_coords,
                const std::vector<float>& radius,
                const std::vector<int>& active_inds,
                const std::vector<int>& player_inds);

    void addToStripe(int player_ind, int stripe_ind, float x_left, float x_right);
    void addToRevealed(int player_ind, int stripe_ind, float x_left, float x_right);


    bool isVisibleBy(int player_ind, const sf::Vector2f& r) const {
        const int stripe_ind = std::floor(r.y / dy_);
        const auto& stripes =  player2vision_field_.at(player_ind).stripes_;
        const auto& stripe = stripes.at(stripe_ind);

        int n_in_vision = 0;
        for (const auto& [left, right] : stripe) {
            n_in_vision += (left < r.x and r.x <= right);
        }
        return n_in_vision == 1;
    }
    
    void draw(int player_ind, sf::RenderWindow& window);
private:
    void addTo(VisionField::StripeVec& stripe_vec, float x_left, float x_right);
    void reveal() ;


    void drawStripe(VisionField::StripeVec& intervals, float y_pos, sf::VertexArray& vertices, sf::Color color, int& last);

    sf::Color grey_color = {0, 0, 1, 69};
    float dy_;
    float dx_;
};

#endif //BOIDS_FOGOFWAR_H