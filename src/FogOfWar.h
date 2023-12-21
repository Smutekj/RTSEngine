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

class FogOfWar {

    BloomEffect effect;

  public:
    float dy_;
    float dx_;
    int n_stripes;

    struct StripeData {
        float min_x;
        float max_x;
        float min_y;
        float max_y;

        StripeData() = default;
        StripeData(float x_min, float x_max, float y_min, float y_max)
            : min_x(x_min)
            , max_x(x_max)
            , min_y(y_min)
            , max_y(y_max) {}
    };

    std::vector<std::vector<std::pair<float, float>>> stripes_;
    std::vector<std::vector<StripeData>> stripe_data_;

    std::vector<std::vector<int>> stripe_ind2grid_1dind_;
    
    sf::RectangleShape vision;
    std::vector<bool> visited;

    struct FOWGridData {
        float x_min;
        float x_max;
    };

    std::vector<std::array<FOWGridData, 500>> stripe_ind2fow_data_;
    std::vector<int> stripe_ind2last_;
    std::vector<int> active_stripe_inds_; 

    sf::VertexArray fow_vertices_;  //! this holds vertices for drawing

    FogOfWar(sf::Vector2i box_size, sf::Vector2f cell_size);

    void update(const std::vector<sf::Vector2f>& vision_data, const std::vector<float>& radii, const std::vector<int>& active_inds);
    void update2(const std::vector<sf::Vector2f>& vision_data, const std::vector<int>& active_inds);
    void update3(const std::vector<sf::Vector2f>& vision_data, const std::vector<int>& active_inds);
    void update4(const std::vector<sf::Vector2f>& vision_data, const std::vector<int>& active_inds);

    void addToStripe2(int stripe_ind, float x_left, float x_right, float y_min, float y_max) ;
    unsigned long long fucked = 0;
    unsigned long long total = 0;

    void addToStripe(int stripe_ind, float x_left, float x_right);
    struct VisionData {
        sf::Vector2f position;
        float radius;
    };

    bool isVisible(const sf::Vector2f& r) const {
        const auto stripe_ind = std::floor(r.y / dy_);
        const auto& stripe = stripes_.at(stripe_ind);

        int n_in_vision = 0;
        for (const auto& [left, right] : stripe) {
            n_in_vision += (left < r.x and r.x < right);
        }
        return n_in_vision == 1;
    }

    sf::Color grey_color = {0, 0, 1, 69};

    void draw(sf::RenderWindow& window);
};



class FogOfWarV2 {

  public:
    FogOfWarSettings settings_;

    struct StripeData {
        float y;
        int next;
    };

    typedef std::deque<std::pair<float, float>> StripeVec;
    std::array<StripeVec, FOW::N_STRIPES> stripes_;
    std::array<StripeVec, FOW::N_STRIPES> revealed_stripes_;

    sf::VertexArray fow_vertices_;  //! this holds vertices for drawing
    sf::VertexArray revealed_fow_vertices_;  //! this holds vertices for drawing

    FogOfWarV2(sf::Vector2i box_size, sf::Vector2f cell_size);

    struct VisionData {
        sf::Vector2f r;
        float radius_sq;
        int player_ind = 0;
    };

    std::array<std::vector<VisionData>, FOW::N_STRIPES> stripe2particle_data_;
    std::array<int, FOW::N_STRIPES> stripe2;


    void update(const std::vector<sf::Vector2f>& r_coords,
                const std::vector<float>& radius,
                const std::vector<int>& active_inds,
                const std::vector<int>& player_inds);
    void update2(const std::vector<sf::Vector2f>& r_coords,
                const std::vector<float>& radius,
                const std::vector<int>& active_inds,
                const std::vector<int>& player_inds);
    void addTo(StripeVec& stripe_vec, float x_left, float x_right);
    
    void reveal() ;

    bool isVisible(const sf::Vector2f& r) const {
        const int stripe_ind = std::floor(r.y / dy_);
        const auto& stripe = stripes_.at(stripe_ind);

        int n_in_vision = 0;
        for (const auto& [left, right] : stripe) {
            n_in_vision += (left < r.x and r.x <= right);
        }
        return n_in_vision == 1;
    }
    
    void draw(sf::RenderWindow& window);
private:
    void addToRevealed(int stripe_ind, float x_left, float x_right);
    void addToStripe(int stripe_ind, float x_left, float x_right);
    void drawStripe(FogOfWarV2::StripeVec& stripe, int stripe_ind, sf::VertexArray& vertices, sf::Color color); 

    sf::Color grey_color = {0, 0, 1, 69};
    float dy_;
    float dx_;
};

#endif //BOIDS_FOGOFWAR_H