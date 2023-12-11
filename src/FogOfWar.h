#include "core.h"
#include "BloomEffect.hpp"
#include "Geometry.h"

namespace FOW {

#include <array>
#include <limits>

//!
namespace Detail {
float constexpr sqrtNewtonRaphson(float x, float curr, float prev) {
    return curr == prev ? curr : sqrtNewtonRaphson(x, 0.5f * (curr + x / curr), curr);
}
} // namespace Detail

/*
 * Constexpr version of the square root
 * Return value:
 *   - For a finite and non-negative value of "x", returns an approximation for the square root of "x"
 *   - Otherwise, returns NaN
 *   - I stole this from stackoverflow! :D
 */ 
float constexpr sqrt_static(float x) {
    return x >= 0 && x < std::numeric_limits<float>::infinity() ? Detail::sqrtNewtonRaphson(x, x, 0)
                                                                : std::numeric_limits<float>::quiet_NaN();
}

static constexpr float DY_VISION = 2*Geometry::CELL_SIZE;
static constexpr int N_STRIPES = Geometry::BOX[1] / DY_VISION;
static constexpr float R_MAX_VISION = 50;
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

} // namespace FOW


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

    sf::VertexArray fow_vertices_;

    FogOfWar(sf::Vector2i box_size, sf::Vector2f cell_size);

    void update(const std::vector<sf::Vector2f>& vision_data, const std::vector<int>& active_inds);
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

    // void initializeGridData(const std::vector<VisionData>& vision_data) {

    //     for (const auto& [r, radius] : vision_data) {
    //         const int stripe_ind = std::floor(r.y / dy_);
    //         const int grid_1dind = std::floor(r.x / dx_);

    //         const auto di_max = std::ceil(radius / dy_);
    //         for (int i = -di_max; i <= di_max; ++i) {
    //             const auto delta_x = std::sqrt(radius * radius - (dy_ * i) * (dy_ * i));
    //             addToStripe(stripe_ind + i, r.x - delta_x, r.x + delta_x);
    //         }
    //     }
    // }

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
