#pragma once

#include "../core.h"
#include "../ECS.h"
#include "../Settings.h"

namespace FOW
{

#include <array>
#include <limits>

    //! constexpr signel iteration of newton Raphson
    float constexpr sqrtNewtonRaphson(float x, float curr, float prev)
    {
        return curr == prev ? curr : sqrtNewtonRaphson(x, 0.5f * (curr + x / curr), curr);
    }

    /*
     * Constexpr version of the square root
     * Return value:
     *   - For a finite and non-negative value of "x", returns an approximation for the square root of "x"
     *   - Otherwise, returns NaN
     *   - I stole this from stackoverflow! :D
     */
    float constexpr sqrt_static(float x)
    {
        return x >= 0 && x < std::numeric_limits<float>::infinity() ? sqrtNewtonRaphson(x, x, 0)
                                                                    : std::numeric_limits<float>::quiet_NaN();
    }

    static constexpr float DY_VISION = 2 * Geometry::CELL_SIZE;
    static constexpr int N_STRIPES = Geometry::BOX[1] / DY_VISION + 2; //! two is for ssafety :D
    static constexpr float R_MAX_VISION = 150;
    static constexpr int N_MAX_DELTA_STRIPEIND = static_cast<int>(R_MAX_VISION / DY_VISION);

    template <int N>
    struct DeltaY
    {
        constexpr DeltaY()
            : arr()
        {
            constexpr auto dy_sq = FOW::DY_VISION * FOW::DY_VISION;
            constexpr auto r_sq = FOW::R_MAX_VISION * FOW::R_MAX_VISION;
            for (auto i = 0; i != N; ++i)
            {
                arr[i] = sqrt_static(r_sq - i * i * dy_sq);
            }
        }
        float arr[N];
    };

    static constexpr auto DELTAS = DeltaY<N_MAX_DELTA_STRIPEIND>();
} //! Namespace FOW

struct VisionField
{

    //! \struct left and right coordinates of the interval that the stripe represents
    struct Interval
    {
        float x_left;
        float x_right;
    };

    typedef std::deque<Interval> StripeVec;
    std::array<StripeVec, FOW::N_STRIPES> stripes_;
    std::array<StripeVec, FOW::N_STRIPES> revealed_stripes_;

    sf::VertexArray fow_vertices_;          //! this holds vertices for drawing
    sf::VertexArray revealed_fow_vertices_; //! this holds vertices for drawing

    void addToRevealedStripes(int stripe_ind, Interval interv);

    void addToStripes(int stripe_ind, Interval interv);

private:
    void addTo(StripeVec &stripe_vec, const Interval &interv);
    void addTo(StripeVec &stripe_vec, float x_left, float x_right);
};

struct VisionComponent : Component<ComponentID::VISION>
{
    TransformComponent trans;
    float vision_radius_sq;
    int player_ind = 0;
};

struct VisionSystem : System2
{

    struct VisionData
    {
        sf::Vector2f r;
        float radius_sq;
    };

    int N_STRIPES;

    typedef ComponentArray<VisionComponent> VisionArray;

    std::array<VisionField, N_PLAYERS> player2vision_field_;
    std::array<std::array<std::vector<VisionData>, FOW::N_STRIPES>, N_PLAYERS> player2stripe2particle_data_; //! https://www.youtube.com/watch?v=lD_ag67tH3I

    float dy_ = FOW::DY_VISION;

    int selected_player_ind = 0;
    FogOfWarSettings settings_;

public:
    VisionSystem(ComponentID id);
    ~VisionSystem() = default;

    void draw(sf::RenderTarget &target);
    void setTransform(const int &ind, const TransformComponent &trans)
    {
        auto &comps = static_cast<ComponentArray<VisionComponent> &>(*p_comps_.get()).components_;
        comps.at(entity2compvec_ind_.at(ind)).trans = trans;
    }
    void updateSharedData(const std::array<SharedData, N_MAX_ENTITIES>& new_data, const std::vector<Entity>& active_entity_inds)
    {
        auto &comps = static_cast<ComponentArray<VisionComponent> &>(*p_comps_.get()).components_;
        for(const auto ent : active_entity_inds){
            const auto compvec_ind = entity2compvec_ind_.at(ent.ind);
            comps.at(compvec_ind).trans = new_data.at(ent.ind).transform;
            // comps.at(compvec_ind).state = new_data.at(ent_ind).state;
        }
    }
    virtual void communicate(std::array<SharedData, N_MAX_ENTITIES>& entity2shared_data)const{
        
    }


    void update();

    void reveal();

    bool isVisibleBy(int player_ind, sf::Vector2f r) const
    {
        const auto &vf = player2vision_field_.at(player_ind);
        const int stripe_ind = std::floor(r.y / dy_);
        const auto& stripes = vf.stripes_.at(stripe_ind);
        for (const auto &stripe : stripes)
        {
            if (stripe.x_left <= r.x && r.x < stripe.x_right)
            {
                return true;
            }
        }
        return false;
    }

private:
    void addOnGrid(VisionComponent &vc)
    {
    }
    void drawStripe(VisionField::StripeVec &intervals, float y_pos, sf::VertexArray &vertices, sf::Color color, int &last);

private:
    sf::Color grey_color = {0, 0, 1, 69};
};
