#include <omp.h>

#include "FogOfWar.h"

FogOfWarV2::FogOfWarV2(sf::Vector2i box_size, sf::Vector2f cell_size)
    : dy_(FOW::DY_VISION)
    , dx_(4 * cell_size.x)
    , player2vision_field_(N_PLAYERS)
    {}


//! \brief adds interval [ \p x_left, \p x_right ] to the overall intersection of visions in stripe \p stripe_ind
//! \param stripe_vec   vector containing data in one stripe
//! \param x_rleft
//! \param x_right
void FogOfWarV2::addTo(VisionField::StripeVec& stripe_vec, float x_left, float x_right)
{       
        if(std::abs(x_left - x_right) < FOW::DY_VISION/1000.f ){return;}
        if (stripe_vec.size() == 0 or x_left >= stripe_vec.back().x_right) {
            stripe_vec.push_back({x_left, x_right});
            return;
        }
        
        // auto first_low_left =
        //     std::lower_bound(stripe_vec.begin(), stripe_vec.end(), x_left,
        //                      [](const auto& p1, float to_find) { return p1.first < to_find; });

        // auto first_low_right =
        //     std::lower_bound(stripe_vec.begin(), stripe_vec.end(), x_right,
        //                      [](const auto& p1, float to_find) { return p1.first < to_find; });

        auto first_low_left = 
            std::find_if_not(stripe_vec.begin(), stripe_vec.end(), [&x_left](const auto& p1) { return p1.x_left < x_left; });
        auto first_low_right = 
            std::find_if_not(stripe_vec.begin(), stripe_vec.end(), [&x_right](const auto& p1) { return p1.x_left < x_right; });

        const auto ind_left = first_low_left - stripe_vec.begin() - 1;
        const auto ind_right = first_low_right - stripe_vec.begin() - 1;

        if (ind_right == -1) { //! the entire interval lies left from the left_most interval in stripe_vec
            stripe_vec.push_front({x_left, x_right});
            return;
        }
        
        first_low_right--;

        if (ind_left == -1) { 
            stripe_vec.at(0).x_left = x_left;
            bool right_is_in = x_right > first_low_right->x_left and x_right < first_low_right->x_right;
            if (!right_is_in) {
                stripe_vec.at(0).x_right = x_right;
            } else {
                stripe_vec.at(0).x_right = first_low_right->x_right;
            }
            if (first_low_right + 1 * (right_is_in) >= stripe_vec.begin() + 1) {
                stripe_vec.erase(stripe_vec.begin() + 1, first_low_right + 1 * (right_is_in));
            }
            assert(std::is_sorted(stripe_vec.begin(), stripe_vec.end(),
                                  [](const auto& p1, const auto& p2) {
                                      return p1.x_left < p2.x_left;
                                  }));

            return;
        }

        first_low_left--;

        bool left_is_in = x_left > first_low_left->x_left and x_left < first_low_left->x_right;
        bool right_is_in = x_right > first_low_right->x_left and x_right < first_low_right->x_right;

        if (first_low_left == first_low_right and !left_is_in and !right_is_in) {
            stripe_vec.insert(first_low_left + 1, {x_left, x_right});
            return;
        }

        if (!left_is_in) {
            first_low_right->x_left = x_left;
            first_low_right->x_right = std::max(x_right, first_low_right->x_right);
            if (first_low_right >= first_low_left + 1) {
                stripe_vec.erase(first_low_left + 1, first_low_right);
            }
        } else {
            first_low_left->x_right = std::max(x_right, first_low_right->x_right);
            if (first_low_right >= first_low_left) {
                stripe_vec.erase(first_low_left + 1, first_low_right + 1);
            }
        }
        assert(std::is_sorted(
            stripe_vec.begin(), stripe_vec.end(),
            [](const auto& p1, const auto& p2) { return p1.x_left < p2.x_left; }));
}

void FogOfWarV2::addToStripe(int player_ind, int stripe_ind, float x_left, float x_right) {
        auto& stripes = player2vision_field_.at(player_ind).stripes_;
        addTo(stripes.at(stripe_ind), x_left, x_right);
    }


//! \brief updates revealed_stripes_with current stripe
void FogOfWarV2::reveal() {

#pragma omp parallel num_threads(NUM_OMP_FOW_THREADS)
{
    for(int player_ind = 0; player_ind < N_PLAYERS; ++player_ind){
        const auto& stripes = player2vision_field_.at(player_ind).stripes_;
        auto& revealed_stripes = player2vision_field_.at(player_ind).revealed_stripes_;
#pragma omp for
        for (int stripe_ind = 0; stripe_ind < FOW::N_STRIPES; ++stripe_ind) {
            const auto& stripe = stripes.at(stripe_ind);
            const auto n_intervals = stripe.size();
            for (int i = 0; i < n_intervals; ++i) {
                addTo(revealed_stripes[stripe_ind], std::max({stripe.at(i).x_left, 0.f}), std::min({stripe.at(i).x_right, (float)Geometry::BOX[0]}));
            }
        }
    }
}
}

//! \brief draws a single stripe defined by \p stripe
//! \param intervals    representing vision
//! \param y_pos        y position of the stripe
//! \param vertices     that will be drawn
//! \param color        of the stripe
//! \param last         index of last drawn vertex in \p vertices
void FogOfWarV2::drawStripe(VisionField::StripeVec& intervals, float y_pos, sf::VertexArray& vertices, sf::Color color, int& last) {
        
        intervals.push_front({0,0});
        intervals.push_back({(float)Geometry::BOX[0], (float)Geometry::BOX[0]});
        const auto n_intervals = intervals.size();
        
        for (int i = 0; i < n_intervals-1; ++i) {
            auto x_left = intervals[i].x_right;
            auto x_right = intervals[i+1].x_left;
            vertices[last + i * 4 + 0] = {{x_left, y_pos}, color};
            vertices[last + i * 4 + 1] = {{x_right, y_pos}, color};
            vertices[last + i * 4 + 2] = {{x_right, y_pos + dy_}, color};
            vertices[last + i * 4 + 3] = {{x_left, y_pos + dy_}, color};
        }
        last += (n_intervals-1)*4;

        intervals.pop_back();
        intervals.pop_front();
}

void FogOfWarV2::draw(int player_ind, sf::RenderWindow& window) {
    auto& vf = player2vision_field_.at(player_ind); 
    auto& fow_vertices = vf.fow_vertices_;
    auto& revealed_fow_vertices = vf.revealed_fow_vertices_;
    fow_vertices.clear();
    fow_vertices.setPrimitiveType(sf::Quads);
    revealed_fow_vertices.clear();
    revealed_fow_vertices.setPrimitiveType(sf::Quads);
    int n_revealed_vertices  = 0;
    int n_fow_vertices  = 0;

    auto& stripes_ = vf.stripes_;
    auto& revealed_stripes_ = vf.revealed_stripes_;

    for (int stripe_ind = 0; stripe_ind < FOW::N_STRIPES; ++stripe_ind) {
        n_revealed_vertices       += 4*(revealed_stripes_.at(stripe_ind).size()+1);
        n_fow_vertices            += 4*(stripes_.at(stripe_ind).size()+1);
    }   

    revealed_fow_vertices.resize(n_revealed_vertices);
    fow_vertices.resize(n_fow_vertices);

    int last_ind_s = 0; 
    int last_ind_rs = 0;
    for (int stripe_ind = 0; stripe_ind < FOW::N_STRIPES; ++stripe_ind) {
        const float y_pos = stripe_ind*dy_;
        drawStripe(stripes_.at(stripe_ind), y_pos, fow_vertices, {grey_color}, last_ind_s);
        drawStripe(revealed_stripes_.at(stripe_ind), y_pos, revealed_fow_vertices, sf::Color::Black, last_ind_rs);
    }


    if(settings_.hasAttribute(FogOfWarSettings::Options::REVEAL)){
        window.draw(revealed_fow_vertices);
    }
    if(settings_.hasAttribute(FogOfWarSettings::Options::FOGOFWAR)){
        window.draw(fow_vertices);
    }
}

// //! \brief updates vision data given coordinates vision radii and player indices of agents 
// //! \param r_coords 
// //! \param radius
// //! \param active_inds
// //! \param player_inds 
// void FogOfWarV2::update( const std::vector<sf::Vector2f>& r_coords,
//                             const std::vector<float>& radius,
//                             const std::vector<int>& active_inds,
//                             const std::vector<int>& player_inds){
    

//     for (const auto ind : active_inds) {
//         int stripe_ind = std::max({0.f, std::floor(r_coords[ind].y / dy_)});
//         stripe_ind = std::min(stripe_ind,FOW::N_STRIPES-1);    
//         stripe2particle_data_[stripe_ind].push_back({r_coords[ind], radius[ind]*radius[ind], player_inds[ind]});
//     }

// #pragma omp parallel num_threads(NUM_OMP_FOW_THREADS)
// {
// #pragma omp for 
//     for (int stripe_ind_i = 0; stripe_ind_i < FOW::N_STRIPES; ++stripe_ind_i)
//         {
//         stripes_[stripe_ind_i].clear();

//         const int ind_max = std::min({FOW::N_STRIPES-1, stripe_ind_i + FOW::N_MAX_DELTA_STRIPEIND});
//         const int ind_min = std::max({0, stripe_ind_i - FOW::N_MAX_DELTA_STRIPEIND});

        
//         //! central stripe is done separately here
//         for(const auto& data : stripe2particle_data_[stripe_ind_i]){
//             const float delta_x = std::sqrt(data.radius_sq);
//             addToStripe(stripe_ind_i, data.r.x - delta_x, data.r.x + delta_x);
//         }

//         for(int delta_i = 1; delta_i <= FOW::N_MAX_DELTA_STRIPEIND; ++delta_i){   
//             const float dy = delta_i*FOW::DY_VISION;

//             int stripe_ind_left = stripe_ind_i - delta_i;
//             if(stripe_ind_left < 0){continue;} 
//             for(const auto& data : stripe2particle_data_[stripe_ind_left]){
//                 const float delta_x = std::sqrt(data.radius_sq - dy*dy);
//                 addToStripe(stripe_ind_i, data.r.x - delta_x, data.r.x + delta_x);
//             }

//             int stripe_ind_right = stripe_ind_i + delta_i;
//             if(stripe_ind_right >= FOW::N_STRIPES){continue;}
//             for(const auto& data : stripe2particle_data_[stripe_ind_right]){
//                 const float delta_x = std::sqrt(data.radius_sq - dy*dy);
//                 addToStripe(stripe_ind_i, data.r.x - delta_x, data.r.x + delta_x);
//             }
//         }
//     }
// } //! OMP_PARALLEL_END

//     std::for_each(stripe2particle_data_.begin(), stripe2particle_data_.end(), [](auto& stripe_data){
//         stripe_data.clear();
//     });
//     reveal();
// }

void FogOfWarV2::update( const std::vector<sf::Vector2f>& r_coords,
                            const std::vector<float>& radius,
                            const std::vector<int>& active_inds,
                            const std::vector<int>& player_inds){
    
    for (const auto ind : active_inds) {
        const int stripe_ind = std::floor(r_coords[ind].y / dy_);
        stripe2particle_data_[stripe_ind].push_back({r_coords[ind], radius[ind]*radius[ind], player_inds[ind]});
    }

size_t max_n_stripes = 0;

#pragma omp parallel num_threads(NUM_OMP_THREADS_FOW)
{   
    for(int player_ind = 0; player_ind < N_PLAYERS; ++player_ind){
    auto& vf = player2vision_field_.at(player_ind);
    #pragma omp for 
        for (int stripe_ind_i = 0; stripe_ind_i < FOW::N_STRIPES; ++stripe_ind_i)
         {
            vf.stripes_[stripe_ind_i].clear();
         }
    }
}

#pragma omp parallel num_threads(NUM_OMP_THREADS_FOW)
{   
    #pragma omp for 
        for (int stripe_ind_i = 0; stripe_ind_i < FOW::N_STRIPES; ++stripe_ind_i)
         {

            const int ind_max = std::min({FOW::N_STRIPES-1, stripe_ind_i + FOW::N_MAX_DELTA_STRIPEIND});
            const int ind_min = std::max({0, stripe_ind_i - FOW::N_MAX_DELTA_STRIPEIND});

            //! central stripe is done separately here because it does not have a counterpart
            for(const auto& data : stripe2particle_data_[stripe_ind_i]){
                const float delta_x = std::sqrt(data.radius_sq);
                addToStripe(data.player_ind, stripe_ind_i, data.r.x - delta_x, data.r.x + delta_x);
            }
            for(int delta_i = 1; delta_i <= FOW::N_MAX_DELTA_STRIPEIND; ++delta_i){   
                const float dy = delta_i*FOW::DY_VISION;

                int stripe_ind_left = stripe_ind_i - delta_i;
                if(stripe_ind_left < 0){continue;} 
                for(const auto& data : stripe2particle_data_[stripe_ind_left]){
                    const float delta_x = std::sqrt(data.radius_sq - dy*dy);
                    addToStripe(data.player_ind, stripe_ind_i, data.r.x - delta_x, data.r.x + delta_x);
                }

                int stripe_ind_right = stripe_ind_i + delta_i;
                if(stripe_ind_right >= FOW::N_STRIPES){continue;}
                for(const auto& data : stripe2particle_data_[stripe_ind_right]){
                    const float delta_x = std::sqrt(data.radius_sq - dy*dy);
                    addToStripe(data.player_ind, stripe_ind_i, data.r.x - delta_x, data.r.x + delta_x);
                }
            }
        }
    
} //! OMP_PARALLEL_END

        std::for_each(stripe2particle_data_.begin(), stripe2particle_data_.end(), [](auto& stripe_data){
            stripe_data.clear();
        });
        reveal();
    }
