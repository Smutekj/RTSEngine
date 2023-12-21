#include <omp.h>

#include "FogOfWar.h"

FogOfWarV2::FogOfWarV2(sf::Vector2i box_size, sf::Vector2f cell_size)
    : dy_(FOW::DY_VISION)
    , dx_(4 * cell_size.x)
    {}


//! \brief adds interval [ \p x_left, \p x_right ] to the overall intersection of visions in stripe \p stripe_ind
//! \param stripe_vec   vector containing data in one stripe
//! \param x_rleft
//! \param x_right
void FogOfWarV2::addTo(FogOfWarV2::StripeVec& stripe_vec, float x_left, float x_right)
{
        if (stripe_vec.size() == 0 or x_left > stripe_vec.back().second) {
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
            std::find_if_not(stripe_vec.begin(), stripe_vec.end(), [&x_left](const auto& p1) { return p1.first < x_left; });
        auto first_low_right = 
            std::find_if_not(stripe_vec.begin(), stripe_vec.end(), [&x_right](const auto& p1) { return p1.first < x_right; });

        const auto ind_left = first_low_left - stripe_vec.begin() - 1;
        const auto ind_right = first_low_right - stripe_vec.begin() - 1;

        if (ind_right == -1) { //! the entire interval lies left from the left_most interval in stripe_vec
            stripe_vec.push_front({x_left, x_right});
            return;
        }
        
        first_low_right--;

        if (ind_left == -1) { 
            stripe_vec.at(0).first = x_left;
            bool right_is_in = x_right > first_low_right->first and x_right < first_low_right->second;
            if (!right_is_in) {
                stripe_vec.at(0).second = x_right;
            } else {
                stripe_vec.at(0).second = first_low_right->second;
            }
            if (first_low_right + 1 * (right_is_in) >= stripe_vec.begin() + 1) {
                stripe_vec.erase(stripe_vec.begin() + 1, first_low_right + 1 * (right_is_in));
            }
            assert(std::is_sorted(stripe_vec.begin(), stripe_vec.end(),
                                  [](const auto& p1, const auto& p2) {
                                      return p1.first < p2.first;
                                  }));

            return;
        }

        first_low_left--;

        bool left_is_in = x_left > first_low_left->first and x_left < first_low_left->second;
        bool right_is_in = x_right > first_low_right->first and x_right < first_low_right->second;

        if (first_low_left == first_low_right and !left_is_in and !right_is_in) {
            stripe_vec.insert(first_low_left + 1, {x_left, x_right});
            return;
        }

        if (!left_is_in) {
            first_low_right->first = x_left;
            first_low_right->second = std::max(x_right, first_low_right->second);
            if (first_low_right >= first_low_left + 1) {
                stripe_vec.erase(first_low_left + 1, first_low_right);
            }
        } else {
            first_low_left->second = std::max(x_right, first_low_right->second);
            if (first_low_right >= first_low_left) {
                stripe_vec.erase(first_low_left + 1, first_low_right + 1);
            }
        }
        assert(std::is_sorted(
            stripe_vec.begin(), stripe_vec.end(),
            [](const auto& p1, const auto& p2) { return p1.first < p2.first; }));
}

void FogOfWarV2::addToStripe(int stripe_ind, float x_left, float x_right) {
        addTo(stripes_.at(stripe_ind), x_left, x_right);
    }


//! \brief updates revealed_stripes_with current stripe
    void FogOfWarV2::reveal() {
        for (int stripe_ind = 0; stripe_ind < FOW::N_STRIPES; ++stripe_ind) {
            const auto& stripe = stripes_.at(stripe_ind);
            const auto n_intervals = stripe.size();
            for (int i = 0; i < n_intervals; ++i) {
               addTo(revealed_stripes_[stripe_ind], stripe[i].first, stripe[i].second);
            }
        }
    }

//! \brief draws a single stripe defined by \p stripe
void FogOfWarV2::drawStripe(FogOfWarV2::StripeVec& stripe, int stripe_ind, sf::VertexArray& vertices, sf::Color color) {
        
        stripe.push_front({0,0});
        stripe.push_back({Geometry::BOX[0], Geometry::BOX[0]});
        const auto n_intervals = stripe.size();

        const auto last = vertices.getVertexCount();
        vertices.resize(last + 4*(n_intervals-1));

        for (int i = 0; i < n_intervals-1; ++i) {
            auto x_left = stripe[i].second;
            auto x_right = stripe[i+1].first;
            vertices[last + i * 4 + 0] = {{x_left, stripe_ind * dy_}, color};
            vertices[last + i * 4 + 1] = {{x_right, stripe_ind * dy_}, color};
            vertices[last + i * 4 + 2] = {{x_right, (stripe_ind + 1) * dy_}, color};
            vertices[last + i * 4 + 3] = {{x_left, (stripe_ind + 1) * dy_}, color};
        }
        stripe.pop_back();
        stripe.pop_front();
}

void FogOfWarV2::draw(sf::RenderWindow& window) {
    fow_vertices_.clear();
    fow_vertices_.setPrimitiveType(sf::Quads);
    revealed_fow_vertices_.clear();
    revealed_fow_vertices_.setPrimitiveType(sf::Quads);

    for (int stripe_ind = 0; stripe_ind < FOW::N_STRIPES; ++stripe_ind) {
        drawStripe(stripes_.at(stripe_ind), stripe_ind, fow_vertices_, {grey_color});
        drawStripe(revealed_stripes_.at(stripe_ind), stripe_ind, revealed_fow_vertices_, sf::Color::Black);
    }

    if(settings_.hasAttribute(FogOfWarSettings::Options::REVEAL)){
        window.draw(revealed_fow_vertices_);
    }
    if(settings_.hasAttribute(FogOfWarSettings::Options::FOGOFWAR)){
        window.draw(fow_vertices_);
    }
}

void FogOfWarV2::update( const std::vector<sf::Vector2f>& r_coords,
                            const std::vector<float>& radius,
                            const std::vector<int>& active_inds,
                            const std::vector<int>& player_inds){
    
    for (const auto ind : active_inds) {
        const int stripe_ind = std::floor(r_coords[ind].y / dy_);
        stripe2particle_data_[stripe_ind].push_back({r_coords[ind], radius[ind]*radius[ind], player_inds[ind]});
    }

#pragma omp parallel num_threads(4)
{
#pragma omp for 
    for (int stripe_ind_i = 0; stripe_ind_i < FOW::N_STRIPES; ++stripe_ind_i)
        {
        stripes_[stripe_ind_i].clear();

        const int ind_max = std::min({FOW::N_STRIPES-1, stripe_ind_i + FOW::N_MAX_DELTA_STRIPEIND});
        const int ind_min = std::max({0, stripe_ind_i - FOW::N_MAX_DELTA_STRIPEIND});

        
        //! central stripe is done separately here
        for(const auto& data : stripe2particle_data_[stripe_ind_i]){
            const float delta_x = std::sqrt(data.radius_sq);
            addToStripe(stripe_ind_i, data.r.x - delta_x, data.r.x + delta_x);
        }

        for(int delta_i = 1; delta_i <= FOW::N_MAX_DELTA_STRIPEIND; ++delta_i){   
            const float dy = delta_i*FOW::DY_VISION;

            int stripe_ind_left = stripe_ind_i - delta_i;
            if(stripe_ind_left < 0){continue;} 
            for(const auto& data : stripe2particle_data_[stripe_ind_left]){
                const float delta_x = std::sqrt(data.radius_sq - dy*dy);
                addToStripe(stripe_ind_i, data.r.x - delta_x, data.r.x + delta_x);
            }

            int stripe_ind_right = stripe_ind_i + delta_i;
            if(stripe_ind_right >= FOW::N_STRIPES){continue;}
            for(const auto& data : stripe2particle_data_[stripe_ind_right]){
                const float delta_x = std::sqrt(data.radius_sq - dy*dy);
                addToStripe(stripe_ind_i, data.r.x - delta_x, data.r.x + delta_x);
            }
            
        }
    }
} //! OMP_PARALLEL_END


    std::for_each(stripe2particle_data_.begin(), stripe2particle_data_.end(), [](auto& stripe_data){
        stripe_data.clear();
    });
    reveal();
}

void insertionSort(std::vector<std::pair<float, float>>& arr, int n)
{
    int i, key, j;
    for (i = 1; i < n; i++) {
        key = arr[i].first;
        j = i - 1;
 
        // Move elements of arr[0..i-1],
        // that are greater than key, to one
        // position ahead of their
        // current position
        while (j >= 0 && arr[j].first > key) {
            arr[j + 1] = arr[j];
            j = j - 1;
        }
        arr[j + 1].first = key;
    }
}


    void FogOfWarV2::update2( const std::vector<sf::Vector2f>& r_coords,
                             const std::vector<float>& radius,
                             const std::vector<int>& active_inds,
                             const std::vector<int>& player_inds){
        
        for (const auto ind : active_inds) {
            const int stripe_ind = std::floor(r_coords[ind].y / dy_);
            stripe2particle_data_[stripe_ind].push_back({r_coords[ind], radius[ind]*radius[ind], player_inds[ind]});
        }

size_t max_n_stripes = 0;

#pragma omp parallel num_threads(4)
{
    #pragma omp for 
        for (int stripe_ind_i = 0; stripe_ind_i < FOW::N_STRIPES; ++stripe_ind_i)
         {
            stripes_[stripe_ind_i].clear();

            const int ind_max = std::min({FOW::N_STRIPES-1, stripe_ind_i + FOW::N_MAX_DELTA_STRIPEIND});
            const int ind_min = std::max({0, stripe_ind_i - FOW::N_MAX_DELTA_STRIPEIND});

            //! central stripe is done separately here
            for(const auto& data : stripe2particle_data_[stripe_ind_i]){
                const float delta_x = std::sqrt(data.radius_sq);
                addToStripe(stripe_ind_i, data.r.x - delta_x, data.r.x + delta_x);
            }
            for(int delta_i = 1; delta_i <= FOW::N_MAX_DELTA_STRIPEIND; ++delta_i){   
                const float dy = delta_i*FOW::DY_VISION;

                int stripe_ind_left = stripe_ind_i - delta_i;
                if(stripe_ind_left < 0){continue;} 
                for(const auto& data : stripe2particle_data_[stripe_ind_left]){
                    const float delta_x = std::sqrt(data.radius_sq - dy*dy);
                    addToStripe(stripe_ind_i, data.r.x - delta_x, data.r.x + delta_x);
                }

                int stripe_ind_right = stripe_ind_i + delta_i;
                if(stripe_ind_right >= FOW::N_STRIPES){continue;}
                for(const auto& data : stripe2particle_data_[stripe_ind_right]){
                    const float delta_x = std::sqrt(data.radius_sq - dy*dy);
                    addToStripe(stripe_ind_i, data.r.x - delta_x, data.r.x + delta_x);
                }
               
            }
        }
} //! OMP_PARALLEL_END

        std::for_each(stripe2particle_data_.begin(), stripe2particle_data_.end(), [](auto& stripe_data){
            stripe_data.clear();
        });
        reveal();
    }
