#include <omp.h>

#include "FogOfWar.h"

FogOfWar::FogOfWar(sf::Vector2i box_size, sf::Vector2f cell_size)
    : dy_(FOW::DY_VISION)
    , dx_(4 * cell_size.x)
    , n_stripes(FOW::N_STRIPES)
    , stripes_(n_stripes)
    , stripe_data_(n_stripes)
    , stripe_ind2grid_1dind_(n_stripes)
    , stripe_ind2fow_data_(n_stripes)
    , stripe_ind2last_(n_stripes, 0)
    , visited(n_stripes, false) {
    vision.setFillColor({0, 0, 1, 69});
}

//! \brief produces union of vision rectangles from each agent in \p active_inds  
//! \param r_coords
//! \param active_inds
//! \note so far it is assumed that all agents share the same viison range
void FogOfWar::update(const std::vector<sf::Vector2f>& r_coords, const std::vector<int>& active_inds) {
        constexpr float radius = FOW::R_MAX_VISION;

        for (int i = 0; i < n_stripes; ++i) {
            stripes_[i].clear();
            stripe_data_[i].clear();
        }

        for (const auto boid_ind : active_inds) {
            const auto r = r_coords[boid_ind];
            const int stripe_ind = std::floor(r.y / dy_);

            const int di = std::floor(radius / dy_);
            const int i_max = std::min({stripe_ind + di, n_stripes - 1});
            const int i_min = std::max({stripe_ind - di, 0});

            for (int ind = i_min + 1; ind < i_max; ++ind) {
                const auto delta_i = std::abs(ind - stripe_ind);
                const auto delta_x = FOW::DELTAS.arr[delta_i]; // std::sqrt(radius * radius - (dy_ * delta_i) * (dy_ * delta_i));
                addToStripe(ind, r.x - delta_x, r.x + delta_x);
            }

        }
}



//! \brief adds interval [ \p x_left, \p x_right ] to the overall intersection of visions in stripe \p stripe_ind
//! \param stripe_ind
//! \param x_rleft
//! \param x_right
void FogOfWar::addToStripe(int stripe_ind, float x_left, float x_right) {
        auto& stripe = stripes_.at(stripe_ind);

        assert(std::is_sorted(
            stripe.begin(), stripe.end(),
            [](const std::pair<float, float>& p1, const std::pair<float, float>& p2) { return p1.first < p2.first; }));

        if (stripe.size() == 0 or x_left > stripe.back().second) {
            stripe.push_back({x_left, x_right});
            return;
        }

        auto first_low_left =
            std::lower_bound(stripe.begin(), stripe.end(), x_left,
                             [](const std::pair<float, float>& p1, float to_find) { return p1.first < to_find; });

        auto first_low_right =
            std::lower_bound(stripe.begin(), stripe.end(), x_right,
                             [](const std::pair<float, float>& p1, float to_find) { return p1.first < to_find; });

        const auto ind_left = first_low_left - stripe.begin() - 1;
        const auto ind_right = first_low_right - stripe.begin() - 1;

        if (ind_right == -1) {
            stripe.insert(stripe.begin(), {x_left, x_right});
            assert(std::is_sorted(stripe.begin(), stripe.end(),
                                  [](const std::pair<float, float>& p1, const std::pair<float, float>& p2) {
                                      return p1.first < p2.first;
                                  }));
            return;
        }
        
        first_low_right--;

        if (ind_left == -1) {
            stripe.at(0).first = x_left;
            bool right_is_in = x_right > first_low_right->first and x_right < first_low_right->second;
            if (!right_is_in) {
                stripe.at(0).second = x_right;
            } else {
                stripe.at(0).second = first_low_right->second;
            }
            if (first_low_right + 1 * (right_is_in) >= stripe.begin() + 1) {
                stripe.erase(stripe.begin() + 1, first_low_right + 1 * (right_is_in));
            }
            assert(std::is_sorted(stripe.begin(), stripe.end(),
                                  [](const std::pair<float, float>& p1, const std::pair<float, float>& p2) {
                                      return p1.first < p2.first;
                                  }));

            return;
        }

        first_low_left--;

        bool left_is_in = x_left > first_low_left->first and x_left < first_low_left->second;
        bool right_is_in = x_right > first_low_right->first and x_right < first_low_right->second;

        if (first_low_left == first_low_right and !left_is_in and !right_is_in) {
            stripe.insert(first_low_left + 1, {x_left, x_right});
            return;
        }

        if (!left_is_in) {
            first_low_right->first = x_left;
            first_low_right->second = std::max(x_right, first_low_right->second);
            if (first_low_right >= first_low_left + 1) {
                stripe.erase(first_low_left + 1, first_low_right);
            }
        } else {
            first_low_left->second = std::max(x_right, first_low_right->second);
            if (first_low_right >= first_low_left) {
                stripe.erase(first_low_left + 1, first_low_right + 1);
            }
        }
        assert(std::is_sorted(
            stripe.begin(), stripe.end(),
            [](const std::pair<float, float>& p1, const std::pair<float, float>& p2) { return p1.first < p2.first; }));
    }

void FogOfWar::addToStripe2(int stripe_ind, float x_left, float x_right, float y_min, float y_max) {

        auto& stripe = stripe_data_.at(stripe_ind);
        if (stripe.size() == 0 or x_left > stripe.back().max_x) {
            stripe.emplace_back(x_left, x_right, y_min, y_max);
            return;
        }

        auto first_low_left = std::lower_bound(stripe.begin(), stripe.end(), x_left,
                                               [](const StripeData& sd, float to_find) { return sd.min_x < to_find; });

        auto first_low_right = std::lower_bound(stripe.begin(), stripe.end(), x_right,
                                                [](const StripeData& sd, float to_find) { return sd.min_x < to_find; });

        const auto ind_left = first_low_left - stripe.begin() - 1;
        const auto ind_right = first_low_right - stripe.begin() - 1;

        if (ind_right == -1) {
            stripe.insert(stripe.begin(), {x_left, x_right, y_min, y_max});
            return;
        }

        first_low_right--;

        if (ind_left == -1) {
            stripe.at(0).min_x = x_left;
            bool right_is_in = x_right > first_low_right->min_x and x_right < first_low_right->max_x;
            if (!right_is_in) {
                stripe.at(0).max_x = x_right;
            } else {
                stripe.at(0).max_x = first_low_right->max_x;
            }
            if (first_low_right + 1 * (right_is_in) >= stripe.begin() + 1) {
                stripe.erase(stripe.begin() + 1, first_low_right + 1 * (right_is_in));
            }
            return;
        }

        first_low_left--;

        bool left_is_in = x_left > first_low_left->min_x and x_left < first_low_left->max_x;
        bool right_is_in = x_right > first_low_right->min_x and x_right < first_low_right->max_x;

        if (first_low_left == first_low_right and !left_is_in and !right_is_in) {
            stripe.insert(first_low_left + 1, {x_left, x_right, y_min, y_max});
            return;
        }

        if (!left_is_in) {
            first_low_right->min_x = x_left;
            first_low_right->max_x = std::max(x_right, first_low_right->max_x);
            if (first_low_right >= first_low_left + 1) {
                stripe.erase(first_low_left + 1, first_low_right);
            }
        } else {
            first_low_left->max_x = std::max(x_right, first_low_right->max_x);
            if (first_low_right >= first_low_left) {
                stripe.erase(first_low_left + 1, first_low_right + 1);
            }
        }
    }



void FogOfWar::update2(const std::vector<sf::Vector2f>& vision_data, const std::vector<int>& active_inds) {
        const float radius = 50 * RHARD;

        for (int i = 0; i < n_stripes; ++i) {
            stripes_[i].clear();
            stripe_data_[i].clear();
        }

        std::vector<std::vector<int>> stripe2grid_inds_;
        std::vector<std::vector<float>> stripe2data(n_stripes);

        std::vector<int> active_stripes;
        std::vector<bool> visited(n_stripes, false);

        std::vector<std::pair<float, float>> stripe2min_max_y(n_stripes, {MAXFLOAT, -MAXFLOAT});
        for (const auto boid_ind : active_inds) {
            const auto r = vision_data[boid_ind];
            const int stripe_ind = std::floor(r.y / dy_);
            // const int grid_1dind = std::floor(r.x / dx_);

            // stripe2grid_inds_.at(stripe_ind).push_back(grid_1dind);
            stripe2data.at(stripe_ind).push_back(r.x);
            stripe2min_max_y.at(stripe_ind).first = std::min({r.y, stripe2min_max_y.at(stripe_ind).first});
            stripe2min_max_y.at(stripe_ind).second = std::max({r.y, stripe2min_max_y.at(stripe_ind).second});
            if (!visited[stripe_ind]) {
                active_stripes.push_back(stripe_ind);
                visited[stripe_ind] = true;
            }
        }

        std::vector<FOWGridData> clusters;
        for (const auto& stripe_ind : active_stripes) {
            clusters.clear();
            auto& stripe_data = stripe2data[stripe_ind];
            std::sort(stripe_data.begin(), stripe_data.end());
            FOWGridData cluster;
            float x_left = stripe_data[0];
            float x_right = stripe_data[0];
            clusters.push_back({x_left, x_right});
            for (auto x : stripe_data) {
                if (x - clusters.back().x_max > 2 * dx_) {
                    clusters.push_back({x, x});
                } else {
                    clusters.back().x_max = x;
                }
            }
            for (const auto& cluster : clusters) {
                const int di = std::floor(radius / dy_);
                const int i_max = std::min({stripe_ind + di, n_stripes - 1});
                const int i_min = std::max({stripe_ind - di, 0});

                const auto di_max = i_max - stripe_ind;
                float y_real_min = stripe2min_max_y.at(stripe_ind).first - stripe_ind * dy_ + radius - di * dy_;
                float y_real_max = dy_ - y_real_min;

                const auto wtf_y = radius - y_real_max;
                const auto delta_x = std::sqrt(radius * radius - (wtf_y) * (wtf_y));
                // addToStripe2(i_max, r.x - delta_x, r.x + delta_x, 0, y_real_min);
                // addToStripe2(i_min, r.x - delta_x, r.x + delta_x, y_real_max, dy_);

                for (int ind = i_min + 1; ind < i_max; ++ind) {
                    const auto delta_i = ind - stripe_ind;
                    const auto delta_x = std::sqrt(radius * radius - (dy_ * delta_i) * (dy_ * delta_i));
                    addToStripe(ind, cluster.x_min - delta_x, cluster.x_max + delta_x);
                }
            }
        }
    }

void FogOfWar::update3(const std::vector<sf::Vector2f>& vision_data, const std::vector<int>& active_inds) {
    const float radius = 50 * RHARD;
    fucked = 0;
    total = 0;
    for (int i = 0; i < n_stripes; ++i) {
        stripes_[i].clear();
        stripe_data_[i].clear();
        visited[i] = false;
        stripe_ind2last_[i] = 0;
    }

    active_stripe_inds_.clear();

    for (const auto boid_ind : active_inds) {
        const auto r = vision_data[boid_ind];
        const int stripe_ind = std::floor(r.y / dy_);

        const int di = std::floor(radius / dy_);
        const int i_max = std::min({stripe_ind + di, n_stripes - 1});
        const int i_min = std::max({stripe_ind - di, 0});

        const auto di_max = i_max - stripe_ind;
        float y_real_min = r.y - stripe_ind * dy_ + radius - di * dy_;
        float y_real_max = dy_ - y_real_min;

        const auto wtf_y = radius - y_real_max;

        for (int ind = i_min + 1; ind < i_max; ++ind) {
            const auto delta_i = std::abs(ind - stripe_ind);
            const auto delta_x =
                FOW::DELTAS.arr[delta_i]; // std::sqrt(radius * radius - (dy_ * delta_i) * (dy_ * delta_i));
            // addToStripe(ind, r.x - delta_x, r.x + delta_x);
            stripe_ind2fow_data_[ind][stripe_ind2last_[ind]] = {r.x - delta_x, r.x + delta_x};
            stripe_ind2last_[ind]++;
        }

        for (int ind = i_min + 1; ind < i_max; ++ind) {
            if (!visited[ind]) {
                active_stripe_inds_.push_back(ind);
                visited[ind] = true;
            }
        }
    }
    // std::cout << "n_stripes = " << active_stripe_inds_.size() << "\n";
    const auto n_active_stripes = active_stripe_inds_.size();
#pragma omp parallel for
    for (int i = 0; i < n_active_stripes; ++i) {
        const auto stripe_ind = active_stripe_inds_[i];
        const auto& fow_data = stripe_ind2fow_data_[stripe_ind];
        // std::sort(fow_data.begin(), fow_data.end(), [](const FOWGridData& f1, const FOWGridData& f2) {
        //     return f1.x_max - f1.x_min > f2.x_max - f2.x_min;
        // });
        const auto last = stripe_ind2last_[stripe_ind];
        for (int fow_data_ind = 0; fow_data_ind < last; ++fow_data_ind) {
            const auto& [x_left, x_right] = fow_data[fow_data_ind];
            addToStripe(stripe_ind, x_left, x_right);
        }
    }

    // std::cout << "percentage of stripes fucked: " << static_cast<float>(fucked) / total * 100 << " %\n";
}


    
void FogOfWar::update4(const std::vector<sf::Vector2f>& vision_data, const std::vector<int>& active_inds) {
    const float radius = 50 * RHARD;
    fucked = 0;
    total = 0;
    for (int i = 0; i < n_stripes; ++i) {
        stripes_[i].clear();
        stripe_data_[i].clear();
    }

    std::fill(visited.begin(), visited.end(), false);
    std::fill(stripe_ind2last_.begin(), stripe_ind2last_.end(), 0);


    active_stripe_inds_.clear();

    for (const auto boid_ind : active_inds) {
        const auto r = vision_data[boid_ind];
        const int stripe_ind = std::floor(r.y / dy_);

        const int di = std::floor(radius / dy_);
        const int i_max = std::min({stripe_ind + di, n_stripes - 1});
        const int i_min = std::max({stripe_ind - di, 0});

        for (int ind = i_min + 1; ind < i_max; ++ind) {
            const auto delta_i = std::abs(ind - stripe_ind);
            const auto delta_x =
                FOW::DELTAS.arr[delta_i]; // std::sqrt(radius * radius - (dy_ * delta_i) * (dy_ * delta_i));
            // addToStripe(ind, r.x - delta_x, r.x + delta_x);
            stripe_ind2fow_data_[ind][stripe_ind2last_[ind]] = {r.x - delta_x, r.x + delta_x};
            stripe_ind2last_[ind]++;
        }

        for (int ind = i_min + 1; ind < i_max; ++ind) {
            if (!visited[ind]) {
                active_stripe_inds_.push_back(ind);
                visited[ind] = true;
            }
        }
    }
    // std::cout << "n_stripes = " << active_stripe_inds_.size() << "\n";
    const auto n_active_stripes = active_stripe_inds_.size();
    omp_set_num_threads(6);
#pragma omp parallel for
    for (int i = 0; i < n_active_stripes; ++i) {
        const auto stripe_ind = active_stripe_inds_[i];
        auto& fow_data = stripe_ind2fow_data_[stripe_ind];
        const auto last = stripe_ind2last_[stripe_ind];
        std::sort(fow_data.begin(), fow_data.begin() + last,
                  [](const FOWGridData& f1, const FOWGridData& f2) { return f1.x_min < f2.x_min; });
        float x_left_last = -MAXFLOAT;
        float x_right_last = -MAXFLOAT;
        for (int fow_data_ind = 0; fow_data_ind < last; ++fow_data_ind) {
            const auto& [x_left, x_right] = fow_data[fow_data_ind];
            if (x_left > x_right_last) {
                stripes_[stripe_ind].push_back({x_left, x_right});
                x_left_last = x_left;
                x_right_last = x_right;
                continue;
            }
            if (x_left < x_right_last and x_right > x_right_last) {
                x_right_last = x_right;
                stripes_[stripe_ind].back().second = x_right;
            }

            // addToStripe(stripe_ind, x_left, x_right);
        }
    }
}

void FogOfWar::draw(sf::RenderWindow& window) {
        int i = 0;
        fow_vertices_.clear();
        fow_vertices_.setPrimitiveType(sf::Quads);
         
        sf::RenderTexture wtf;
        wtf.create(500, 512);
        sf::Vertex vertex;
        int stripe_ind = 0;
        for (const auto& stripe : stripes_) {
            const auto last = fow_vertices_.getVertexCount();
            fow_vertices_.resize(fow_vertices_.getVertexCount() + 4 * stripe.size());

            const auto n_stripes = stripe.size();
            for (int i = 0; i < n_stripes; ++i) {
                // if(first){
                // }
                const auto x_left = stripe[i].first;
                const auto x_right = stripe[i].second;
                // wtf.create(x_right - x_left, dy_);
                // vertex.position = {x_left, stripe_ind * dy_};
                fow_vertices_[last + i * 4 + 0] = {{x_left, stripe_ind * dy_}, {grey_color}};
                fow_vertices_[last + i * 4 + 1] = {{x_right, stripe_ind * dy_}, {grey_color}};
                fow_vertices_[last + i * 4 + 2] = {{x_right, (stripe_ind + 1) * dy_}, {grey_color}};
                fow_vertices_[last + i * 4 + 3] = {{x_left, (stripe_ind + 1) * dy_}, {grey_color}};

                // memset(&fow_vertices_[last], , n_stripes)

                // fow_vertices_[last + i * 6 + 0].position = {x_left, stripe_ind * dy_};
                // fow_vertices_[last + i * 6 + 1].position = {x_right, stripe_ind * dy_};
                // fow_vertices_[last + i * 6 + 2].position = {x_right, (stripe_ind + 1) * dy_};
                // fow_vertices_[last + i * 6 + 3].position = {x_left, (stripe_ind + 1) * dy_};
                // fow_vertices_[last + i * 6 + 4].position = {x_right, (stripe_ind + 1) * dy_};
                // fow_vertices_[last + i * 6 + 5].position = {x_left, stripe_ind * dy_};

                // fow_vertices_[last + i * 6 + 0].color = {0, 0, 1, 69};
                // fow_vertices_[last + i * 6 + 1].color = {0, 0, 1, 69};
                // fow_vertices_[last + i * 6 + 2].color = {0, 0, 1, 69};
                // fow_vertices_[last + i * 6 + 3].color = {0, 0, 1, 69};
                // fow_vertices_[last + i * 6 + 4].color = {0, 0, 1, 69};
                // fow_vertices_[last + i * 6 + 5].color = {0, 0, 1, 69};
            }
            stripe_ind++;
            // for (const auto& [x_left, x_right] : stripe) {
            //     vision.setPosition(x_left, (i)*dy_);
            //     vision.setSize({x_right - x_left, dy_});
            //     window.draw(vision);
            // }
            // i++;
        }

        // wtf.draw(fow_vertices_);
        window.draw(fow_vertices_);
        // effect.apply(wtf, window);
        // for (const auto& stripe : stripe_data_) {
        //     for (const auto& [x_left, x_right, y_min, y_max] : stripe) {
        //         vision.setPosition(x_left, (i)*dy_ + y_min);
        //         vision.setSize({x_right - x_left, y_max - y_min});
        //         window.draw(vision);
        //     }
        //     i++;
        // }
    }