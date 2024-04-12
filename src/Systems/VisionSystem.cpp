#include "VisionSystem.h"
#include "../MapGrid.h"

VisionSystem::VisionSystem(ComponentID id) : System2(id)
{}

void VisionField::addToRevealedStripes(int stripe_ind, Interval interv)
{
    addTo(revealed_stripes_.at(stripe_ind), interv);
}

void VisionField::addToStripes(int stripe_ind, Interval interv)
{
    addTo(stripes_.at(stripe_ind), interv);
}

void VisionSystem::updateWallFromMap(const MapGrid& map){
    int n_x = map.n_cells_.x;
    int n_y = map.n_cells_.y;
    const auto dx = map.cell_size_.x;
    const auto dy = map.cell_size_.y;
    for(int stripe_ind = 0; stripe_ind < FOW::N_STRIPES; ++stripe_ind){
        wall_stripes.at(stripe_ind).clear();
    }

    VisionField::Interval wall_interv;
    for(int j = 0; j < n_y;  ++j){

        bool encountered_wall = false;
        for(int i = 0; i < n_x;  ++i){

            const auto cell_ind = i  + j * n_x;
            sf::Vector2f cell_pos = {i * dx, j * dy}; 
            const int stripe_ind = cell_pos.y / FOW::DY_VISION;
            if(map.tiles.at(cell_ind) == MapGrid::TileType::WALL){
                if(!encountered_wall){wall_interv.x_left = cell_pos.x; encountered_wall = true;} //! if we hit wall for the first time;
                
            }
            if(encountered_wall && map.tiles.at(cell_ind) != MapGrid::TileType::WALL){ // if we stepped from wall down to ground
                wall_interv.x_right = cell_pos.x + dx;
                player2vision_field_.at(0).addTo(wall_stripes.at(stripe_ind), wall_interv);
                encountered_wall = false;
                // wall_stripes.at(stripe_ind).push_back(wall_interv);
            }
        }
    }
    // for(int stripe_ind = 0; stripe_ind < FOW::N_STRIPES; ++stripe_ind){
    //     auto& stripes = wall_stripes.at(stripe_ind);
    //     std::sort(stripes.begin(), stripes.end(), [](const auto& i1, const auto& i2){
    //         return i1.x_left < i2.x_left;});
    // }

}

VisionField::Interval VisionSystem::cutIntervalByWalls(VisionField::Interval& interv, int stripe_ind){
    const auto& walls = wall_stripes.at(stripe_ind);
    VisionField::Interval new_interv = {-1,-1};
    auto x_center = (interv.x_left + interv.x_right)/2.f;
    for(const auto& wall : walls){
        if(wall.x_left > interv.x_left && wall.x_right < interv.x_right){
            new_interv = {wall.x_right, interv.x_right};
            interv.x_right = wall.x_left;
            break;
        }
        if(interv.x_left < wall.x_left && wall.x_left < interv.x_right){
            interv.x_right = wall.x_left;
            break;
        }
        if(interv.x_right < wall.x_right && wall.x_right < interv.x_right){
            interv.x_left = wall.x_right;
            break;
        }
        
    }
    return new_interv;
}

void VisionSystem::update()
{

    auto &comp_array = static_cast<VisionArray &>(*p_comps_);
        
        
    std::chrono::high_resolution_clock clock;
    auto t_start = clock.now();


    for (auto &comp : comp_array.components_)
    {
        comp.trans.r += comp.trans.vel * dt;
        comp.trans.angle += comp.trans.angle_vel * dt;

        const int stripe_ind = std::floor(comp.trans.r.y / dy_);
        player2stripe2particle_data_[comp.player_ind][stripe_ind].push_back({comp.trans.r, comp.vision_radius_sq});
    }

#pragma omp parallel num_threads(NUM_OMP_FOW_THREADS)
    {
        for (int player_ind = 0; player_ind < N_PLAYERS; ++player_ind)
        {
            auto &vf = player2vision_field_.at(player_ind);
#pragma omp for
            for (int stripe_ind_i = 0; stripe_ind_i < FOW::N_STRIPES; ++stripe_ind_i)
            {
                vf.stripes_[stripe_ind_i].clear();
            }
        }
    }

#pragma omp parallel num_threads(NUM_OMP_FOW_THREADS)
    {
        for (int player_ind = 0; player_ind < N_PLAYERS; ++player_ind)
        {
            auto &player_data = player2stripe2particle_data_.at(player_ind);
            auto &vision_field = player2vision_field_.at(player_ind);
#pragma omp for
            for (int stripe_ind_i = 0; stripe_ind_i < FOW::N_STRIPES; ++stripe_ind_i)
            {
                const auto &data_in_stripe = player_data.at(stripe_ind_i);
                const int ind_max = std::min({FOW::N_STRIPES - 1, stripe_ind_i + FOW::N_MAX_DELTA_STRIPEIND});
                const int ind_min = std::max({0, stripe_ind_i - FOW::N_MAX_DELTA_STRIPEIND});

                //! central stripe is done separately here because it does not have a counterpart
                for (const auto &data : data_in_stripe)
                {
                    const float delta_x = std::sqrt(data.radius_sq);
                    VisionField::Interval interv = {data.r.x - delta_x, data.r.x + delta_x};
                    auto new_interv = cutIntervalByWalls(interv, stripe_ind_i);
                    vision_field.addToStripes(stripe_ind_i, interv);
                    if(new_interv.x_left >= 0){
                        vision_field.addToStripes(stripe_ind_i, new_interv);
                    }
                }
                for (int delta_i = 1; delta_i <= FOW::N_MAX_DELTA_STRIPEIND; ++delta_i)
                {
                    const float dy = delta_i * FOW::DY_VISION;

                    int stripe_ind_left = stripe_ind_i - delta_i;
                    if (stripe_ind_left < 0)
                    {
                        continue;
                    }
                    for (const auto &data : player_data[stripe_ind_left])
                    {
                        if(data.radius_sq - dy * dy < 0) {continue;}
                        const float delta_x = std::sqrt(data.radius_sq - dy * dy);
                        VisionField::Interval interv = {data.r.x - delta_x, data.r.x + delta_x};
                        auto new_interv = cutIntervalByWalls(interv, stripe_ind_i);
                        vision_field.addToStripes(stripe_ind_i, interv);
                        if(new_interv.x_left >= 0){
                            vision_field.addToStripes(stripe_ind_i, new_interv);
                        }
                    }

                    int stripe_ind_right = stripe_ind_i + delta_i;
                    if (stripe_ind_right >= FOW::N_STRIPES)
                    {
                        continue;
                    }
                    for (const auto &data : player_data[stripe_ind_right])
                    {
                        if(data.radius_sq - dy * dy < 0) {continue;}
                        const float delta_x = std::sqrt(data.radius_sq - dy * dy);
                        VisionField::Interval interv = {data.r.x - delta_x, data.r.x + delta_x};
                        auto new_interv = cutIntervalByWalls(interv, stripe_ind_i);
                        vision_field.addToStripes(stripe_ind_i, interv);
                        if(new_interv.x_left >= 0){
                            vision_field.addToStripes(stripe_ind_i, new_interv);
                        }
                        // vision_field.addToStripes(stripe_ind_i, {data.r.x - delta_x, data.r.x + delta_x});
                    }
                }
            }

            std::for_each(player_data.begin(), player_data.end(), [](auto &stripe_data)
                          { stripe_data.clear(); });
        }
    } //! OMP_PARALLEL_END


    reveal(); 
    auto delta_t =  std::chrono::duration_cast<std::chrono::microseconds>(clock.now() - t_start);
    // std::cout << delta_t << "\n";
}


//! \brief draws a single stripe defined by \p stripe
//! \param intervals    representing vision
//! \param y_pos        y position of the stripe
//! \param vertices     that will be drawn
//! \param color        of the stripe
//! \param last         index of last drawn vertex in \p vertices
void VisionSystem::drawStripe(VisionField::StripeVec &intervals, float y_pos, sf::VertexArray &vertices, sf::Color color, int &last)
{

    intervals.push_front({0, 0});
    intervals.push_back({(float)Geometry::BOX[0], (float)Geometry::BOX[0]});
    const auto n_intervals = intervals.size();

    for (int i = 0; i < n_intervals - 1; ++i)
    {
        auto x_left = intervals[i].x_right;
        auto x_right = intervals[i + 1].x_left;
        vertices[last + i * 6 + 0] = {{x_left, y_pos}, color};
        vertices[last + i * 6 + 1] = {{x_right, y_pos}, color};
        vertices[last + i * 6 + 2] = {{x_right, y_pos + dy_}, color};

        vertices[last + i * 6 + 3] = {{x_right, y_pos + dy_}, color};
        vertices[last + i * 6 + 4] = {{x_left, y_pos + dy_}, color};
        vertices[last + i * 6 + 5] = vertices[last + i * 6 + 0];
    }
    last += (n_intervals - 1) * 6;

    intervals.pop_back();
    intervals.pop_front();
}

void VisionField::addTo(VisionField::StripeVec &stripe_vec, const Interval &interv)
{
    addTo(stripe_vec, interv.x_left, interv.x_right);
}

//! \brief adds interval [ \p x_left, \p x_right ] to the overall intersection of visions in stripe \p stripe_ind
//! \param stripe_vec   vector containing data in one stripe
//! \param x_rleft
//! \param x_right
void VisionField::addTo(VisionField::StripeVec &stripe_vec, float x_left, float x_right)
{
    if (std::abs(x_left - x_right) < FOW::DY_VISION / 1000.f)
    {
        return;
    }
    if (stripe_vec.size() == 0 or x_left >= stripe_vec.back().x_right)
    {
        stripe_vec.push_back({x_left, x_right});
        return;
    }
    auto first_low_left =
        std::find_if_not(stripe_vec.begin(), stripe_vec.end(), [&x_left](const auto &p1)
                         { return p1.x_left < x_left; });
    auto first_low_right =
        std::find_if_not(stripe_vec.begin(), stripe_vec.end(), [&x_right](const auto &p1)
                         { return p1.x_left < x_right; });

    const auto ind_left = first_low_left - stripe_vec.begin() - 1;
    const auto ind_right = first_low_right - stripe_vec.begin() - 1;

    if (ind_right == -1)
    { //! the entire interval lies left from the left_most interval in stripe_vec
        stripe_vec.push_front({x_left, x_right});
        return;
    }

    first_low_right--;

    if (ind_left == -1)
    {
        stripe_vec.at(0).x_left = x_left;
        bool right_is_in = x_right > first_low_right->x_left and x_right < first_low_right->x_right;
        if (!right_is_in)
        {
            stripe_vec.at(0).x_right = x_right;
        }
        else
        {
            stripe_vec.at(0).x_right = first_low_right->x_right;
        }
        if (first_low_right + 1 * (right_is_in) >= stripe_vec.begin() + 1)
        {
            stripe_vec.erase(stripe_vec.begin() + 1, first_low_right + 1 * (right_is_in));
        }
        assert(std::is_sorted(stripe_vec.begin(), stripe_vec.end(),
                              [](const auto &p1, const auto &p2)
                              {
                                  return p1.x_left < p2.x_left;
                              }));

        return;
    }

    first_low_left--;

    bool left_is_in = x_left > first_low_left->x_left and x_left < first_low_left->x_right;
    bool right_is_in = x_right > first_low_right->x_left and x_right < first_low_right->x_right;

    if (first_low_left == first_low_right and !left_is_in and !right_is_in)
    {
        stripe_vec.insert(first_low_left + 1, {x_left, x_right});
        return;
    }

    if (!left_is_in)
    {
        first_low_right->x_left = x_left;
        first_low_right->x_right = std::max(x_right, first_low_right->x_right);
        if (first_low_right >= first_low_left + 1)
        {
            stripe_vec.erase(first_low_left + 1, first_low_right);
        }
    }
    else
    {
        first_low_left->x_right = std::max(x_right, first_low_right->x_right);
        if (first_low_right >= first_low_left)
        {
            stripe_vec.erase(first_low_left + 1, first_low_right + 1);
        }
    }
    assert(std::is_sorted(
        stripe_vec.begin(), stripe_vec.end(),
        [](const auto &p1, const auto &p2)
        { return p1.x_left < p2.x_left; }));
}

void VisionSystem::reveal()
{
#pragma omp parallel num_threads(NUM_OMP_FOW_THREADS)
    {
        for (int player_ind = 0; player_ind < N_PLAYERS; ++player_ind)
        {
            const auto &stripes = player2vision_field_.at(player_ind).stripes_;
            auto &vision_field = player2vision_field_.at(player_ind);

#pragma omp for
            for (int stripe_ind = 0; stripe_ind < FOW::N_STRIPES; ++stripe_ind)
            {
                const auto &stripe = stripes.at(stripe_ind);
                const auto n_intervals = stripe.size();
                for (int i = 0; i < n_intervals; ++i)
                {
                    VisionField::Interval interv = {std::max({stripe.at(i).x_left, 0.f}), std::min({stripe.at(i).x_right, (float)Geometry::BOX[0]})};
                    vision_field.addToRevealedStripes(stripe_ind, interv);
                }
            }
        }
    }
}

// void VisionSystem::draw(sf::RenderTarget &window)
// {

//     auto &vf = player2vision_field_.at(selected_player_ind);
//     auto &fow_vertices = vf.fow_vertices_;
//     auto &revealed_fow_vertices = vf.revealed_fow_vertices_;
//     fow_vertices.clear();
//     fow_vertices.setPrimitiveType(sf::Quads);
//     revealed_fow_vertices.clear();
//     revealed_fow_vertices.setPrimitiveType(sf::Quads);
//     int n_revealed_vertices = 0;
//     int n_fow_vertices = 0;

//     auto &stripes_ = vf.stripes_;
//     auto &revealed_stripes_ = vf.revealed_stripes_;

//     for (int stripe_ind = 0; stripe_ind < FOW::N_STRIPES; ++stripe_ind)
//     {
//         n_revealed_vertices += 4 * (revealed_stripes_.at(stripe_ind).size() + 1);
//         n_fow_vertices += 4 * (stripes_.at(stripe_ind).size() + 1);
//     }

//     revealed_fow_vertices.resize(n_revealed_vertices);
//     fow_vertices.resize(n_fow_vertices);

//     int last_ind_s = 0;
//     int last_ind_rs = 0;
//     for (int stripe_ind = 0; stripe_ind < FOW::N_STRIPES; ++stripe_ind)
//     {
//         const float y_pos = stripe_ind * dy_;
//         drawStripe(stripes_.at(stripe_ind), y_pos, fow_vertices, {grey_color}, last_ind_s);
//         drawStripe(revealed_stripes_.at(stripe_ind), y_pos, revealed_fow_vertices, sf::Color::Black, last_ind_rs);
//     }

//     if (settings_.hasAttribute(FogOfWarSettings::Options::REVEAL))
//     {
//         window.draw(revealed_fow_vertices);
//     }
//     if (settings_.hasAttribute(FogOfWarSettings::Options::FOGOFWAR))
//     {
//         window.draw(fow_vertices);
//     }
// }
