#include "BoidWorld.h"
#include "BoidControler.h"

//! \brief updates positions of agents that are not HOLDING
//! \param dt time step
void BoidWorld::update(float dt) {
    for (const auto i : active_inds) {
        auto v_tot =  velocities_[i] + intertia_velocities_[i];
        bool is_holding = (move_states_[i] == MoveState::HOLDING);
        if(is_holding){
            v_tot -= velocities_[i];
        }
        r_coords_[i] += (v_tot) * (dt * !is_holding);
    }

    //    for(auto wtf : changed_cell){
    //        auto new_cell_ind = std::get<0>(wtf);
    //        auto i = std::get<1>(wtf);
    //
    //        auto& old_cell_boid_inds_ = cell2boid_inds_[boid_inds2cellind_[i]];
    //        auto last = old_cell_boid_inds_.size()-1;
    //        old_cell_boid_inds_[boid_inds2ind_in_cell_[i]] = old_cell_boid_inds_[last];
    //        boid_inds2ind_in_cell_[old_cell_boid_inds_[last]] = boid_inds2ind_in_cell_[i];
    //        old_cell_boid_inds_.pop_back();
    //
    //        auto& new_cell_boid_inds_ = cell2boid_inds_[new_cell_ind];
    //        new_cell_boid_inds_.push_back(i);
    //        boid_inds2cellind_[i] = new_cell_ind;
    //        boid_inds2ind_in_cell_[i] = new_cell_boid_inds_.size()-1;
    //    }
}


//! \brief activates new agent
//! \param player_ind   index of player controlling the agent
//! \param r_coord      position of agent
//! \param vel          velocity of agent 
//! \returns            boid_ind of the new agent
int BoidWorld::activate(int player_ind, sf::Vector2f r_coord, sf::Vector2f vel) {

    n_alive_of_players_.at(player_ind)++;
    n_active_++;
    auto new_boid_ind = last_active_ind + 1;

    if (!inactive_inds.empty()) {
        new_boid_ind = inactive_inds.back();
        inactive_inds.pop_back();
    } else {
        last_active_ind++;
    }

    r_coords_[new_boid_ind] = r_coord;
    velocities_[new_boid_ind] = vel;
    move_states_[new_boid_ind] = MoveState::STANDING;

    ind2player[new_boid_ind] = player_ind;
    active_inds.push_back(new_boid_ind);
    ind_in_active_inds[new_boid_ind] = active_inds.size() - 1;

    return new_boid_ind;
}

//! \brief deactivates existing agent (when it dies e.g.)
//! \param boid_ind    index of player controlling the agent
void BoidWorld::deactivate(int boid_ind) {
    n_active_--;
    const auto player_ind = ind2player[boid_ind];
    //    r_coords_[boid_ind] = r_coords_[last];
    //    velocities_[boid_ind] = velocities_[last];
    //    boid_inds2draw_data_[boid_ind] = boid_inds2draw_data_[last];

    n_alive_of_players_.at(player_ind)--;

    //    auto& active_inds = player2active_inds.at(player_ind);
    //    auto& inactive_inds = player2inactive_inds.at(player_ind);

    if (boid_ind != last_active_ind) //! if it is not last active;
    {
        inactive_inds.push_back(boid_ind);
        auto last_in_active = active_inds.back();
        active_inds[ind_in_active_inds[boid_ind]] = last_in_active;
        ind_in_active_inds[last_in_active] = ind_in_active_inds[boid_ind];
        active_inds.pop_back();
    } else {
        last_active_ind--;
    }

    //    player2active_inds[u_ind.player_ind].erase(player2active_inds[u_ind.player_ind].begin()+boid_ind);
    //    auto& old_cell_boid_inds_ = cell2boid_inds_[boid_inds2cellind_[boid_ind]];
    //    auto last_in_cell = *(old_cell_boid_inds_.end()-1);
    //    old_cell_boid_inds_[boid_inds2ind_in_cell_[boid_ind]] = last_in_cell ;
    //    boid_inds2ind_in_cell_[last_in_cell] = boid_inds2ind_in_cell_[boid_ind];
    //    old_cell_boid_inds_.pop_back();
    //
    //    boid_inds2cellind_[boid_ind] = boid_inds2cellind_[last];
    //    cell2boid_inds_.at(boid_inds2cellind_[boid_ind]).;
    //
    //    boid_inds2cellind_.pop_back();
}

//! \brief draw agents into the window (so far agents are just triangles :D)
//! \param window
void BoidWorld::draw(sf::RenderWindow& window) {

    vertices_.setPrimitiveType(sf::Triangles);
    vertices_.resize(active_inds.size() * 3);

    sf::ConvexShape tri;
    tri.setPointCount(3);
    tri.setPoint(0, {0, -10 * RHARD / 2.f});
    tri.setPoint(1, {10 * RHARD, 0});
    tri.setPoint(2, {0, 10 * RHARD / 2.f});

    for (const auto i : active_inds) {
        const auto& r = r_coords_[i];
        const auto angle = orientation_[i];
        const auto radius = 3.f; //boid_inds2draw_data_[i].circle.getRadius();
        const auto player_ind = ind2player[i];
        tri.setPoint(0, {-0.7f * radius, -0.5f * radius});
        tri.setPoint(1, {radius, 0});
        tri.setPoint(2, {-0.7f * radius, 0.5f * radius});
        sf::Transform a;
        a.rotate(angle);

        sf::Color color;
        player_ind == 0 ? color = sf::Color::Red : color = sf::Color::Black;
        color.b = static_cast<sf::Uint8>(std::floor(norm(velocities_[i])/1.0f * 255.f));

        vertices_[i * 3 + 0] = {r_coords_[i] + a.transformPoint(tri.getPoint(0)), color};
        vertices_[i * 3 + 1] = {r_coords_[i] + a.transformPoint(tri.getPoint(1)), color};
        vertices_[i * 3 + 2] = {r_coords_[i] + a.transformPoint(tri.getPoint(2)), color};
    }
    window.draw(vertices_);

    // sf::RectangleShape arrow1;
    // arrow1.setSize({1, 20});

    // for (const auto i : active_inds) {

    //     auto& circle = boid_inds2draw_data_[i].circle;
    //     circle.setPosition(r_coords_[i] - sf::Vector2f{circle.getRadius(), circle.getRadius()});
    //     window.draw(circle);

    //     arrow1.setPosition(r_coords_[i]);
    //     arrow1.setRotation(orientation_[i] - 90);
    //     arrow1.setFillColor(sf::Color::Magenta);
    //     window.draw(arrow1);
    // }
}

//! \brief draw agents into the window (so far agents are just triangles :D)
//! \param window
void BoidWorld::draw(sf::RenderWindow& window, BoidControler& bc) {

    vertices_.setPrimitiveType(sf::Triangles);
    vertices_.resize(active_inds.size() * 3);

    sf::ConvexShape tri;
    tri.setPointCount(3);
    tri.setPoint(0, {0, -10 * RHARD / 2.f});
    tri.setPoint(1, {10 * RHARD, 0});
    tri.setPoint(2, {0, 10 * RHARD / 2.f});

    auto& forces = bc.getForces();
    auto max_force = std::max_element(forces.begin(), forces.end(),  [](const ControlForces& f1,
                                        const ControlForces& f2){
                                        return norm2(f1.repulse) < norm2(f2.repulse);});
    const auto max_force_norm = 0.05f*std::max({norm(max_force->repulse), 1.f});
    max_force = std::max_element(forces.begin(), forces.end(),  [](const ControlForces& f1,
                                        const ControlForces& f2){
                                        return norm2(f1.push) < norm2(f2.push);});
    const auto max_force_norm_push = 0.5f*std::max({norm(max_force->push), 1.f});

    max_force = std::max_element(forces.begin(), forces.end(),  [](const ControlForces& f1,
                                        const ControlForces& f2){
                                        return norm2(f1.align) < norm2(f2.align);});
    const auto max_force_norm_align = std::max({norm(max_force->align), 1.f});

    for (const auto i : active_inds) {
        const auto& r = r_coords_[i];
        const auto angle = orientation_[i];
        const auto radius = 3.f;//boid_inds2draw_data_[i].circle.getRadius();
        const auto player_ind = ind2player[i];
        tri.setPoint(0, {-0.7f * radius, -0.5f * radius});
        tri.setPoint(1, {radius, 0});
        tri.setPoint(2, {-0.7f * radius, 0.5f * radius});
        sf::Transform a;
        a.rotate(angle);

        sf::Color color;
        player_ind == 0 ? color = sf::Color::Blue : color = sf::Color::Green;
        
        if(norm2(forces[i].repulse) > norm2(forces[i].align) and norm2(forces[i].repulse) > norm2(forces[i].push)){
            color.r = static_cast<sf::Uint8>(std::floor(norm(forces[i].repulse)/max_force_norm * 255.f));
        }else if ( norm2(forces[i].align) > norm2(forces[i].push) and norm2(forces[i].align) > norm2(forces[i].repulse)){
            color.b = static_cast<sf::Uint8>(std::floor(norm(forces[i].align)/max_force_norm_align * 255.f));
            color = sf::Color::Blue;
        } else if(norm2(forces[i].push) > norm2(forces[i].align) and norm2(forces[i].push) > norm2(forces[i].repulse)){
            color = sf::Color::Yellow;
        }
        // // color.b = static_cast<sf::Uint8>(std::floor(norm(velocities_[i])/1.0f * 255.f));
        // color.g = static_cast<sf::Uint8>(std::floor(norm(forces[i].push)/max_force_norm_push * 255.f));


        vertices_[i * 3 + 0] = {r_coords_[i] + a.transformPoint(tri.getPoint(0)), color};
        vertices_[i * 3 + 1] = {r_coords_[i] + a.transformPoint(tri.getPoint(1)), color};
        vertices_[i * 3 + 2] = {r_coords_[i] + a.transformPoint(tri.getPoint(2)), color};
        forces[i].reset();
    }
    window.draw(vertices_);
}

