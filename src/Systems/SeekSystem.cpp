#include "SeekSystem.h"
#include "../PathFinding/PathFinder2.h"
#include "../NeighbourSearcherStrategy.h"

SeekSystem::SeekSystem(ComponentID id) : System2(id)
{
    sf::Vector2f box = {(float)Geometry::BOX[0], (float)Geometry::BOX[1]};
    p_ns_ = std::make_unique<NeighbourSearcherContext<PathFinderComponent, int>>(box, 10 * RHARD);
    p_ns_->setStrategy(NS_STRATEGY::BASIC);
}

void SeekSystem::issuePaths(std::vector<int> &entity_inds, sf::Vector2f path_end)
{
    auto &components = static_cast<CompArray &>(*p_comps_.get()).components_;
    std::vector<int> compvec_inds;
    compvec_inds.reserve(entity_inds.size());
    for (const auto e_ind : entity_inds)
    {
        const auto compvec_ind = entity2compvec_ind_.at(e_ind);
        if(compvec_ind == -1){//! already removed
            continue;
        } 
        compvec_inds.push_back(compvec_ind);
        components.at(compvec_ind).path_end = path_end;
        components.at(compvec_ind).transform.angle_vel = components.at(compvec_ind).turn_rate; 
    }
    commander_groups_.addGroup(compvec_inds);
    p_pathfinder_->issuePaths2(components, entity_inds, entity2compvec_ind_, path_end);

}

void SeekSystem::update()
{
    auto &components = static_cast<CompArray &>(*p_comps_.get()).components_;
    p_ns_->update(components);

    //! remove death agents ... (this should be done somewhere else i guess?)
    auto& groups = p_pathfinder_->to_update_groups_;
    for(auto& group_data : groups){
        for(const auto ind : group_data.inds_to_update){

        }     
    }

    p_pathfinder_->updatePaths(components, 5000);

    int compvec_ind = 0;
    for (auto &comp : components)
    {
        if (comp.state == MoveState::MOVING)
        {
            const auto &r = comp.transform.r;
            const auto &v = comp.transform.vel;
            const auto &move_target = comp.r_next;

            const auto &path_end = comp.path_end;
            auto dr_to_target = move_target - r;
            const auto desired_angle = s_angle_calculator.angle(dr_to_target);
            
            turnTowards(comp, dr_to_target);

            float norm_dr = norm(dr_to_target);
            if (norm_dr > comp.radius && comp.transform.angle_vel == 0 )
            {
                comp.transform.vel = dr_to_target / norm_dr * comp.max_speed;
            }

            if (comp.transform.angle_vel == 0 && needsUpdating(comp))
            {
                updateTarget(comp, r);
            }

            if (vequal(path_end, move_target))
            {
                finalizeSeek(comp, dr_to_target);
            }
        }
        else
        {
            
        }
        compvec_ind++;
    }

    //  //! we separate paths into buckets based on the end point
    // std::unordered_map<TriInd, std::vector<int>> path_end_tri_ind2compvec_ind;
    // while (!components2update_.empty())
    // {

    //     auto& pd = components2update_.front();
    //     path_end_tri_ind2compvec_ind.at(pd.s) = ;
    //     components2update_.pop();
    // }

    while (!comp_inds_to_update_.empty())
    {
        std::vector<int> entity_ind = {compvec_ind2entity_ind_.at(comp_inds_to_update_.front())};
        p_pathfinder_->issuePaths2(components, entity_ind, entity2compvec_ind_,
                                     components.at(comp_inds_to_update_.back()).path_end);
        comp_inds_to_update_.pop();
    }
}

void SeekSystem::draw(sf::RenderTarget &window)
{
    for(int i = 0; i < N_MAX_THREADS; ++i){
        const auto &path = p_pathfinder_->pap[i].path;
        const auto &portals = p_pathfinder_->pap[i].portals;
        sf::RectangleShape line;
        auto color = sf::Color::Cyan;
        color.r += i / 255.f * N_MAX_THREADS;
        line.setFillColor(color);
        sf::CircleShape node;
        node.setRadius(1.f);
        node.setFillColor(sf::Color::Cyan);

        sf::RectangleShape portal_line;
        portal_line.setFillColor(sf::Color::Black);

        for (int i = 1; i < path.size(); ++i)
        {
            const auto dr = path.at(i) - path.at(i - 1);
            const auto dr_norm = norm(dr);
            const auto dr2 = portals[i].t;
            const auto angle = 180.f / (M_PIf)*std::acos(dot(dr / dr_norm, {0, 1})) * (2.f * (dr.x < 0.f) - 1.f);
            const auto angle2 = 180.f / (M_PIf)*std::acos(dot(dr2, {0, 1})) * (2.f * (dr2.x < 0.f) - 1.f);
            line.setPosition(path.at(i - 1));
            line.setSize({1, dr_norm});
            line.setRotation(angle);

            portal_line.setPosition(portals[i].from);
            portal_line.setSize({1, portals[i].l});
            portal_line.setRotation(angle2);

            node.setPosition(path.at(i) - sf::Vector2f{1.f, 1.f});

            window.draw(portal_line);
            window.draw(line);
            window.draw(node);
        }
    }

    //! draw funnels
    for(int i = 0; i < N_MAX_THREADS; ++i){
        const auto &funnel = p_pathfinder_->funnels[i];
        sf::RectangleShape line_l;
        sf::RectangleShape line_r;
        
        auto color = sf::Color::Magenta;
        line_l.setFillColor(color);
        line_r.setFillColor(color);
        sf::CircleShape node;
        node.setRadius(1.f);
        node.setFillColor(sf::Color::Black);


        for (int i = 1; i < funnel.size(); ++i)
        {
            const auto dr_l = funnel.at(i).first - funnel.at(i - 1).first;
            const auto dr_r = funnel.at(i).second - funnel.at(i - 1).second;
            const auto drl_norm = norm(dr_l);
            const auto drr_norm = norm(dr_r);
            const auto angle_l = 180.f / (M_PIf)*std::acos(dot(dr_l / drl_norm, {0, 1})) * (2.f * (dr_l.x < 0.f) - 1.f);
            const auto angle_r = 180.f / (M_PIf)*std::acos(dot(dr_r / drr_norm, {0, 1})) * (2.f * (dr_r.x < 0.f) - 1.f);
            line_l.setPosition(funnel.at(i - 1).first);
            line_l.setSize({1, drl_norm});
            line_l.setRotation(angle_l);

            line_r.setPosition(funnel.at(i - 1).second);
            line_r.setSize({1, drr_norm});
            line_r.setRotation(angle_r);

            window.draw(line_l);
            window.draw(line_r);
            window.draw(node);
        }
    }
}

//! \brief stops moving towards seek target and removes agent from his command group
//! \param selected
//! \param r        position of the selected agent
void SeekSystem::stopSeeking(PathFinderComponent &comp)
{
    comp.state = MoveState::STANDING;
    comp.standing_pos = comp.transform.r;
    comp.n_steps_since_all_in_front_standing = 0;
    comp.transform.vel *= 0.f;
    comp.transform.angle_vel = 0;
    commander_groups_.removeFromGroup(compvecInd(comp));
}

int SeekSystem::compvecInd(const PathFinderComponent &comp) const
{
    const auto &components = static_cast<CompArray &>(*p_comps_.get()).components_;
    return (&comp) - &(components.front());
}

//! \brief one possible stopping condition
bool SeekSystem::neighboursInFrontReachedEnd(const PathFinderComponent &comp, float radius, sf::Vector2f dr_to_target)
{
    const auto &comps = static_cast<CompArray &>(*p_comps_.get()).components_;

    int n_in_front = 0;
    int n_in_front_standing = 0;
    const auto r = comp.transform.r;
    const auto compvec_ind = compvecInd(comp);
    const auto &neighbour_inds = p_ns_->getInteractionData(compvec_ind);
    const auto entity_ind = compvec_ind2entity_ind_.at(compvec_ind);
    for (int i = 0; i < p_ns_->last_i[compvec_ind]; i++)
    {
        const auto comp_ind_j = neighbour_inds.at(i);
        const sf::Vector2f d_r = r - comps.at(comp_ind_j).transform.r;
        const auto state_j = comps.at(comp_ind_j).state;

        if (norm(d_r) < settings_.values_[SeekSystemSettings::Values::FINISH_CONE_LEN] && commander_groups_.isSameGroup(compvec_ind, comp_ind_j))
        {
            if (std::abs(s_angle_calculator.angleBetween(d_r, dr_to_target)) < settings_.values_[SeekSystemSettings::Values::FINISH_ANGLE])
            {
                n_in_front_standing += state_j == MoveState::STANDING;
                n_in_front++;
            }
        }
    }
    return n_in_front != 0 && n_in_front == n_in_front_standing;
}

bool SeekSystem::needsUpdating(PathFinderComponent &comp)
{
    auto &move_target = comp.r_next;
    auto &next_move_target = comp.r_next_next;
    auto &portal = comp.portal_next;
    const auto &path_end = comp.path_end;
    const auto &r = comp.transform.r;

    // if(isStuck(selected)){ return true; } //! HELP STEP BRO, I'M STUCK!!!
    if (vequal(move_target, path_end))
    {
        return false;
    }

    auto dr_to_target = move_target - r;
    const float norm_dr = norm(dr_to_target);

    const auto dr_to_next_target = next_move_target - r;
    const auto norm_dr_next = norm(dr_to_next_target);

    auto t = next_move_target - move_target;
    bool can_move_to_next_target = dot(-dr_to_target, t) > 0;
    bool update_next_target = norm_dr < RHARD && can_move_to_next_target || move_target == next_move_target;
    const auto closest_point_on_next_portal = closestPointOnEdge(r, comp.portal_next);

    const auto current_tri_ind = p_cdt_->findTriangle(comp.transform.r, false);
    // if(current_tri_ind == comp.next_tri_ind && current_tri_ind != comp.start_tri_ind){
    //     //! I should ray-cast here to check if we can move directly to next point
    //     update_next_target = true;

    // }

    if (norm_dr < comp.radius || dot(dr_to_target, t) < 0)
    {
        update_next_target = true;
    }
    else
    {
        if (dist2(closest_point_on_next_portal, r) < dist2(r, move_target) && portal.l > 0 &&
            dist2(closest_point_on_next_portal, r) < 16 * RHARD * RHARD and norm2(t) != 0)
        {
            move_target = closest_point_on_next_portal;
            dr_to_target = move_target - r;
            update_next_target = false;
        }
        else if (vequal(move_target, next_move_target) and move_target != path_end)
        {
            update_next_target = true;
        }
    }

    return update_next_target;
}

void SeekSystem::finalizeSeek(PathFinderComponent &comp, sf::Vector2f dr_to_target)
{
    const auto radius = comp.radius;
    const auto path_end = comp.path_end;

    auto &comps = static_cast<CompArray &>(*p_comps_.get()).components_;
    if (neighboursInFrontReachedEnd(comp, radius, dr_to_target))
    {
        comp.n_steps_since_all_in_front_standing++;
        if (comp.n_steps_since_all_in_front_standing > 0)
        {
            stopSeeking(comp);
        }
    }
    else
    {
        comp.n_steps_since_all_in_front_standing = 0;
    }

    auto compvec_ind = compvecInd(comp);

    const auto finish_area = commander_groups_.getFinishedAreaOf(compvec_ind);
    auto finish_radius = std::sqrt(finish_area / M_PIf);
    auto dist_to_end = dist(path_end, comp.transform.r);
    if (dist_to_end < 2 * radius + finish_radius || comp.state == MoveState::STANDING)
    {
        //! we inform all neighbours in our command group within some radius that they have reached the end
        const auto &neighbour_comp_inds = p_ns_->getInteractionData(compvec_ind);
        for (int i = 0; i < p_ns_->last_i[compvec_ind]; i++)
        {
            const auto comp_ind_j = neighbour_comp_inds.at(i);
            const auto dr = comp.transform.r - comps.at(comp_ind_j).transform.r;
            const auto dist_to_neighbour = norm(dr);
            if (dist_to_neighbour < settings_.values_[SeekSystemSettings::Values::FINISH_RADIUS] && commander_groups_.isSameGroup(compvec_ind, comp_ind_j))
            {
                stopSeeking(comps.at(comp_ind_j));
            }
        }
        stopSeeking(comp);
    }
}

void SeekSystem::updateTarget(PathFinderComponent &comp, const sf::Vector2f r)
{
    auto &move_target = comp.r_next;
    auto &next_move_target = comp.r_next_next;
    const auto &path_end = comp.path_end;

    const auto dr_to_next_target = next_move_target - r;
    const auto norm_dr_next = norm(dr_to_next_target);

    move_target = next_move_target;
    comp.portal_next = comp.portal_next_next;

    if (norm_dr_next > 0)
    {
        comp.transform.vel = dr_to_next_target / norm_dr_next * comp.max_speed;
    }

    if (next_move_target != path_end)
    {
        comp.needs_update = true;
        
        PathData pd;
        pd.start_tri_ind =  p_cdt_->findTriangle(comp.transform.r, false);
        pd.r_end = comp.path_end;
        components2update_.push(pd);
        comp_inds_to_update_.push(compvecInd(comp));
    }
}


void SeekSystem::updateSharedData(const std::array<SharedData, N_MAX_ENTITIES> &new_data, const std::vector<Entity> &active_entity_inds)
{
    auto &comps = static_cast<ComponentArray<PathFinderComponent> &>(*p_comps_.get()).components_;
    for (const auto ent : active_entity_inds)
    {
        const auto compvec_ind = entity2compvec_ind_.at(ent.ind);
        comps.at(compvec_ind).transform = new_data.at(ent.ind).transform;
        comps.at(compvec_ind).state = new_data.at(ent.ind).state;
    }
}


void SeekSystem::communicate(std::array<SharedData, N_MAX_ENTITIES> &entity2shared_data) const
{
    int comp_ind = 0;
    auto &components = static_cast<CompArray &>(*p_comps_).components_;

    for (const auto &comp : components)
    {
        auto entity_ind = compvec_ind2entity_ind_.at(comp_ind);
        entity2shared_data.at(entity_ind).state = comp.state;
        entity2shared_data.at(entity_ind).transform.r = comp.transform.r;
        entity2shared_data.at(entity_ind).transform.vel += comp.transform.vel;
        entity2shared_data.at(entity_ind).transform.angle_vel = comp.transform.angle_vel;

        comp_ind++;
    }
}

    void SeekSystem::turnTowards(PathFinderComponent &comp, sf::Vector2f direction)
    {
        float norm_dr = norm(direction);
        float desired_angle;
        repairAngleBounds(comp);

        if (norm2(direction) == 0)
        {
            desired_angle = comp.desired_angle;
        }
        else
        {
            desired_angle = s_angle_calculator.angle(direction);
        }
        auto d_angle = desired_angle - comp.transform.angle;
        d_angle > 180 ? d_angle = -360 + d_angle : d_angle = d_angle;
        d_angle < -180 ? d_angle = 360 + d_angle : d_angle = d_angle;
        comp.transform.angle_vel = std::min({comp.turn_rate, d_angle});
        if (d_angle < 0)
        {
            comp.transform.angle_vel = std::max({-comp.turn_rate, d_angle});
        }
        if(std::abs(d_angle) < comp.turn_rate){ //! we can turn to desired angle in this frame
            comp.transform.angle = desired_angle;
            comp.transform.angle_vel = 0;
        }
        repairAngleBounds(comp);

    }

    void SeekSystem::repairAngleBounds(PathFinderComponent& comp)
    {

        if (comp.transform.angle > 180)
        {
            comp.transform.angle -= 360;
        }
        if (comp.transform.angle < -180)
        {
            comp.transform.angle += 360;
        }
        
    }

