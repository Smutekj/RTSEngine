#include "SeekSystem.h"
#include "../PathFinding/PathFinder2.h"
#include "../Utils/NeighbourSearcherStrategy.h"

SeekSystem::SeekSystem(ComponentID id) : System2(id)
{
    sf::Vector2f box = {(float)Geometry::BOX[0], (float)Geometry::BOX[1]};
    p_ns_ = std::make_unique<NeighbourSearcherContext<PathFinderComponent, int>>(box, 10 * RHARD);
    p_ns_->setStrategy(NS_STRATEGY::BASIC);
}

void SeekSystem::issuePaths2(const std::vector<int> &entity_inds, sf::Vector2f path_end)
{
    auto &components = static_cast<CompArray &>(*p_comps_.get()).components_;

    for (const auto e_ind : entity_inds)
    {
        const auto compvec_ind = entity2compvec_ind_.at(e_ind);
        if (compvec_ind == -1)
        { //! already removed
            continue;
        }
        // components.at(compvec_ind).r_next = path_end;
        components.at(compvec_ind).path_end = path_end;
        components.at(compvec_ind).transform.angle_vel = components.at(compvec_ind).turn_rate;
    }
    commander_groups_.addGroup(entity_inds);
    p_pathfinder_->issuePaths2(components, entity_inds, entity2compvec_ind_, path_end);
}


void SeekSystem::issuePaths(const std::vector<int> &entity_inds, sf::Vector2f path_end)
{
    auto &components = static_cast<CompArray &>(*p_comps_.get()).components_;
    std::vector<int> compvec_inds;
    for (const auto e_ind : entity_inds)
    {
        const auto compvec_ind = entity2compvec_ind_.at(e_ind);
        if (compvec_ind == -1)
        { //! already removed
            continue;
        }
        compvec_inds.push_back(compvec_ind);
        components.at(compvec_ind).path_end = path_end;
        components.at(compvec_ind).transform.angle_vel = components.at(compvec_ind).turn_rate;
    }
    commander_groups_.addGroup(compvec_inds);
    p_pathfinder_->issuePaths(components, entity_inds, path_end);
}

void SeekSystem::update()
{
    auto &components = static_cast<CompArray &>(*p_comps_.get()).components_;
    p_ns_->update(components);

    //! remove dead agents ... (this should be done somewhere else i guess?)
    auto &groups = p_pathfinder_->to_update_groups_;
    for (auto &group_data : groups)
    {
        for (const auto ind : group_data.inds_to_update)
        {
        }
    }

    p_pathfinder_->updatePaths(components, entity2compvec_ind_, 5000);

        //! check that each entity is there at most once
    std::unordered_set<int> ents;
    for(const auto& group : commander_groups_.group2_entities_){
        for(auto ind : group){
            ents.insert(ind);
            assert(ents.count(ind) <= 1);
        }
    }

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
            if (norm_dr > comp.radius)
            {
                comp.transform.vel = dr_to_target / norm_dr * comp.max_speed;
            }

            if (needsUpdating(comp))
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
        const auto compvec_ind = comp_inds_to_update_.front();
        std::vector<int> entity_ind = {compvec_ind2entity_ind_.at(compvec_ind)};
        p_pathfinder_->issuePaths2(components, entity_ind, entity2compvec_ind_,
                                   components.at(compvec_ind).path_end);
        comp_inds_to_update_.pop();
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
        pd.start_tri_ind = p_cdt_->findTriangle(comp.transform.r, false);
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

void SeekSystem::onComponentRemoval(int comp_ind){
        p_pathfinder_->to_update_groups_;
    }

void SeekSystem::communicate(std::array<SharedData, N_MAX_ENTITIES> &entity2shared_data) const
{
    int comp_ind = 0;
    auto &components = static_cast<CompArray &>(*p_comps_).components_;

    for (const auto &comp : components)
    {
        auto entity_ind = compvec_ind2entity_ind_.at(comp_ind);
        entity2shared_data.at(entity_ind).target = comp.r_next;
        entity2shared_data.at(entity_ind).state = comp.state;
        entity2shared_data.at(entity_ind).transform.r = comp.transform.r;
        auto phys_vel = entity2shared_data.at(entity_ind).transform.vel;
        // if(dot(comp.transform.vel, phys_vel) >= 0 && norm(phys_vel) < 5.f){
        // auto alpha = std::exp(-norm(phys_vel) /50.f);
        entity2shared_data.at(entity_ind).transform.vel += comp.transform.vel;
        // }
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
    if (std::abs(d_angle) < comp.turn_rate)
    { //! we can turn to desired angle in this frame
        comp.transform.angle = desired_angle;
        comp.transform.angle_vel = 0;
    }
    repairAngleBounds(comp);
}

void SeekSystem::repairAngleBounds(PathFinderComponent &comp)
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
