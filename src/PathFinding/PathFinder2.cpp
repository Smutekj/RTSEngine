#include <future>
#include <thread>
#include <chrono>
#include <fstream>

#include "PathFinder2.h"
#include "Triangulation.h"

PathFinder2::PathFinder2(Triangulation *cdt)
    : cdt_(cdt)
{
    p_rtg_ = std::make_shared<ReducedTriangulationGraph>();
    update_has_been_issued_.resize(N_MAX_NAVIGABLE_BOIDS, false);
    entity2update_group_ind_.fill(-1);
}

//! \brief checks if reduced2tri_ind and tri_ind2reduced mappings are consistent with each other
//! \brief also checks if corridors lead to correct triangles
bool PathFinder2::reducedGraphConsistentWithTriangulation() const
{
    const auto &triangles = cdt_->triangles_;
    const auto &rtg = *p_rtg_;

    auto triangle_is_within_boundary = [&](const Triangle &tri)
    {
        const auto tri_center = cdt_->calcTriangleCenter(tri);
        return cdt_->boundary_.x >= tri_center.x and tri_center.x > 0 and cdt_->boundary_.y >= tri_center.y and
               tri_center.y > 0;
    };

    for (int ti = 0; ti < triangles.size(); ti++)
    {
        auto &vertex = rtg.tri_ind2vertex[ti];
        if (triangle_is_within_boundary(triangles[ti]) and triangles[ti].countFreeEdges() == 3)
        {
            assert(rtg.vertex2tri_ind[vertex] == ti);
            for (auto edge_ind : rtg.vertex2edge_inds2[vertex])
            {
                auto tj = ti;
                if (rtg.edges.at(edge_ind).end.current == ti)
                {
                    tj = rtg.edges.at(edge_ind).start.current;
                }
                else
                {
                    tj = rtg.edges.at(edge_ind).end.current;
                }
                // assert(triangles[tj].countFreeEdges() != 2);
            }
        }
        else if (triangles[ti].countFreeEdges() == 2)
        {
            auto edge_ind = vertex;
            auto &edge = rtg.edges.at(edge_ind);
            // assert( std::find(edge.tri_inds.begin(), edge.tri_inds.end(), ti) != edge.tri_inds.end() );
        }
    }
    return true;
}

//! \brief creates everything from p_triangulation_
//! \brief should be called on triangulation change
void PathFinder2::update()
{

    const auto n_triangles = cdt_->triangles_.size();

    triangle2tri_widths_.resize(n_triangles);
    for (int tri_ind = 0; tri_ind < n_triangles; ++tri_ind)
    {
        triangle2tri_widths_[tri_ind] = TriangleWidth(cdt_->triangles_[tri_ind], *cdt_);
    }

    p_rtg_->constructFromTriangulationCenters(*cdt_, tri_ind2component_);
    cdt_->updateCellGrid();
    // reducedGraphConsistentWithTriangulation();

    for (int thread_id = 0; thread_id < N_MAX_THREADS; ++thread_id)
    {
        thread2reduced_vertex2astar_data_[thread_id].resize(p_rtg_->reduced_vertices.size() + 2);
    }
    reduced_vertex2astar_data_.resize(p_rtg_->reduced_vertices.size() + 2);
}

void PathFinder2::updatePaths(std::vector<PathFinderComponent> &comps, std::array<int, N_MAX_ENTITIES> &entity2compvec_ind,
                              const int available_time)
{

    // sf::Clock time;
    const int n_threads = std::thread::hardware_concurrency() - 1;
    thread_status_.resize(n_threads, THREAD_STATUS::FREE);

    std::function<int(const GroupPathData, int)> do_pathfinding = [this, &comps, &entity2compvec_ind](const GroupPathData data, int thread_id)
    {
        try
        {
            doPathFinding(data.r_starts, data.r_end, comps, entity2compvec_ind, data.radius, data.inds_to_update, thread_id);
        }
        catch (std::exception &e)
        {
            std::cout << e.what() << "\n";
            throw e;
        }
        return 1;
    };

    //! check that each entity is there at most once
    std::unordered_set<int> ents;
    for (const auto &group : to_update_groups_)
    {
        for (auto ind : group.inds_to_update)
        {
            ents.insert(ind);
            assert(ents.count(ind) <= 1);
        }
    }

    int thread_id = 0;
    const int last = to_update_groups_.size() - 1;

    std::priority_queue to_update_pq(
        to_update_groups_.begin(), to_update_groups_.end(), [](const GroupPathData &gpd1, const GroupPathData &gpd2)
        { return dist2(gpd1.r_end, gpd1.r_starts[0]) < dist2(gpd2.r_end, gpd2.r_starts[0]); });
    futures_.resize(n_threads);

    // sf::Clock clock;
    int n_paths_found = 0;
    for (int i = last; i >= std::max(last + 1 - n_threads, 0); --i)
    {
        auto data = to_update_pq.top();
        for (int j = 0; j < data.inds_to_update.size(); ++j)
        {
            auto compvec_ind = entity2compvec_ind.at(data.inds_to_update[j]);
            if(compvec_ind == -1){continue;}
            data.r_starts.at(j) = comps.at(compvec_ind).transform.r;
        }
        futures_.at(thread_id) = std::async(std::launch::async, do_pathfinding, data, thread_id);
        thread_status_.at(thread_id) = THREAD_STATUS::RUNNING;
        n_paths_found++;
        for (const auto &entity_ind : data.inds_to_update)
        {
            auto compvec_ind = entity2compvec_ind.at(entity_ind);
            if(compvec_ind == -1){continue;}
            comps.at(entity2compvec_ind.at(entity_ind)).needs_update = true;
        }
        thread_id++;
        to_update_pq.pop();
    }

    bool all_threads_free = allThreadsFree(std::min(last + 1, n_threads));
    // auto run_time = clock.getElapsedTime().asMicroseconds();
    double run_time = 0;
    bool all_jobs_are_done = all_threads_free && to_update_pq.empty();
    bool time_has_run_out = run_time > available_time;
    //! search for queried paths on all available threads until time runs out or until we finish the job
    while (!(all_jobs_are_done))
    {
        if (time_has_run_out && all_threads_free)
        {
            std::cout << "time has run out! path_finding took: " << run_time << " available: " << available_time << "\n";
            break;
        }

        for (int thread_id = 0; thread_id < std::min(last + 1, n_threads); ++thread_id)
        {
            if (thread_status_[thread_id] == THREAD_STATUS::FREE and !to_update_pq.empty())
            {
                auto data = to_update_pq.top();
                for (int i = 0; i < data.inds_to_update.size(); ++i)
                {
                    auto compvec_ind = entity2compvec_ind.at(data.inds_to_update[i]);
                    if(compvec_ind == -1){continue;}
                    data.r_starts[i] = comps.at(compvec_ind).transform.r;
                }

                futures_[thread_id] = std::async(std::launch::async, do_pathfinding, data, thread_id);
                thread_status_[thread_id] = THREAD_STATUS::RUNNING;

                n_paths_found++;
                for (const auto &entity_ind : data.inds_to_update)
                {
                    auto compvec_ind = entity2compvec_ind.at(entity_ind);
                    if(compvec_ind == -1){continue;}
                    comps.at(compvec_ind).needs_update = true;
                }
                to_update_pq.pop();
            }
        }
        all_threads_free = allThreadsFree(std::min(last + 1, n_threads));
        all_jobs_are_done = all_threads_free && to_update_pq.empty();

        // run_time = clock.getElapsedTime().asMicroseconds();
        time_has_run_out = run_time > available_time;
    }
    to_update_groups_.resize(to_update_groups_.size() - n_paths_found);
}

bool PathFinder2::allThreadsFree(int last_thread_id)
{
    bool all_threads_free = true;
    for (int thread_id = 0; thread_id < last_thread_id; ++thread_id)
    {

        bool is_valid = futures_.at(thread_id).valid();
        std::future_status status = std::future_status::timeout;
        if (is_valid)
        {
            status = futures_.at(thread_id).wait_for(std::chrono::microseconds(0));
        }
        if (status == std::future_status::ready)
        {
            thread_status_.at(thread_id) = THREAD_STATUS::FREE;
            int n;
            try
            {
                if (futures_.at(thread_id).valid())
                {
                    n = futures_.at(thread_id).get();
                }
            }
            catch (const std::exception &e)
            {
                std::cout << "YOU PROBABLY FORGOT TO CREATE TRIANGULATION, LOL: "
                          << "\"\nMessage: \"" << e.what() << "\"\n";
                thread_status_.at(thread_id) = THREAD_STATUS::FAILED;
                throw e;
            }
        }
        else if (is_valid)
        {
            all_threads_free = false;
        }
    }
    return all_threads_free;
}



void dumpFunnelToFile(const Funnel& funnel, float radius, std::string filename){
    std::ofstream file(filename);
    
    for(const auto [r_left, r_right] : funnel){
        file << r_left.x << " " << r_left.y << " " << r_right.x << " " << r_right.y << '\n';
    }
    file.close();
}

void dumpPathToFile(const std::deque<sf::Vector2f>& path, float radius, std::string filename){
    std::ofstream file(filename);
    
    for(const auto r : path){
        file << r.x << " " << r.y  << '\n';
    }
    file.close();
}

void dumpFunnelToFile2(const std::vector<Vertex>& r_lefts, const std::vector<Vertex>& r_rights, std::string filename){
    std::ofstream file(filename);
    
    for( auto r_left : r_lefts){
        file << r_left.x << " " << r_left.y << '\n';
    }
    file << "\n";

    for( auto r_right : r_rights){
        file << r_right.x << " " << r_right.y << '\n';
    }

    file.close();
}

//! \brief calls threads that will do path-finding of the issued pathfinding work
//! \param comps components which have to contain a transform component
//! \param available_time total time available for pathfinding
//! \note  We use here the most recent coordinates ( \p r_coords )
//! \note because the actual pathfinding does not have to be done in the same frame when the pathfinding was issued!
void PathFinder2::updatePaths2(std::vector<PathFinderComponent> &comps,
                               const int available_time)
{

    // sf::Clock time;
    const int n_threads = 1; // std::thread::hardware_concurrency();
    thread_status_.resize(n_threads, THREAD_STATUS::FREE);
    std::function<int(const GroupPathData, int)> do_pathfinding = [this, &comps](const GroupPathData data, int thread_id)
    {
        try
        {
            doPathFinding2(data.r_starts, data.r_end, comps, data.radius, data.inds_to_update, thread_id);
        }
        catch (std::exception &e)
        {
            std::cout << "fuck\n";
            throw e;
        }
        return 1;
    };

    std::priority_queue to_update_pq(
        to_update_groups_.begin(), to_update_groups_.end(), [](const GroupPathData &gpd1, const GroupPathData &gpd2)
        { return dist2(gpd1.r_end, gpd1.r_starts[0]) < dist2(gpd2.r_end, gpd2.r_starts[0]); });
    futures_.resize(n_threads);

    // sf::Clock clock;
    int n_paths_found = 0;
    int thread_id = 0;
    const int last = to_update_groups_.size() - 1;
    for (int i = last; i >= std::max(last + 1 - n_threads, 0); --i)
    {
        auto data = to_update_pq.top();
        for (int j = 0; j < data.inds_to_update.size(); ++j)
        {
            data.r_starts.at(j) = comps.at(data.inds_to_update.at(j)).transform.r;
        }
        futures_.at(thread_id) = std::async(std::launch::async, do_pathfinding, data, thread_id);
        thread_status_.at(thread_id) = THREAD_STATUS::RUNNING;
        n_paths_found++;
        for (const auto &ind : data.inds_to_update)
        {
            update_has_been_issued_.at(ind) = false;
        }
        thread_id++;
        to_update_pq.pop();
    }

    bool all_threads_free = allThreadsFree(std::min(last + 1, n_threads));
    auto run_time = 0.f;//clock.getElapsedTime().asMicroseconds();

    bool all_jobs_are_done = all_threads_free && to_update_pq.empty();
    bool time_has_run_out = run_time > available_time;
    //! search for queried paths on all available threads until time runs out or until we finish the job
    while (!(all_jobs_are_done))
    {
        if (time_has_run_out and all_threads_free)
        {
            std::cout << "time has run out! path_finding took: " << run_time << " available: " << available_time << "\n";
            break;
        }

        for (int thread_id = 0; thread_id < std::min(last + 1, n_threads); ++thread_id)
        {
            if (thread_status_[thread_id] == THREAD_STATUS::FREE and !to_update_pq.empty())
            {
                auto data = to_update_pq.top();
                for (int i = 0; i < data.inds_to_update.size(); ++i)
                {
                    data.r_starts[i] = comps.at(data.inds_to_update[i]).transform.r;
                }

                futures_[thread_id] = std::async(std::launch::async, do_pathfinding, data, thread_id);
                thread_status_[thread_id] = THREAD_STATUS::RUNNING;

                n_paths_found++;
                for (const auto &ind : data.inds_to_update)
                {
                    update_has_been_issued_[ind] = false;
                }
                to_update_pq.pop();
            }
        }
        all_threads_free = allThreadsFree(std::min(last + 1, n_threads));
        all_jobs_are_done = all_threads_free && to_update_pq.empty();
        if (to_update_pq.empty())
        {
            std::cout << "to_update_pq empty\n";
        }

        // run_time = clock.getElapsedTime().asMicroseconds();
        time_has_run_out = run_time > available_time;
    }
    to_update_groups_.resize(to_update_groups_.size() - n_paths_found);

}

//! \brief sets relevant data relating to pathfinding of agent with index \p ind to controler \p bc
//! \param ind      index of the agnet
//! \param r_ind  current position of the agnet
//! \param r_end  target position of the agent
//! \param bc   controler object
//! \param path_and_portals
void PathFinder2::setPathOfAgent(PathFinderComponent &comp, sf::Vector2f r_end, PathAndPortals &path_and_portals) const
{
    auto &path = path_and_portals.path;
    auto &portals = path_and_portals.portals;
    comp.start_tri_ind = cdt_->findTriangle(path.at(0), true);
    comp.next_tri_ind = cdt_->findTriangle(path.at(1), true);

    if (path.back() == path.at(path.size() - 2))
    {
        path.pop_back();
    }

    comp.r_next = path.at(1);
    comp.portal_next = portals.at(1);
    if (path.size() >= 3)
    {
        if (dot(comp.transform.r - path.at(1), path.at(2) - path.at(1)) < 0)
        {
            comp.r_next_next = path.at(2);
            comp.portal_next_next = portals.at(2);
        }
        else
        {
            if (path.size() > 3)
            {
                comp.next_tri_ind = cdt_->findTriangle(path.at(2), true);
                comp.r_next = path.at(2);
                comp.portal_next = portals.at(2);

                comp.next_next_tri_ind = cdt_->findTriangle(path.at(3), false);
                comp.r_next_next = path.at(3);
                comp.portal_next_next = portals.at(3);
            }
            else
            {
                comp.next_tri_ind = cdt_->findTriangle(path.at(2), true);
                comp.r_next = path.at(2);
                comp.portal_next = portals.at(2);

                comp.next_next_tri_ind = cdt_->findTriangle(r_end, true);
                comp.r_next_next = r_end;
                comp.portal_next_next = Edgef();
            }
        }
    }
    else
    {
        comp.r_next = path.at(1);
        comp.portal_next = portals.at(1);

        comp.next_next_tri_ind = comp.next_tri_ind;
        comp.r_next_next = path.at(1);
        comp.portal_next_next = portals.at(1);
    }
}

// void PathFinder2::doPathfinding(PathFinder2::GroupPathData& gd, int thread_id){

//     doPathFinding

// }

void PathFinder2::doPathFinding(const std::vector<sf::Vector2f> r_coords, const sf::Vector2f r_end,
                                std::vector<PathFinderComponent> &comps,
                                std::array<int, N_MAX_ENTITIES> &entity2compvec_ind,
                                const float max_radius_of_agent, const std::vector<int> agent_indices, const int thread_id)
{

    const auto r_start = r_coords.at(0);
    Funnel funnel;

    try
    {
        findSubOptimalPathCenters(r_start, r_end, max_radius_of_agent, funnel, thread_id);
    }
    catch (std::exception &e)
    {
        std::cout << "wtf: " << e.what() << "\n";
        throw e;
    }
    funnel.push_back({r_start, r_start});
    std::reverse(funnel.begin(), funnel.end());
    funnel.push_back({r_end, r_end});

    try
    {
        auto path_and_portals = pathFromFunnel(r_start, r_end, max_radius_of_agent, funnel);
        auto &path = path_and_portals.path;

        for (int i = 0; i < agent_indices.size(); ++i)
        {
            const auto comp_ind = entity2compvec_ind.at(agent_indices[i]);
            if(comp_ind == -1){continue;}
            setPathOfAgent(comps.at(comp_ind), r_end, path_and_portals);
        }
    }
    catch (std::exception &e)
    {
        std::cout << "funnel creation got fucked: " << e.what() << "\n";
        throw e;
    }

    // pap[comp_ind] = path_and_portals;
    // funnels[comp_ind] = funnel;
}

void PathFinder2::doPathFinding2(const std::vector<sf::Vector2f> r_coords, const sf::Vector2f r_end, std::vector<PathFinderComponent> &comps,
                                 const float max_radius_of_agent, const std::vector<int> agent_indices, const int thread_id)
{

    const auto r_start = r_coords.at(0);
    Funnel funnel;

    try
    {
        findSubOptimalPathCenters(r_start, r_end, max_radius_of_agent, funnel, thread_id);
    }
    catch (std::exception &e)
    {
        std::cout << "wtf: " << e.what() << "\n";
        throw e;
    }
    funnel.push_back({r_start, r_start});
    std::reverse(funnel.begin(), funnel.end());
    funnel.push_back({r_end, r_end});

    auto path_and_portals = pathFromFunnel(r_start, r_end, max_radius_of_agent, funnel);
    auto &path = path_and_portals.path;

    for (int i = 0; i < agent_indices.size(); ++i)
    {
        const auto comp_ind = agent_indices[i];
        setPathOfAgent(comps.at(comp_ind), r_end, path_and_portals);
    }
    pap[thread_id] = path_and_portals;
    funnels[thread_id] = funnel;
}

//! \brief divides agents by triangles and for each triangle we do one Astar search
void PathFinder2::issuePaths(std::vector<PathFinderComponent> &comps, const std::vector<int> &selection,
                             const sf::Vector2f r_end)
{

    float max_radius_of_agent = 0.;

    // sf::Clock time;

    start_tri2indices_and_rstarts_.clear();
    start_tri_inds_.clear();

    for (const auto selected_ind : selection)
    {

        auto &comp = comps.at(selected_ind);
        auto start_tri_ind = cdt_->findTriangle(comp.transform.r, false);
        comp.start_tri_ind = start_tri_ind;
        //! I should do ray casting here, but it is not done yet
        // if (rayCast(r_coords[selected_ind], r_end) >=
        //     1) { //! if I see the end directly result is straight line (I should write a setter for path_data)
        //     bc.setPathData(selected_ind, r_end, Edgef());
        //     bc.setPathDataNext(selected_ind, r_end, Edgef());
        //     continue;
        // }

        if (!comp.needs_update)
        { //! no need to update something twice
            continue;
        }

        comp.needs_update = false;

        if (start_tri2indices_and_rstarts_.count(start_tri_ind) == 0)
        {
            start_tri2indices_and_rstarts_[start_tri_ind] = {{selected_ind, comp.transform.r}};
            start_tri_inds_.push_back(start_tri_ind);
        }
        else
        {
            start_tri2indices_and_rstarts_[start_tri_ind].push_back({selected_ind, comp.transform.r});
        }

        if (comps.at(selected_ind).radius > max_radius_of_agent)
        {
            max_radius_of_agent = comps.at(selected_ind).radius;
        }
    }
    // const auto time_of_first_part = time.restart().asMicroseconds();

    const auto end_tri_ind = cdt_->findTriangle(r_end, false);

    for (const auto start_tri_ind : start_tri_inds_)
    {
        auto& inds = start_tri2indices_and_rstarts_.at(start_tri_ind);
        auto r_start = cdt_->calcTriangleCenter(cdt_->triangles_[start_tri_ind]);
        to_update_groups_.push_back({start_tri_ind, end_tri_ind, r_end, max_radius_of_agent, inds});
        for (auto [entity_ind, r_start] : inds)
        {
            entity2update_group_ind_.at(entity_ind) = to_update_groups_.size() - 1;
        }
    }
    // const auto time_of_second_part = time.restart().asMicroseconds();
    // std::cout << "1. part took: " << time_of_first_part << " us\n"
    //           << "2. part took: " << time_of_second_part << " us\n";
    // std::cout << "there are : " << to_update_groups_.size() << " paths to find\n";
}

//! \brief divides agents by triangles and for each triangle we do one Astar search
void PathFinder2::issuePaths2(std::vector<PathFinderComponent> &comps,
                              const std::vector<int> &selected_entity_inds,
                              const std::array<int, N_MAX_ENTITIES> &entity2compvec_ind,
                              const sf::Vector2f r_end)
{

    float max_radius_of_agent = 0.;

    // sf::Clock time;

    start_tri2indices_and_rstarts_.clear();
    start_tri_inds_.clear();

    for (const auto selected_ind : selected_entity_inds)
    {
        const auto compvec_ind = entity2compvec_ind.at(selected_ind);
        auto &comp = comps.at(compvec_ind);
        auto start_tri_ind = cdt_->findTriangle(comp.transform.r, false);
        comp.start_tri_ind = start_tri_ind;
        //! I should do ray casting here, but it is not done yet
        // if (rayCast(r_coords[selected_ind], r_end) >=
        //     1) { //! if I see the end directly result is straight line (I should write a setter for path_data)
        //     bc.setPathData(selected_ind, r_end, Edgef());
        //     bc.setPathDataNext(selected_ind, r_end, Edgef());
        //     continue;
        // }

        // if (!comp.needs_update)
        // { //! no need to update something twice
        //     continue;
        // }

        comp.needs_update = false;

        if (start_tri2indices_and_rstarts_.count(start_tri_ind) == 0)
        {
            start_tri2indices_and_rstarts_[start_tri_ind] = {{selected_ind, comp.transform.r}};
            start_tri_inds_.push_back(start_tri_ind);
        }
        else
        {
            start_tri2indices_and_rstarts_[start_tri_ind].emplace_back(selected_ind, comp.transform.r);
        }

        if (comps.at(compvec_ind).radius > max_radius_of_agent)
        {
            max_radius_of_agent = comps.at(compvec_ind).radius;
        }
    }
    // const auto time_of_first_part = time.restart().asMicroseconds();

    const auto end_tri_ind = cdt_->findTriangle(r_end, false);

    bool wtf = true;
    int i = 0;
    for (const auto start_tri_ind : start_tri_inds_)
    {
        std::vector<std::pair<int, sf::Vector2f>> &inds_and_r_starts = start_tri2indices_and_rstarts_.at(start_tri_ind);
        auto r_start = cdt_->calcTriangleCenter(cdt_->triangles_[start_tri_ind]);
        to_update_groups_.push_back({start_tri_ind, end_tri_ind, r_end, max_radius_of_agent, inds_and_r_starts});
    }
    // const auto time_of_second_part = time.restart().asMicroseconds();
    // std::cout << "1. part took: " << time_of_first_part << " us\n"
    //           << "2. part took: " << time_of_second_part << " us\n";
    // std::cout << "there are : " << to_update_groups_.size() << " paths to find\n";
}

//! \brief casts a ray connecting points from and to
//! \param from
//! \param to
//! \returns returns 1 if there are no walls in between points and number smaller than 1 otherwise
//! \returns in case a wall is hit, the number represents how far away from the start point the point of hit lies
//! \returns(the contact point can be found like: from + result * (from - to) / norm(from - to)
float PathFinder2::rayCast(const sf::Vector2f &from, const sf::Vector2f &to) const
{
    const auto &triangles = cdt_->triangles_;
    const auto &vertices = cdt_->vertices_;

    const auto start_tri_ind = cdt_->findTriangle(from, true);
    const auto end_tri_ind = cdt_->findTriangle(to, false);
    if (start_tri_ind == end_tri_ind)
    {
        return 1;
    }

    const auto &start_tri = triangles[start_tri_ind];
    const auto &end_tri = triangles[end_tri_ind];
    const auto &vs0 = start_tri.verts[0];
    const auto &vs1 = start_tri.verts[1];
    const auto &vs2 = start_tri.verts[2];

    const auto new_alpha1s = cdt_->linesIntersect(from, to, vs0, vs1);
    const auto new_alpha2s = cdt_->linesIntersect(from, to, vs1, vs2);
    const auto new_alpha3s = cdt_->linesIntersect(from, to, vs2, vs0);

    int prev_tri_ind = start_tri_ind;
    int tri_ind = start_tri_ind;
    float alpha = -1;
    bool is_constrained = false;
    if (new_alpha1s <= 1 and new_alpha1s >= 0)
    {
        alpha = new_alpha1s;
        is_constrained = start_tri.is_constrained[0];
        tri_ind = start_tri.neighbours[0];
    }
    if (new_alpha2s <= 1 and new_alpha2s >= 0)
    {
        alpha = new_alpha2s;
        is_constrained = start_tri.is_constrained[1];
        tri_ind = start_tri.neighbours[1];
    }
    if (new_alpha3s <= 1 and new_alpha3s >= 0)
    {
        alpha = new_alpha3s;
        is_constrained = start_tri.is_constrained[2];
        tri_ind = start_tri.neighbours[2];
    }

    //! this should not happen but sometimes does and I don't know why yet :(
    if (new_alpha1s < 0 and new_alpha2s < 0 and new_alpha3s < 0)
    {
        //        assert(isInTriangle(from, start_tri, vertices));
        return 0;
    }
    assert(!(new_alpha1s < 0 and new_alpha2s < 0 and new_alpha3s < 0));

    while (tri_ind != end_tri_ind and !is_constrained)
    {
        const auto &tri = triangles[tri_ind];
        const int k = indInTriOf(tri, prev_tri_ind);

        const auto &v0 = tri.verts[next(k)];
        const auto &v1 = tri.verts[prev(k)];
        const auto &v2 = tri.verts[k];

        const auto new_alpha_left = cdt_->linesIntersect(from, to, v0, v1);
        const auto new_alpha_right = cdt_->linesIntersect(from, to, v1, v2);

        if ((new_alpha_left < 0 and new_alpha_right < 0))
        {
            return 0;
        }

        prev_tri_ind = tri_ind;
        if (new_alpha_left <= 1 and new_alpha_left >= 0)
        {
            alpha = new_alpha_left;
            is_constrained = tri.is_constrained[next(k)];
            tri_ind = tri.neighbours[next(k)];
        }
        if (new_alpha_right <= 1 and new_alpha_right >= 0)
        {
            alpha = new_alpha_right;
            is_constrained = tri.is_constrained[prev(k)];
            tri_ind = tri.neighbours[prev(k)];
        }
    }

    if (tri_ind == end_tri_ind and !is_constrained)
    {
        return 1;
    }
    return alpha;
}

//! \brief initializes stuff for Astar on a reduced triangulation
//! \brief walks from start_tri to the nearest crossroads/deadends
//! \brief and calculates distance by Funnel Algorithm from r_start to the found crossroads centers
//! \param r_start
//! \param r_end
//! \param start index in triangulation of the starting triangle
//! \param reduced_start index of a start vertex/edge in the reduced graph
//! \param end   index in triangulation of the finish triangle
//! \param current_shortest_distance
//! \param radius how big the navigated agent is
//! \returns reduced vertices connected directly to the start triangle
std::vector<PathFinder2::AstarReducedDataPQ>
PathFinder2::initializeReducedAstar(const sf::Vector2f &r_start, const sf::Vector2f &r_end, const TriInd start,
                                    const ReducedVertexInd reduced_start, const TriInd end,
                                    float &current_shortest_distance, const float radius,
                                    std::vector<AstarReducedData> &reduced_vertex2astar_data) const
{
    const auto &triangles = cdt_->triangles_;
    const auto &start_tri = triangles[start];
    const auto &rtg_ = *p_rtg_;

    const auto n_reduced = rtg_.vertex2edge_inds2.size();
    for (int reduced_i = 0; reduced_i < n_reduced; ++reduced_i)
    {
        reduced_vertex2astar_data[reduced_i].g_value = MAXFLOAT;
        reduced_vertex2astar_data[reduced_i].h_value =
            dist(cdt_->calcTriangleCenter(triangles[rtg_.vertex2tri_ind[reduced_i]]), r_end);
    }

    std::vector<AstarReducedDataPQ> to_visitx;

    const auto n_free_edges_start = start_tri.countFreeEdges();

    int ind_in_tri_corridor = -1;
    TriInd tri_ind_corridor = -1;
    int prev_in_in_tri_corridor = -1;
    TriInd prev_tri_ind_corridor = -1;

    //! we look from starting triangle to nearest crossroads/deadends
    for (int ind_in_tri = 0; ind_in_tri < 3; ++ind_in_tri)
    {
        if (start_tri.is_constrained[ind_in_tri])
        {
            continue;
        }
        assert(start_tri.neighbours[ind_in_tri] != -1);
        if (n_free_edges_start != 2)
        {
            const auto vertex_ind = rtg_.tri_ind2vertex[start];
            const auto &neighbour_corridor_width = rtg_.edges[rtg_.vertex2edge_inds2[vertex_ind][ind_in_tri]].width;
            if (2 * radius >= neighbour_corridor_width)
            {
                continue;
            }
        }

        ind_in_tri_corridor = ind_in_tri;
        tri_ind_corridor = start_tri.neighbours[ind_in_tri];
        prev_tri_ind_corridor = start;
        int prev_ind_in_tri_corridor = ind_in_tri;

        int n_free = 0;
        Funnel funnel;
        funnel.push_back({r_start, r_start});
        while (!(n_free == 2 || tri_ind_corridor == end))
        { //! walk until end or next crossroads and construct portals along the way

            const auto &tri = triangles[tri_ind_corridor];
            n_free = 0;
            for (int k = 0; k < 3; ++k)
            {
                if (!tri.is_constrained[k] and tri.neighbours[k] != prev_tri_ind_corridor)
                {
                    n_free++;
                    ind_in_tri_corridor = k;
                }
            }
            if (n_free == 2 or tri_ind_corridor == end or n_free == 0)
            {
                break;
            }

            auto left_vertex_of_portal = asFloat(tri.verts[next(ind_in_tri_corridor)]);
            auto right_vertex_of_portal = asFloat(tri.verts[ind_in_tri_corridor]);
            funnel.emplace_back(right_vertex_of_portal, left_vertex_of_portal);

            prev_tri_ind_corridor = tri_ind_corridor;
            prev_ind_in_tri_corridor = ind_in_tri_corridor;
            tri_ind_corridor = tri.neighbours[ind_in_tri_corridor];
        }

        const auto reduced_ind = rtg_.tri_ind2vertex[tri_ind_corridor];
        const auto &tri = triangles[tri_ind_corridor];
        int i = indInTriOf(tri, prev_tri_ind_corridor);

        if (tri_ind_corridor == end)
        { //! we found end

            funnel.emplace_back(r_end, r_end);
            float distance = rtg_.funnelDistance(r_start, r_end, funnel, *cdt_);
            if (distance <= current_shortest_distance)
            {
                reduced_vertex2astar_data.back().prev = reduced_start;
                reduced_vertex2astar_data.back().ind_in_tri = ind_in_tri;
                reduced_vertex2astar_data.back().g_value = distance;
                current_shortest_distance = distance;
            }
        }
        else
        {
            auto new_clock_point = asFloat(tri.verts[i]);
            auto new_anticlock_point = asFloat(tri.verts[next(i)]);
            auto entry_mid_point = (new_clock_point + new_anticlock_point) / 2.0f;
            funnel.push_back({entry_mid_point, entry_mid_point});

            float distance_start_to_center = rtg_.funnelDistance(r_start, entry_mid_point, funnel, *cdt_);

            reduced_vertex2astar_data[reduced_ind].prev = {reduced_start};
            reduced_vertex2astar_data[reduced_ind].ind_in_tri = i;
            reduced_vertex2astar_data[reduced_ind].g_value = distance_start_to_center;
            to_visitx.push_back(
                {distance_start_to_center + reduced_vertex2astar_data[reduced_ind].h_value, reduced_ind});
        }
    }

    return to_visitx;
}

void PathFinder2::findSubOptimalPathCenters(sf::Vector2f r_start, sf::Vector2f r_end, float radius, Funnel &funnel)
{
    findSubOptimalPathCenters(r_start, r_end, radius, funnel, 0);
}

//! \brief finds sequence of triangles such that the path going through centers of the triangles is the shortest
//! \param r_start starting position
//! \param r_end end position
//! \param radius to block paths that are too narrow
//! \param funnel stores data used to create real path going through the triangles
//! \param thread_id which thread runs the job
void PathFinder2::findSubOptimalPathCenters(sf::Vector2f r_start, sf::Vector2f r_end, float radius, Funnel &funnel,
                                            const int thread_id)
{

    const auto &triangles = cdt_->triangles_;
    const auto &vertices = cdt_->vertices_;
    const auto &rtg = *p_rtg_;

    int start = cdt_->findTriangle(r_start, false);
    int end = cdt_->findTriangle(r_end, false);

    auto triangle_is_within_boundary = [&](const Triangle &tri)
    {
        const auto tri_center = cdt_->calcTriangleCenter(tri);
        return cdt_->boundary_.x >= tri_center.x and tri_center.x > 0 and cdt_->boundary_.y >= tri_center.y and
               tri_center.y > 0;
    };
    assert(triangle_is_within_boundary(triangles.at(start)));
    assert(triangle_is_within_boundary(triangles.at(end)));

    //! if we cannot reach the destination due to it being in a different graph component
    if (tri_ind2component_.at(start) != tri_ind2component_.at(end))
    {
        const auto new_end_tri_and_edge =
            closestPointOnNavigableComponent(r_start, end, tri_ind2component_.at(end), tri_ind2component_.at(start));
        end = new_end_tri_and_edge.first;
        const auto to = new_end_tri_and_edge.second;
        r_end = cdt_->calcTriangleCenter(triangles[end]);
    }
    if (start == -1 or end == -1 or start == end)
    {
        return;
    }

    // std::vector<AstarReducedData> reduced_vertex2astar_data(p_rtg_->reduced_vertices.size() + 2);

    const auto &start_tri = triangles.at(start);
    const auto &end_tri = triangles.at(end);

    const int n_free_edges_end = end_tri.countFreeEdges();
    const int n_free_edges_start = start_tri.countFreeEdges();
    int end_edge_ind = -1;
    int reduced_start = rtg.reduced_vertices.size();
    // reduced_vertex2astar_data_[reduced_start].prev = start;
    auto &reduced_vertex2astar_data = thread2reduced_vertex2astar_data_[thread_id];
    reduced_vertex2astar_data.at(reduced_start).prev = start;
    int reduced_end = rtg.reduced_vertices.size() + 1;
    if (n_free_edges_end == 2)
    {
        end_edge_ind = rtg.tri_ind2vertex.at(end); //! this holds edge_indices for triangles that are corridors
    }

    float current_shortest_distance = MAXFLOAT;

    auto to_visit = initializeReducedAstar(r_start, r_end, start, reduced_start, end, current_shortest_distance, radius,
                                           reduced_vertex2astar_data);

    std::priority_queue to_visitx(
        to_visit.begin(), to_visit.end(),
        [&](const AstarReducedDataPQ &a1, const AstarReducedDataPQ &a2)
        { return a1.f_value > a2.f_value; });

    //! if end lies in a corridor we remember which crossorads it connects and search for on of possible ends
    int first_reduced_end;
    int second_reduced_end;
    if (n_free_edges_end == 2)
    {
        second_reduced_end = rtg.tri_ind2vertex[rtg.edges[end_edge_ind].end.current];
        first_reduced_end = rtg.tri_ind2vertex[rtg.edges[end_edge_ind].start.current];
    }
    else
    { //! if it lies on a  reduced vertex we search only for one ind;
        first_reduced_end = rtg.tri_ind2vertex[end];
        second_reduced_end = first_reduced_end;
    }
    auto reached_end = [first_reduced_end, second_reduced_end](int reduced_ind)
    {
        return reduced_ind == first_reduced_end or reduced_ind == second_reduced_end;
    };

    //! this is where Astar starts;
    int prev_reduced_ind = reduced_start;
    TriInd prev_tri_ind = -1;
    TriInd entry_tri_ind = -1;
    int entry_ind_in_tri = -1;
    TriInd current_tri_ind = -1;
    while (!(to_visitx.empty()))
    {

        const auto reduced_ind = to_visitx.top().next;
        const auto &reduced_vertex = rtg.reduced_vertices[reduced_ind];
        to_visitx.pop();

        const float distance_start_to_current_tri_center = reduced_vertex2astar_data[reduced_ind].g_value;

        if (reached_end(reduced_ind))
        {
            // finaliseAstar(...);
            int entry_ind_in_tri;
            int entry_reduced_ind;
            if (n_free_edges_end == 2)
            { //! end triangle is in corridor corresponding to edge or in crossroads
                entry_reduced_ind = reduced_ind;
                if (first_reduced_end == reduced_ind)
                {
                    entry_ind_in_tri = std::find(reduced_vertex.neighbours.begin(), reduced_vertex.neighbours.end(),
                                                 second_reduced_end) -
                                       reduced_vertex.neighbours.begin();
                }
                if (reduced_ind == second_reduced_end)
                {
                    entry_ind_in_tri = std::find(reduced_vertex.neighbours.begin(), reduced_vertex.neighbours.end(),
                                                 first_reduced_end) -
                                       reduced_vertex.neighbours.begin();
                }
            }
            else
            {
                entry_ind_in_tri = reduced_vertex2astar_data[reduced_ind].ind_in_tri;
                entry_reduced_ind = reduced_vertex2astar_data[reduced_ind].prev;
            }

            int n_free;
            int ind_in_tri_corridor;
            int current_tri_ind = rtg.vertex2tri_ind.at(reduced_ind);
            const auto r_center = cdt_->calcTriangleCenter(cdt_->triangles_.at(current_tri_ind));
            Funnel end_funnel;
            end_funnel.push_back({r_center, r_center}); //! funnel is from center of current triangle to r_end;
            if (triangles[current_tri_ind].neighbours[entry_ind_in_tri] != end)
            {
                const auto edge_ind = rtg.vertex2edge_inds2.at(reduced_ind).at(entry_ind_in_tri);
                const auto &edge = rtg.edges.at(edge_ind);

                int end_ind_in_edge = std::find(edge.tri_inds.begin(), edge.tri_inds.end(), end) - edge.tri_inds.begin();
                if (first_reduced_end == reduced_ind)
                {
                    fillFunnelWithCorridorFront(edge, 0, end_ind_in_edge, end_funnel, true);
                }
                if (second_reduced_end == reduced_ind)
                {
                    fillFunnelWithCorridorReversed(edge, edge.tri_inds.size() - 1, end_ind_in_edge, end_funnel, true);
                }
            }
            else
            {
                const auto &tri = triangles.at(current_tri_ind);
                const auto ind_in_tri_corridor = std::find(tri.neighbours.begin(), tri.neighbours.end(), end) - tri.neighbours.begin();
                auto left_vertex_of_portal = asFloat(tri.verts[next(ind_in_tri_corridor)]);
                auto right_vertex_of_portal = asFloat(tri.verts[(ind_in_tri_corridor)]);
                end_funnel.emplace_back(right_vertex_of_portal, left_vertex_of_portal);
            }

            const auto distance_to_end = rtg.funnelDistance(r_center, r_end, end_funnel, *cdt_);

            if (distance_start_to_current_tri_center + distance_to_end <=
                current_shortest_distance)
            { //! update backpointer if path is better
                reduced_vertex2astar_data.back().prev = entry_reduced_ind;
                reduced_vertex2astar_data.back().ind_in_tri = entry_ind_in_tri;
                current_shortest_distance = distance_start_to_current_tri_center + distance_to_end;
            }
            continue;
        }
        for (int k = 0; k < 3; ++k)
        { // look at neighbours in reduced graph
            const auto &next_reduced_ind = reduced_vertex.neighbours[k];
            if (next_reduced_ind != -1)
            {
                const auto &neighbour_corridor_width = reduced_vertex.widths[k];
                //                if(2*radius >= neighbour_corridor_width){ continue; }

                float new_g_value = distance_start_to_current_tri_center + reduced_vertex.lengths[k];
                new_g_value += (neighbour_corridor_width <= 2 * radius) * cdt_->boundary_.x;

                const float &h_value = reduced_vertex2astar_data[next_reduced_ind].h_value;

                if (new_g_value < reduced_vertex2astar_data[next_reduced_ind].g_value)
                {
                    to_visitx.push({new_g_value + h_value, next_reduced_ind});
                    reduced_vertex2astar_data[next_reduced_ind].g_value = new_g_value;
                    reduced_vertex2astar_data[next_reduced_ind].prev = reduced_ind;
                    reduced_vertex2astar_data[next_reduced_ind].ind_in_tri = k;
                }
            }
        }
    }
    fillFunnelData(start, reduced_start, end, reduced_end, funnel, reduced_vertex2astar_data);
}

//! \brief finds sequence of triangles such that the path going through centers of the triangles is the shortest
//! \param r_start starting position
//! \param r_end end position
//! \param radius to block paths that are too narrow
//! \param funnel stores data used to create real path going through the triangles
//! \param thread_id which thread runs the job
void PathFinder2::findSubOptimalPathCentersWater(sf::Vector2f r_start, sf::Vector2f r_end, float radius, Funnel &funnel, int unit_type,
                                                 const int thread_id)
{

    const auto &triangles = cdt_->triangles_;
    const auto &vertices = cdt_->vertices_;
    const auto &rtg = *p_rtg_;

    int start = cdt_->findTriangle(r_start, false);
    int end = cdt_->findTriangle(r_end, false);

    auto triangle_is_within_boundary = [&](const Triangle &tri)
    {
        const auto tri_center = cdt_->calcTriangleCenter(tri);
        return cdt_->boundary_.x >= tri_center.x and tri_center.x > 0 and cdt_->boundary_.y >= tri_center.y and
               tri_center.y > 0;
    };
    assert(triangle_is_within_boundary(triangles.at(start)));
    assert(triangle_is_within_boundary(triangles.at(end)));

    //! if we cannot reach the destination due to it being in a different graph component
    if (tri_ind2component_.at(start) != tri_ind2component_.at(end))
    {
        const auto new_end_tri_and_edge =
            closestPointOnNavigableComponent(r_start, end, tri_ind2component_.at(end), tri_ind2component_.at(start));
        end = new_end_tri_and_edge.first;
        const auto to = new_end_tri_and_edge.second;
        r_end = cdt_->calcTriangleCenter(triangles[end]);
    }
    if (start == -1 or end == -1 or start == end)
    {
        return;
    }

    // std::vector<AstarReducedData> reduced_vertex2astar_data(p_rtg_->reduced_vertices.size() + 2);

    const auto &start_tri = triangles.at(start);
    const auto &end_tri = triangles.at(end);

    const int n_free_edges_end = end_tri.countFreeEdges();
    const int n_free_edges_start = start_tri.countFreeEdges();
    int end_edge_ind = -1;
    int reduced_start = rtg.reduced_vertices.size();
    // reduced_vertex2astar_data_[reduced_start].prev = start;
    auto &reduced_vertex2astar_data = thread2reduced_vertex2astar_data_[thread_id];
    reduced_vertex2astar_data.at(reduced_start).prev = start;
    int reduced_end = rtg.reduced_vertices.size() + 1;
    if (n_free_edges_end == 2)
    {
        end_edge_ind = rtg.tri_ind2vertex.at(end); //! this holds edge_indices for triangles that are corridors
    }

    float current_shortest_distance = MAXFLOAT;

    auto to_visit = initializeReducedAstar(r_start, r_end, start, reduced_start, end, current_shortest_distance, radius,
                                           reduced_vertex2astar_data);

    std::priority_queue to_visitx(
        to_visit.begin(), to_visit.end(),
        [&](const AstarReducedDataPQ &a1, const AstarReducedDataPQ &a2)
        { return a1.f_value > a2.f_value; });

    //! if end lies in a corridor we remember which crossorads it connects and search for on of possible ends
    int first_reduced_end;
    int second_reduced_end;
    if (n_free_edges_end == 2)
    {
        second_reduced_end = rtg.tri_ind2vertex[rtg.edges[end_edge_ind].end.current];
        first_reduced_end = rtg.tri_ind2vertex[rtg.edges[end_edge_ind].start.current];
    }
    else
    { //! if it lies on a  reduced vertex we search only for one ind;
        first_reduced_end = rtg.tri_ind2vertex[end];
        second_reduced_end = first_reduced_end;
    }
    auto reached_end = [first_reduced_end, second_reduced_end](int reduced_ind)
    {
        return reduced_ind == first_reduced_end or reduced_ind == second_reduced_end;
    };

    //! this is where Astar starts;
    int prev_reduced_ind = reduced_start;
    TriInd prev_tri_ind = -1;
    TriInd entry_tri_ind = -1;
    int entry_ind_in_tri = -1;
    TriInd current_tri_ind = -1;
    while (!(to_visitx.empty()))
    {

        const auto reduced_ind = to_visitx.top().next;
        const auto &reduced_vertex = rtg.reduced_vertices[reduced_ind];
        to_visitx.pop();

        const float distance_start_to_current_tri_center = reduced_vertex2astar_data[reduced_ind].g_value;

        if (reached_end(reduced_ind))
        {
            // finaliseAstar(...);
            int entry_ind_in_tri;
            int entry_reduced_ind;
            if (n_free_edges_end == 2)
            { //! end triangle is in corridor corresponding to edge or in crossroads
                entry_reduced_ind = reduced_ind;
                if (first_reduced_end == reduced_ind)
                {
                    entry_ind_in_tri = std::find(reduced_vertex.neighbours.begin(), reduced_vertex.neighbours.end(),
                                                 second_reduced_end) -
                                       reduced_vertex.neighbours.begin();
                }
                if (reduced_ind == second_reduced_end)
                {
                    entry_ind_in_tri = std::find(reduced_vertex.neighbours.begin(), reduced_vertex.neighbours.end(),
                                                 first_reduced_end) -
                                       reduced_vertex.neighbours.begin();
                }
            }
            else
            {
                entry_ind_in_tri = reduced_vertex2astar_data[reduced_ind].ind_in_tri;
                entry_reduced_ind = reduced_vertex2astar_data[reduced_ind].prev;
            }

            int n_free;
            int ind_in_tri_corridor;
            int current_tri_ind = rtg.vertex2tri_ind.at(reduced_ind);
            const auto r_center = cdt_->calcTriangleCenter(cdt_->triangles_.at(current_tri_ind));
            Funnel end_funnel;
            end_funnel.push_back({r_center, r_center}); //! funnel is from center of current triangle to r_end;
            if (triangles[current_tri_ind].neighbours[entry_ind_in_tri] != end)
            {
                const auto edge_ind = rtg.vertex2edge_inds2.at(reduced_ind).at(entry_ind_in_tri);
                const auto &edge = rtg.edges.at(edge_ind);

                int end_ind_in_edge = std::find(edge.tri_inds.begin(), edge.tri_inds.end(), end) - edge.tri_inds.begin();
                if (first_reduced_end == reduced_ind)
                {
                    fillFunnelWithCorridorFront(edge, 0, end_ind_in_edge, end_funnel, true);
                }
                if (second_reduced_end == reduced_ind)
                {
                    fillFunnelWithCorridorReversed(edge, edge.tri_inds.size() - 1, end_ind_in_edge, end_funnel, true);
                }
            }
            else
            {
                const auto &tri = triangles.at(current_tri_ind);
                const auto ind_in_tri_corridor = std::find(tri.neighbours.begin(), tri.neighbours.end(), end) - tri.neighbours.begin();
                auto left_vertex_of_portal = asFloat(tri.verts[next(ind_in_tri_corridor)]);
                auto right_vertex_of_portal = asFloat(tri.verts[(ind_in_tri_corridor)]);
                end_funnel.emplace_back(right_vertex_of_portal, left_vertex_of_portal);
            }

            const auto distance_to_end = rtg.funnelDistance(r_center, r_end, end_funnel, *cdt_);

            if (distance_start_to_current_tri_center + distance_to_end <=
                current_shortest_distance)
            { //! update backpointer if path is better
                reduced_vertex2astar_data.back().prev = entry_reduced_ind;
                reduced_vertex2astar_data.back().ind_in_tri = entry_ind_in_tri;
                current_shortest_distance = distance_start_to_current_tri_center + distance_to_end;
            }
            continue;
        }
        for (int k = 0; k < 3; ++k)
        { // look at neighbours in reduced graph
            const auto &next_reduced_ind = reduced_vertex.neighbours[k];
            if (next_reduced_ind != -1)
            {
                const auto &neighbour_corridor_width = reduced_vertex.widths[k];
                //                if(2*radius >= neighbour_corridor_width){ continue; }

                float new_g_value = distance_start_to_current_tri_center + reduced_vertex.lengths[k];
                new_g_value += (neighbour_corridor_width <= 2 * radius) * cdt_->boundary_.x;
                const float &h_value = reduced_vertex2astar_data[next_reduced_ind].h_value;

                if (new_g_value < reduced_vertex2astar_data[next_reduced_ind].g_value)
                {
                    to_visitx.push({new_g_value + h_value, next_reduced_ind});
                    reduced_vertex2astar_data[next_reduced_ind].g_value = new_g_value;
                    reduced_vertex2astar_data[next_reduced_ind].prev = reduced_ind;
                    reduced_vertex2astar_data[next_reduced_ind].ind_in_tri = k;
                }
            }
        }
    }
    fillFunnelData(start, reduced_start, end, reduced_end, funnel, reduced_vertex2astar_data);
}

//! \brief finds sequence of triangles such that the path going through centers of the triangles is the shortest
//! \param r_start starting position
//! \param r_end end position
//! \param radius to block paths that are too narrow
//! \param funnel stores data used to create real path going through the triangles
//! \param thread_id which thread runs the job?
void PathFinder2::findSubOptimalPathCenters2(sf::Vector2f r_start, sf::Vector2f r_end, float radius, Funnel &funnel,
                                             const int thread_id)
{

    const auto &triangles = cdt_->triangles_;
    const auto &vertices = cdt_->vertices_;
    const auto &rtg = *p_rtg_;

    int start = cdt_->findTriangle(r_start, false);
    int end = cdt_->findTriangle(r_end, false);

    auto triangle_is_within_boundary = [&](const Triangle &tri)
    {
        const auto tri_center = cdt_->calcTriangleCenter(tri);
        return cdt_->boundary_.x >= tri_center.x and tri_center.x > 0 and cdt_->boundary_.y >= tri_center.y and
               tri_center.y > 0;
    };
    assert(triangle_is_within_boundary(triangles.at(start)));
    assert(triangle_is_within_boundary(triangles.at(end)));

    //! if we cannot reach the destination due to it being in a different graph component
    if (tri_ind2component_.at(start) != tri_ind2component_.at(end))
    {
        const auto new_end_tri_and_edge =
            closestPointOnNavigableComponent(r_start, end, tri_ind2component_.at(end), tri_ind2component_.at(start));
        end = new_end_tri_and_edge.first;
        const auto to = new_end_tri_and_edge.second;
        r_end = cdt_->calcTriangleCenter(triangles[end]);
    }
    if (start == -1 or end == -1 or start == end)
    {
        return;
    }

    // std::vector<AstarReducedData> reduced_vertex2astar_data(p_rtg_->reduced_vertices.size() + 2);

    const auto &start_tri = triangles.at(start);
    const auto &end_tri = triangles.at(end);

    //! find correspondign reduced indices from triangles
    const int n_free_edges_end = end_tri.countFreeEdges();
    const int n_free_edges_start = start_tri.countFreeEdges();
    int end_edge_ind = -1;
    int reduced_start = rtg.reduced_vertices.size();
    // reduced_vertex2astar_data_[reduced_start].prev = start;
    auto &reduced_vertex2astar_data = thread2reduced_vertex2astar_data_[thread_id];
    reduced_vertex2astar_data.at(reduced_start).prev = start;
    int reduced_end = rtg.reduced_vertices.size() + 1;
    if (n_free_edges_end == 2)
    {
        end_edge_ind = rtg.tri_ind2vertex.at(end); //! this holds edge_indices for triangles that are corridors
    }

    float current_shortest_distance = MAXFLOAT;

    auto to_visit = initializeReducedAstar(r_start, r_end, start, reduced_start, end, current_shortest_distance, radius,
                                           reduced_vertex2astar_data);

    std::priority_queue to_visitx(
        to_visit.begin(), to_visit.end(),
        [&](const AstarReducedDataPQ &a1, const AstarReducedDataPQ &a2)
        { return a1.f_value > a2.f_value; });

    //! if end lies in a corridor we remember which crossorads it connects and search for on of possible ends
    int first_reduced_end;
    int second_reduced_end;
    if (n_free_edges_end == 2)
    {
        second_reduced_end = rtg.tri_ind2vertex[rtg.edges[end_edge_ind].end.current];
        first_reduced_end = rtg.tri_ind2vertex[rtg.edges[end_edge_ind].start.current];
    }
    else
    { //! if it lies on a  reduced vertex we search only for one ind;
        first_reduced_end = rtg.tri_ind2vertex[end];
        second_reduced_end = first_reduced_end;
    }
    auto reached_end = [first_reduced_end, second_reduced_end](int reduced_ind)
    {
        return reduced_ind == first_reduced_end or reduced_ind == second_reduced_end;
    };

    //! this is where Astar starts;
    int prev_reduced_ind = reduced_start;
    TriInd prev_tri_ind = -1;
    TriInd entry_tri_ind = -1;
    int entry_ind_in_tri = -1;
    TriInd current_tri_ind = -1;
    while (!(to_visitx.empty()))
    {

        const auto reduced_ind = to_visitx.top().next;
        const auto &reduced_vertex = rtg.reduced_vertices[reduced_ind];
        to_visitx.pop();

        const float distance_start_to_current_tri_center = reduced_vertex2astar_data[reduced_ind].g_value;

        if (reached_end(reduced_ind))
        {
            // finaliseAstar(...);
            int entry_ind_in_tri;
            int entry_reduced_ind;
            if (n_free_edges_end == 2)
            { //! end triangle is in corridor corresponding to edge or in crossroads
                entry_reduced_ind = reduced_ind;
                if (first_reduced_end == reduced_ind)
                {
                    entry_ind_in_tri = std::find(reduced_vertex.neighbours.begin(), reduced_vertex.neighbours.end(),
                                                 second_reduced_end) -
                                       reduced_vertex.neighbours.begin();
                }
                if (reduced_ind == second_reduced_end)
                {
                    entry_ind_in_tri = std::find(reduced_vertex.neighbours.begin(), reduced_vertex.neighbours.end(),
                                                 first_reduced_end) -
                                       reduced_vertex.neighbours.begin();
                }
            }
            else
            {
                entry_ind_in_tri = reduced_vertex2astar_data[reduced_ind].ind_in_tri;
                entry_reduced_ind = reduced_vertex2astar_data[reduced_ind].prev;
            }

            int n_free;
            int ind_in_tri_corridor;
            int current_tri_ind = rtg.vertex2tri_ind.at(reduced_ind);

            fillFunnelData(start, reduced_start, end, reduced_end, funnel, reduced_vertex2astar_data);

            continue;
        }
        for (int k = 0; k < 3; ++k)
        { // look at neighbours in reduced graph
            const auto &next_reduced_ind = reduced_vertex.neighbours[k];
            if (next_reduced_ind != -1)
            {
                const auto &neighbour_corridor_width = reduced_vertex.widths[k];
                //                if(2*radius >= neighbour_corridor_width){ continue; }

                float new_g_value = distance_start_to_current_tri_center + reduced_vertex.lengths[k];
                new_g_value += (neighbour_corridor_width <= 2 * radius) * cdt_->boundary_.x;

                const float &h_value = reduced_vertex2astar_data[next_reduced_ind].h_value;

                if (new_g_value < reduced_vertex2astar_data[next_reduced_ind].g_value)
                {
                    to_visitx.push({new_g_value + h_value, next_reduced_ind});
                    reduced_vertex2astar_data[next_reduced_ind].g_value = new_g_value;
                    reduced_vertex2astar_data[next_reduced_ind].prev = reduced_ind;
                    reduced_vertex2astar_data[next_reduced_ind].ind_in_tri = k;
                }
            }
        }
    }
    fillFunnelData(start, reduced_start, end, reduced_end, funnel, reduced_vertex2astar_data);
}

//! \param r  starting point
//! \param start_tri_ind index of triangle containing starting point of the sought path
//! \param end_component triangulation component of the end point
//! \param navigable_component triangulation component of the start point
//! \returns first in pair is triangle ind of the given navigable_component
//! \returns  second is ind_in_tri looking into end_component
std::pair<TriInd, int> PathFinder2::closestPointOnNavigableComponent(const sf::Vector2f &r, const TriInd start_tri_ind,
                                                                     const int end_component,
                                                                     const int navigable_component) const
{
    const auto &triangles = cdt_->triangles_;
    const auto &vertices = cdt_->vertices_;
    auto current_tri_ind = start_tri_ind;
    std::queue<TriInd> to_visit({current_tri_ind});

    auto current_component = end_component;
    auto prev_tri_ind = current_tri_ind;
    while (current_component != navigable_component and !to_visit.empty())
    {
        current_tri_ind = to_visit.front();
        const auto &tri = triangles[current_tri_ind];
        to_visit.pop();
        const auto v0 = tri.verts[0];
        const auto v1 = tri.verts[1];
        const auto v2 = tri.verts[2];

        Edgef line1(cdt_->calcTriangleCenter(triangles[start_tri_ind]), r);

        for (int k = 0; k < 3; ++k)
        {
            const auto neighbour = tri.neighbours[k];
            if (neighbour != prev_tri_ind and neighbour != -1)
            {
                Edgef line2(asFloat(tri.verts[k]), asFloat(tri.verts[next(k)]));
                if (cdt_->linesIntersect(line1, line2))
                {
                    to_visit.push(neighbour);
                }
            }
        }
        prev_tri_ind = current_tri_ind;
        current_component = tri_ind2component_[current_tri_ind];
    }
    const auto to_end_component = indInTriOf(triangles[current_tri_ind], prev_tri_ind);
    return {current_tri_ind, to_end_component};
}

void PathFinder2::fillFunnelData(const TriInd start, const int reduced_start, const TriInd end, const int reduced_end,
                                 Funnel &funnel,
                                 const std::vector<AstarReducedData> &reduced_vertex2astar_data) const
{

    int next_tri_ind = reduced_vertex2astar_data.back().prev;
    int next_ind_in_tri = reduced_vertex2astar_data.back().ind_in_tri;
    int prev_ind_in_tri;

    const auto &rtg_ = *p_rtg_;
    const auto &next_tri = cdt_->triangles_[next_tri_ind];
    const auto &start_tri = cdt_->triangles_[start];
    const auto &end_tri = cdt_->triangles_[end];

    auto count_free_edges = [](const Triangle &tri)
    {
        return static_cast<int>(!tri.is_constrained[0] + !tri.is_constrained[1] + !tri.is_constrained[2]);
    };

    const int n_free_edges_end = count_free_edges(end_tri);
    const int n_free_edges_start = count_free_edges(start_tri);

    int end_edge_ind = -1;
    if (n_free_edges_end == 2)
    {
        end_edge_ind = rtg_.tri_ind2vertex.at(end); //! this holds edge_indices for triangles that are corridors
        //! and tri_indices for crossroad and dead-ends
    }

    if (next_tri_ind == reduced_start)
    {

        int edge_ind = rtg_.tri_ind2vertex[start];
        if (n_free_edges_start != 2)
        {
            edge_ind = rtg_.vertex2edge_inds2.at(rtg_.tri_ind2vertex[start])[next_ind_in_tri];
        }
        const auto &edge = rtg_.edges.at(edge_ind);

        bool edge_is_reversed = false;
        bool with_end = false;
        bool end_is_corridor = n_free_edges_end == 2;
        bool start_is_corridor = n_free_edges_start == 2;

        //! corridors are generally not too big (tens of triangles max) and this gets called once per path search so
        //! linear search is fine
        const auto start_ind_in_corridor =
            std::find(edge.tri_inds.begin(), edge.tri_inds.end(), start) - edge.tri_inds.begin();
        const auto end_ind_in_corridor =
            std::find(edge.tri_inds.begin(), edge.tri_inds.end(), end) - edge.tri_inds.begin();

        int last_ind = edge.tri_inds.size() - 1;
        if (end_is_corridor && !start_is_corridor)
        {
            edge.start.current == start ? edge_is_reversed = true : edge_is_reversed = false;
            fillFunnelWithCorridor(edge, end_ind_in_corridor, !edge_is_reversed * last_ind, funnel, edge_is_reversed,
                                   with_end = false);
        }
        if (end_is_corridor && start_is_corridor)
        { //! minus one because last triangle in corridor contains start
          //! point and so we do not leave it
            end_ind_in_corridor < start_ind_in_corridor ? edge_is_reversed = false : edge_is_reversed = true;
            fillFunnelWithCorridor(edge, end_ind_in_corridor, start_ind_in_corridor + 2 * (edge_is_reversed)-1, funnel,
                                   edge_is_reversed, with_end = false);
        }
        if (!end_is_corridor and !start_is_corridor)
        {
            edge.start.current == start ? edge_is_reversed = true : edge_is_reversed = false;
            fillFunnelWithCorridor(edge, edge_is_reversed * last_ind, !edge_is_reversed * last_ind, funnel,
                                   edge_is_reversed, with_end = true);
        }
        if (!end_is_corridor and start_is_corridor)
        {
            edge.start.current == end ? edge_is_reversed = false : edge_is_reversed = true;
            fillFunnelWithCorridor(edge, edge_is_reversed * last_ind, start_ind_in_corridor + 2 * (edge_is_reversed)-1,
                                   funnel, edge_is_reversed, with_end = true);
        }
        return;
    }
    int prev_tri_ind = -1;
    int prev_reduced_ind = -1;
    int reduced_ind = reduced_vertex2astar_data.back().prev;
    // int reduced_ind = reduced_vertex2astar_data_.back().prev;
    while (reduced_ind != reduced_start)
    {
        next_tri_ind = rtg_.vertex2tri_ind[reduced_ind];

        const int edge_ind = rtg_.vertex2edge_inds2.at(reduced_ind)[next_ind_in_tri];
        if (edge_ind == -1 or edge_ind > rtg_.edges.size() - 1)
        {
            return;
        }
        const auto &edge = rtg_.edges.at(edge_ind);

        int first_ind_in_corridor = 0;
        bool with_start_or_end = true;
        if (edge_ind == end_edge_ind and
            n_free_edges_end ==
                2)
        { //! end is in corridor this part is done only when walking from end triangle to next crossroads
            first_ind_in_corridor = std::find(edge.tri_inds.begin(), edge.tri_inds.end(), end) - edge.tri_inds.begin();
            with_start_or_end = false;
        }

        int tri_ind;
        if (edge.end.current == next_tri_ind)
        {
            fillFunnelWithCorridor(edge, first_ind_in_corridor, edge.tri_inds.size() - 1, funnel, false,
                                   with_start_or_end);
            tri_ind = edge.end.current;
        }
        if (edge.start.current == next_tri_ind)
        {
            if (!(edge_ind == end_edge_ind and n_free_edges_end == 2))
            {
                first_ind_in_corridor = edge.tri_inds.size() - 1;
            }
            fillFunnelWithCorridor(edge, first_ind_in_corridor, 0, funnel, true, with_start_or_end);
            tri_ind = edge.start.current;
        }

        prev_tri_ind = next_tri_ind;
        prev_ind_in_tri = next_ind_in_tri;
        //        next_ind_in_tri = tri_edge2shortest_path_[tri_ind][prev_ind_in_tri].second;
        assert(reduced_ind >= 0 and reduced_ind < rtg_.reduced_vertices.size());
        prev_reduced_ind = reduced_ind;
        // next_ind_in_tri = reduced_vertex2astar_data_[reduced_ind].ind_in_tri;
        // reduced_ind = reduced_vertex2astar_data_[reduced_ind].prev;
        next_ind_in_tri = reduced_vertex2astar_data[reduced_ind].ind_in_tri;
        reduced_ind = reduced_vertex2astar_data[reduced_ind].prev;
        if (prev_reduced_ind == reduced_ind)
        { //! this means no path was found! path will be straight line
            return;
        }
    }

    int edge_ind = rtg_.tri_ind2vertex[start];
    if (n_free_edges_start != 2)
    {
        edge_ind = rtg_.vertex2edge_inds2[prev_reduced_ind][next_ind_in_tri];
    }

    const auto &edge = rtg_.edges.at(edge_ind);
    if (edge.end.current == prev_tri_ind)
    {
        int last_ind = 0;
        if (n_free_edges_start == 2)
        { //! end is in corridor
            last_ind = std::find(edge.tri_inds.begin(), edge.tri_inds.end(), start) - edge.tri_inds.begin();
        }
        fillFunnelWithCorridor(edge, edge.tri_inds.size() - 1, last_ind + 1, funnel, true, true);
    }
    if (edge.start.current == prev_tri_ind)
    {
        int last_ind = edge.tri_inds.size();
        if (n_free_edges_start == 2)
        { //! end is in corridor
            last_ind = std::find(edge.tri_inds.begin(), edge.tri_inds.end(), start) - edge.tri_inds.begin();
        }
        fillFunnelWithCorridor(edge, 0, last_ind - 1, funnel, false, true);
    }
}

PathFinder2::TriangleWidth::TriangleWidth(Triangle &tri, const Triangulation &cdt)
{

    int n_constraints = 0;
    int free_vertex_ind_in_tri = 0;

    for (int i = 0; i < 3; ++i)
    {
        if (tri.is_constrained[i])
        {
            n_constraints++;
            free_vertex_ind_in_tri = (i + 2) % 3;
            widths[i] = MAXFLOAT;
        }
    }
    if (n_constraints == 1)
    { //! only one vertex can be "circled around" so need to calc. just one width
        const auto v_free = tri.verts[free_vertex_ind_in_tri];
        const auto v1 = tri.verts[next(free_vertex_ind_in_tri)];
        const auto v2 = tri.verts[prev(free_vertex_ind_in_tri)];
        //        const auto v1 = v1.vertices_[v_ind_1];
        //        const auto v2 = cdt.vertices_[v_ind_2];

        const auto d1 = dist(v_free, v1);
        const auto d2 = dist(v_free, v2);

        const auto v21 = static_cast<sf::Vector2f>(v2 - v1);
        const auto vf1 = static_cast<sf::Vector2f>(v_free - v1);
        const auto dv_free_to_edge = vf1 - dot(vf1, v21) / dot(v21, v21) * v21;
        const auto d_norm = dot(dv_free_to_edge, dv_free_to_edge);

        widths[free_vertex_ind_in_tri] = std::min({d1, d2, d_norm});
    }
    else if (n_constraints == 0)
    { //! each vertex can be "circled around" and thus has width
        const auto l_edge_0 = dist(tri.verts[0], tri.verts[1]);
        const auto l_edge_1 = dist(tri.verts[1], tri.verts[2]);
        const auto l_edge_2 = dist(tri.verts[2], tri.verts[0]);
        widths[0] = std::min(l_edge_0, l_edge_2);
        widths[1] = std::min(l_edge_0, l_edge_1);
        widths[2] = std::min(l_edge_1, l_edge_2);
    }
}

//! \brief imagine r_to_push being between r_prev and r_next
//! \brief r_to_push is pushed in the average of normal directions of (r_to_push - r_prev) and (r_next - r_to_push)
//! \param r_to_push point to push away
//! \param r_prev
//! \param r_next
//! \param distance how far away the point is pushed
//! \returns path portal coming from pushed
Edgef pushAwayFromCorner2(sf::Vector2f &r_to_push, const sf::Vector2f &r_prev, const sf::Vector2f &r_next,
                          const float distance, bool left)
{

    sf::Vector2f n10({0, 0});
    sf::Vector2f n20({0, 0});

    const auto v0 = r_to_push;
    const auto v1 = r_prev;
    const auto v2 = r_next;
    n10 = {(v1.y - v0.y), -(v1.x - v0.x)};
    n20 = {(v2.y - v0.y), -(v2.x - v0.x)};
    n10 /= norm(n10);
    n20 /= norm(n20);
    if (dot(n10, v2 - v0) < 0)
    {
        n10 *= -1.f;
    }
    if (dot(n20, v0 - v1) < 0)
    {
        n20 *= -1.f;
    }

    r_to_push += (n10 + n20) * distance / norm(n10 + n20) * (float)left;
    Edgef p;
    p.from = r_to_push;
    p.t = (n10 + n20) / norm(n10 + n20);
    p.l = 100 * distance; //!
    return p;
}

//! \brief imagine r_to_push being between r_prev and r_next
//! \brief r_to_push is pushed in the average of normal directions of (r_to_push - r_prev) and (r_next - r_to_push)
//! \param r_to_push point to push away
//! \param r_prev
//! \param r_next
//! \param distance how far away the point is pushed
//! \returns path portal coming from pushed
Edgef createPortal(sf::Vector2f &r_to_push, const sf::Vector2f &r_prev, const sf::Vector2f &r_next, const float distance, bool left)
{

    const auto v0 = r_to_push;
    const auto v1 = r_prev;
    const auto v2 = r_next;
    const auto dv20 = (v2 - v0) / norm(v2 - v0);
    const auto dv10 = (v1 - v0) / norm(v1 - v0);
    auto vertex_normal = (dv10 + dv20) / norm(dv20 + dv10);
    if ((2.f * left - 1.f) * sign(r_next, r_prev, r_to_push) > 0)
    {
        vertex_normal *= -1.f;
    }

    Edgef p;
    p.from = r_to_push + vertex_normal * distance;
    r_to_push += vertex_normal * distance;
    p.t = vertex_normal;
    p.l = 10 * distance; //!
    return p;
}

//! \brief crates shorte.st path inside a Polygon defined by FunnelData
//! \param r_start starting position of the path
//! \param r_end end position of the path
//! \param radius defines how much the path will be pushed away from corners
//! \returns shortest path inside funnel and portals (line segments from path points indicating that I passed the path
//! point)
PathFinder2::PathAndPortals PathFinder2::pathFromFunnel2(const sf::Vector2f r_start, const sf::Vector2f r_end,
                                                         const float radius, Funnel &funnel) const
{

    PathAndPortals path_and_portals;
    auto &smoothed_path = path_and_portals.path;
    auto &portals = path_and_portals.portals;
    path_and_portals.path = {r_start};
    path_and_portals.portals = {Edgef()};

    const auto &triangles = cdt_->triangles_;
    const auto &vertices = cdt_->vertices_;

    sf::Vector2f right;
    sf::Vector2f left;
    sf::Vector2f portal_apex = r_start;
    sf::Vector2f portal_right = r_start;
    sf::Vector2f portal_left = r_start;

    int right_index = 0;
    int left_index = 0;
    int apex_index = 0;

    std::vector<Edgef> left_portals;
    std::vector<Edgef> right_portals;

    left_portals.push_back(Edgef());
    right_portals.push_back(Edgef());
    sf::Vector2f prev_left = r_start;
    sf::Vector2f prev_right = r_start;
    int i_first_same = 0;
    bool is_first = true;

    std::vector<int> unique_left({0});
    std::vector<int> unique_right({0});
    for (int i = 1; i < funnel.size(); ++i)
    {
        auto &next_r = funnel[i].first;
        auto &next_l = funnel[i].second;
        if (!vequal(next_l, prev_left))
        {
            unique_left.push_back(i);
            prev_left = next_l;
        }
        if (!vequal(next_r, prev_right))
        {
            unique_right.push_back(i);
            prev_right = next_r;
        }
    }

    //! push path points away from walls
    const auto push_distance = radius;
    for (int i = 1; i < unique_left.size() - 1; ++i)
    {
        const auto &prev_unique_left = funnel[unique_left[i - 1]].second;
        const auto &next_unique_left = funnel[unique_left[i + 1]].second;
        for (int j = unique_left[i]; j < unique_left[i + 1]; ++j)
        {
            auto &mid_left = funnel[j].second;
            const auto left_portal = pushAwayFromCorner2(mid_left, prev_unique_left, next_unique_left, push_distance, false);
            left_portals.push_back(left_portal);
        }
    }
    for (int i = 1; i < unique_right.size() - 1; ++i)
    {
        const auto &prev_unique_right = funnel[unique_right[i - 1]].first;
        const auto &next_unique_right = funnel[unique_right[i + 1]].first;
        for (int j = unique_right[i]; j < unique_right[i + 1]; ++j)
        {
            auto &mid_right = funnel[j].first;
            const auto right_portal =
                pushAwayFromCorner2(mid_right, prev_unique_right, next_unique_right, push_distance, false);
            right_portals.push_back(right_portal);
        }
    }

    left_portals.push_back(Edgef());
    right_portals.push_back(Edgef());
    for (int i = 1; i < funnel.size(); ++i)
    {

        right = funnel[i].first;
        left = funnel[i].second;

        if (sign(portal_apex, portal_right, right) <= 0.f)
        { //! if the portal shrank from right
            auto is_same_point = vequal(portal_apex, portal_right);
            if (is_same_point || sign(portal_apex, portal_left, right) >
                                     0.f)
            { //! if the new right segment of the portal crosses the left segment
                portal_right = right;
                right_index = i;
            }
            else if (vequal(portal_right, right))
            {
                right_index = i;
            }
            else
            {
                portals.push_back(left_portals.at(left_index));

                smoothed_path.push_back(portal_left);
                portal_apex = portal_left;
                portal_left = portal_apex;
                portal_right = portal_apex;
                apex_index = left_index;
                right_index = left_index;
                i = left_index;
                continue;
            }
        }

        if (sign(portal_apex, portal_left, left) >= 0.f)
        { //! same as above but we move left portal segment
            auto is_same_point = vequal(portal_apex, portal_left);
            //            if(vequal(portal_left, left)){ continue;}
            if (is_same_point or sign(portal_apex, portal_right, left) < 0.f)
            {
                portal_left = left;
                left_index = i;
            }
            else if (vequal(portal_left, left))
            {
                left_index = i;
            }
            else
            {
                portals.push_back(right_portals.at(right_index));

                smoothed_path.push_back(portal_right);
                portal_apex = portal_right;
                portal_right = portal_apex;
                portal_left = portal_apex;
                apex_index = right_index;
                left_index = right_index;
                i = right_index;
                continue;
            }
        }
    }
    smoothed_path.push_back(r_end);
    portals.push_back(Edgef());
    return path_and_portals;
}


//! \brief crates shorte.st path inside a Polygon defined by FunnelData
//! \param r_start starting position of the path
//! \param r_end end position of the path
//! \param radius defines how much the path will be pushed away from corners
//! \returns shortest path inside funnel and portals (line segments from path points indicating that I passed the path
//! point)
PathFinder2::PathAndPortals PathFinder2::pathFromFunnel(const sf::Vector2f r_start, const sf::Vector2f r_end,
                                                        const float radius, Funnel &funnel) const
{

    PathAndPortals path_and_portals;
    auto &smoothed_path = path_and_portals.path;
    auto &portals = path_and_portals.portals;
    path_and_portals.path = {r_start};
    path_and_portals.portals = {Edgef()};

    const auto &triangles = cdt_->triangles_;
    const auto &vertices = cdt_->vertices_;

    sf::Vector2f right;
    sf::Vector2f left;
    sf::Vector2f portal_apex = r_start;
    sf::Vector2f portal_right = r_start;
    sf::Vector2f portal_left = r_start;

    int right_index = 0;
    int left_index = 0;
    int apex_index = 0;

    std::vector<Edgef> left_portals;
    std::vector<Edgef> right_portals;

    left_portals.push_back(Edgef());
    right_portals.push_back(Edgef());
    Vertex prev_left = static_cast<Vertex>(r_start);
    Vertex prev_right = static_cast<Vertex>(r_start);
    int i_first_same = 0;
    bool is_first_l = true;
    bool is_first_r = true;

    std::unordered_map<Vertex, sf::Vector2f, VertexHash> vertex2normal_left;
    std::unordered_map<Vertex, sf::Vector2f, VertexHash> vertex2normal_right;

    std::vector<int> unique_left({0});
    std::vector<Vertex> unique_left_points({ static_cast<Vertex>(funnel[0].first)});
    std::vector<int> unique_right({0});
    std::vector<Vertex> unique_right_points({static_cast<Vertex>(funnel[0].first)});
    for (int i = 1; i < funnel.size(); ++i)
    {
        auto next_r = static_cast<Vertex>(funnel[i].first);
        auto next_l = static_cast<Vertex>(funnel[i].second);
        if(!vequal(next_l, prev_left) || is_first_r)
        {
            unique_left.push_back(i);
            unique_left_points.push_back(next_l);
            prev_left = next_l;
            is_first_r = false;
        }
        if (!vequal(next_r, prev_right) || is_first_l)
        {
            unique_right.push_back(i);
            unique_right_points.push_back(next_r);
            prev_right = next_r;
            is_first_l = false;
        }
    }

    //! push path points away from walls
    const auto push_distance = radius;
    for (int i = 1; i < unique_left_points.size() - 1; ++i)
    {
        const auto &prev_left = funnel[unique_left[i - 1]].second;
        const auto &next_left = funnel[unique_left[i + 1]].second;
        const auto &left = funnel[unique_left[i]].second;

        sf::Vector2f t1 = (prev_left - left) / norm(prev_left - left);
        sf::Vector2f t0 = (next_left - left) / norm(next_left - left);
        sf::Vector2f vertex_normal = (t1 + t0) / (norm(t1 + t0));

        if (sign(next_left, prev_left, left) < 0)
        {
            vertex_normal *= -1.f;
        }
        vertex2normal_left[static_cast<Vertex>(left)] = vertex_normal;

        for (int j = unique_left[i]; j < unique_left[i + 1]; ++j)
        {
            auto &mid_left = funnel[j].second;
            const auto left_portal = createPortal(mid_left, prev_left, next_left, push_distance, false);
            left_portals.push_back(left_portal);
        }
    }

    //! push path points away from walls
    for (int i = 1; i < unique_right_points.size() - 1; ++i)
    {
        const auto &prev_right = funnel[unique_right[i - 1]].first;
        const auto &next_right = funnel[unique_right[i + 1]].first;
        const auto &right = funnel[unique_right[i]].first;

        sf::Vector2f t1 = (prev_right - right) / norm(prev_right - right);
        sf::Vector2f t0 = (next_right - right) / norm(next_right - right);
        sf::Vector2f vertex_normal = (t1 + t0) / (norm(t1 + t0));

        if (sign(next_right, prev_right, right) < 0)
        {
            vertex_normal *= -1.f;
        }
        vertex2normal_right[static_cast<Vertex>(right)] = vertex_normal;

        for (int j = unique_right[i]; j < unique_right[i + 1]; ++j)
        {
            auto &mid_right = funnel[j].first;
            const auto right_portal = createPortal(mid_right, prev_right, next_right, push_distance, true);
            right_portals.push_back(right_portal);
        }
    }

    // if(right_portals.size() != left_portals.size()){
    //     smoothed_path.push_back(r_end);
    //     portals.push_back(Edgef());
    //     return path_and_portals;
    // }
    // dumpFunnelToFile2(unique_left_points, unique_right_points, "funnel-test-unique.dat");
    // dumpFunnelToFile(funnel, 3, "funnel-test.dat");

    try{
    left_portals.push_back(Edgef());
    right_portals.push_back(Edgef());
    for (int i = 1; i < funnel.size(); ++i)
    {

        right = funnel[i].first;
        left = funnel[i].second;

        if (sign(portal_apex, portal_right, right) <= 0.f)
        { //! if the portal shrank from right
            auto is_same_point = vequal(portal_apex, portal_right);
            if (is_same_point || sign(portal_apex, portal_left, right) >
                                     0.f)
            { //! if the new right segment of the portal crosses the left segment
                portal_right = right;
                right_index = i;
            }
            else if (vequal(portal_right, right))
            {
                right_index = i;
            }
            else
            {
                portals.push_back(left_portals[left_index]);

                //! this is where the path "bends", so we push the point away from wall here.
                auto v_port_l = static_cast<Vertex>(portal_left);
                if (vertex2normal_left.count(v_port_l) > 0)
                {
                    // portal_left += radius * vertex2normal_left.at(v_port_l);
                }
                smoothed_path.push_back(portal_left);

                portal_apex = portal_left;
                portal_left = portal_apex;
                portal_right = portal_apex;
                apex_index = left_index;
                right_index = left_index;
                i = left_index;
                continue;
            }
        }

        if (sign(portal_apex, portal_left, left) >= 0.f)
        { //! same as above but we move left portal segment
            auto is_same_point = vequal(portal_apex, portal_left);
            //            if(vequal(portal_left, left)){ continue;}
            if (is_same_point or sign(portal_apex, portal_right, left) < 0.f)
            {
                portal_left = left;
                left_index = i;
            }
            else if (vequal(portal_left, left))
            {
                left_index = i;
            }
            else
            {
                portals.push_back(right_portals.at(right_index));

                auto v_port_r = static_cast<Vertex>(portal_right);
                if (vertex2normal_right.count(v_port_r) > 0)
                {
                    // portal_right += radius * vertex2normal_right.at(v_port_r);
                }
                smoothed_path.push_back(portal_right);

                portal_apex = portal_right;
                portal_right = portal_apex;
                portal_left = portal_apex;
                apex_index = right_index;
                left_index = right_index;
                i = right_index;
                continue;
            }
        }
    }
    }
    catch(std::exception& e){
        std::cout << "a" << e.what();
        throw e;
    }
    smoothed_path.push_back(r_end);
    // dumpPathToFile(smoothed_path, 3, "path-test.dat");
    portals.push_back(Edgef());
    return path_and_portals;
}

//! \brief fills FunnelData with left and right points forming a corridor
//! \param edge of reduced Triangulation graph (contains corridor data)
//! \param first index of first triangle I want from corridor
//! \param last  index of last triangle I want from corridor
//! \param revers is true if we walk from edge.end to edge.start
//! \param with_start_or_end is true if I start in edge.start or edge.end (and not in the middle of the corridor)
void PathFinder2::fillFunnelWithCorridor(const ReducedTriangulationGraph::Corridor &edge, int first, int last,
                                         Funnel &funnel, bool reverse, bool with_start_or_end) const
{

    const auto &triangles = cdt_->triangles_;
    auto &corridor = edge.tri_inds;

    if (!reverse)
    {
        if (with_start_or_end)
        {
            assert(first == 0);
            const auto &tri = triangles[edge.start.current];
            int index_of_next_neighbour = edge.start.to;

            const auto left_vertex_of_portal = asFloat(tri.verts[index_of_next_neighbour]);
            const auto right_vertex_of_portal = asFloat(tri.verts[next(index_of_next_neighbour)]);
            funnel.emplace_back(right_vertex_of_portal, left_vertex_of_portal);
        }
        if (last < 0 or first < 0 or last >= edge.tri_inds.size())
        {
            return;
        }
        for (int i = first; i <= last; ++i)
        {
            const auto &tri = triangles[edge.tri_inds[i]];
            int index_of_next_neighbour = edge.from_start[i];

            const auto left_vertex_of_portal = asFloat(tri.verts[index_of_next_neighbour]);
            const auto right_vertex_of_portal = asFloat(tri.verts[next(index_of_next_neighbour)]);
            funnel.emplace_back(right_vertex_of_portal, left_vertex_of_portal);
        }
    }
    else
    {
        if (with_start_or_end)
        {
            assert(first == edge.tri_inds.size() - 1);
            const auto &tri = triangles[edge.end.current];
            int index_of_next_neighbour = edge.end.to;

            const auto left_vertex_of_portal = asFloat(tri.verts[index_of_next_neighbour]);
            const auto right_vertex_of_portal = asFloat(tri.verts[next(index_of_next_neighbour)]);
            funnel.emplace_back(right_vertex_of_portal, left_vertex_of_portal);
        }
        if (last < 0 or first < 0 or first >= edge.tri_inds.size())
        {
            return;
        }
        for (int i = first; i >= last; --i)
        {
            const auto &tri = triangles[edge.tri_inds[i]];
            int index_of_next_neighbour = edge.from_end[i];

            const auto left_vertex_of_portal = asFloat(tri.verts[index_of_next_neighbour]);
            const auto right_vertex_of_portal = asFloat(tri.verts[next(index_of_next_neighbour)]);
            funnel.emplace_back(right_vertex_of_portal, left_vertex_of_portal);
        }
    }
}

//! \brief fills FunnelData with left and right points forming a corridor
//! \param edge of reduced Triangulation graph (contains corridor data)
//! \param first index of first triangle I want from corridor
//! \param last  index of last triangle I want from corridor
//! \param revers is true if we walk from edge.end to edge.start
//! \param with_start_or_end is true if I start in edge.start or edge.end (and not in the middle of the corridor)
void PathFinder2::fillFunnelWithCorridorFront(const ReducedTriangulationGraph::Corridor &edge, int first, int last,
                                              Funnel &funnel, bool with_start_or_end) const
{

    const auto &triangles = cdt_->triangles_;

    auto &corridor = edge.tri_inds;

    if (with_start_or_end)
    {
        assert(first == 0);
        const auto &tri = triangles[edge.start.current];
        int index_of_next_neighbour = edge.start.to;

        const auto left_vertex_of_portal = asFloat(tri.verts[index_of_next_neighbour]);
        const auto right_vertex_of_portal = asFloat(tri.verts[next(index_of_next_neighbour)]);
        funnel.emplace_back(right_vertex_of_portal, left_vertex_of_portal);
    }
    if (last < 0 or first < 0 or last >= edge.tri_inds.size())
    {
        return;
    }
    for (int i = first; i <= last; ++i)
    {
        const auto &tri = triangles[edge.tri_inds[i]];
        int index_of_next_neighbour = edge.from_start[i];

        const auto left_vertex_of_portal = asFloat(tri.verts[index_of_next_neighbour]);
        const auto right_vertex_of_portal = asFloat(tri.verts[next(index_of_next_neighbour)]);
        funnel.emplace_back(right_vertex_of_portal, left_vertex_of_portal);
    }
}

//! \brief fills FunnelData with left and right points forming a corridor
//! \param edge of reduced Triangulation graph (contains corridor data)
//! \param first index of first triangle I want from corridor
//! \param last  index of last triangle I want from corridor
//! \param with_start_or_end is true if I start in edge.start or edge.end (and not in the middle of the corridor)
void PathFinder2::fillFunnelWithCorridorReversed(const ReducedTriangulationGraph::Corridor &edge, int first, int last,
                                                 Funnel &funnel, bool with_start_or_end) const
{

    const auto &triangles = cdt_->triangles_;

    auto &corridor = edge.tri_inds;

    if (with_start_or_end)
    {
        assert(first == edge.tri_inds.size() - 1);
        const auto &tri = triangles[edge.end.current];
        int index_of_next_neighbour = edge.end.to;

        const auto left_vertex_of_portal = asFloat(tri.verts[index_of_next_neighbour]);
        const auto right_vertex_of_portal = asFloat(tri.verts[next(index_of_next_neighbour)]);
        funnel.emplace_back(right_vertex_of_portal, left_vertex_of_portal);
    }
    if (last < 0 or first < 0 or first >= edge.tri_inds.size())
    {
        return;
    }
    for (int i = first; i >= last; --i)
    {
        const auto &tri = triangles[edge.tri_inds[i]];
        int index_of_next_neighbour = edge.from_end[i];

        const auto left_vertex_of_portal = asFloat(tri.verts[index_of_next_neighbour]);
        const auto right_vertex_of_portal = asFloat(tri.verts[next(index_of_next_neighbour)]);
        funnel.emplace_back(right_vertex_of_portal, left_vertex_of_portal);
    }
}

//! TODO: figure out why I get segfault when I use sign(..) function from "core.h"
//!       but not this one with optimisation lvl at least -O2?
//!
float PathFinder2::sign(sf::Vector2f a, sf::Vector2f b, sf::Vector2f c) const
{
    const auto ax = b.x - a.x;
    const auto ay = b.y - a.y;
    const auto bx = c.x - a.x;
    const auto by = c.y - a.y;
    const auto area = by * ax - ay * bx;
    return area;
}
