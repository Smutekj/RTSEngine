#include <future>
#include <thread>
#include <chrono>
#include "PathFinder.h"
#include "Triangulation.h"
#include "BoidControler.h"
#include "MapGrid.h"
#include "Selection.h"
#include "MapGrid.h"

FunnelFan::FunnelFan(const sf::Vector2f& apex, const sf::Vector2f& left_point, const sf::Vector2f& right_point)
    : apex(apex)
    , left_points({left_point})
    , right_points({right_point})
    , left_min_dists({dist(left_point, apex)})
    , right_min_dists({dist(right_point, apex)}) {}

FunnelFan::FunnelFan(const sf::Vector2f& apex)
    : apex(apex) {}

//! \brief calculates distance from funnel fan apex to the end_point
//! \param end_point
//! \returns the distance
float FunnelFan::calcDistanceToEndPoint(const sf::Vector2f end_point) const {
    const auto n_left = left_points.size();
    const auto n_right = right_points.size();

    if (n_right == 0 and n_left == 0) {
        return dist(end_point, apex) + dist_apex_to_start;
    }
    Edgef line_l;
    Edgef line_r;

    if (n_left == 0) {
        line_l.from = apex;
        line_l.t = right_points.at(0) - apex;
        if (!isClockwise(end_point, line_l)) {
            return dist(end_point, apex) + dist_apex_to_start;
        }
        for (int i = 0; i < n_right - 1; ++i) {
            if (i == 0) {
                line_l.t = right_points.at(0) - apex;
            }
            line_l.from = right_points.at(i);
            line_r.from = right_points.at(i);
            line_r.t = right_points.at(i + 1) - right_points.at(i);
            if (isInFan(end_point, line_l, line_r)) {
                return dist(end_point, right_points.at(i)) + right_min_dists.at(i) + dist_apex_to_start;
            }
            line_l = line_r;
        }
        return dist(end_point, right_points.back()) + right_min_dists.back() + dist_apex_to_start;
    }
    if (n_right == 0) {
        line_r.from = apex;
        line_r.t = left_points.at(0) - apex;
        if (!isClockwise(end_point, line_r)) {
            return dist(end_point, apex) + dist_apex_to_start;
        }
        for (int i = 0; i < n_left - 1; ++i) {
            if (i == 0) {
                line_r.t = left_points.at(0) - apex;
            }
            line_r.from = left_points.at(i);
            line_l.from = left_points.at(i);
            line_l.t = left_points.at(i + 1) - left_points.at(i);
            if (isInFan(end_point, line_l, line_r)) {
                return dist(end_point, left_points.at(i)) + left_min_dists.at(i) + dist_apex_to_start;
            }
            line_r = line_l;
        }
        return dist(end_point, left_points.back()) + left_min_dists.back() + dist_apex_to_start;
    }

    Edgef portal_line;
    portal_line.from = right_points.back();
    portal_line.t = left_points.back() - right_points.back();
    portal_line.l = norm(portal_line.t);
    portal_line.t /= portal_line.l;

    if (isClockwise(end_point, portal_line)) {
        line_l.from = apex;
        line_r.from = apex;
        line_r.t = right_points.at(0) - apex;
        line_l.t = left_points.at(0) - apex;
        if (isInFan(end_point, line_l, line_r)) {
            return dist(end_point, apex) + dist_apex_to_start;
        }

        for (int i = 0; i < n_left - 1; ++i) {
            line_r = line_l;
            if (i == 0) {
                line_r.t = left_points.at(0) - apex;
            }
            line_r.from = left_points.at(i);
            line_l.from = left_points.at(i);
            line_l.t = left_points.at(i + 1) - left_points.at(i);
            if (isInFan(end_point, line_l, line_r)) {
                return dist(end_point, left_points.at(i)) + left_min_dists.at(i) + dist_apex_to_start;
            }
        }

        for (int i = 0; i < n_right - 1; ++i) {
            line_l = line_r;
            if (i == 0) {
                line_l.t = right_points.at(0) - apex;
            }
            line_l.from = right_points.at(i);
            line_r.from = right_points.at(i);
            line_r.t = right_points.at(i + 1) - right_points.at(i);
            if (isInFan(end_point, line_l, line_r)) {
                return dist(end_point, right_points.at(i)) + right_min_dists.at(i) + dist_apex_to_start;
            }
        }
    }

    const auto dist_to_right = dist(end_point, right_points.back()) + right_min_dists.back() + dist_apex_to_start;
    const auto dist_to_left = dist(end_point, left_points.back()) + left_min_dists.back() + dist_apex_to_start;
    return std::min(dist_to_right, dist_to_left);
}

//! \brief inserts next left and right point int the funnel fan
void FunnelFan::addEdge(const sf::Vector2f& new_anticlock_point, const sf::Vector2f& new_clock_point) {
    const auto n_right = right_points.size();
    const auto n_left = left_points.size();

    if (n_right > 0 and vequal(new_clock_point, right_points.back())) {
        addLeftPoint(new_anticlock_point);
    } else if (n_left > 0 and vequal(new_anticlock_point, left_points.back())) {
        addRightPoint(new_clock_point);
    } else {
        addLeftPoint(new_anticlock_point);
        addRightPoint(new_clock_point);
    }
}

void FunnelFan::addLeftPoint(const sf::Vector2f new_point) {
    int i = 0;
    const int n_left = left_points.size();
    const int n_right = right_points.size();
    Edgef line1;
    Edgef line2;

    if (n_left >= 1) {
        line1.t = left_points.at(0) - apex;
    }
    line1.from = apex;
    if (n_right >= 1) {
        line2.t = right_points.at(0) - apex;
    }
    line2.from = apex;

    if (n_right == 0 or !isClockwise(new_point, line2)) {
        if (n_left == 1 and isInFan(new_point, line1, line2)) {
            left_points.at(0) = new_point;
            left_min_dists.at(0) = dist(apex, new_point);
            return;
        }
        //! find in which triangle fan the new point lies, if
        while (i < n_left - 1) {
            if (isInFan(new_point, line1, line2)) {
                while (left_points.size() != i) {
                    left_points.pop_back();
                    left_min_dists.pop_back();
                }
                break;
            }
            line2 = line1;
            line2.from = left_points.at(i);
            line1.from = left_points.at(i);
            line1.t = left_points.at(i + 1) - line1.from;
            line1.t /= norm(line1.t);
            i++;
        }
        if (left_points.size() == 0) {
            left_min_dists.push_back(dist(new_point, apex));
        } else {
            left_min_dists.push_back(left_min_dists.back() + dist(new_point, left_points.back()));
        }
    } else {
        if (n_right == 1) {
            dist_apex_to_start += dist(apex, right_points.at(0));
            apex = right_points.at(0);
            right_points.pop_front();
            right_min_dists.pop_front();
        }
        left_points.clear();
        left_min_dists.clear();
        i = 0;
        //! find in which right triangle fan the new point lies, if
        while (i < n_right - 1) {
            line2 = line1;
            line2.from = right_points.at(i);
            if (i == 0) {
                line2.t = right_points.at(i) - apex;
            };
            line1.from = right_points.at(i);
            line1.t = right_points.at(i + 1) - line1.from;

            if (isInFan(new_point, line2, line1)) {
                float distance_to_last_removed = 0;
                sf::Vector2f last_removed_point;
                while (right_points.size() != n_right - i - 1) {
                    last_removed_point = right_points.front();
                    right_points.pop_front();
                    distance_to_last_removed = right_min_dists.front();
                    right_min_dists.pop_front();
                }

                apex = last_removed_point;
                dist_apex_to_start += distance_to_last_removed;
                for (int j = 0; j < right_points.size(); ++j) {
                    right_min_dists[j] -= distance_to_last_removed;
                }
                break;
            }
            i++;
        }
        left_min_dists.push_back(dist(apex, new_point));
    }
    left_points.push_back(new_point);
}

void FunnelFan::addRightPoint(const sf::Vector2f new_point) {
    int i = 0;
    const int n_left = left_points.size();
    const int n_right = right_points.size();
    Edgef line1;
    Edgef line2;

    if (n_right >= 1) {
        line1.t = right_points.at(0) - apex;
    }
    line1.from = apex;
    if (n_left >= 1) {
        line2.t = left_points.at(0) - apex;
    }
    line2.from = apex;

    if (n_left == 0 or isClockwise(new_point, line2)) {
        if (n_right == 1 and isInFan(new_point, line2, line1)) {
            right_points.at(0) = new_point;
            right_min_dists.at(0) = dist(apex, new_point);
            return;
        }
        //! find in which triangle fan the new point lies, if
        while (i < n_right - 1) {
            if (isInFan(new_point, line2, line1)) {
                while (right_points.size() != i) {
                    right_points.pop_back();
                    right_min_dists.pop_back();
                }
                break;
            }
            line2 = line1;
            line2.from = right_points.at(i);
            line1.from = right_points.at(i);
            line1.t = right_points.at(i + 1) - line1.from;
            line1.t /= norm(line1.t);

            i++;
        }
        if (right_points.size() == 0) {
            right_min_dists.push_back(dist(new_point, apex));
        } else {
            right_min_dists.push_back(right_min_dists.back() + dist(new_point, right_points.back()));
        }
    } else {
        if (n_left == 1) {
            dist_apex_to_start += dist(apex, left_points.at(0));
            apex = left_points.at(0);
            left_points.pop_front();
            left_min_dists.pop_front();
        }
        right_points.clear();
        right_min_dists.clear();
        i = 0;
        //! find in which right triangle fan the new point lies, if
        while (i < n_left - 1) {
            line1 = line2;
            line1.from = left_points.at(i);
            if (i == 0) {
                line1.t = left_points.at(i) - apex;
            }
            line2.from = left_points.at(i);
            line2.t = left_points.at(i + 1) - line2.from;
            if (isInFan(new_point, line2, line1)) {
                float distance_to_last_removed = 0;
                sf::Vector2f last_removed_point;
                while (left_points.size() != n_left - i - 1) {
                    last_removed_point = left_points.front();
                    left_points.pop_front();
                    distance_to_last_removed = left_min_dists.front();
                    left_min_dists.pop_front();
                }

                apex = last_removed_point;
                dist_apex_to_start += distance_to_last_removed;
                for (int j = 0; j < left_points.size(); ++j) {
                    left_min_dists[j] -= distance_to_last_removed;
                }
                break;
            }
            i++;
        }
        right_min_dists.push_back(dist(apex, new_point));
    }
    right_points.push_back(new_point);
}

//! \brief returns true if query_point lies in a clockwise orientation relative to line
//! \param query_point
//! \param line
bool FunnelFan::isClockwise(const sf::Vector2f query_point, Edgef& line) const {
    const auto dr_to_apex = query_point - line.from;
    const sf::Vector2f n2 = {-line.t.y, line.t.x};
    return dot(dr_to_apex, n2) > 0;
}

//! \brief returns true if query_point lies in between the line segments line1 and line2
bool FunnelFan::isInFan(const sf::Vector2f query_point, Edgef& line1, Edgef& line2) const {

    const auto dr_to_apex = query_point - line1.from;
    const sf::Vector2f n1 = {-line1.t.y, line1.t.x};
    const sf::Vector2f n2 = {-line2.t.y, line2.t.x};

    return dot(dr_to_apex, n1) > 0 and dot(dr_to_apex, n2) < 0;
}

PathFinder::PathFinder(Triangulation* cdt)
    : cdt_(cdt) {
    p_rtg_ = std::make_shared<ReducedTriangulationGraph>();
    update_has_been_issued_.resize(N_MAX_NAVIGABLE_BOIDS, false);
}

//! \brief checks if reduced2tri_ind and tri_ind2reduced mappings are consistent with each other
//! \brief also checks if corridors lead to correct triangles  
bool PathFinder::reducedGraphConsistentWithTriangulation()const {
    const auto& triangles = cdt_->triangles_;
    const auto& rtg = *p_rtg_;
    
    auto triangle_is_within_boundary = [&](const Triangle& tri) {
        const auto tri_center = cdt_->calcTriangleCenter(tri);
        return cdt_->boundary_.x >= tri_center.x and tri_center.x > 0 and cdt_->boundary_.y >= tri_center.y and
               tri_center.y > 0;
    };

    for(int ti = 0; ti<triangles.size(); ti++){
        auto& vertex = rtg.tri_ind2vertex[ti];
        if(triangle_is_within_boundary(triangles[ti]) and triangles[ti].countFreeEdges() == 3){
            assert(rtg.vertex2tri_ind[vertex] == ti);
            for(auto edge_ind : rtg.vertex2edge_inds2[vertex]){
                auto tj = ti;
                if(rtg.edges.at(edge_ind).end.current == ti){
                    tj = rtg.edges.at(edge_ind).start.current;
                }else{
                    tj = rtg.edges.at(edge_ind).end.current;
                }
                // assert(triangles[tj].countFreeEdges() != 2);
            }
        }else if(triangles[ti].countFreeEdges() == 2){
            auto edge_ind = vertex;
            auto& edge = rtg.edges.at(edge_ind);
            // assert( std::find(edge.tri_inds.begin(), edge.tri_inds.end(), ti) != edge.tri_inds.end() );
        }
    }
    return true;
}

//! \brief creates everything from triangulation
//! \brief should be called on triangulation change
void PathFinder::update() {

    const auto n_triangles = cdt_->triangles_.size();
    shortest_path.resize(n_triangles);
    tri_ind2shortest_path_.resize(n_triangles);

    tri_edge2g_function_.resize(n_triangles);
    tri_edge2h_function_.resize(n_triangles);
    // tri_edge2astar_data_.resize(n_triangles);
    tri_edge2portal_.resize(3 * n_triangles);
    tri_edge2shortest_path_.resize(n_triangles);

    vertex2shortest_distance.resize(n_triangles);
    vertex2h_function_.resize(n_triangles);

    triangle2tri_widths_.resize(n_triangles);
    for (int tri_ind = 0; tri_ind < n_triangles; ++tri_ind) {
        triangle2tri_widths_[tri_ind] = TriangleWidth(cdt_->triangles_[tri_ind], *cdt_);
    }

    p_rtg_->constructFromTriangulationCenters(*cdt_, tri_ind2component_);
    cdt_->updateCellGrid();
    reducedGraphConsistentWithTriangulation();

    for (int thread_id = 0; thread_id < 12; ++thread_id) {
        thread2reduced_vertex2astar_data_[thread_id].resize(p_rtg_->reduced_vertices.size() + 2);
    }
    reduced_vertex2astar_data_.resize(p_rtg_->reduced_vertices.size() + 2);
}

bool PathFinder::allThreadsFree(int last_thread_id){
    bool all_threads_free = true;
    for (int thread_id = 0; thread_id < last_thread_id; ++thread_id) {
        
        bool is_valid = futures_.at(thread_id).valid();
        std::future_status status = std::future_status::timeout;
        if(is_valid){
            status = futures_.at(thread_id).wait_for(std::chrono::microseconds(0));
        }
        
        if (status == std::future_status::ready) {
           thread_status_.at(thread_id) = THREAD_STATUS::FREE;
           int n;
            try{
                if(futures_.at(thread_id).valid()){
                    n = futures_.at(thread_id).get();
                }
            }
            catch (const std::exception& e)
            {
                std::cout << "something got fucked at: "
                        << "\"\nMessage: \"" << e.what() << "\"\n";
                thread_status_.at(thread_id) = THREAD_STATUS::FAILED;
            }
        }else if(is_valid){
            all_threads_free = false;
        }
    }
    return all_threads_free;
}

//! \brief calls threads that will do path-finding of the issued pathfinding work  
//! \param bc BoidControler object  
//! \param r_coords coordinates of all particles
//! \param available_time total time available for pathfinding
//! \note  We use here the most recent coordinates ( \p r_coords )
//! \note because the actual pathfinding does not have to be done in the same frame when the pathfinding was issued!
void PathFinder::updatePaths(const std::vector<sf::Vector2f>& r_coords, BoidControler& bc,
                             const sf::Int64 available_time) {

    sf::Clock time;
    const int n_threads = std::thread::hardware_concurrency()-1;
    threads_.resize(0);
    thread_status_.resize(n_threads, THREAD_STATUS::FREE);
    
    std::function<int(const GroupPathData, int)> do_pathfinding = [this, &bc](const GroupPathData data, int thread_id) {
        try{
            doPathFinding(data.r_starts, data.r_end, bc, data.radius, data.inds_to_update, thread_id);
        }catch(std::exception& e){
            std::cout << "fuck\n";
            throw e;
        }
        return 1;
    };

    int thread_id = 0;
    const int last = to_update_groups_.size() - 1;

    std::priority_queue to_update_pq(
        to_update_groups_.begin(), to_update_groups_.end(), [](const GroupPathData& gpd1, const GroupPathData& gpd2) {
            return dist2(gpd1.r_end, gpd1.r_starts[0]) < dist2(gpd2.r_end, gpd2.r_starts[0]);
        });
    futures_.resize(n_threads);

    sf::Clock clock;
    int n_paths_found = 0;
    for (int i = last; i >= std::max(last + 1 - n_threads, 0); --i) {
        auto data = to_update_pq.top();
        for (int j = 0; j < data.inds_to_update.size(); ++j) {
            data.r_starts[j] = r_coords[data.inds_to_update[j]];
        }
        futures_[thread_id] = std::async(std::launch::async, do_pathfinding, data, thread_id);
        thread_status_[thread_id] = THREAD_STATUS::RUNNING;
        n_paths_found++;
        for (const auto& ind : data.inds_to_update) {
            update_has_been_issued_[ind] = false;
        }
        thread_id++;
        to_update_pq.pop();
    }

    bool all_threads_free = allThreadsFree(std::min(last+1, n_threads));
    auto run_time = clock.getElapsedTime().asMicroseconds();

    bool all_jobs_are_done = all_threads_free && to_update_pq.empty();
    bool time_has_run_out = run_time > available_time;
    //! search for queried paths on all available threads until time runs out or until we finish the job 
    while (!(all_jobs_are_done)) {
        
        if (time_has_run_out and all_threads_free) { 
            std::cout << "time has run out! path_finding took: " << run_time <<  " available: " << available_time<< "\n"; 
            break;
        }

        
        for (int thread_id = 0; thread_id < std::min(last+1, n_threads); ++thread_id) {
            if (thread_status_[thread_id] == THREAD_STATUS::FREE and !to_update_pq.empty()) {
                auto data = to_update_pq.top();
                for (int i = 0; i < data.inds_to_update.size(); ++i) {
                    data.r_starts[i] = r_coords[data.inds_to_update[i]];
                }
                
                futures_[thread_id] = std::async(std::launch::async, do_pathfinding, data, thread_id);
                thread_status_[thread_id] = THREAD_STATUS::RUNNING;

                n_paths_found++;
                for (const auto& ind : data.inds_to_update) {
                    update_has_been_issued_[ind] = false;
                }
                to_update_pq.pop();
            }
        }
        all_threads_free = allThreadsFree(std::min(last+1, n_threads));
        all_jobs_are_done = all_threads_free && to_update_pq.empty();

        run_time = clock.getElapsedTime().asMicroseconds();
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
void PathFinder::setPathOfAgent(int ind, sf::Vector2f r_ind, sf::Vector2f r_end, BoidControler& bc, PathAndPortals& path_and_portals)const{
        auto& path = path_and_portals.path;
        auto& portals = path_and_portals.portals;
        
        bc.setPathData(ind, path.at(1), portals.at(1));
        if (path.size() >= 3) {
            if (dot(r_ind - path.at(1), path.at(2) - path.at(1)) < 0) {
                bc.setPathData(ind, path.at(1), portals.at(1));
                bc.setPathDataNext(ind, path.at(2), portals.at(2));
            } else {
                if (path.size() > 3) {
                    bc.setPathData(ind, path.at(2), portals.at(2));
                    bc.setPathDataNext(ind, path.at(3), portals.at(3));
                } else {
                    bc.setPathData(ind, path.at(2), portals.at(2));
                    bc.setPathDataNext(ind, r_end, Edgef());
                }
            }
        } else {
            bc.setPathData(ind, path.at(1), portals.at(1));
            bc.setPathDataNext(ind, path.at(1), portals.at(1));
        }
}

// void PathFinder::doPathfinding(PathFinder::GroupPathData& gd, int thread_id){

//     doPathFinding

// }

void PathFinder::doPathFinding(const std::vector<sf::Vector2f> r_coords, const sf::Vector2f r_end, BoidControler& bc,
                         const float max_radius_of_agent, const std::vector<int> agent_indices, const int thread_id) {
    
    const auto r_start = r_coords[0];
    Funnel funnel;
    
    try{
        findSubOptimalPathCenters(r_start, r_end, max_radius_of_agent, funnel, thread_id);
    }catch(std::exception& e){
        std::cout << "wtf" << e.what() << "\n";
        throw e;
    }
    funnel.push_back({r_start, r_start});
    std::reverse(funnel.begin(), funnel.end()); 
    funnel.push_back({r_end, r_end});
    auto path_and_portals = pathFromFunnel(r_start, r_end, bc.radii_[agent_indices[0]], funnel);
    auto& path = path_and_portals.path;

    for (int i = 0; i < agent_indices.size(); ++i) {
        const auto ind = agent_indices[i];
        setPathOfAgent(ind, r_coords[ind], r_end, bc, path_and_portals);

        // //! find if agent needs to turn
        const auto current_angle = bc.orientation_[ind];
        const auto desired_angle = bc.angle_calculator_.angle(path.at(1) - r_coords[i]);
        if (!bc.is_turning[ind] and std::abs(current_angle - desired_angle) > 15) {
            bc.is_turning[ind] = true;
        }
    }
}

//! \brief divides agents by triangles and for each triangle we do one Astar search
void PathFinder::issuePaths(BoidControler& bc, const std::vector<sf::Vector2f>& r_coords,
                            const std::vector<float>& radii, const std::vector<int>& selection,
                            const sf::Vector2f r_end) {

    float max_radius_of_agent = 0.;

    sf::Clock time;

    start_tri2indices_.clear();
    start_tri_inds_.clear();

    for (const auto selected_ind : selection) {

        auto start_tri_ind = cdt_->findTriangle(r_coords[selected_ind], false);
        //! I should do ray casting here, but it is not done yet
        // if (rayCast(r_coords[selected_ind], r_end) >=
        //     1) { //! if I see the end directly result is straight line (I should write a setter for path_data)
        //     bc.setPathData(selected_ind, r_end, Edgef());
        //     bc.setPathDataNext(selected_ind, r_end, Edgef());
        //     continue;
        // }
    
        if (update_has_been_issued_[selected_ind]) { //! no need to update something twice
            continue;
        }
        update_has_been_issued_[selected_ind] = true;

        if (start_tri2indices_.count(start_tri_ind) == 0) {
            start_tri2indices_[start_tri_ind] = {selected_ind};
            start_tri_inds_.push_back(start_tri_ind);
        } else {
            start_tri2indices_[start_tri_ind].push_back(selected_ind);
        }

        if (radii[selected_ind] > max_radius_of_agent) {
            max_radius_of_agent = radii[selected_ind];
        }
    }
    const auto time_of_first_part = time.restart().asMicroseconds();

    const auto end_tri_ind = cdt_->findTriangle(r_end, false);

    bool wtf = true;
    int i = 0;
    for (const auto start_tri_ind : start_tri_inds_) {
        std::vector<int>& inds = start_tri2indices_.at(start_tri_ind);
        auto r_start = cdt_->calcTriangleCenter(cdt_->triangles_[start_tri_ind]);
        to_update_groups_.push_back({start_tri_ind, end_tri_ind, r_coords, r_end, max_radius_of_agent, inds});
        // threads.back().detach();
        // const auto r_start = r_coords[start_tri2indices.at(start_tri_ind).at(0)];
        // // auto funnel = findOptimalPath(r_start, r_end, max_radius_of_agent);
        // FunnelData funnel;
        // findSubOptimalPathCenters(r_start, r_end, max_radius_of_agent, funnel);

        // funnel.funnel.push_back({r_start, r_start});
        // std::reverse(funnel.funnel.begin(), funnel.funnel.end()); //! Should just use deque ...
        // funnel.funnel.push_back({r_end, r_end});
        // const auto path_and_portals =
        //     pathFromFunnel(r_start, r_end, radii[start_tri2indices.at(start_tri_ind)[0]], funnel);

        // const auto& selected_inds_in_start_triangle = start_tri2indices.at(start_tri_ind);
        // for (const auto selected_ind : selected_inds_in_start_triangle) {

        //     funnel.funnel[0] = {r_coords[selected_ind], r_coords[selected_ind]};
        //     const auto& path = path_and_portals.path;
        //     const auto& portals = path_and_portals.portals;
        //     if (wtf) {
        //         result.path = std::vector<sf::Vector2f>(path.begin(), path.end());
        //         result.portals = std::vector<Edgef>(portals.begin(), portals.end());
        //         wtf = false;
        //     } //! this is so that I can draw one path for demonstrations

        //     bc.setPathData(selected_ind, path.at(1), portals.at(1));
        //     if (path.size() >= 3) {
        //         bc.setPathDataNext(selected_ind, path.at(2), portals.at(2));
        //     } else {
        //         bc.setPathDataNext(selected_ind, path.at(1), Edgef());
        //     }
        // }
    }
    const auto time_of_second_part = time.restart().asMicroseconds();
    // std::cout << "1. part took: " << time_of_first_part << " us\n"
    //           << "2. part took: " << time_of_second_part << " us\n";
    // std::cout << "there are : " << to_update_groups_.size() << " paths to find\n";
}

//! \brief updates path of an agent with boid_index ind
//! \brief and tells the result to the Boidcontroler
//! \brief Should be used in case we do not have yet acces to a reduced graph
//! \param bc       reference to the BoidControler object (maybe I should use only necessary data from bc and not the
//! entire object?) \param r_coords position of all agents \param radii    radii of all agents \param selection indices
//! of selected agents which need path finding \param r_end    finish position \returns path of one selected agent so
//! that I can draw it for demonstartion (will be removed :D)
PathFinder::PathAndPortals PathFinder::calcPathOfSelection(BoidControler& bc, const std::vector<sf::Vector2f>& r_coords,
                                                           const std::vector<float>& radii,
                                                           const std::vector<int>& selection,
                                                           const sf::Vector2f r_end) {

    PathFinder::PathAndPortals result;

    std::unordered_map<TriInd, std::vector<int>> start_tri2indices;
    std::vector<int> start_tri_inds;
    float max_radius_of_agent = 0.;
    for (const auto selected_ind : selection) {

        auto start_tri_ind = cdt_->findTriangle(r_coords[selected_ind], false);
        if (rayCast(r_coords[selected_ind], r_end) >=
            1) { //! if I see the end directly result is straight line (I should write a setter for path_data)
            bc.setPathData(selected_ind, r_end, Edgef());
            bc.setPathDataNext(selected_ind, r_end, Edgef());
            continue;
        }

        if (start_tri2indices.count(start_tri_ind) == 0) {
            start_tri2indices[start_tri_ind] = {selected_ind};
            start_tri_inds.push_back(start_tri_ind);
        } else {
            start_tri2indices[start_tri_ind].push_back(selected_ind);
        }

        if (radii[selected_ind] > max_radius_of_agent) {
            max_radius_of_agent = radii[selected_ind];
        }
    }

    const auto end_tri_ind = cdt_->findTriangle(r_end, false);

    const int n_threads = 12;
    bool wtf = true;
    std::vector<std::thread> threads;
    threads.reserve(start_tri_inds.size());
    int thread_id = 0;
    for (const auto start_tri_ind : start_tri_inds) {

        std::function<void(int)> do_pathfinding = [&](int thread_id) {
            doPathFinding(r_coords, r_end, bc, max_radius_of_agent, start_tri2indices.at(start_tri_ind), thread_id);
        };

        threads.push_back(std::thread(do_pathfinding, thread_id));
        thread_id++;
        if (thread_id == n_threads) {
            thread_id = 0;
            for (auto& thread : threads) {
                thread.join();
            }
            threads.clear();
        }
    }

    for (auto& job : threads) {
        job.join();
    }

    return paf;
}

//! \brief casts a ray connecting points from and to
//! \param from
//! \param to
//! \returns returns 1 if there are no walls in between points and number smaller than 1 otherwise
//! \returns in case a wall is hit, the number represents how far away from the start point the point of hit lies
//! \returns(the contact point can be found like: from + result * (from - to) / norm(from - to)
float PathFinder::rayCast(const sf::Vector2f& from, const sf::Vector2f& to) const {
    const auto& triangles = cdt_->triangles_;
    const auto& vertices = cdt_->vertices_;

    const auto start_tri_ind = cdt_->findTriangle(from, true);
    const auto end_tri_ind = cdt_->findTriangle(to, false);
    if (start_tri_ind == end_tri_ind) {
        return 1;
    }

    const auto& start_tri = triangles[start_tri_ind];
    const auto& end_tri = triangles[end_tri_ind];
    const auto& vs0 = start_tri.verts[0];
    const auto& vs1 = start_tri.verts[1];
    const auto& vs2 = start_tri.verts[2];

    const auto new_alpha1s = cdt_->linesIntersect(from, to, vs0, vs1);
    const auto new_alpha2s = cdt_->linesIntersect(from, to, vs1, vs2);
    const auto new_alpha3s = cdt_->linesIntersect(from, to, vs2, vs0);

    int prev_tri_ind = start_tri_ind;
    int tri_ind = start_tri_ind;
    float alpha = -1;
    bool is_constrained = false;
    if (new_alpha1s <= 1 and new_alpha1s >= 0) {
        alpha = new_alpha1s;
        is_constrained = start_tri.is_constrained[0];
        tri_ind = start_tri.neighbours[0];
    }
    if (new_alpha2s <= 1 and new_alpha2s >= 0) {
        alpha = new_alpha2s;
        is_constrained = start_tri.is_constrained[1];
        tri_ind = start_tri.neighbours[1];
    }
    if (new_alpha3s <= 1 and new_alpha3s >= 0) {
        alpha = new_alpha3s;
        is_constrained = start_tri.is_constrained[2];
        tri_ind = start_tri.neighbours[2];
    }

    //! this should not happen but sometimes does and I don't know why yet :(
    if (new_alpha1s < 0 and new_alpha2s < 0 and new_alpha3s < 0) {
        //        assert(isInTriangle(from, start_tri, vertices));
        return 0;
    }
    assert(!(new_alpha1s < 0 and new_alpha2s < 0 and new_alpha3s < 0));

    while (tri_ind != end_tri_ind and !is_constrained) {
        const auto& tri = triangles[tri_ind];
        const int k = indInTriOf(tri, prev_tri_ind);

        const auto& v0 = tri.verts[next(k)];
        const auto& v1 = tri.verts[prev(k)];
        const auto& v2 = tri.verts[k];

        const auto new_alpha_left = cdt_->linesIntersect(from, to, v0, v1);
        const auto new_alpha_right = cdt_->linesIntersect(from, to, v1, v2);

        if ((new_alpha_left < 0 and new_alpha_right < 0)) {
            return 0;
        }

        prev_tri_ind = tri_ind;
        if (new_alpha_left <= 1 and new_alpha_left >= 0) {
            alpha = new_alpha_left;
            is_constrained = tri.is_constrained[next(k)];
            tri_ind = tri.neighbours[next(k)];
        }
        if (new_alpha_right <= 1 and new_alpha_right >= 0) {
            alpha = new_alpha_right;
            is_constrained = tri.is_constrained[prev(k)];
            tri_ind = tri.neighbours[prev(k)];
        }
    }

    if (tri_ind == end_tri_ind and !is_constrained) {
        return 1;
    }
    return alpha;
}

//! \brief updates path of an agent with boid_index ind
//! \brief and tells the result to the Boidcontroler
//! \brief Should be used in case we do not have yet acces to a reduced graph
//! \param r_start starting position
//! \param r_end    target position
//! \param bc       reference to the BoidControler object (maybe I should use only necessary data from bc and not the
//! entire object?) \param radius how big the navigated agent is
void PathFinder::updatePathOf(const int ind, sf::Vector2f r_start, BoidControler& bc, float radius) {

    //    r_start = bc.move_targets_[ind];
    const auto r_end = bc.getPathEnd(ind);
    Funnel funnel;
    findSubOptimalPathCenters(r_start, r_end, radius, funnel);
    funnel.push_back({r_start, r_start});
    // std::reverse(funnel.funnel.begin(), funnel.funnel.end()); // Should just use deque ...
    funnel.push_back({r_end, r_end});
    auto path_and_portals = pathFromFunnel(r_start, r_end, radius, funnel);

    const auto& real_path = path_and_portals.path;
    const auto& portals = path_and_portals.portals;

    float real_path_distance = 0;
    for (int i = 1; i < real_path.size(); ++i) {
        real_path_distance += dist(real_path[i], real_path[i - 1]);
    }

    if (real_path.size() > 2) {
        if (dot(r_start - real_path.at(1), real_path.at(2) - real_path.at(1)) < 0) {
            bc.setPathData(ind, real_path.at(1), portals.at(1));
            bc.setPathDataNext(ind, real_path.at(2), portals.at(2));
        } else {
            if (real_path.size() > 3) {
                bc.setPathData(ind, real_path.at(2), portals.at(2));
                bc.setPathDataNext(ind, real_path.at(3), portals.at(3));
            } else {
                bc.setPathData(ind, real_path.at(2), portals.at(2));
                bc.setPathDataNext(ind, r_end, Edgef());
            }
        }
    }
}

//! \brief finds path using triangle centers on a full triangulation
//! \brief Should be used in case we do not have yet acces to a reduced graph
//! \param r_start starting position
//! \param r_end    target position
//! \param radius how big the navigated agent is
//! \returns funnel formed from triangles, which contains the desired path
Funnel PathFinder::findShortestPath(sf::Vector2f r_start, sf::Vector2f r_end, float radius) {

    const auto& triangles = cdt_->triangles_;
    const auto& vertices = cdt_->vertices_;
    const auto start = cdt_->findTriangle(r_start, false);
    const auto end = cdt_->findTriangle(r_end, false);

    if (start == end or start == -1 or end == -1) {
        return Funnel();
    }

    std::vector<AstarDataPQ> to_visit;
    for (int i = 0; i < triangles.size(); ++i) {
        //        to_visit.push_back(i);
        vertex2shortest_distance[i] = MAXFLOAT;
        vertex2h_function_[i] = dist(cdt_->calcTriangleCenter(triangles[i]), r_end);
        shortest_path[i] = -1;
    }

    std::priority_queue to_visit_pque(
        to_visit.begin(), to_visit.end(),
        [&](const AstarDataPQ& a1, const AstarDataPQ& a2) { return a1.f_value > a2.f_value; });

    vertex2shortest_distance.at(start) = 0;
    to_visit_pque.push({start, 0});

    int current_tri_ind = -1;
    int prev_tri_ind = -1;
    while (!to_visit_pque.empty()) {
        prev_tri_ind = current_tri_ind;
        current_tri_ind = to_visit_pque.top().next;
        to_visit_pque.pop();

        const auto& current_tri = cdt_->triangles_[current_tri_ind];
        std::vector<int> free_neighbours;
        int prev_ind_in_tri = -1;
        for (int ind_in_tri = 0; ind_in_tri < 3; ++ind_in_tri) {
            if (current_tri.neighbours[ind_in_tri] != prev_tri_ind and !current_tri.is_constrained[ind_in_tri]) {
                free_neighbours.push_back(ind_in_tri);
            }
        }

        for (const auto ind_in_tri : free_neighbours) {
            const auto neighbour = current_tri.neighbours[ind_in_tri];
            const auto& neighbour_tri = cdt_->triangles_[neighbour];
            if (neighbour == -1) { //! This means that the edge is a wall (Most likely)
                continue;
            }

            float width = MAXFLOAT;
            width = triangle2tri_widths_[current_tri_ind].widths[ind_in_tri];

            const auto t1 = cdt_->calcTriangleCenter(current_tri);
            const auto t2 = cdt_->calcTriangleCenter(neighbour_tri);
            const auto distance = dist(t1, t2);
            const auto distance_to_end = dist(r_end, t1);
            const auto h_value = dist(r_end, t2);
            const auto new_g_value = vertex2shortest_distance.at(current_tri_ind) + distance;
            if (vertex2shortest_distance.at(neighbour) > new_g_value and width > 2 * radius) {

                vertex2shortest_distance.at(neighbour) = vertex2shortest_distance.at(current_tri_ind) + distance;
                vertex2h_function_[neighbour] = h_value;
                shortest_path.at(neighbour) = current_tri_ind;
                to_visit_pque.push({neighbour, h_value + new_g_value});
            }
        }
    }
    //! walk backwards from finish to start;
    //! current_tri_ind = end;
    Funnel path_funnel;
    current_tri_ind = end;
    while (current_tri_ind != start) {
        if (current_tri_ind == -1) {
            return path_funnel;
        }
        const auto& tri = cdt_->triangles_[current_tri_ind];

        auto next_neighbour = std::find(tri.neighbours.begin(), tri.neighbours.end(), shortest_path[current_tri_ind]);
        auto index_of_next_neighbour = next_neighbour - tri.neighbours.begin();
        if (index_of_next_neighbour >= 3) {
            return Funnel();
        }

        const auto left_vertex_of_portal = asFloat(tri.verts[index_of_next_neighbour]);
        const auto right_vertex_of_portal = asFloat(tri.verts[(index_of_next_neighbour + 1) % 3]);
        path_funnel.emplace_back(right_vertex_of_portal, left_vertex_of_portal);

        current_tri_ind = shortest_path[current_tri_ind];
    }

    return path_funnel;
}

//! \brief finds expensively a funnel of triangles containing always the optimal path
//! \brief should only be used for small distances!
//! \param r_start starting position
//! \param r_end    target position
//! \param radius how big the navigated agent is
//! \returns funnel formed from triangles, which contains the desired path
Funnel PathFinder::findOptimalPath(sf::Vector2f r_start, sf::Vector2f r_end, float radius) {

    const auto& triangles = cdt_->triangles_;
    const auto& vertices = cdt_->vertices_;

    const auto start = cdt_->findTriangle(r_start, false);
    const auto end = cdt_->findTriangle(r_end, false);

    if (start == -1 or end == -1 or start == end) {
        return Funnel();
    }

    for (int i = 0; i < triangles.size(); ++i) {
        vertex2shortest_distance[i] = MAXFLOAT;
        shortest_path[i] = -1;
        for (int k = 0; k < 3; ++k) {
            if (!triangles[i].is_constrained[k]) {
                tri_edge2g_function_.at(i)[k] = MAXFLOAT;
                tri_edge2h_function_.at(i)[k] = 0;
            }
        }
    }

    std::vector<std::pair<TriInd, int>> to_visit2; //! not sure why this is needed?
    std::priority_queue to_visit(to_visit2.begin(), to_visit2.end(), [&](const auto& p1, const auto& p2) {
        return tri_edge2g_function_.at(p1.first)[p1.second] + tri_edge2h_function_.at(p1.first)[p1.second] >
               tri_edge2g_function_.at(p2.first)[p2.second] + tri_edge2h_function_.at(p2.first)[p2.second];
    });
    vertex2shortest_distance.at(start) = 0;

    int ind_in_tri_corridor;
    TriInd tri_ind_corridor;
    TriInd prev_tri_ind_corridor;

    const auto& start_tri = triangles[start];
    for (int ind_in_tri = 0; ind_in_tri < 3; ++ind_in_tri) {
        if (start_tri.is_constrained[ind_in_tri]) {
            continue;
        }

        FunnelFan current_portal =
            FunnelFan(r_start, asFloat(start_tri.verts[next(ind_in_tri)]), asFloat(start_tri.verts[ind_in_tri]));

        ind_in_tri_corridor = ind_in_tri;
        tri_ind_corridor = start_tri.neighbours[ind_in_tri];
        prev_tri_ind_corridor = start;
        int prev_ind_in_tri_corridor = ind_in_tri;

        int n_free = 0;
        while (!(n_free == 2 or
                 tri_ind_corridor == end)) { //! walk until end or next crossroads and construct portals along the way

            const auto& tri = triangles[tri_ind_corridor];
            n_free = 0;
            for (int k = 0; k < 3; ++k) {
                if (!tri.is_constrained[k] and tri.neighbours[k] != prev_tri_ind_corridor) {
                    n_free++;
                    ind_in_tri_corridor = k;
                }
            }
            if (n_free == 2 or tri_ind_corridor == end or n_free == 0) {
                break;
            }
            const auto new_clock_point = asFloat(tri.verts[ind_in_tri_corridor]);
            const auto new_anticlock_point = asFloat(tri.verts[next(ind_in_tri_corridor)]);
            current_portal.addEdge(new_anticlock_point, new_clock_point);

            shortest_path.at(tri_ind_corridor) = prev_tri_ind_corridor;
            tri_edge2shortest_path_.at(tri_ind_corridor)[ind_in_tri_corridor] = {prev_tri_ind_corridor,
                                                                                 prev_ind_in_tri_corridor};
            prev_tri_ind_corridor = tri_ind_corridor;
            prev_ind_in_tri_corridor = ind_in_tri_corridor;
            tri_ind_corridor = tri.neighbours[ind_in_tri_corridor];
        }

        if (tri_ind_corridor == end) { //! if we found end we backtrack to start
            float distance = current_portal.calcDistanceToEndPoint(r_end);
            if (vertex2shortest_distance.at(end) >= distance) {
                vertex2shortest_distance.at(end) = distance;
                shortest_path[end] = prev_tri_ind_corridor;
                tri_edge2shortest_path_[end][0] = {prev_tri_ind_corridor, prev_ind_in_tri_corridor};
                int tri_ind = end;
                int prev = end;
                int k = 0;
                while (tri_ind != start) {
                    auto a = tri_edge2shortest_path_[tri_ind][k];
                    tri_ind = a.first;
                    k = a.second;
                    shortest_path[prev] = tri_ind;
                    tri_edge2g_function_.at(tri_ind)[k] = distance;
                    prev = tri_ind;
                }
            }
        } else {
            const auto& tri = triangles[tri_ind_corridor];
            for (int k = 0; k < 3; ++k) {
                if (!tri.is_constrained[k] and tri.neighbours[k] != prev_tri_ind_corridor) {
                    FunnelFan new_portal = current_portal;
                    to_visit.push({tri_ind_corridor, k});
                    const auto new_clock_point = asFloat(tri.verts[k]);
                    const auto new_anticlock_point = asFloat(tri.verts[next(k)]);
                    new_portal.addEdge(new_anticlock_point, new_clock_point);

                    tri_edge2portal_[tri_ind_corridor * 3 + k] = new_portal;

                    const auto g_h_values = calcAstarData(new_portal, r_end);

                    tri_edge2shortest_path_.at(tri_ind_corridor)[k] = {prev_tri_ind_corridor, prev_ind_in_tri_corridor};
                    tri_edge2g_function_.at(tri_ind_corridor)[k] = g_h_values.g_value;
                    tri_edge2h_function_.at(tri_ind_corridor)[k] = g_h_values.h_value;
                }
            }
        }
    }

    TriInd prev_tri_ind = -1;
    TriInd entry_tri_ind = -1;
    int current_ind_in_tri = -1;
    TriInd current_tri_ind = -1;
    auto count_free_edges = [](const Triangle& tri, int prev_tri_ind) {
        int n_free_edges = 0;
        for (int k = 0; k < 3; ++k) {
            if (!tri.is_constrained[k] and tri.neighbours[k] != prev_tri_ind) {
                n_free_edges += 1;
            }
        }
        return n_free_edges;
    };

    //! this is where the Astar starts;
    while (!(to_visit.empty())) {
        entry_tri_ind = to_visit.top().first;
        current_ind_in_tri = to_visit.top().second;
        current_tri_ind = triangles[entry_tri_ind].neighbours[current_ind_in_tri];
        to_visit.pop();

        FunnelFan& cp = tri_edge2portal_.at(entry_tri_ind * 3 + current_ind_in_tri);

        const auto& current_tri = triangles[current_tri_ind];

        tri_ind_corridor = current_tri_ind;
        prev_tri_ind_corridor = entry_tri_ind;
        ind_in_tri_corridor = current_ind_in_tri;
        int prev_ind_in_tri_corridor = current_ind_in_tri;
        int n_free = 0;
        while (!(n_free == 2 or tri_ind_corridor ==
                                    end)) { //! we walk along the corridor until either end, dead-end or next crossroads
            const auto& tri = triangles[tri_ind_corridor];
            n_free = 0;
            for (int k = 0; k < 3; ++k) {
                if (!tri.is_constrained[k] and tri.neighbours[k] != prev_tri_ind_corridor) {
                    n_free++;
                    ind_in_tri_corridor = k;
                }
            }
            if (tri_ind_corridor == end or n_free == 2 or n_free == 0) {
                break;
            }
            const auto new_clock_point = asFloat(tri.verts[ind_in_tri_corridor]);
            const auto new_anticlock_point = asFloat(tri.verts[next(ind_in_tri_corridor)]);
            cp.addEdge(new_anticlock_point, new_clock_point);

            tri_edge2shortest_path_[tri_ind_corridor][ind_in_tri_corridor] = {prev_tri_ind_corridor,
                                                                              prev_ind_in_tri_corridor};
            prev_tri_ind_corridor = tri_ind_corridor;
            tri_ind_corridor = tri.neighbours[ind_in_tri_corridor];
            prev_ind_in_tri_corridor = ind_in_tri_corridor;
        }
        current_tri_ind = tri_ind_corridor;
        if (current_tri_ind == end) { //! when end is found we backtrack to start noting the way in shortest_path vector
            float distance = cp.calcDistanceToEndPoint(r_end);
            if (vertex2shortest_distance.at(end) >= distance) {
                vertex2shortest_distance.at(end) = distance;
                shortest_path[end] = prev_tri_ind_corridor;
                tri_edge2shortest_path_[end][0] = {prev_tri_ind_corridor, prev_ind_in_tri_corridor};
                int tri_ind = end;
                int prev = end;
                int k = 0;
                while (tri_ind != start) {
                    auto a = tri_edge2shortest_path_[tri_ind][k];
                    tri_ind = a.first;
                    k = a.second;
                    shortest_path[prev] = tri_ind;
                    tri_edge2g_function_.at(tri_ind)[k] = distance;
                    prev = tri_ind;
                }
            }
            continue;
        }

        const auto& tri = triangles[tri_ind_corridor];
        for (int k = 0; k < 3; ++k) {
            if (!tri.is_constrained[k] and tri.neighbours[k] != prev_tri_ind_corridor) {
                FunnelFan new_portal = cp;
                const auto new_clock_point = asFloat(tri.verts[k]);
                const auto new_anticlock_point = asFloat(tri.verts[next(k)]);
                new_portal.addEdge(new_anticlock_point, new_clock_point);

                const auto g_h_values = calcAstarData(new_portal, r_end);
                if (g_h_values.g_value < tri_edge2g_function_[current_tri_ind][k]) {
                    tri_edge2portal_[3 * tri_ind_corridor + k] = new_portal;
                    to_visit.push({tri_ind_corridor, k});
                    tri_edge2shortest_path_[current_tri_ind][k] = {prev_tri_ind_corridor, prev_ind_in_tri_corridor};
                    tri_edge2g_function_[current_tri_ind][k] = g_h_values.g_value;
                    tri_edge2h_function_[current_tri_ind][k] = g_h_values.h_value;
                }
            }
        }
    }

    //! walk back to start from end and construct polygon for funnel algorithm
    Funnel funnel;
    TriInd next_tri_ind = end;
    while (next_tri_ind != start) {
        if (next_tri_ind == -1) {
            return Funnel();
        }
        const auto& next_triangle = triangles[next_tri_ind];

        auto next_neighbour =
            std::find(next_triangle.neighbours.begin(), next_triangle.neighbours.end(), shortest_path[next_tri_ind]);
        int index_of_next_neighbour = next_neighbour - next_triangle.neighbours.begin();
        if (index_of_next_neighbour >= 3) {
            return Funnel();
        }
        assert(index_of_next_neighbour >= 0 and index_of_next_neighbour < 3);

        auto left_vertex_of_portal = asFloat(next_triangle.verts[index_of_next_neighbour]);
        auto right_vertex_of_portal = asFloat(next_triangle.verts[next(index_of_next_neighbour)]);
        funnel.emplace_back(right_vertex_of_portal, left_vertex_of_portal);

        next_tri_ind = shortest_path[next_tri_ind];
    }

    return funnel;
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
std::vector<PathFinder::AstarReducedDataPQ>
PathFinder::initializeReducedAstar(const sf::Vector2f& r_start, const sf::Vector2f& r_end, const TriInd start,
                                   const ReducedVertexInd reduced_start, const TriInd end,
                                   float& current_shortest_distance, const float radius,
                                   std::vector<AstarReducedData>& reduced_vertex2astar_data) const {
    const auto& triangles = cdt_->triangles_;
    const auto& start_tri = triangles[start];
    const auto& rtg_ = *p_rtg_;

    const auto n_reduced = rtg_.vertex2edge_inds2.size();
    for (int reduced_i = 0; reduced_i < n_reduced; ++reduced_i) {
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
    for (int ind_in_tri = 0; ind_in_tri < 3; ++ind_in_tri) {
        if (start_tri.is_constrained[ind_in_tri]) {
            continue;
        }
        assert(start_tri.neighbours[ind_in_tri] != -1);
        if (n_free_edges_start != 2) {
            const auto vertex_ind = rtg_.tri_ind2vertex[start];
            const auto& neighbour_corridor_width = rtg_.edges[rtg_.vertex2edge_inds2[vertex_ind][ind_in_tri]].width;
            if (2 * radius >= neighbour_corridor_width) {
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
        while (!(n_free == 2 or
                 tri_ind_corridor == end)) { //! walk until end or next crossroads and construct portals along the way

            const auto& tri = triangles[tri_ind_corridor];
            n_free = 0;
            for (int k = 0; k < 3; ++k) {
                if (!tri.is_constrained[k] and tri.neighbours[k] != prev_tri_ind_corridor) {
                    n_free++;
                    ind_in_tri_corridor = k;
                }
            }
            if (n_free == 2 or tri_ind_corridor == end or n_free == 0) {
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
        const auto& tri = triangles[tri_ind_corridor];
        int i = indInTriOf(tri, prev_tri_ind_corridor);

        if (tri_ind_corridor == end) { //! we found end

            funnel.emplace_back(r_end, r_end);
            float distance = rtg_.funnelDistance(r_start, r_end, funnel, *cdt_);
            if (current_shortest_distance >= distance) {
                reduced_vertex2astar_data.back().prev = reduced_start;
                reduced_vertex2astar_data.back().ind_in_tri = ind_in_tri;
                reduced_vertex2astar_data.back().g_value = distance;
                current_shortest_distance = distance;
            }
        } else {
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

void PathFinder::findSubOptimalPathCenters(sf::Vector2f r_start, sf::Vector2f r_end, float radius, Funnel& funnel) {
    findSubOptimalPathCenters(r_start, r_end, radius, funnel, 0);
}


//! \brief finds sequence of triangles such that the path going through centers of the triangles is the shortest
//! \param r_start starting position
//! \param r_end end position
//! \param radius to block paths that are too narrow
//! \param funnel stores data used to create real path going through the triangles
//! \param thread_id which thread runs the job?
void PathFinder::findSubOptimalPathCenters(sf::Vector2f r_start, sf::Vector2f r_end, float radius, Funnel& funnel,
                                           const int thread_id) {

    const auto& triangles = cdt_->triangles_;
    const auto& vertices = cdt_->vertices_;
    const auto& rtg = *p_rtg_;

    int start = cdt_->findTriangle(r_start, false);
    int end = cdt_->findTriangle(r_end, false);

    auto triangle_is_within_boundary = [&](const Triangle& tri) {
        const auto tri_center = cdt_->calcTriangleCenter(tri);
        return cdt_->boundary_.x >= tri_center.x and tri_center.x > 0 and cdt_->boundary_.y >= tri_center.y and
               tri_center.y > 0;
    };
    assert(triangle_is_within_boundary(triangles[start]));
    assert(triangle_is_within_boundary(triangles[end]));
    
    //! if we cannot reach the destination due to it being in a different graph component
    if (tri_ind2component_.at(start) != tri_ind2component_.at(end)) {
        const auto new_end_tri_and_edge =
            closestPointOnNavigableComponent(r_start, end, tri_ind2component_[end], tri_ind2component_[start]);
        end = new_end_tri_and_edge.first;
        const auto to = new_end_tri_and_edge.second;
        r_end = cdt_->calcTriangleCenter(triangles[end]);
    }
    if (start == -1 or end == -1 or start == end) {
        return;
    }

    // std::vector<AstarReducedData> reduced_vertex2astar_data(p_rtg_->reduced_vertices.size() + 2);

    const auto& start_tri = triangles[start];
    const auto& end_tri = triangles[end];

    const int n_free_edges_end = end_tri.countFreeEdges();
    const int n_free_edges_start = start_tri.countFreeEdges();
    int end_edge_ind = -1;
    int reduced_start = rtg.reduced_vertices.size();
    // reduced_vertex2astar_data_[reduced_start].prev = start;
    auto& reduced_vertex2astar_data = thread2reduced_vertex2astar_data_[thread_id];
    reduced_vertex2astar_data[reduced_start].prev = start;
    int reduced_end = rtg.reduced_vertices.size() + 1;
    if (n_free_edges_end == 2) {
        end_edge_ind = rtg.tri_ind2vertex.at(end); //! this holds edge_indices for triangles that are corridors
    }

    float current_shortest_distance = MAXFLOAT;

    auto to_visit = initializeReducedAstar(r_start, r_end, start, reduced_start, end, current_shortest_distance, radius,
                                           reduced_vertex2astar_data);

    std::priority_queue to_visitx(
        to_visit.begin(), to_visit.end(),
        [&](const AstarReducedDataPQ& a1, const AstarReducedDataPQ& a2) { return a1.f_value > a2.f_value; });

    //! if end lies in a corridor we remember which crossorads it connects and search for on of possible ends
    int first_reduced_end;
    int second_reduced_end;
    if (n_free_edges_end == 2) {
        second_reduced_end = rtg.tri_ind2vertex[rtg.edges[end_edge_ind].end.current];
        first_reduced_end = rtg.tri_ind2vertex[rtg.edges[end_edge_ind].start.current];
    } else { //! if it lies on a  reduced vertex we search only for one ind;
        first_reduced_end = rtg.tri_ind2vertex[end];
        second_reduced_end = first_reduced_end;
    }
    auto reached_end = [first_reduced_end, second_reduced_end](int reduced_ind) {
        return reduced_ind == first_reduced_end or reduced_ind == second_reduced_end;
    };
    
    //! this is where Astar starts;
    int prev_reduced_ind = reduced_start;
    TriInd prev_tri_ind = -1;
    TriInd entry_tri_ind = -1;
    int entry_ind_in_tri = -1;
    TriInd current_tri_ind = -1;
    while (!(to_visitx.empty())) {

        const auto reduced_ind = to_visitx.top().next;
        const auto& reduced_vertex = rtg.reduced_vertices[reduced_ind];
        to_visitx.pop();

        const float distance_start_to_current_tri_center = reduced_vertex2astar_data[reduced_ind].g_value;

        if (reached_end(reduced_ind)) {
            int entry_ind_in_tri;
            int entry_reduced_ind;
            if (n_free_edges_end == 2) { //! end triangle is in corridor corresponding to edge or in crossroads
                entry_reduced_ind = reduced_ind;
                if (first_reduced_end == reduced_ind) {
                    entry_ind_in_tri = std::find(reduced_vertex.neighbours.begin(), reduced_vertex.neighbours.end(),
                                                 second_reduced_end) -
                                       reduced_vertex.neighbours.begin();
                }
                if (reduced_ind == second_reduced_end) {
                    entry_ind_in_tri = std::find(reduced_vertex.neighbours.begin(), reduced_vertex.neighbours.end(),
                                                 first_reduced_end) -
                                       reduced_vertex.neighbours.begin();
                    
                }
            } else {
                entry_ind_in_tri = reduced_vertex2astar_data[reduced_ind].ind_in_tri;
                entry_reduced_ind = reduced_vertex2astar_data[reduced_ind].prev;
            }

            int n_free; int ind_in_tri_corridor;
            int current_tri_ind = rtg.vertex2tri_ind.at(reduced_ind);
            const auto r_center = cdt_->calcTriangleCenter(cdt_->triangles_.at(current_tri_ind));
            Funnel end_funnel;
            end_funnel.push_back({r_center, r_center}); //! funnel is from center of current triangle to r_end;
            if(triangles[current_tri_ind].neighbours[entry_ind_in_tri] != end){
                const auto edge_ind = rtg.vertex2edge_inds2.at(reduced_ind).at(entry_ind_in_tri);
                const auto& edge = rtg.edges.at(edge_ind);
                int end_ind_in_edge = std::find(edge.tri_inds.begin(), edge.tri_inds.end(), end) - edge.tri_inds.begin();
                if (first_reduced_end == reduced_ind) {
                    for(int ind_in_edge = 0; ind_in_edge < end_ind_in_edge; ++ind_in_edge)
                    {
                        const auto ind_in_tri_corridor = edge.from_start.at(ind_in_edge);
                        const auto& tri = triangles.at(edge.tri_inds.at(ind_in_edge));
                        auto left_vertex_of_portal = asFloat(tri.verts[(ind_in_tri_corridor)]);
                        auto right_vertex_of_portal = asFloat(tri.verts[next(ind_in_tri_corridor)]);
                        end_funnel.emplace_back(right_vertex_of_portal, left_vertex_of_portal);
                    }
                }
                if (second_reduced_end == reduced_ind) {
                    for(int ind_in_edge = end_ind_in_edge-1; ind_in_edge >= 0; --ind_in_edge)
                    {
                        const auto ind_in_tri_corridor = edge.from_end.at(ind_in_edge);
                        const auto& tri = triangles.at(edge.tri_inds.at(ind_in_edge));
                        auto left_vertex_of_portal = asFloat(tri.verts[next(ind_in_tri_corridor)]);
                        auto right_vertex_of_portal = asFloat(tri.verts[(ind_in_tri_corridor)]);
                        end_funnel.emplace_back(right_vertex_of_portal, left_vertex_of_portal);
                    }
                }
            }else{
                const auto& tri = triangles.at(current_tri_ind);
                const auto ind_in_tri_corridor =  std::find(tri.neighbours.begin(), tri.neighbours.end(), end) - tri.neighbours.begin();
                auto left_vertex_of_portal = asFloat(tri.verts[next(ind_in_tri_corridor)]);
                auto right_vertex_of_portal = asFloat(tri.verts[(ind_in_tri_corridor)]);
                end_funnel.emplace_back(right_vertex_of_portal, left_vertex_of_portal);
                // throw std::runtime_error("hi, you forgot to finish me !!!");
            }

            const auto distance_to_end = rtg.funnelDistance(r_center, r_end, end_funnel, *cdt_);

            if (distance_start_to_current_tri_center + distance_to_end <=
                current_shortest_distance) { //! update backpointer if path is better
                reduced_vertex2astar_data.back().prev = entry_reduced_ind;
                reduced_vertex2astar_data.back().ind_in_tri = entry_ind_in_tri;
                current_shortest_distance = distance_start_to_current_tri_center + distance_to_end;
            }
            continue;
        }

        for (int k = 0; k < 3; ++k) {
            const auto& next_reduced_ind = reduced_vertex.neighbours[k];
            if (next_reduced_ind != -1) {
                const auto& neighbour_corridor_width = reduced_vertex.widths[k];
                //                if(2*radius >= neighbour_corridor_width){ continue; }

                float new_g_value = distance_start_to_current_tri_center + reduced_vertex.lengths[k];
                new_g_value += (neighbour_corridor_width <= 2 * radius) * cdt_->boundary_.x;

                const float& h_value = reduced_vertex2astar_data[next_reduced_ind].h_value;
                // const float& h_value = reduced_vertex2astar_data_[next_reduced_ind].h_value;

                // if (new_g_value < reduced_vertex2astar_data[next_reduced_ind].g_value) {
                if (new_g_value < reduced_vertex2astar_data[next_reduced_ind].g_value) {
                    to_visitx.push({new_g_value + h_value, next_reduced_ind});
                    reduced_vertex2astar_data[next_reduced_ind].g_value = new_g_value;
                    reduced_vertex2astar_data[next_reduced_ind].prev = reduced_ind;
                    reduced_vertex2astar_data[next_reduced_ind].ind_in_tri = k;
                }
            }
        }
    }
    //! walk back to start from end and construct polygon for funnel algorithm
    fillFunnelData3(start, reduced_start, end, reduced_end, funnel, reduced_vertex2astar_data);
}

//! \param r  starting point
//! \param start_tri_ind index of triangle containing starting point of the sought path
//! \param end_component triangulation component of the end point
//! \param navigable_component triangulation component of the start point
//! \returns first in pair is triangle ind of the given navigable_component
//! \returns  second is ind_in_tri looking into end_component
std::pair<TriInd, int> PathFinder::closestPointOnNavigableComponent(const sf::Vector2f& r, const TriInd start_tri_ind,
                                                                    const int end_component,
                                                                    const int navigable_component) const {
    const auto& triangles = cdt_->triangles_;
    const auto& vertices = cdt_->vertices_;
    auto current_tri_ind = start_tri_ind;
    std::queue<TriInd> to_visit({current_tri_ind});

    auto current_component = end_component;
    auto prev_tri_ind = current_tri_ind;
    while (current_component != navigable_component and !to_visit.empty()) {
        current_tri_ind = to_visit.front();
        const auto& tri = triangles[current_tri_ind];
        to_visit.pop();
        const auto v0 = tri.verts[0];
        const auto v1 = tri.verts[1];
        const auto v2 = tri.verts[2];

        Edgef line1(cdt_->calcTriangleCenter(triangles[start_tri_ind]), r);

        for (int k = 0; k < 3; ++k) {
            const auto neighbour = tri.neighbours[k];
            if (neighbour != prev_tri_ind and neighbour != -1) {
                Edgef line2(asFloat(tri.verts[k]), asFloat(tri.verts[next(k)]));
                if (cdt_->linesIntersect(line1, line2)) {
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

void PathFinder::fillFunnelData2(const TriInd start, const int reduced_start, const TriInd end, const int reduced_end,
                                 Funnel& funnel) const {

    int next_tri_ind = reduced_vertex2astar_data_.back().prev;
    int next_ind_in_tri = reduced_vertex2astar_data_.back().ind_in_tri;
    int prev_ind_in_tri;

    const auto& rtg_ = *p_rtg_;
    const auto& next_tri = cdt_->triangles_[next_tri_ind];
    const auto& start_tri = cdt_->triangles_[start];
    const auto& end_tri = cdt_->triangles_[end];

    auto count_free_edges = [](const Triangle& tri) {
        return static_cast<int>(!tri.is_constrained[0] + !tri.is_constrained[1] + !tri.is_constrained[2]);
    };

    const int n_free_edges_end = count_free_edges(end_tri);
    const int n_free_edges_start = count_free_edges(start_tri);

    int end_edge_ind = -1;
    if (n_free_edges_end == 2) {
        end_edge_ind = rtg_.tri_ind2vertex.at(end); //! this holds edge_indices for triangles that are corridors
        //! and tri_indices for crossroad and dead-ends
    }

    if (next_tri_ind == reduced_start) {

        int edge_ind = rtg_.tri_ind2vertex[start];
        if (n_free_edges_start != 2) {
            edge_ind = rtg_.vertex2edge_inds2.at(rtg_.tri_ind2vertex[start])[next_ind_in_tri];
        }
        const auto& edge = rtg_.edges.at(edge_ind);

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
        if (end_is_corridor and !start_is_corridor) {
            edge.start.current == start ? edge_is_reversed = true : edge_is_reversed = false;
            fillFunnelWithCorridor(edge, end_ind_in_corridor, !edge_is_reversed * last_ind, funnel, edge_is_reversed,
                                   with_end = false);
        }
        if (end_is_corridor and start_is_corridor) { //! minus one because last triangle in corridor contains start
                                                     //! point and so we do not leave it
            end_ind_in_corridor < start_ind_in_corridor ? edge_is_reversed = false : edge_is_reversed = true;
            fillFunnelWithCorridor(edge, end_ind_in_corridor, start_ind_in_corridor + 2 * (edge_is_reversed)-1, funnel,
                                   edge_is_reversed, with_end = false);
        }
        if (!end_is_corridor and !start_is_corridor) {
            edge.start.current == start ? edge_is_reversed = true : edge_is_reversed = false;
            fillFunnelWithCorridor(edge, edge_is_reversed * last_ind, !edge_is_reversed * last_ind, funnel,
                                   edge_is_reversed, with_end = true);
        }
        if (!end_is_corridor and start_is_corridor) {
            edge.start.current == end ? edge_is_reversed = false : edge_is_reversed = true;
            fillFunnelWithCorridor(edge, edge_is_reversed * last_ind, start_ind_in_corridor + 2 * (edge_is_reversed)-1,
                                   funnel, edge_is_reversed, with_end = true);
        }

        return;
    }
    int prev_tri_ind = -1;
    int prev_reduced_ind = -1;
    int reduced_ind = reduced_vertex2astar_data_.back().prev;
    while (reduced_ind != reduced_start) {
        next_tri_ind = rtg_.vertex2tri_ind[reduced_ind];

        const int edge_ind = rtg_.vertex2edge_inds2.at(reduced_ind)[next_ind_in_tri];
        if (edge_ind == -1 or edge_ind > rtg_.edges.size() - 1) {
            return;
        }
        const auto& edge = rtg_.edges.at(edge_ind);

        int first_ind_in_corridor = 0;
        bool with_start_or_end = true;
        if (edge_ind == end_edge_ind and
            n_free_edges_end ==
                2) { //! end is in corridor this part is done only when walking from end triangle to next crossroads
            first_ind_in_corridor = std::find(edge.tri_inds.begin(), edge.tri_inds.end(), end) - edge.tri_inds.begin();
            with_start_or_end = false;
        }

        int tri_ind;
        if (edge.end.current == next_tri_ind) {
            fillFunnelWithCorridor(edge, first_ind_in_corridor, edge.tri_inds.size() - 1, funnel, false,
                                   with_start_or_end);
            tri_ind = edge.end.current;
        }
        if (edge.start.current == next_tri_ind) {
            if (!(edge_ind == end_edge_ind and n_free_edges_end == 2)) {
                first_ind_in_corridor = edge.tri_inds.size() - 1;
            }
            fillFunnelWithCorridor(edge, first_ind_in_corridor, 0, funnel, true, with_start_or_end);
            tri_ind = edge.start.current;
        }

        prev_tri_ind = next_tri_ind;
        prev_ind_in_tri = next_ind_in_tri;
        //        next_ind_in_tri = tri_edge2shortest_path_[tri_ind][prev_ind_in_tri].second;

        prev_reduced_ind = reduced_ind;
        next_ind_in_tri = reduced_vertex2astar_data_[reduced_ind].ind_in_tri;
        reduced_ind = reduced_vertex2astar_data_[reduced_ind].prev;
    }

    int edge_ind = rtg_.tri_ind2vertex[start];
    if (n_free_edges_start != 2) {
        edge_ind = rtg_.vertex2edge_inds2[prev_reduced_ind][next_ind_in_tri];
    }

    const auto& edge = rtg_.edges.at(edge_ind);
    if (edge.end.current == prev_tri_ind) {
        int last_ind = 0;
        if (n_free_edges_start == 2) { //! end is in corridor
            last_ind = std::find(edge.tri_inds.begin(), edge.tri_inds.end(), start) - edge.tri_inds.begin();
        }
        fillFunnelWithCorridor(edge, edge.tri_inds.size() - 1, last_ind + 1, funnel, true, true);
    }
    if (edge.start.current == prev_tri_ind) {
        int last_ind = edge.tri_inds.size();
        if (n_free_edges_start == 2) { //! end is in corridor
            last_ind = std::find(edge.tri_inds.begin(), edge.tri_inds.end(), start) - edge.tri_inds.begin();
        }
        fillFunnelWithCorridor(edge, 0, last_ind - 1, funnel, false, true);
    }
}

void PathFinder::fillFunnelData3(const TriInd start, const int reduced_start, const TriInd end, const int reduced_end,
                                 Funnel& funnel,
                                 const std::vector<AstarReducedData>& reduced_vertex2astar_data) const {

    int next_tri_ind = reduced_vertex2astar_data.back().prev;
    int next_ind_in_tri = reduced_vertex2astar_data.back().ind_in_tri;
    int prev_ind_in_tri;

    const auto& rtg_ = *p_rtg_;
    const auto& next_tri = cdt_->triangles_[next_tri_ind];
    const auto& start_tri = cdt_->triangles_[start];
    const auto& end_tri = cdt_->triangles_[end];

    auto count_free_edges = [](const Triangle& tri) {
        return static_cast<int>(!tri.is_constrained[0] + !tri.is_constrained[1] + !tri.is_constrained[2]);
    };

    const int n_free_edges_end = count_free_edges(end_tri);
    const int n_free_edges_start = count_free_edges(start_tri);

    int end_edge_ind = -1;
    if (n_free_edges_end == 2) {
        end_edge_ind = rtg_.tri_ind2vertex.at(end); //! this holds edge_indices for triangles that are corridors
        //! and tri_indices for crossroad and dead-ends
    }

    if (next_tri_ind == reduced_start) {

        int edge_ind = rtg_.tri_ind2vertex[start];
        if (n_free_edges_start != 2) {
            edge_ind = rtg_.vertex2edge_inds2.at(rtg_.tri_ind2vertex[start])[next_ind_in_tri];
        }
        const auto& edge = rtg_.edges.at(edge_ind);

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
        if (end_is_corridor && !start_is_corridor) {
            edge.start.current == start ? edge_is_reversed = true : edge_is_reversed = false;
            fillFunnelWithCorridor(edge, end_ind_in_corridor, !edge_is_reversed * last_ind, funnel, edge_is_reversed,
                                   with_end = false);
        }
        if (end_is_corridor && start_is_corridor) { //! minus one because last triangle in corridor contains start
                                                     //! point and so we do not leave it
            end_ind_in_corridor < start_ind_in_corridor ? edge_is_reversed = false : edge_is_reversed = true;
            fillFunnelWithCorridor(edge, end_ind_in_corridor, start_ind_in_corridor + 2 * (edge_is_reversed)-1, funnel,
                                   edge_is_reversed, with_end = false);
        }
        if (!end_is_corridor and !start_is_corridor) {
            edge.start.current == start ? edge_is_reversed = true : edge_is_reversed = false;
            fillFunnelWithCorridor(edge, edge_is_reversed * last_ind, !edge_is_reversed * last_ind, funnel,
                                   edge_is_reversed, with_end = true);
        }
        if (!end_is_corridor and start_is_corridor) {
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
    while (reduced_ind != reduced_start) {
        next_tri_ind = rtg_.vertex2tri_ind[reduced_ind];

        const int edge_ind = rtg_.vertex2edge_inds2.at(reduced_ind)[next_ind_in_tri];
        if (edge_ind == -1 or edge_ind > rtg_.edges.size() - 1) {
            return;
        }
        const auto& edge = rtg_.edges.at(edge_ind);

        int first_ind_in_corridor = 0;
        bool with_start_or_end = true;
        if (edge_ind == end_edge_ind and
            n_free_edges_end ==
                2) { //! end is in corridor this part is done only when walking from end triangle to next crossroads
            first_ind_in_corridor = std::find(edge.tri_inds.begin(), edge.tri_inds.end(), end) - edge.tri_inds.begin();
            with_start_or_end = false;
        }

        int tri_ind;
        if (edge.end.current == next_tri_ind) {
            fillFunnelWithCorridor(edge, first_ind_in_corridor, edge.tri_inds.size() - 1, funnel, false,
                                   with_start_or_end);
            tri_ind = edge.end.current;
        }
        if (edge.start.current == next_tri_ind) {
            if (!(edge_ind == end_edge_ind and n_free_edges_end == 2)) {
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
        if (prev_reduced_ind == reduced_ind) { //! this means no path was found! path will be straight line
            return;
        }
    }

    int edge_ind = rtg_.tri_ind2vertex[start];
    if (n_free_edges_start != 2) {
        edge_ind = rtg_.vertex2edge_inds2[prev_reduced_ind][next_ind_in_tri];
    }

    const auto& edge = rtg_.edges.at(edge_ind);
    if (edge.end.current == prev_tri_ind) {
        int last_ind = 0;
        if (n_free_edges_start == 2) { //! end is in corridor
            last_ind = std::find(edge.tri_inds.begin(), edge.tri_inds.end(), start) - edge.tri_inds.begin();
        }
        fillFunnelWithCorridor(edge, edge.tri_inds.size() - 1, last_ind + 1, funnel, true, true);
    }
    if (edge.start.current == prev_tri_ind) {
        int last_ind = edge.tri_inds.size();
        if (n_free_edges_start == 2) { //! end is in corridor
            last_ind = std::find(edge.tri_inds.begin(), edge.tri_inds.end(), start) - edge.tri_inds.begin();
        }
        fillFunnelWithCorridor(edge, 0, last_ind - 1, funnel, false, true);
    }
}

void PathFinder::fillFunnelData(const TriInd start, const TriInd end, Funnel& fd) const {

    const auto& triangles = cdt_->triangles_;
    const auto& vertices = cdt_->vertices_;
    const auto& rtg_ = *p_rtg_;

    const auto& start_tri = triangles[start];
    const auto& end_tri = triangles[end];

    auto count_free_edges = [](const Triangle& tri) {
        return static_cast<int>(!tri.is_constrained[0] + !tri.is_constrained[1] + !tri.is_constrained[2]);
    };

    const int n_free_edges_end = count_free_edges(end_tri);
    const int n_free_edges_start = count_free_edges(start_tri);

    int end_edge_ind = -1;
    if (n_free_edges_end == 2) {
        end_edge_ind = rtg_.tri_ind2vertex.at(end); //! this holds edge_indices for triangles that are corridors
        //! and tri_indices for crossroad and dead-ends
    }

    //! walk back to start from end and construct polygon for funnel algorithm
    Funnel funnel;
    int next_tri_ind = tri_ind2shortest_path_[end].first;
    int next_ind_in_tri = tri_ind2shortest_path_[end].second;
    int prev_ind_in_tri;
    const auto& next_tri = triangles[next_tri_ind];
    if (next_tri_ind == start) {

        int edge_ind = rtg_.tri_ind2vertex[start];
        if (n_free_edges_start != 2) {
            edge_ind = rtg_.vertex2edge_inds2.at(rtg_.tri_ind2vertex[start])[next_ind_in_tri];
        }
        const auto& edge = rtg_.edges.at(edge_ind);

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
        if (end_is_corridor and !start_is_corridor) {
            edge.start.current == start ? edge_is_reversed = true : edge_is_reversed = false;
            fillFunnelWithCorridor(edge, end_ind_in_corridor, !edge_is_reversed * last_ind, funnel, edge_is_reversed,
                                   with_end = false);
        }
        if (end_is_corridor and start_is_corridor) { //! minus one because last triangle in corridor contains start
                                                     //! point and so we do not leave it
            end_ind_in_corridor < start_ind_in_corridor ? edge_is_reversed = false : edge_is_reversed = true;
            fillFunnelWithCorridor(edge, end_ind_in_corridor, start_ind_in_corridor + 2 * (edge_is_reversed)-1, funnel,
                                   edge_is_reversed, with_end = false);
        }
        if (!end_is_corridor and !start_is_corridor) {
            edge.start.current == start ? edge_is_reversed = true : edge_is_reversed = false;
            fillFunnelWithCorridor(edge, edge_is_reversed * last_ind, !edge_is_reversed * last_ind, funnel,
                                   edge_is_reversed, with_end = true);
        }
        if (!end_is_corridor and start_is_corridor) {
            edge.start.current == end ? edge_is_reversed = false : edge_is_reversed = true;
            fillFunnelWithCorridor(edge, edge_is_reversed * last_ind, start_ind_in_corridor + 2 * (edge_is_reversed)-1,
                                   funnel, edge_is_reversed, with_end = true);
        }
    }

    int prev_tri_ind = -1;
    while (next_tri_ind != start) {

        const int edge_ind = rtg_.vertex2edge_inds2.at(rtg_.tri_ind2vertex[next_tri_ind])[next_ind_in_tri];
        if (edge_ind == -1 or edge_ind > rtg_.edges.size() - 1) {
            fd = Funnel();
            return;
        }
        const auto& edge = rtg_.edges.at(edge_ind);

        int first_ind_in_corridor = 0;
        bool with_start_or_end = true;
        if (edge_ind == end_edge_ind and
            n_free_edges_end ==
                2) { //! end is in corridor this part is done only when walking from end triangle to next crossroads
            first_ind_in_corridor = std::find(edge.tri_inds.begin(), edge.tri_inds.end(), end) - edge.tri_inds.begin();
            with_start_or_end = false;
        }

        int tri_ind;
        if (edge.end.current == next_tri_ind) {
            fillFunnelWithCorridor(edge, first_ind_in_corridor, edge.tri_inds.size() - 1, funnel, false,
                                   with_start_or_end);
            tri_ind = edge.end.current;
        }
        if (edge.start.current == next_tri_ind) {
            if (!(edge_ind == end_edge_ind and n_free_edges_end == 2)) {
                first_ind_in_corridor = edge.tri_inds.size() - 1;
            }
            fillFunnelWithCorridor(edge, first_ind_in_corridor, 0, funnel, true, with_start_or_end);
            tri_ind = edge.start.current;
        }
        prev_tri_ind = next_tri_ind;
        prev_ind_in_tri = next_ind_in_tri;
        next_tri_ind = tri_edge2shortest_path_[tri_ind][prev_ind_in_tri].first;
        next_ind_in_tri = tri_edge2shortest_path_[tri_ind][prev_ind_in_tri].second;
    }

    //! this should be always true I think?
    if (next_tri_ind == start) {
        int edge_ind = rtg_.tri_ind2vertex[start];
        if (n_free_edges_start != 2) {
            edge_ind = rtg_.vertex2edge_inds2[rtg_.tri_ind2vertex[prev_tri_ind]][next_ind_in_tri];
        }

        const auto& edge = rtg_.edges.at(edge_ind);
        if (edge.end.current == prev_tri_ind) {
            int last_ind = 0;
            if (n_free_edges_start == 2) { //! end is in corridor
                last_ind = std::find(edge.tri_inds.begin(), edge.tri_inds.end(), start) - edge.tri_inds.begin();
            }
            fillFunnelWithCorridor(edge, edge.tri_inds.size() - 1, last_ind + 1, funnel, true, true);
        }
        if (edge.start.current == prev_tri_ind) {
            int last_ind = edge.tri_inds.size();
            if (n_free_edges_start == 2) { //! end is in corridor
                last_ind = std::find(edge.tri_inds.begin(), edge.tri_inds.end(), start) - edge.tri_inds.begin();
            }
            fillFunnelWithCorridor(edge, 0, last_ind - 1, funnel, false, true);
        }
    } else {
        fd = Funnel();
        return;
    }
}

//! TODO: figure out why I get segfault when I use sign(..) function from "core.h"
//!       but not this one with optimisation lvl at least -O2?
float PathFinder::sign(sf::Vector2f a, sf::Vector2f b, sf::Vector2f c) const {
    const auto ax = b.x - a.x;
    const auto ay = b.y - a.y;
    const auto bx = c.x - a.x;
    const auto by = c.y - a.y;
    const auto area = by * ax - ay * bx;
    return area;
}

PathFinder::TriangleWidth::TriangleWidth(Triangle& tri, const Triangulation& cdt) {

    int n_constraints = 0;
    int free_vertex_ind_in_tri = 0;

    for (int i = 0; i < 3; ++i) {
        if (tri.is_constrained[i]) {
            n_constraints++;
            free_vertex_ind_in_tri = (i + 2) % 3;
            widths[i] = MAXFLOAT;
        }
    }
    if (n_constraints == 1) { //! only one vertex can be "circled around" so need to calc. just one width
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
    } else if (n_constraints == 0) { //! each vertex can be "circled around" and thus has width
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
Edgef pushAwayFromCorner2(sf::Vector2f& r_to_push, const sf::Vector2f& r_prev, const sf::Vector2f& r_next,
                          const float distance) {

    sf::Vector2f n10({0, 0});
    sf::Vector2f n20({0, 0});

    const auto v0 = r_to_push;
    const auto v1 = r_prev;
    const auto v2 = r_next;
    n10 = {(v1.y - v0.y), -(v1.x - v0.x)};
    n20 = {(v2.y - v0.y), -(v2.x - v0.x)};
    n10 /= norm(n10);
    n20 /= norm(n20);
    if (dot(n10, v0 - v2) < 0) {
        n10 *= -1.f;
    }
    if (dot(n20, v0 - v1) < 0) {
        n20 *= -1.f;
    }

    r_to_push += (n10 + n20) * distance / norm(n10 + n20);
    Edgef p;
    p.from = r_to_push;
    p.t = (n10 + n20) / norm(n10 + n20);
    p.l = 100 * distance; //!
    return p;
}

//! \brief crates shortest path inside a Polygon defined by FunnelData
//! \param r_start starting position of the path
//! \param r_end end position of the path
//! \param radius defines how much the path will be pushed away from corners
//! \returns shortest path inside funnel and portals (line segments from path points indicating that I passed the path
//! point)
PathFinder::PathAndPortals PathFinder::pathFromFunnel(const sf::Vector2f r_start, const sf::Vector2f r_end,
                                                      const float radius, Funnel& funnel) const {

    PathAndPortals path_and_portals;
    auto& smoothed_path = path_and_portals.path;
    auto& portals = path_and_portals.portals;
    path_and_portals.path = {r_start};
    path_and_portals.portals = {Edgef()};

    const auto& triangles = cdt_->triangles_;
    const auto& vertices = cdt_->vertices_;

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
    for (int i = 1; i < funnel.size(); ++i) {
        auto& next_r = funnel[i].first;
        auto& next_l = funnel[i].second;
        if (!vequal(next_l, prev_left)) {
            unique_left.push_back(i);
            prev_left = next_l;
        }
        if (!vequal(next_r, prev_right)) {
            unique_right.push_back(i);
            prev_right = next_r;
        }
    }

    //! push path points away from walls
    const auto push_distance = radius;
    for (int i = 1; i < unique_left.size() - 1; ++i) {
        const auto& prev_unique_left = funnel[unique_left[i - 1]].second;
        const auto& next_unique_left = funnel[unique_left[i + 1]].second;
        for (int j = unique_left[i]; j < unique_left[i + 1]; ++j) {
            auto& mid_left = funnel[j].second;
            const auto left_portal = pushAwayFromCorner2(mid_left, prev_unique_left, next_unique_left, push_distance);
            left_portals.push_back(left_portal);
        }
    }
    for (int i = 1; i < unique_right.size() - 1; ++i) {
        const auto& prev_unique_right = funnel[unique_right[i - 1]].first;
        const auto& next_unique_right = funnel[unique_right[i + 1]].first;
        for (int j = unique_right[i]; j < unique_right[i + 1]; ++j) {
            auto& mid_right = funnel[j].first;
            const auto right_portal =
                pushAwayFromCorner2(mid_right, prev_unique_right, next_unique_right, push_distance);
            right_portals.push_back(right_portal);
        }
    }

    left_portals.push_back(Edgef());
    right_portals.push_back(Edgef());
    for (int i = 1; i < funnel.size(); ++i) {

        right = funnel[i].first;
        left = funnel[i].second;

        if (sign(portal_apex, portal_right, right) <= 0.f) { //! if the portal shrank from right
            auto is_same_point = vequal(portal_apex, portal_right);
            if (is_same_point || sign(portal_apex, portal_left, right) >
                                     0.f) { //! if the new right segment of the portal crosses the left segment
                portal_right = right;
                right_index = i;
            } else if (vequal(portal_right, right)) {
                right_index = i;
            } else {
                portals.push_back(left_portals[left_index]);

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

        if (sign(portal_apex, portal_left, left) >= 0.f) { //! same as above but we move left portal segment
            auto is_same_point = vequal(portal_apex, portal_left);
            //            if(vequal(portal_left, left)){ continue;}
            if (is_same_point or sign(portal_apex, portal_right, left) < 0.f) {
                portal_left = left;
                left_index = i;
            } else if (vequal(portal_left, left)) {
                left_index = i;
            } else {
                portals.push_back(right_portals[right_index]);

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

//! \brief fills FunnelData with left and right points forming a corridor
//! \param edge of reduced Triangulation graph (contains corridor data)
//! \param first index of first triangle I want from corridor
//! \param last  index of last triangle I want from corridor
//! \param revers is true if we walk from edge.end to edge.start
//! \param with_start_or_end is true if I start in edge.start or edge.end (and not in the middle of the corridor)
void PathFinder::fillFunnelWithCorridor(const ReducedTriangulationGraph::Corridor& edge, int first, int last,
                                        Funnel& funnel, bool reverse, bool with_start_or_end) const {

    const auto& triangles = cdt_->triangles_;

    auto& corridor = edge.tri_inds;

    if (!reverse) {
        if (with_start_or_end) {
            assert(first == 0);
            const auto& tri = triangles[edge.start.current];
            int index_of_next_neighbour = edge.start.to;

            const auto left_vertex_of_portal = asFloat(tri.verts[index_of_next_neighbour]);
            const auto right_vertex_of_portal = asFloat(tri.verts[next(index_of_next_neighbour)]);
            funnel.emplace_back(right_vertex_of_portal, left_vertex_of_portal);
        }
        if (last < 0 or first < 0 or last >= edge.tri_inds.size()) {
            return;
        }
        for (int i = first; i <= last; ++i) {
            const auto& tri = triangles[edge.tri_inds[i]];
            int index_of_next_neighbour = edge.from_start[i];

            const auto left_vertex_of_portal = asFloat(tri.verts[index_of_next_neighbour]);
            const auto right_vertex_of_portal = asFloat(tri.verts[next(index_of_next_neighbour)]);
            funnel.emplace_back(right_vertex_of_portal, left_vertex_of_portal);
        }
    } else {
        if (with_start_or_end) {
            assert(first == edge.tri_inds.size() - 1);
            const auto& tri = triangles[edge.end.current];
            int index_of_next_neighbour = edge.end.to;

            const auto left_vertex_of_portal = asFloat(tri.verts[index_of_next_neighbour]);
            const auto right_vertex_of_portal = asFloat(tri.verts[next(index_of_next_neighbour)]);
            funnel.emplace_back(right_vertex_of_portal, left_vertex_of_portal);
        }
        if (last < 0 or first < 0 or first >= edge.tri_inds.size()) {
            return;
        }
        for (int i = first; i >= last; --i) {
            const auto& tri = triangles[edge.tri_inds[i]];
            int index_of_next_neighbour = edge.from_end[i];

            const auto left_vertex_of_portal = asFloat(tri.verts[index_of_next_neighbour]);
            const auto right_vertex_of_portal = asFloat(tri.verts[next(index_of_next_neighbour)]);
            funnel.emplace_back(right_vertex_of_portal, left_vertex_of_portal);
        }
    }
}

PathFinder::AstarVertexData PathFinder::calcAstarData(const FunnelFan& portal, const sf::Vector2f r_end) {
    AstarVertexData result;
    result.h_value = portal.calcDistanceToEndPoint(r_end) - portal.dist_apex_to_start;
    float left_dist = 0;
    float right_dist = 0;
    sf::Vector2f left_point = portal.apex;
    sf::Vector2f right_point = portal.apex;
    if (portal.left_points.size() > 0) {
        left_dist = portal.left_min_dists.back();
        left_point = portal.left_points.back();
    }
    if (portal.right_points.size() > 0) {
        right_dist = portal.right_min_dists.back();
        right_point = portal.right_points.back();
    }

    result.g_value =
        portal.dist_apex_to_start + 0 * dist(left_point, right_point) + std::max(left_dist, right_dist);
    return result;
}
