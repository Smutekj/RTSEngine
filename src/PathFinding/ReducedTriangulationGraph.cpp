#include "ReducedTriangulationGraph.h"

//! \param funnel 
//! \returns minimal width of a triangle inside a corridor 
float ReducedTriangulationGraph::calcCorridorWidth(
    const Funnel& funnel) const {
    if (funnel.size() == 0) {
        return MAXFLOAT;
    }

    float min_width = dist(funnel[0].first, funnel[0].second);
    for (int i = 1; i < funnel.size(); ++i) {
        const auto& l1 = funnel[i - 1].first;
        const auto& r1 = funnel[i - 1].second;
        const auto& l2 = funnel[i].first;
        const auto& r2 = funnel[i].second;

        Edgef left_segment(l1, l2);
        Edgef right_segment(r1, r2);

        const auto new_width = distanceOfSegments(left_segment, right_segment);
        if (new_width < min_width) {
            min_width = new_width;
        }
    }
    return min_width;
}

float ReducedTriangulationGraph::sign2(const sf::Vector2f& p1, const sf::Vector2f& p2, const sf::Vector2f& p3) const {
    return (p3.x - p1.x) * (p2.y - p1.y) - (p2.x - p1.x) * (p3.y - p1.y);
}

//! \brief computes path distance within triangles funnel using \p funnel_data  
//! \param r_start
//! \param r_end
//! \param funnel
//! \param cdt Triangulation object (is this really needed here?)
//! \returns distance of the shortes path within given funnel
float ReducedTriangulationGraph::funnelDistance(const sf::Vector2f r_start, const sf::Vector2f r_end, Funnel& funnel,
                                                const Triangulation& cdt) const {

    std::deque<sf::Vector2f> smoothed_path = {r_start};

    sf::Vector2f right;
    sf::Vector2f left;
    sf::Vector2f portal_apex = r_start;
    sf::Vector2f portal_right = r_start;
    sf::Vector2f portal_left = r_start;

    int right_index = 0;
    int left_index = 0;
    int apex_index = 0;
    for (int i = 1; i < funnel.size(); ++i) {

        right = funnel[i].first;
        left = funnel[i].second;

        if (sign2(portal_apex, portal_right, right) >= 0.f) { //! if the portal shrank from right
            bool is_same_point = vequal(portal_apex, portal_right);
            if (is_same_point or sign2(portal_apex, portal_left, right) <
                                     0.f) { //! if the new right segment of the portal crosses the left segment
                portal_right = right;
                right_index = i;
            } else {
                smoothed_path.push_back(portal_left);
                portal_apex = portal_left;
                apex_index = left_index;
                portal_right = portal_apex;
                right_index = apex_index;
                i = left_index;
                continue;
            }
        }
        if (sign2(portal_apex, portal_left, left) <= 0.f) { //! same as above but we move left portal segment
            bool is_same_point = vequal(portal_apex, portal_left);
            if (is_same_point or sign2(portal_apex, portal_right, left) > 0.f) {
                portal_left = left;
                left_index = i;
            } else {
                smoothed_path.push_back(portal_right);
                portal_apex = portal_right;
                apex_index = right_index;
                portal_left = portal_apex;
                left_index = apex_index;
                i = right_index;
                continue;
            }
        }
    }
    smoothed_path.push_back(r_end);
    float distance = 0;
    for (int i = 0; i < smoothed_path.size() - 1; ++i) {
        distance += dist(smoothed_path[i], smoothed_path[i + 1]);
    }
    return distance;
}

//! \brief constructs reduced triangulation graph from full triangulation 
//! \brief also assings graph-component to each triangle ind and writes it to \p tri_ind2component  
//! \param cdt full triangulation object
//! \param tri_ind2component  
void ReducedTriangulationGraph::constructFromTriangulation(Triangulation& cdt, std::vector<TriInd>& tri_ind2component) {

    const auto& triangles = cdt.triangles_;
    const auto& vertices = cdt.vertices_;
    tri_ind2vertex.resize(triangles.size());
    vertex2tri_ind.clear();
    vertex2edge_inds2.clear();
    edges.clear();

    auto triangle_is_within_boundary = [&](const Triangle& tri) {
        const auto tri_center = cdt.calcTriangleCenter(tri);
        return cdt.boundary_.x >= tri_center.x and tri_center.x > 0 and cdt.boundary_.y >= tri_center.y and
               tri_center.y > 0;
    };

    auto count_free_edges = [](const Triangle& tri) {
        int n_free_edges = 0;
        for (int k = 0; k < 3; ++k) {
            if (!tri.is_constrained[k]) {
                n_free_edges += 1;
            }
        }
        return n_free_edges;
    };

    auto can_walk_through = [](const Triangle& tri, int ind_in_tri) {
        return !tri.is_constrained[ind_in_tri] and tri.neighbours[ind_in_tri] != -1;
    };

    tri_ind2component.resize(triangles.size());
    std::cout << "there are " << std::to_string(triangles.size()) << " in triangulation\n";

    std::vector<bool> visited_tris(triangles.size(), false);
    int component_ind = 0;
    auto flood = [&](const TriInd tri_ind) {
        if (visited_tris[tri_ind]) {
            return;
        }
        std::queue<TriInd> to_visit({tri_ind});

        while (!to_visit.empty()) {
            const auto entry_tri_ind = to_visit.front();
            const auto& entry_tri = triangles[entry_tri_ind];
            to_visit.pop();
            if (visited_tris[entry_tri_ind]) {
                continue;
            }
            visited_tris[entry_tri_ind] = true;
            tri_ind2component[entry_tri_ind] = component_ind;
            for (int k = 0; k < 3; ++k) {
                if (can_walk_through(entry_tri, k)) {
                    to_visit.push(entry_tri.neighbours[k]);
                }
            }
        }
        component_ind++;
    };

    std::queue<TriPathData> to_visit;
    //! we want to start from all crossroads
    int edge_ind = 0;
    int n_vertices = 0;
    for (TriInd tri_ind = 0; tri_ind < triangles.size(); ++tri_ind) {
        flood(tri_ind);
        const auto& tri = triangles[tri_ind];
        const auto n_free_edges = count_free_edges(tri);
        if (n_free_edges != 2 and triangle_is_within_boundary(tri)) { //! crossroads or dead end within boundaries
            for (int ind_in_tri = 0; ind_in_tri < 3; ++ind_in_tri) {
                if (can_walk_through(tri, ind_in_tri)) {
                    to_visit.push({tri_ind, ind_in_tri});
                }
            }
            tri_ind2vertex[tri_ind] = n_vertices;
            vertex2tri_ind.push_back(tri_ind);
            n_vertices += 1;
            //                }
        }
    }
    vertex2edge_inds2.resize(n_vertices);
    visited.resize(n_vertices, std::array<bool, 3>({false, false, false}));
    // std::fill(visited.begin(), visited.end(), std::array<bool,3>({false, false, false}));

    assert(cdt.triangulationIsConsistent());

    int tri_ind_corridor;
    int prev_tri_ind_corridor;
    int ind_in_tri_corridor;
    while (!to_visit.empty()) {
        const auto entry_tri_ind = to_visit.front().current;
        const auto entry_ind_in_tri = to_visit.front().to;
        to_visit.pop();
        const auto& entry_tri = triangles[entry_tri_ind];

        assert(tri_ind2vertex[entry_tri_ind] >= 0 and tri_ind2vertex[entry_tri_ind] < n_vertices);
        if (visited[tri_ind2vertex[entry_tri_ind]][entry_ind_in_tri]) { //! no visiting more than once and no buildings
            continue;
        }

        visited[tri_ind2vertex[entry_tri_ind]][entry_ind_in_tri] = true;
        tri_ind_corridor = entry_tri.neighbours[entry_ind_in_tri];
        prev_tri_ind_corridor = entry_tri_ind;
        ind_in_tri_corridor = entry_ind_in_tri;

        int n_free = 0;
        Corridor e;
        Funnel funnel;
        const auto left_start = asFloat(entry_tri.verts[next(entry_ind_in_tri)]);
        const auto right_start = asFloat(entry_tri.verts[entry_ind_in_tri]);
        const auto r_start = (right_start + left_start) / 2.f;
        funnel.push_back({right_start, left_start});

        e.start = {entry_tri_ind, entry_ind_in_tri};
        while (tri_ind_corridor !=
               entry_tri_ind) { //! we walk along the corridor until either end, dead-end or next crossroads
            const auto& tri = triangles[tri_ind_corridor];

            n_free = 0;
            for (int k = 0; k < 3; ++k) {
                if (!tri.is_constrained[k] and tri.neighbours[k] != prev_tri_ind_corridor) {
                    n_free++;
                    ind_in_tri_corridor = k;
                }
            }
            if (n_free == 2 or n_free == 0) {
                break;
            }
            e.tri_inds.push_back(tri_ind_corridor);
            e.from_start.push_back(ind_in_tri_corridor);
            tri_ind2vertex[tri_ind_corridor] = edge_ind;

            auto left_vertex_of_portal = asFloat(tri.verts[next(ind_in_tri_corridor)]);
            auto right_vertex_of_portal = asFloat(tri.verts[ind_in_tri_corridor]);

            prev_tri_ind_corridor = tri_ind_corridor;
            tri_ind_corridor = tri.neighbours[ind_in_tri_corridor];

            funnel.emplace_back(right_vertex_of_portal, left_vertex_of_portal);
            //                funnel_vertex_inds.emplace_back(right_index, left_index);
        }

        auto& end_tri_ind = tri_ind_corridor;
        auto& end_tri = triangles[end_tri_ind];
        int ind_in_tri_from_end = indInTriOf(end_tri, prev_tri_ind_corridor);
        e.end = {static_cast<TriInd>(end_tri_ind), ind_in_tri_from_end};

        if (e.tri_inds.size() > 0) {
            const auto& tri = triangles[e.tri_inds.front()];
            e.from_end.push_back(indInTriOf(tri, entry_tri_ind));
        }
        for (int i = 1; i < e.tri_inds.size(); i++) {
            const auto& tri = triangles.at(e.tri_inds[i]);
            e.from_end.push_back(indInTriOf(tri, e.tri_inds[i - 1]));
        }

        //            left_index = end_tri.vertinds[ind_in_tri_from_end];
        //            right_index = end_tri.vertinds[next(ind_in_tri_from_end)];
        const auto right_end = asFloat(end_tri.verts[next(ind_in_tri_from_end)]);
        const auto left_end = asFloat(end_tri.verts[ind_in_tri_from_end]);
        const auto r_end = (right_end + left_end) / 2.f;

        funnel.push_back({right_end, left_end});
        e.corridor_points = funnel;
        e.width = calcCorridorWidth(funnel);

        funnel.front() = {r_start, r_start};
        funnel.back() = {r_end, r_end};
        e.length = funnelDistance(r_start, r_end, funnel, cdt);

        vertex2edge_inds2[tri_ind2vertex[end_tri_ind]][ind_in_tri_from_end] = edge_ind;
        vertex2edge_inds2[tri_ind2vertex[entry_tri_ind]][entry_ind_in_tri] = edge_ind;
        visited[tri_ind2vertex[end_tri_ind]][ind_in_tri_from_end] = true;
        visited[tri_ind2vertex[entry_tri_ind]][entry_ind_in_tri] = true;

        //            cdt.renumber(end_tri, end_tri_ind, entry_tri, entry_tri_ind);

        edges.push_back(e);

        edge_ind++;
    }
}

//! \brief constructs reduced triangulation graph from full triangulation 
//! \brief also assings graph-component to each triangle ind and writes it to \p tri_ind2component  
//! \param cdt full triangulation object
//! \param tri_ind2component  
void ReducedTriangulationGraph::constructFromTriangulationCenters(Triangulation& cdt,
                                                                  std::vector<TriInd>& tri_ind2component) {

    const auto& triangles = cdt.triangles_;
    const auto& vertices = cdt.vertices_;
    tri_ind2vertex.resize(triangles.size());
    vertex2tri_ind.clear();
    reduced_vertices.clear();
    vertex2edge_inds2.clear();
    edges.clear();

    auto triangle_is_within_boundary = [&](const Triangle& tri) {
        const auto tri_center = cdt.calcTriangleCenter(tri);
        return cdt.boundary_.x >= tri_center.x and tri_center.x > 0 and cdt.boundary_.y >= tri_center.y and
               tri_center.y > 0;
    };

    auto count_free_edges = [](const Triangle& tri) {
        int n_free_edges = 0;
        for (int k = 0; k < 3; ++k) {
            if (!tri.is_constrained[k]) {
                n_free_edges += 1;
            }
        }
        return n_free_edges;
    };

    auto can_walk_through = [](const Triangle& tri, int ind_in_tri) {
        return !tri.is_constrained[ind_in_tri] and tri.neighbours[ind_in_tri] != -1;
    };

    tri_ind2component.resize(triangles.size());
    // std::cout << "there are " << std::to_string(triangles.size()) << " in triangulation\n";

    std::vector<bool> visited_tris(triangles.size(), false);
    int component_ind = 0;
    auto flood = [&](const TriInd tri_ind) {
        if (visited_tris[tri_ind]) {
            return;
        }
        std::queue<TriInd> to_visit({tri_ind});

        while (!to_visit.empty()) {
            const auto entry_tri_ind = to_visit.front();
            const auto& entry_tri = triangles[entry_tri_ind];
            to_visit.pop();
            if (visited_tris[entry_tri_ind]) {
                continue;
            }
            visited_tris[entry_tri_ind] = true;
            tri_ind2component[entry_tri_ind] = component_ind;
            for (int k = 0; k < 3; ++k) {
                if (can_walk_through(entry_tri, k)) {
                    to_visit.push(entry_tri.neighbours[k]);
                }
            }
        }
        component_ind++;
    };

    std::queue<TriPathData> to_visit;
    //! we want to start from all crossroads
    int edge_ind = 0;
    int n_vertices = 0;
    for (TriInd tri_ind = 0; tri_ind < triangles.size(); ++tri_ind) {
        flood(tri_ind);
        const auto& tri = triangles[tri_ind];
        const auto n_free_edges = count_free_edges(tri);
        if (n_free_edges != 2 and triangle_is_within_boundary(tri)) { //! crossroads or dead end within boundaries
            for (int ind_in_tri = 0; ind_in_tri < 3; ++ind_in_tri) {
                if (can_walk_through(tri, ind_in_tri)) {
                    to_visit.push({tri_ind, ind_in_tri});
                }
            }
            tri_ind2vertex[tri_ind] = n_vertices;
            vertex2tri_ind.push_back(tri_ind);
            n_vertices += 1;
        }
    }
    vertex2edge_inds2.resize(n_vertices);
    reduced_vertices.resize(n_vertices);
    visited.resize(n_vertices); //! not sure why I can't initialize in .resize  but it does not work :D
    std::fill(visited.begin(), visited.end(), std::array<bool, 3>({false, false, false}));

    assert(cdt.triangulationIsConsistent());

    int tri_ind_corridor;
    int prev_tri_ind_corridor;
    int ind_in_tri_corridor;
    while (!to_visit.empty()) {
        const auto entry_tri_ind = to_visit.front().current;
        const auto entry_ind_in_tri = to_visit.front().to;
        to_visit.pop();
        const auto& entry_tri = triangles[entry_tri_ind];

        assert(tri_ind2vertex[entry_tri_ind] >= 0 and tri_ind2vertex[entry_tri_ind] < n_vertices);
        if (visited[tri_ind2vertex[entry_tri_ind]][entry_ind_in_tri]) { //! no visiting more than once and no buildings
            continue;
        }

        visited[tri_ind2vertex[entry_tri_ind]][entry_ind_in_tri] = true;
        tri_ind_corridor = entry_tri.neighbours[entry_ind_in_tri];
        prev_tri_ind_corridor = entry_tri_ind;
        ind_in_tri_corridor = entry_ind_in_tri;

        int n_free = 0;
        Corridor e;
        Funnel funnel;
        const auto left_start = asFloat(entry_tri.verts[next(entry_ind_in_tri)]);
        const auto right_start = asFloat(entry_tri.verts[entry_ind_in_tri]);
        const auto r_start = cdt.calcTriangleCenter(entry_tri);
        funnel.push_back({right_start, left_start});

        e.start = {entry_tri_ind, entry_ind_in_tri};
        while (tri_ind_corridor !=
               entry_tri_ind) { //! we walk along the corridor until either end, dead-end or next crossroads
            const auto& tri = triangles[tri_ind_corridor];

            n_free = 0;
            for (int k = 0; k < 3; ++k) {
                if (!tri.is_constrained[k] and tri.neighbours[k] != prev_tri_ind_corridor) {
                    n_free++;
                    ind_in_tri_corridor = k;
                }
            }
            if (n_free == 2 or n_free == 0) {
                break;
            }
            e.tri_inds.push_back(tri_ind_corridor);
            e.from_start.push_back(ind_in_tri_corridor);
            tri_ind2vertex[tri_ind_corridor] = edge_ind;

            auto left_vertex_of_portal = asFloat(tri.verts[next(ind_in_tri_corridor)]);
            auto right_vertex_of_portal = asFloat(tri.verts[ind_in_tri_corridor]);

            prev_tri_ind_corridor = tri_ind_corridor;
            tri_ind_corridor = tri.neighbours[ind_in_tri_corridor];

            funnel.emplace_back(right_vertex_of_portal, left_vertex_of_portal);
        }

        auto& end_tri_ind = tri_ind_corridor;
        auto& end_tri = triangles[end_tri_ind];
        int ind_in_tri_from_end = indInTriOf(end_tri, prev_tri_ind_corridor);
        e.end = {static_cast<TriInd>(end_tri_ind), ind_in_tri_from_end};

        if (e.tri_inds.size() > 0) {
            const auto& tri = triangles[e.tri_inds.front()];
            e.from_end.push_back(indInTriOf(tri, entry_tri_ind));
        }
        for (int i = 1; i < e.tri_inds.size(); i++) {
            const auto& tri = triangles.at(e.tri_inds[i]);
            e.from_end.push_back(indInTriOf(tri, e.tri_inds[i - 1]));
        }

        const auto right_end = asFloat(end_tri.verts[next(ind_in_tri_from_end)]);
        const auto left_end = asFloat(end_tri.verts[ind_in_tri_from_end]);
        const auto r_end = cdt.calcTriangleCenter(end_tri);

        funnel.push_back({right_end, left_end});
        //            funnel_vertex_inds.push_back({-1, -1});
        e.width = calcCorridorWidth(funnel);
        funnel.front() = {r_start, r_start};
        funnel.back() = {r_end, r_end};

        e.length = funnelDistance(r_start, r_end, funnel, cdt);

        reduced_vertices[tri_ind2vertex[end_tri_ind]].neighbours[ind_in_tri_from_end] = tri_ind2vertex[entry_tri_ind];
        reduced_vertices[tri_ind2vertex[end_tri_ind]].widths[ind_in_tri_from_end] = e.width;
        reduced_vertices[tri_ind2vertex[end_tri_ind]].lengths[ind_in_tri_from_end] = e.length;

        reduced_vertices[tri_ind2vertex[entry_tri_ind]].neighbours[entry_ind_in_tri] = tri_ind2vertex[end_tri_ind];
        reduced_vertices[tri_ind2vertex[entry_tri_ind]].widths[entry_ind_in_tri] = e.width;
        reduced_vertices[tri_ind2vertex[entry_tri_ind]].lengths[entry_ind_in_tri] = e.length;

        vertex2edge_inds2[tri_ind2vertex[end_tri_ind]][ind_in_tri_from_end] = edge_ind;
        vertex2edge_inds2[tri_ind2vertex[entry_tri_ind]][entry_ind_in_tri] = edge_ind;
        visited[tri_ind2vertex[end_tri_ind]][ind_in_tri_from_end] = true;
        visited[tri_ind2vertex[entry_tri_ind]][entry_ind_in_tri] = true;

        //            cdt.renumber(end_tri, end_tri_ind, entry_tri, entry_tri_ind);

        edges.push_back(e);

        edge_ind++;
    }
    
}