#pragma once

#include <future>
#include <queue>
#include <thread>
#include <unordered_map>

#include "ReducedTriangulationGraph.h"
#include "../ECS.h"
#include "../Edges.h"
#include "../core.h"


class Triangulation;

//! \class contains data and methods for pathfinding on a constrianed Delaunay triangulation
class PathFinder2
{
        struct PathAndPortals
    {
        std::deque<sf::Vector2f> path;
        std::deque<Edgef> portals;
    };

    //! \struct holds data needed by prority_queue in Astar
    struct AstarDataPQ
    {
        TriInd next;   //! nexttriangle to visit
        float f_value; //! sum of heuristic (h_value) and g_value
    };

    typedef int ReducedVertexInd;
    //! \struct contains together all the properties needed during a single Astar iteration
    struct AstarReducedData
    {
        float g_value = 0;
        float h_value = 0;
        ReducedVertexInd prev; //! back-pointer to a reduced vertex used to backtrack path
        int ind_in_tri;
    };

    //! data needed by priority_queue in Astar on reduced graph
    struct AstarReducedDataPQ
    {
        float f_value;
        ReducedVertexInd next; //! next reduced vertex to visit in Astar
    };

    enum class THREAD_STATUS
    {
        RUNNING,
        FREE,
        FAILED
    };

    Triangulation *cdt_; //! underlying triangulation

    std::shared_ptr<ReducedTriangulationGraph> p_rtg_; //! pointer to reduced triangulation data
    std::vector<int> component2building_ind_;          //! this should probably not be here...

    std::vector<AstarReducedData> reduced_vertex2astar_data_;                                   //! has size n_reduced_vertices + 2 (1 for start_tri_ind 1 for end...)
    std::array<std::vector<AstarReducedData>, N_MAX_THREADS> thread2reduced_vertex2astar_data_; //!

    std::unordered_map<TriInd, std::vector<int>> start_tri2indices_; //! each triangle ind holds vector of agent indices starting from TriInd
                                                                     //! used when distributing path finding work
    std::vector<TriInd> start_tri_inds_;

    //! \struct stores data neede by pathfinding work (all particles that have the same start and end triangles)
    struct GroupPathData
    {
        int start = -1;                          //! start triangle ind
        int end = -1;                            //! end triangle ind
        sf::Vector2f r_end = {0, 0};             //! end position
        float radius = RHARD;                    //! maximum radius of the group
        std::vector<int> inds_to_update = {};    //! indices of the agents
        std::vector<sf::Vector2f> r_starts = {}; //! positions of the agents

        GroupPathData(TriInd start, TriInd end, const std::vector<PathFinderComponent> &comps, sf::Vector2f r_end,
                      float radius, const std::vector<int> &inds)
            : start(start), end(end), r_end(r_end), radius(radius), inds_to_update(inds)
        {
            for (const auto ind : inds)
            {
                r_starts.push_back(comps.at(ind).transform.r);
            }
        }
        GroupPathData() = default;
    };

    //! \struct holding data of  
    struct PathData
    {
        TriInd start;
        TriInd end;
        sf::Vector2f r_start;
        sf::Vector2f r_end;
        float dist_until_update_needed = 0;
    };

    std::vector<std::thread> threads_;
    std::vector<std::future<int>> futures_;    //! holds threads doing pathfinding work, return value used to see if the job has ended
    std::vector<THREAD_STATUS> thread_status_; //! holds threads doing pathfinding work, return value used to see if the job has ended

    std::vector<PathData> to_update_;
    std::vector<bool> update_has_been_issued_;
    

    //! \struct contains information about width of each triangle sides
    struct TriangleWidth
    {
        float widths[3];
        TriangleWidth(Triangle &tri, const Triangulation &cdt);
        TriangleWidth()
        {
            widths[0] = MAXFLOAT;
            widths[1] = MAXFLOAT;
            widths[2] = MAXFLOAT;
        }
    };
    std::vector<TriangleWidth> triangle2tri_widths_;

public:
    std::vector<GroupPathData> to_update_groups_;

    std::vector<TriInd> tri_ind2component_; //! This will be used for identifying buldings

    std::array<PathAndPortals, N_MAX_THREADS> pap;
    std::array<Funnel, N_MAX_THREADS> funnels;

    explicit PathFinder2(Triangulation *cdt);

    //! \brief updates graph data structures so they correspond to triangulation
    void update();

    void findSubOptimalPathCenters(const sf::Vector2f r_start, const sf::Vector2f r_end, float radius, Funnel &funnel);
    void findSubOptimalPathCenters(sf::Vector2f r_start, sf::Vector2f r_end, float radius, Funnel &funnel, const int thread_id);
    void findSubOptimalPathCenters2(sf::Vector2f r_start, sf::Vector2f r_end, float radius, Funnel &funnel, const int thread_id);
    void issuePaths(std::vector<PathFinderComponent> &comps, const std::vector<int> &selection, const sf::Vector2f r_end);
    void issuePaths2(   std::vector<PathFinderComponent> &comps,
                        const std::vector<int> &selected_entity_inds,
                        const std::array<int, N_MAX_ENTITIES>& entity2compvec_ind,
                        const sf::Vector2f r_end);
    
    void updatePaths(std::vector<PathFinderComponent> &comps, const sf::Int64 available_time);
    void updatePaths2(std::vector<PathFinderComponent> &comps, const sf::Int64 available_time);

private:
    void doPathFinding(const std::vector<sf::Vector2f> r_coords, const sf::Vector2f r_end, std::vector<PathFinderComponent> &comps,
                       const float max_radius_of_agent, const std::vector<int> agent_indices, const int thread_id);

    void setPathOfAgent(PathFinderComponent &comp, sf::Vector2f r_end, PathAndPortals &path_and_portals) const;
    bool allThreadsFree(int last_thread_id);
    bool reducedGraphConsistentWithTriangulation() const;

    void fillFunnelWithCorridor(const ReducedTriangulationGraph::Corridor &edge, int first, int last, Funnel &fd,
                                bool reverse, bool with_start_or_end) const;
    void fillFunnelWithCorridorReversed(const ReducedTriangulationGraph::Corridor &edge, int first, int last,
                                        Funnel &funnel, bool with_start_or_end) const;
    void fillFunnelWithCorridorFront(const ReducedTriangulationGraph::Corridor &edge, int first, int last,
                                     Funnel &funnel, bool with_start_or_end) const;

    std::pair<TriInd, int> closestPointOnNavigableComponent(const sf::Vector2f &r, const TriInd start_tri_ind,
                                                            const int end_component,
                                                            const int navigable_component) const;

    void fillFunnelData(const TriInd start, const int reduced_start, const TriInd end, const int reduced_end,
                        Funnel &funnel, const std::vector<AstarReducedData> &reduced_vertex2astar_data) const;

    PathAndPortals pathFromFunnel(const sf::Vector2f r_start, const sf::Vector2f r_end, const float radius,
                                  Funnel &fd) const;
    PathAndPortals pathFromFunnel2(const sf::Vector2f r_start, const sf::Vector2f r_end, const float radius,
                                  Funnel &fd) const;

    std::vector<PathFinder2::AstarReducedDataPQ>
    initializeReducedAstar(const sf::Vector2f &r_start, const sf::Vector2f &r_end, const TriInd start,
                           const ReducedVertexInd reduced_start, const TriInd end, float &current_shortest_distance,
                           const float radius, std::vector<AstarReducedData> &reduced_vertex2astar_data) const;
    float sign(sf::Vector2f a, sf::Vector2f b, sf::Vector2f c) const;

public:
    float rayCast(const sf::Vector2f &from, const sf::Vector2f &to) const;
};
