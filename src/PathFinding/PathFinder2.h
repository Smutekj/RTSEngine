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

struct GroupPathManager
{

    struct GroupPathData
    {
        int start = -1;                              //! start triangle ind
        int end = -1;                                //! end triangle ind
        sf::Vector2f r_end = {0, 0};                 //! end position
        float radius = RHARD;                        //! maximum radius of the group
        std::unordered_set<int> inds_to_update = {}; //! indices of the agents
        std::vector<sf::Vector2f> r_starts = {};     //! positions of the agents

        GroupPathData(TriInd start, TriInd end, const std::vector<PathFinderComponent> &comps, sf::Vector2f r_end,
                      float radius, const std::vector<int> &inds)
            : start(start), end(end), r_end(r_end), radius(radius), inds_to_update(inds.begin(), inds.end())
        {
            for (const auto ind : inds)
            {
                r_starts.push_back(comps.at(ind).transform.r);
            }
        }

        GroupPathData() = default;
    };

    GayVector<std::pair<int, int>, N_MAX_NAVIGABLE_BOIDS> group_path_data_inds;
    std::vector<GroupPathData> group_path_data;

    void addToGroup(int group_data_ind, int entity_ind, sf::Vector2f r_start)
    {
        const auto n_entities_in_group = group_path_data.at(group_data_ind).inds_to_update.size();
        group_path_data_inds.insert({group_data_ind, n_entities_in_group}, entity_ind);

        group_path_data.at(group_data_ind).inds_to_update.insert(entity_ind);
        group_path_data.at(group_data_ind).r_starts.push_back(r_start);
    }

    void removeEntityFromGroup(int entity_ind)
    {
        auto [group_data_ind, ind_in_group] = group_path_data_inds.data.at(group_path_data_inds.entity2ind_in_vec.at(entity_ind));
        group_path_data.at(group_data_ind).inds_to_update.erase(entity_ind);
    }
};

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

    std::unordered_map<TriInd, std::vector<std::pair<int, sf::Vector2f>>> start_tri2indices_and_rstarts_; //! each triangle ind holds vector of agent indices starting from TriInd
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

        GroupPathData(TriInd start, TriInd end,  sf::Vector2f r_end,
                      float radius,  const std::vector<std::pair<int, sf::Vector2f>> &inds_and_rstarts)
            : start(start), end(end), r_end(r_end), radius(radius)
        {
            for (const auto [ind, r_start] : inds_and_rstarts)
            {
                inds_to_update.push_back(ind);
                r_starts.push_back(r_start);
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
    std::vector<std::future<int>> futures_;    //! holds threads ing pathfinding work, return value used to see if the job has ended
    std::vector<THREAD_STATUS> thread_status_; //!

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
    std::array<int, N_MAX_NAVIGABLE_BOIDS> entity2update_group_ind_;
    std::vector<GroupPathData> to_update_groups_;

    std::vector<TriInd> tri_ind2component_; //! This will be used for identifying buldings

    std::array<PathAndPortals, N_MAX_THREADS> pap;
    std::array<Funnel, N_MAX_THREADS> funnels;

    explicit PathFinder2(Triangulation *cdt);

    //! \brief updates graph data structures so they correspond to triangulation
    void update();

    void findSubOptimalPathCenters(const sf::Vector2f r_start, const sf::Vector2f r_end, float radius, Funnel &funnel);
    void findSubOptimalPathCenters(sf::Vector2f r_start, sf::Vector2f r_end, float radius, Funnel &funnel, const int thread_id);
    void findSubOptimalPathCentersWater(sf::Vector2f r_start, sf::Vector2f r_end, float radius, Funnel &funnel, int unit_type, const int thread_id);
    void findSubOptimalPathCenters2(sf::Vector2f r_start, sf::Vector2f r_end, float radius, Funnel &funnel, const int thread_id);
    void issuePaths(std::vector<PathFinderComponent> &comps, const std::vector<int> &selection, const sf::Vector2f r_end);
    void issuePaths2(std::vector<PathFinderComponent> &comps,
                     const std::vector<int> &selected_entity_inds,
                     const std::array<int, N_MAX_ENTITIES> &entity2compvec_ind,
                     const sf::Vector2f r_end);

    void updatePaths(std::vector<PathFinderComponent> &comps, std::array<int, N_MAX_ENTITIES> &entity2compvec_ind,
                     const int available_time);
    void updatePaths2(std::vector<PathFinderComponent> &comps, const int available_time);

    void onUnitRemoval(Entity ent)
    {
        auto update_group_ind = entity2update_group_ind_.at(ent.ind);
        if(update_group_ind == -1){return;}
        auto& entities_in_group= to_update_groups_.at(update_group_ind).inds_to_update;
        auto entity_in_group_it = std::find(entities_in_group.begin(), entities_in_group.end(), ent.ind);
        assert(entity_in_group_it != entities_in_group.end());
        entities_in_group.erase(entity_in_group_it);
        if(entities_in_group.size() == 0){
            to_update_groups_.erase(to_update_groups_.begin() + update_group_ind);
        }

        entity2update_group_ind_.at(ent.ind) = -1;
    }

private:
    void doPathFinding(const sf::Vector2f r_end, std::vector<PathFinderComponent> &comps,
                       std::array<int, N_MAX_ENTITIES> &entity2compvec_ind,
                       const float max_radius_of_agent, const std::vector<int> agent_indices, const int thread_id);

    void doPathFinding2(const std::vector<sf::Vector2f> r_coords, const sf::Vector2f r_end, std::vector<PathFinderComponent> &comps,
                        const float max_radius_of_agent, const std::vector<int> entities, const int thread_id);

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
