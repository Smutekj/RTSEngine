#ifndef BOIDS_PATHFINDER_H
#define BOIDS_PATHFINDER_H

#include <future>
#include <queue>
#include <thread>
#include "core.h"
#include "ReducedTriangulationGraph.h"
#include "Edges.h"
#include <unordered_map>

class Triangulation;
class BoidControler;
class Triangle;

//! \struct Contains data and methods for constructing and maintaining FunnelFans,
//! \struct  which are needed in the Optimal Astar
struct FunnelFan {
    sf::Vector2f apex;
    float dist_apex_to_start = 0;

    std::deque<sf::Vector2f> left_points;
    std::deque<float> left_min_dists; //! dists from apex to a corresponding left_point
    std::deque<sf::Vector2f> right_points;
    std::deque<float> right_min_dists; //! dists from apex to a corresponding right_point

    FunnelFan() = default;

    FunnelFan(const sf::Vector2f& apex, const sf::Vector2f& left_point, const sf::Vector2f& right_point);

    FunnelFan(const sf::Vector2f& apex);

    float calcDistanceToEndPoint(const sf::Vector2f end_point) const;

    void addEdge(const sf::Vector2f& new_anticlock_point, const sf::Vector2f& new_clock_point);
    void addLeftPoint(const sf::Vector2f new_point);
    void addRightPoint(const sf::Vector2f new_point);

    bool isClockwise(const sf::Vector2f query_point, Edgef& line) const;
    bool isInFan(const sf::Vector2f query_point, Edgef& line1, Edgef& line2) const;
};

//! \class contains data and methods for pathfinding on a constrianed Delaunay triangulation
//! TODO: Separate into interface and different implementions of the actual finding of the path
//! TODO: Maybe I could use a Policy pattern?
class PathFinder {

    //! \struct holds data needed by prority_queue in Astar
    struct AstarDataPQ {
        TriInd next;   //! nexttriangle to visit
        float f_value; //! sum of heuristic (h_value) and g_value
    };

    typedef int ReducedVertexInd;
    //! \struct contains together all the properties needed during a single Astar iteration
    struct AstarReducedData {
        float g_value = 0;
        float h_value = 0;
        ReducedVertexInd prev; //! back-pointer to a reduced vertex used to backtrack path
        int ind_in_tri;
    };

    //! data needed by priority_queue in Astar on reduced graph
    struct AstarReducedDataPQ {
        float f_value;
        ReducedVertexInd next; //! next reduced vertex to visit in Astar
    };

    Triangulation* cdt_; //! undrelying triangulation
    std::vector<int> shortest_path;
    std::vector<std::pair<TriInd, int>> tri_ind2shortest_path_;
    std::vector<float> vertex2shortest_distance;
    std::vector<float> vertex2h_function_;

    std::vector<AstarReducedData>
        reduced_vertex2astar_data_; //! has size n_reduced_vertices + 2 (1 for start_tri_ind 1 for end...)

    std::vector<std::array<float, 3>> tri_edge2astar_data_;
    std::vector<std::array<float, 3>> tri_edge2g_function_;
    std::vector<std::array<float, 3>> tri_edge2h_function_;
    //    std::vector<std::array<AstarVertexData, 3>> tri_edge2astar_data_;
    std::vector<FunnelFan> tri_edge2portal_;
    std::vector<std::array<std::pair<TriInd, int>, 3>> tri_edge2shortest_path_;

    std::vector<std::pair<int, int>> tri_vertex2constrained_edges;

    std::shared_ptr<ReducedTriangulationGraph> p_rtg_;      //! pointer to reduced triangulation data
    std::vector<int> component2building_ind_;               //! this should probably not be here...

    std::array<std::vector<AstarReducedData>, N_MAX_THREADS> thread2reduced_vertex2astar_data_; //! each thread 

    std::unordered_map<TriInd, std::vector<int>> start_tri2indices_;    //! each triangle ind holds vector of agent indices starting from TriInd
                                                                        //! used when distributing path finding work
    std::vector<TriInd> start_tri_inds_;


    //! \struct stores data neede by pathfinding work (all particles that have the same start and end triangles)
    struct GroupPathData {
        int start = -1;             //! start triangle ind
        int end = -1;               //! end triangle ind
        sf::Vector2f r_end = {0, 0};    //! end position
        float radius = RHARD;           //! maximum radius of the group
        std::vector<int> inds_to_update = {};   //! indices of the agents
        std::vector<sf::Vector2f> r_starts = {};    //! positions of the agents

        GroupPathData(TriInd start, TriInd end, const std::vector<sf::Vector2f>& r_coords, sf::Vector2f r_end,
                      float radius, const std::vector<int>& inds)
            : start(start)
            , end(end)
            , r_end(r_end)
            , radius(radius)
            , inds_to_update(inds) {
            for (const auto ind : inds) {
                r_starts.push_back(r_coords[ind]);
            }
        }
        GroupPathData() = default;
    };


    struct PathData {
        TriInd start;
        TriInd end;
        sf::Vector2f r_start;
        sf::Vector2f r_end;
        float dist_until_update_needed = 0;
    };


    std::vector<std::thread> threads_;
    std::vector<std::future<int>> futures_;     //! holds threads doing pathfinding work, return value used to see if the job has ended

    std::vector<GroupPathData> to_update_groups_;
    std::vector<PathData> to_update_;
    std::vector<bool> update_has_been_issued_;

    //! \struct contains information about width of each triangle sides
    struct TriangleWidth {
        float widths[3];
        TriangleWidth(Triangle& tri, const Triangulation& cdt);
        TriangleWidth() {
            widths[0] = MAXFLOAT;
            widths[1] = MAXFLOAT;
            widths[2] = MAXFLOAT;
        }
    };
    std::vector<TriangleWidth> triangle2tri_widths_;

  public:
    std::vector<TriInd> tri_ind2component_;

  public:
    void updatePaths(const std::vector<sf::Vector2f>& r_coords, BoidControler& bc, const sf::Int64 available_time);

    explicit PathFinder(Triangulation* cdt);

    //! \brief updates graph data structures so they correspond to triangulation
    void update();

    struct PathAndPortals {
        std::vector<sf::Vector2f> path;
        std::vector<Edgef> portals;
    };

    PathAndPortals calcPathOfSelection(BoidControler& bc, const std::vector<sf::Vector2f>& r_coords,
                                       const std::vector<float>& radii, const std::vector<int>& selection,
                                       const sf::Vector2f);
    void updatePathOf(const int ind, sf::Vector2f r_start, BoidControler& bc, float radius);

    FunnelData findOptimalPath(const sf::Vector2f r_start, const sf::Vector2f r_end, float radius);
    // FunnelData findSubOptimalPathTriangleEdges(const sf::Vector2f& r_start, const sf::Vector2f r_end, float radius);
    void findSubOptimalPathCenters(const sf::Vector2f r_start, const sf::Vector2f r_end, float radius,
                                   FunnelData& funnel);
    void findSubOptimalPathCenters(sf::Vector2f r_start, sf::Vector2f r_end, float radius, FunnelData& funnel,
                                   const int thread_id);

    void doPathFinding(const std::vector<sf::Vector2f> r_coords, const sf::Vector2f r_end, BoidControler& bc,
                 const float max_radius_of_agent, const std::vector<int> agent_indices, const int thread_id);

    FunnelData findSubOptimalPathBasic(const TriInd start_tri_ind, const TriInd end_tri_ind, const sf::Vector2f r_end,
                                       float radius);

    FunnelData findShortestPath(const sf::Vector2f r_start, const sf::Vector2f r_end, float radius);
    void issuePaths(BoidControler& bc, const std::vector<sf::Vector2f>& r_coords, const std::vector<float>& radii,
                    const std::vector<int>& selection, const sf::Vector2f r_end);

  private:
    //! \brief
    float sign(sf::Vector2f a, sf::Vector2f b, sf::Vector2f c) const;

    void fillFunnelWithCorridor(const ReducedTriangulationGraph::Corridor& edge, int first, int last, FunnelData& fd,
                                bool reverse, bool with_start_or_end) const;

    std::pair<TriInd, int> closestPointOnNavigableComponent(const sf::Vector2f& r, const TriInd start_tri_ind,
                                                            const int end_component,
                                                            const int navigable_component) const;

    void fillFunnelData3(const TriInd start, const int reduced_start, const TriInd end, const int reduced_end,
                         FunnelData& funnel, const std::vector<AstarReducedData>& reduced_vertex2astar_data) const;

    void fillFunnelData2(const TriInd start, const int reduced_start, const TriInd end, const int reduced_end,
                         FunnelData& funnel) const;

    void fillFunnelData(const TriInd start, const TriInd end, FunnelData& fd) const;

    PathAndPortals pathFromFunnel(const sf::Vector2f r_start, const sf::Vector2f r_end, const float radius,
                                  FunnelData& fd) const;

    std::vector<PathFinder::AstarReducedDataPQ>
    initializeReducedAstar(const sf::Vector2f& r_start, const sf::Vector2f& r_end, const TriInd start,
                           const ReducedVertexInd reduced_start, const TriInd end, float& current_shortest_distance,
                           const float radius, std::vector<AstarReducedData>& reduced_vertex2astar_data) const;


    struct AstarVertexData {
        float g_value = 0;
        float h_value = 0;
    };

    AstarVertexData calcAstarData(const FunnelFan& portal, const sf::Vector2f r_end);

    float rayCast(const sf::Vector2f& from, const sf::Vector2f& to) const;
};

#endif // BOIDS_PATHFINDER_H
