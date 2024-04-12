#pragma once

#include "../ECS.h"
#include "../CommandGroupManager.h"
#include "../Settings.h"

#include "../Utils/NeighbourSearcherContext.h"

class PathFinder2;
class Triangulation;

//! \brief 
struct SeekSystem : System2
{

    SeekSystemSettings settings_;

    typedef ComponentArray<PathFinderComponent> CompArray;

    const AcosTable<1000> s_angle_calculator;

    CommandGroupManager2 commander_groups_;

    std::array<SharedData, N_MAX_ENTITIES> *master_data_;

    PathFinder2 *p_pathfinder_;
    Triangulation* p_cdt_;

    // std::unique_ptr<NeighbourSearcherT<PathFinderComponent>> p_ns_;
    std::unique_ptr<NeighbourSearcherContext<PathFinderComponent, int>> p_ns_;

    struct PathData{
        int compvec_ind = -1;
        sf::Vector2f r_end;
        int command_group_ind = -1;
        int start_tri_ind = -1;
        int end_tri_ind = -1;
    };
    std::queue<PathData> components2update_;

    std::queue<int> comp_inds_to_update_;
public:
    SeekSystem(ComponentID id);
    virtual ~SeekSystem() = default;

    void issuePaths(const std::vector<int> &entity_inds, sf::Vector2f path_end);
    void issuePaths2(const std::vector<int> &entity_inds, sf::Vector2f path_end);

    virtual void onComponentCreation(GraphicsComponent& comp){}

    virtual void update() override;

    // virtual void draw(sf::RenderTarget &target) override;

    virtual void updateSharedData(const std::array<SharedData, N_MAX_ENTITIES> &new_data,
                          const std::vector<Entity> &active_entity_inds) override ;

    virtual void communicate(std::array<SharedData, N_MAX_ENTITIES> &entity2shared_data) const override;

    void onComponentRemoval(int comp_ind);

private:
    //! \brief stops moving towards seek target and removes agent from his command group
    //! \param selected
    //! \param r        position of the selected agent
    void stopSeeking(PathFinderComponent &comp);

    int compvecInd(const PathFinderComponent &comp) const;

    //! \brief one possible stopping condition
    bool neighboursInFrontReachedEnd(const PathFinderComponent &comp, float radius, sf::Vector2f dr_to_target);

    bool needsUpdating(PathFinderComponent &comp);

    void finalizeSeek(PathFinderComponent &comp, sf::Vector2f dr_to_target);

    void updateTarget(PathFinderComponent &comp, const sf::Vector2f r);

    void turnTowards(PathFinderComponent &comp, sf::Vector2f direction);

    void repairAngleBounds(PathFinderComponent& comp);
};
