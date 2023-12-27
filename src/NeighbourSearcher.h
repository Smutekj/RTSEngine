#pragma once

#include "core.h"
#include "BoidWorld.h"


class SearchGrid;
class NeighbourSearcher {

    struct NeighbourData {
        sf::Vector2f dr;
        float r_collision;
        BoidInd second;
    };

    struct InteractionData {
        sf::Vector2f dr;
        sf::Vector2f dv;
        float r_collision;
        float mass;
        int player_ind;
        BoidInd second;
        MoveState state;
    };

    std::vector<std::array<NeighbourData, N_MAX_NEIGHBOURS>> particle2neighbour_data_;
    std::vector<std::array<InteractionData, N_MAX_NEIGHBOURS>> particle2ninteraction_data_;
    std::shared_ptr<SearchGrid> s_grid_;
    BoidWorld* boids_;

    struct CellData {
        sf::Vector2f r;
        float radius;
        BoidInd ind;
    };

    struct CellData2 {
        sf::Vector2f r;
        sf::Vector2f v;
        float mass;
        float radius;
        int player_ind;
        MoveState state;
        BoidInd ind;
    };

    static constexpr size_t N_MAX_AGENTS_IN_CELL = 500;
    ;
    std::vector<std::array<int, N_MAX_AGENTS_IN_CELL + 1>> cell2boid_inds_;
    std::vector<std::array<CellData, N_MAX_AGENTS_IN_CELL + 1>> cell2data_;
    std::vector<std::array<CellData2, N_MAX_AGENTS_IN_CELL + 1>> cell2data2_;

    std::vector<int> active_cells_;
    std::vector<bool> cell_visited_;

  public:
    std::vector<int> last_i;
    
    NeighbourSearcher(sf::Vector2f box_size, float max_dist, BoidWorld& world);

    std::vector<int> getNeighboursInds(sf::Vector2f coords, const float r_max2);
    std::vector<int> getNeighboursIndsFull(sf::Vector2f r, const float r_max);

    const std::array<InteractionData, N_MAX_NEIGHBOURS>& getInteractionData(const int boid_ind,
                                                                                             const float r_max2) const;
    std::vector<int>& getNeighbourInds(int boid_ind, const float r_max2);
    
    const std::array<NeighbourData, N_MAX_NEIGHBOURS>& getNeighbourData(const int boid_ind,
                                                                            const float r_max2) const;
    void addOnGrid(const std::vector<sf::Vector2f>& r_coords, const std::vector<float>& radii);
    void addOnGrid(const std::vector<sf::Vector2f>& r_coords, const std::vector<float>& radii,
                   const std::vector<int>& selection);
    void addOnGrid(const std::vector<sf::Vector2f>& r_coords,
                                    const std::vector<sf::Vector2f>& vels, 
                                    const std::vector<float>& masses,
                                    const std::vector<float>& radii,
                                    const std::vector<int>& active_inds);

    void update(const std::vector<sf::Vector2f>& r_coords, const std::vector<int>& active_inds, const std::vector<float>& radii);

    void fillNeighbourData( const std::vector<sf::Vector2f>& r_coords, const std::vector<float>& radii,
                             const std::vector<int>& active_inds, const float r_max2);
    void fillNeighbourData2(const std::vector<sf::Vector2f>& r_coords, const std::vector<float>& radii,
                                             const std::vector<int>& active_inds, const float r_max2);
private:

};




























