#pragma once

#include "core.h"
#include "BoidWorld.h"


class SearchGrid;
class NeighbourSearcher {

    struct ParticlePairData {
        sf::Vector2f dr = {MAXFLOAT, MAXFLOAT};
        u_int16_t first = -1;
        u_int16_t second = -1;
        float r_collision = 0;

        ParticlePairData(){}

        ParticlePairData(sf::Vector2f dr, int first, int second, float r_collision)
            : dr(dr)
            , first(first)
            , second(second)
            , r_collision(r_collision){};

        ParticlePairData(const ParticlePairData& old_data)    
            : dr(old_data.dr)
            , first(old_data.first)
            , second(old_data.second)
            , r_collision(old_data.r_collision){}

        ParticlePairData& operator=(const ParticlePairData& old_data) 
        {
            dr = old_data.dr;
            first = old_data.first;
            second = old_data.second;
            r_collision = old_data.r_collision;
            return *this;
        }
    };


    struct ParticlePairData2 {
        std::vector<sf::Vector2f> drs;
        std::vector<std::pair<u_int16_t, u_int16_t>>  inds;
        std::vector<float> r_collisions ;
    };


    struct NeighbourData {
        sf::Vector2f dr;
        float r_collision;
        BoidInd second;
    };

    struct NeighboursData {
        std::vector<sf::Vector2f> drs;
        float r_collision;
        BoidInd second;
    };

    static constexpr int NGRIDX = 1000;
    std::vector<std::array<float, NGRIDX>> ygrid2x_positions;

    std::vector<ParticlePairData> pair_list_;
    std::array<std::vector<ParticlePairData>, 12> pair_lists_;
    std::array<ParticlePairData2, 12> pair_lists2_;


    std::vector<std::array<NeighbourData, N_MAX_NEIGHBOURS>> particle2neighbour_data_;
    std::vector<std::array<NeighbourData, N_MAX_NEIGHBOURS>> particle2neighbour_data2_;

    std::vector<std::vector<int>> particle2verlet_list;

    std::shared_ptr<SearchGrid> s_grid_;
    BoidWorld* boids_;
    // std::unordered_map<int, std::vector<int>> cell2boid_inds_; //! I should implement a quad tree

    struct CellData {
        sf::Vector2f r;
        float radius;
        BoidInd ind;
    };

    static constexpr size_t N_MAX_AGENTS_IN_CELL = 500;
    ;
    std::vector<std::array<int, N_MAX_AGENTS_IN_CELL + 1>> cell2boid_inds_;
    std::vector<std::array<CellData, N_MAX_AGENTS_IN_CELL + 1>> cell2boid_inds2_;

    std::vector<int> active_cells_;

    std::vector<bool> cell_visited_;

  public:
    std::vector<int> last_i;
    
    const std::vector<ParticlePairData>& getPairList() const { return pair_list_; }
    const std::vector<ParticlePairData>& getPairList(int thread_id) const { return pair_lists_[thread_id]; }

    NeighbourSearcher(sf::Vector2f box_size, float max_dist, BoidWorld& world);

    std::vector<int> getNeighboursInds(sf::Vector2f coords, const float r_max2);
    std::vector<int> getNeighboursIndsFull(sf::Vector2f r, const float r_max);

    std::vector<int>& getNeighbourInds(int boid_ind, const float r_max2);
    
    const std::array<NeighbourData, N_MAX_NEIGHBOURS>& getNeighbourData(const int boid_ind,
                                                                            const float r_max2) const;
    void addOnGrid(const std::vector<sf::Vector2f>& r_coords, const std::vector<float>& radii);
    void addOnGrid(const std::vector<sf::Vector2f>& r_coords, const std::vector<float>& radii,
                   const std::vector<int>& selection);
    void update(const std::vector<sf::Vector2f>& r_coords, const std::vector<float>& radii);
    void fillVerletLists(const std::vector<sf::Vector2f>& r_coords, const std::vector<float>& radii,
                         const float r_max2);
    void fillNeighbourData(const std::vector<sf::Vector2f>& r_coords, const std::vector<float>& radii,
                           const float r_max2);
    void fillNeighbourDataSIMD(const std::vector<sf::Vector2f>& r_coords, const std::vector<float>& radii,
                           const float r_max2);
    void fillNeighbourData3(const std::vector<sf::Vector2f>& r_coords, const std::vector<float>& radii,
                           const float r_max2);
    void fillNeighbourData2(const std::vector<sf::Vector2f>& r_coords, const std::vector<float>& radii,
                            const float r_max2);
    void fillNeighbourDataBalanced(const std::vector<sf::Vector2f>& r_coords, const std::vector<float>& radii,
                    const float r_max2) ;

    void fillPairListFromCells(ParticlePairData* __restrict__ pair_list, const int last_in_cell_i,
                        const int last_in_cell_j, const CellData* __restrict__ cell_i, const CellData* __restrict__ cell_j,
                         const float r_max2, const int last_ind);

    void fillPairListFromCells2(std::pair<sf::Vector2f, u_int64_t>* __restrict__ pair_list, const int last_in_cell_i,
                        const int last_in_cell_j,  const CellData* __restrict__  cell_i, const CellData*  __restrict__  cell_j, const float r_max2, const int last_ind);

private:
    struct CellPairData{
        int cell_i = -1;
        int cell_j = -1;
        int n_pairs = 0;
    };
    std::vector<CellPairData> cell_pair_list_;
    std::vector<int> thread2first_cell_pair_ind_;
    std::vector<int> thread2n_cell_pairs_;

};




























