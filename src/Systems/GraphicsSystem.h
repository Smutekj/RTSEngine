#pragma once

#include "../ECS.h"
#include "../core.h"
#include "../Utils/Grid.h"
#include "../Utils/RandomTools.h"

#include "Components.h"
#include "../Graphics/SceneLayer.h"

constexpr float cell_size = 100.f;

constexpr int N_MAX_SPAWNED_POINTS = 100;
// struct PointSpawner{

//     sf::VertexArray points;
//     std::vector<int> life_times;
//     std::vector<sf::Vector2f> velocities;
//     std::vector<sf::Vector2f> directions;
//     std::vector<int> free_inds;

//     sf::Vector2f r_center;
//     sf::Vector2f direction;
//     float max_radius = 31.69;


//     int spawner_life_time = 600;
//     float spawn_frequency = 1; //! n_points per frame  

//     PointSpawner(sf::Vector2f r_center) : r_center(r_center){

//     }

//     // void draw(sf::RenderTarget& target){
//     //     target.draw(points);
//     // }

//     void update(){

//         spawner_life_time--;

//         int n_spawned = 0;

//         std::vector<int> to_remove;
//         int point_ind = 0;
//         for(auto& life_time : life_times){
//             life_time--;
//             if(life_time <= 0){
//                 to_remove.push_back(point_ind);
//                 spawnPoint();
//                 n_spawned++;
//             }
//             points[point_ind].position.x += velocities.at(point_ind).x ;
//             points[point_ind].position.y += velocities.at(point_ind).y ;
//             velocities.at(point_ind) -= directions.at(point_ind)*1.f/60.f;
//             point_ind++;
//         }

//         while(n_spawned < spawn_frequency){
//             spawnPoint();
//             n_spawned++;
//         }

//         while(!to_remove.empty()){
//             const auto ind_in_points = to_remove.back();
//             free_inds.push_back(ind_in_points);
//             points[ind_in_points].color = sf::Color::Transparent;
//             to_remove.pop_back();
//         }
//     }
//     void removePoint(int ind_in_points){
//         free_inds.push_back(ind_in_points);
//         points[ind_in_points].color = sf::Color::Transparent;
//     }

//     void spawnPoint(){


//         sf::Vertex new_pos = {r_center, sf::Color::Black};
//         sf::Vector2f new_dir = {2*randf()-1, 2*randf()-1};

//         int new_point_ind = life_times.size();
//         if(!free_inds.empty()){
//             new_point_ind = free_inds.back();
//             free_inds.pop_back();
//             points[new_point_ind] = new_pos;
//             life_times[new_point_ind] = 60;
//             directions[new_point_ind] = new_dir;
//             velocities[new_point_ind] = direction;
//             return ;
//         }

//         if(points.getVertexCount() > N_MAX_SPAWNED_POINTS){
//             return;
//         }
        
//         new_dir /= norm(new_dir);
//         points.append(new_pos);
//         life_times.push_back(60);
//         directions.push_back(new_dir);
//         velocities.push_back(direction);
//     }
// };


struct GraphicsSystem : System2
{

    typedef ComponentArray<GraphicsComponent> CompArray;

    std::unique_ptr<SearchGrid> p_culling_grid_;
    std::vector<std::vector<int>> grid2comp_inds_;

    // std::unordered_map<int, std::unique_ptr<PointSpawner>> point_spawners_;
    
public:
    SquareScene* p_graphics_layer;

    GraphicsSystem(ComponentID id);
    virtual ~GraphicsSystem() = default;

    virtual void update() override;

    void addOnGrid();

    // void draw(sf::RenderTarget &target) override;

    virtual void updateSharedData(const std::array<SharedData, N_MAX_ENTITIES> &new_data,
                                  const std::vector<Entity> &active_entity_inds ) override;
    
    virtual void communicate(std::array<SharedData, N_MAX_ENTITIES> &entity2shared_data) const override
    {}

    virtual void onComponentCreation(GraphicsComponent& comp);

    void draw(sf::RenderWindow& window){
        p_graphics_layer->initialize();
        p_graphics_layer->draw(0, window.view);   
             
    }

    void createSpawner(sf::Vector2f r_coords){
        // auto p_ps = std::make_unique<PointSpawner>(r_coords);
        // point_spawners_[point_spawners_.size()] = std::move(p_ps);
    }
    private:
        // sf::VertexArray vertices_;

        
};