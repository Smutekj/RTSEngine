#include "GraphicsSystem.h"



void makeUnitFromComponents(UnitLayer& scene, GraphicsComponent& comp){
    
 
    const auto instance_id = scene.n_instances;
    comp.instance_ind = instance_id;

    auto& transforms = scene.transforms;
    transforms[instance_id].trans = comp.transform.r;
    transforms[instance_id].angle = comp.transform.angle;  
    transforms[instance_id].scale = comp.radius;  
    scene.n_instances++;
}


    void GraphicsSystem::onComponentCreation(GraphicsComponent& comp){
    
        makeUnitFromComponents(*p_graphics_layer, comp);   
    }

    GraphicsSystem::GraphicsSystem(ComponentID id) : System2(id)
    {
        sf::Vector2i n_cells = {static_cast<int>(Geometry::BOX[0] / cell_size), static_cast<int>(Geometry::BOX[1] / cell_size)};
        sf::Vector2f cell_sizes(cell_size, cell_size);
        p_culling_grid_ = std::make_unique<SearchGrid>(n_cells, cell_sizes);
        grid2comp_inds_.resize(n_cells.x * n_cells.y);
    }
    

void GraphicsSystem::update()
{
    std::vector<int> spawner_ids_to_remove;
    // for(auto& [ps_id, ps] : point_spawners_){ 
    //     ps->update();
    //     if(ps->spawner_life_time <= 0){
    //         spawner_ids_to_remove.push_back(ps_id);
    //     }
    // }

    while(!spawner_ids_to_remove.empty()){
        auto id = spawner_ids_to_remove.back();
        // point_spawners_.erase(id);
        spawner_ids_to_remove.pop_back();
    }



    auto &components = static_cast<CompArray &>(*p_comps_).components_;
    auto& transforms = p_graphics_layer->transforms;

    for (auto &comp : components)
    {   
        transforms.at(comp.instance_ind).trans = comp.transform.r;
        transforms.at(comp.instance_ind).angle = comp.transform.angle; 
        transforms.at(comp.instance_ind).scale = comp.radius; 
        
        sf::Color c = sf::Color::Red;
        if(comp.player_ind == 1){
            c = sf::Color::Blue;
        }
        transforms.at(comp.instance_ind).color = c; 
        
        //! why the fuck is this here? :D :D 
        comp.transform.r += comp.transform.vel * dt;
        comp.transform.angle += comp.transform.angle_vel * dt;
    

    }
    // p_graphics_layer->update();

    addOnGrid();

}

void GraphicsSystem::addOnGrid()
{
    const auto &components = static_cast<CompArray &>(*p_comps_).components_;
    int compvec_ind = 0;
    for (const auto &comp : components)
    {
        const auto cell_ind = p_culling_grid_->coordToCell(comp.transform.r);
        grid2comp_inds_.at(cell_ind).push_back(compvec_ind);
        compvec_ind++;
    }
}

// void GraphicsSystem::draw(sf::RenderTarget &target)
// {

//     for(auto& [ps_id, p_ps] : point_spawners_){ p_ps->draw(target); }

//     const auto &components = static_cast<CompArray &>(*p_comps_).components_;

//     vertices_.setPrimitiveType(sf::Triangles);
//     vertices_.resize(components.size() * 3);

//     sf::ConvexShape tri;
//     tri.setPointCount(3);

//     int i = 0;
//     for (const auto &comp : components)
//     {
//         const auto &r = comp.transform.r;
//         const auto angle = comp.transform.angle;
//         const auto radius = 3.f;
//         const auto player_ind = comp.player_ind;
//         tri.setPoint(0, {-0.7f * radius, -0.5f * radius});
//         tri.setPoint(1, {radius, 0});
//         tri.setPoint(2, {-0.7f * radius, 0.5f * radius});
//         sf::Transform a;
//         a.rotate(angle);
//         const auto ent_ind = compvec_ind2entity_ind_.at(i); 
//         const float health_ratio = comp.p_shared_data->health / comp.p_shared_data->max_health;
//         sf::Color color = sf::Color::Black;
//         player_ind == 0 ? color = sf::Color::Red : color = sf::Color::Blue;
//         color.r = (health_ratio)*255; 

//         vertices_[i * 3 + 0] = {r + a.transformPoint(tri.getPoint(0)), color};
//         vertices_[i * 3 + 1] = {r + a.transformPoint(tri.getPoint(1)), color};
//         vertices_[i * 3 + 2] = {r + a.transformPoint(tri.getPoint(2)), color};
//         i++;
//     }
//     target.draw(vertices_);
// }

void GraphicsSystem::updateSharedData(const std::array<SharedData, N_MAX_ENTITIES> &new_data,
                                      const std::vector<Entity> &active_entity_inds)
{
    auto &comps = static_cast<ComponentArray<GraphicsComponent> &>(*p_comps_.get()).components_;
    for (const auto ent : active_entity_inds)
    {
        const auto compvec_ind = entity2compvec_ind_.at(ent.ind);
        comps.at(compvec_ind).transform = new_data.at(ent.ind).transform;
        // comps.at(compvec_ind).state = new_data.at(ent_ind).state;
    }
}