#include "GraphicsSystem.h"



void makeUnitFromComponents(SquareScene& scene, GraphicsComponent& comp){
    
    //! take one free graph index and activate it 
    Graph* p_g;
    if(!scene.free_graph_inds.empty()){
        auto new_graph_ind_it = scene.free_graph_inds.begin();
        p_g = &scene.graphs.at(*new_graph_ind_it);

        scene.ind_in_active_graph_inds.at(*new_graph_ind_it) = scene.active_graph_inds.size();
        scene.active_graph_inds.push_back(*new_graph_ind_it);    
        scene.free_graph_inds.erase(new_graph_ind_it);
    }else{
        throw std::runtime_error("ran out of graph slots!");
    }
    auto& g = *p_g;

    auto* body_square = new GraphicalEntity(scene.id2squares.at(0));
    auto* weapon_square = new GraphicalEntity(scene.id2squares.at(1));
    auto* shield_square = new GraphicalEntity(scene.id2squares.at(2));
    body_square->id = 0;
    if(comp.player_ind == 1){}
    weapon_square->id = 1;
    shield_square->id = 2;

    Node* root = new Node();
    Node* child = new Node();
    Node* child2 = new Node();
    root->edges.emplace_back(child, scene.movers[0].get());
    root->edges.emplace_back(child2, scene.movers[1].get());

    root->entity = body_square;
    child->entity = weapon_square;
    child2->entity = shield_square;

    child->parent = root; 
    child2->parent = root;
    g.root = root;    

    auto parent_id = body_square->id;
    auto child_id = weapon_square->id;
    auto child2_id = shield_square->id;

    body_square->instance_id = scene.id2n_instances.at(parent_id);
    scene.id2n_instances.at(parent_id)++;
    weapon_square->instance_id = scene.id2n_instances.at(child_id);
    scene.id2n_instances.at(child_id)++;
    shield_square->instance_id = scene.id2n_instances.at(child2_id);
    scene.id2n_instances.at(child2_id)++;

    comp.graphics_ind = 0;
    comp.instance_ind = scene.id2n_instances.at(parent_id)-1;
    comp.p_graph = p_g;


    body_square->s = &scene.id2squares.at(child_id);
    weapon_square->s = &scene.id2squares.at(parent_id);
    shield_square->s = &scene.id2squares.at(child2_id);

    body_square->s->trans2.trans = comp.transform.r;
    body_square->s->trans2.scale = {3.f, 3.f};
    weapon_square->s->trans2.trans = {0,0};
    shield_square->s->trans2.trans = {0,0};
    body_square->s->trans2.angle = comp.transform.angle;
    weapon_square->s->trans2.angle = 0;
    shield_square->s->trans2.angle = 0;

    //! copy square initial transform into transform buffer
    scene.id2transforms.at(parent_id).at(body_square->instance_id) = body_square->s->trans2;
    scene.id2transforms.at(child_id).at(weapon_square->instance_id) = weapon_square->s->trans2;
    scene.id2transforms.at(child2_id).at(shield_square->instance_id) = shield_square->s->trans2;
    
    body_square->transform = &(scene.id2transforms.at(parent_id).at(body_square->instance_id)); 
    weapon_square->transform = &scene.id2transforms.at(child_id).at(weapon_square->instance_id); 
    shield_square->transform = &scene.id2transforms.at(child2_id).at(shield_square->instance_id); 

    scene.id2graphical_entities.at(parent_id).at(body_square->instance_id) = body_square;
    scene.id2graphical_entities.at(child_id).at(weapon_square->instance_id) = (weapon_square);
    scene.id2graphical_entities.at(child2_id).at(shield_square->instance_id) = (shield_square);

    scene.id_instance2graph.at(parent_id).at(body_square->instance_id) = {&g, root};
    scene.id_instance2graph.at(child_id).at(weapon_square->instance_id) = {&g, child};
    scene.id_instance2graph.at(child2_id).at(shield_square->instance_id) = {&g, child2};    
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
    for (auto &comp : components)
    {   
        p_graphics_layer->id2transforms.at(comp.graphics_ind).at(comp.instance_ind).trans = comp.transform.r;
        p_graphics_layer->id2transforms.at(comp.graphics_ind).at(comp.instance_ind).angle = comp.transform.angle; 
        
        //! why the fuck is this here? :D :D 
        comp.transform.r += comp.transform.vel * dt;
        comp.transform.angle += comp.transform.angle_vel * dt;
    

    }
    p_graphics_layer->update();

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