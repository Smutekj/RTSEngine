#include "SceneLayer.h"

#include "../MapGrid.h"
#include "../Systems/VisionSystem.h"

#include <fstream>
#include <string>


SquareScene::SquareScene() : id2transforms(N_UNIQUE_GRAPHICAL_ENTITES), id2squares(N_UNIQUE_GRAPHICAL_ENTITES), id2instanceVBO(N_UNIQUE_GRAPHICAL_ENTITES, -1),
                             id2graphical_entities(N_UNIQUE_GRAPHICAL_ENTITES), id2n_instances(N_UNIQUE_GRAPHICAL_ENTITES, 0), id_instance2graph(N_UNIQUE_GRAPHICAL_ENTITES)
{
}

void SquareScene::createInstanceOf(u_int g_id)
{
    id2n_instances.at(g_id)++;
}

void SquareScene::connectParentAndChild(Node *parent, Node *child)
{
    parent->edges.emplace_back(child, nullptr);
    child->parent = parent;
}

void SquareScene::removeNodeFromInstances(const std::vector<int> &instance_inds, u_int g_id)
{
    auto &instances = id_instance2graph.at(g_id);
    for (auto inst_ind : instance_inds)
    {
        auto [p_graph, p_node] = instances.at(inst_ind);
        p_graph->removeChildNode(p_node);
    }
}
void SquareScene::addChildToInstances(const std::vector<int> &instance_inds, u_int g_id, GraphicalEntity *child)
{

    auto &instances = id_instance2graph.at(g_id);
    for (auto inst_ind : instance_inds)
    {
        Node *child_node = new Node();

        auto [p_graph, p_node] = instances.at(inst_ind);
        p_graph->addChildTo(p_node, child_node);
    }
}
void SquareScene::destroyInstanceOf(u_int g_id, int instance_ind)
{
    auto [p_graph, p_node] = id_instance2graph.at(g_id).at(instance_ind);

    if (!p_node->parent)
    { //! if the node has no parent we are deleting root and so we destroy the entire graph
        p_graph->deleteGraph();
        free_graph_inds.insert(p_graph - &graphs[0]);
        auto old_ind = ind_in_active_graph_inds.at(p_graph - &graphs[0]);
        active_graph_inds.at(old_ind) = active_graph_inds.back();
        ind_in_active_graph_inds.at(active_graph_inds.back()) = old_ind;
        active_graph_inds.pop_back();
    }
    else
    {
        p_graph->removeChildNode(p_node);
    }
    auto &transforms = id2transforms.at(g_id);

    auto last_instance_ind = id2n_instances.at(g_id) - 1;

    transforms.at(instance_ind) = transforms.at(last_instance_ind);
    id_instance2graph.at(g_id).at(instance_ind) = id_instance2graph.at(g_id).at(last_instance_ind);
    id2graphical_entities.at(g_id).at(instance_ind) = id2graphical_entities.at(g_id).at(last_instance_ind);
    // id2graphical_entities.at(g_id).at(last_instance_ind)->transform = &(transforms.at(instance_ind));

    id2n_instances.at(g_id)--;

    std::queue<Node *> to_destroy;
    for (auto edge : p_node->edges)
    {
        to_destroy.push(edge.p_child);
    }

    while (!to_destroy.empty())
    {
        auto current = to_destroy.front();
        to_destroy.pop();
        auto current_instance_ind = current->entity->instance_id;
        auto current_g_id = current->entity->id;
        last_instance_ind = id2n_instances.at(current_g_id) - 1;
        p_graph->removeChildNode(current);

        id2transforms.at(current_g_id).at(current_instance_ind) = id2transforms.at(current_g_id).at(last_instance_ind);
        ;
        id_instance2graph.at(current_g_id).at(current_instance_ind) = id_instance2graph.at(current_g_id).at(last_instance_ind);
        ;
        id2graphical_entities.at(current_g_id).at(current_instance_ind) = id2graphical_entities.at(current_g_id).at(last_instance_ind);
        ;

        id2graphical_entities.at(current_g_id).at(current_instance_ind)->transform = &(id2transforms.at(current_g_id).at(current_instance_ind));
        // id2graphical_entities.at(current_g_id).pop_back();

        id2n_instances.at(current_g_id)--;

        for (auto edge : current->edges)
        {
            to_destroy.push(edge.p_child);
        }
    }
}

void SquareScene::initialize()
{
    for (int g_id = 0; g_id < N_UNIQUE_GRAPHICAL_ENTITES; ++g_id)
    {
        auto &instanceVBO = id2instanceVBO.at(g_id);
        auto &transforms = id2transforms.at(g_id);
        auto &prototype = id2squares.at(g_id);

        // glGenBuffers(1, &instanceVBO);
        glBindVertexArray(prototype.quadVAO2);
        glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(InstancedData) * transforms.size(), transforms.data(), GL_DYNAMIC_DRAW);
        // glBindBuffer(GL_ARRAY_BUFFER, 0);
        // glBindVertexArray(0);

        // glGenBuffers(1, &instanceVBO);
        glBindVertexArray(prototype.quadVAO2);
        glBindBuffer(GL_ARRAY_BUFFER, prototype.quadVBO2);
        glBufferData(GL_ARRAY_BUFFER, 5 * sizeof(float) * prototype.vertices.size(), prototype.vertices.data(), GL_DYNAMIC_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }
}

void SquareScene::draw(GLint target, View &view)
{
    view.calcMatrix();
    for (int g_id = 0; g_id < N_UNIQUE_GRAPHICAL_ENTITES; ++g_id)
    {
        auto &transforms = id2transforms.at(g_id);

        auto wtf = transforms.data();
        if (id2n_instances.at(g_id) == 0)
        {
            continue;
        }
        auto &prototype = id2squares.at(g_id);

        shader_instanced.use();
        shader_instanced.setMat4("view", view.matrix);

        glBindVertexArray(prototype.quadVAO2);
        if (prototype.texture)
        {
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, prototype.texture);
        }

        glDrawArraysInstanced(GL_TRIANGLE_FAN, 0, 6, id2n_instances.at(g_id));
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glBindVertexArray(0);
    }
}

void SquareScene::update()
{

    u_int g_id = 0;
    for (auto &entities : id2graphical_entities)
    {
        for (int instance_ind = 0; instance_ind < id2n_instances.at(g_id); ++instance_ind)
        {
            auto entity = id2graphical_entities.at(g_id).at(instance_ind);
            auto *p_node = id_instance2graph.at(g_id).at(instance_ind).second;
            if (entity->id == 0)
            {
                continue;
            }
            auto &frame = entity->frame_i;
            frame++;
            // entity->transform->trans = {5.0f*std::sin((float)frame/50.0f), 5.0f*std::cos((float)frame/50.0f)} ;
        }
        id2squares.at(g_id).update();
        g_id++;
    }

    for (auto &mover : movers)
    {
        if (mover)
        {
            mover->frame++;
        }
    }

    for (auto graph_ind : active_graph_inds)
    {
        auto &g = graphs.at(graph_ind);
        if (!g.root)
        {
            continue;
        }

        std::queue<Node *> to_visit;
        for (auto edge : g.root->edges)
        {
            to_visit.push(edge.p_child);
            if (edge.p_mover)
            {
                edge.p_mover->move(*edge.p_child->entity->transform);
            }
            edge.p_child->entity->transform->trans += edge.p_child->parent->entity->transform->trans;
        }
        while (!to_visit.empty())
        {
            auto *current = to_visit.front();
            to_visit.pop();
            for (auto &edge : current->edges)
            {
                to_visit.push(edge.p_child);
                if (edge.p_mover)
                {
                    edge.p_mover->move(*edge.p_child->entity->transform);
                }
                else
                {
                    // edge.p_child->entity->transform->trans = {0,0};
                }
                edge.p_child->entity->transform->trans += edge.p_child->parent->entity->transform->trans;
            }
        }

        // for(auto& node : g.nodes){
        //     if(node.parent){
        //         node.entity->transform->trans += node.parent->entity->transform->trans;
        //     }
        // }
    }
}

void BuildingLayer::draw(sf::RenderWindow &window)
{

    auto &view = window.view;
    view.calcMatrix();
    frame_i++;
    for (int g_id = 0; g_id < static_cast<int>(GraphicsID::COUNT); ++g_id)
    {

        if (id2n_instances.at(g_id) == 0)
        {
            continue;
        }
        auto &prototype = id2squares.at(g_id);
        auto &transforms = id2transforms.at(g_id);

        auto &shader = id2shader.at(static_cast<GraphicsID>(g_id));
        shader.use();
        shader.setMat4("view", view.matrix);
        shader.setFloat("u_time", frame_i);

        glBindVertexArray(prototype.quadVAO2);
        if (prototype.texture)
        {
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, prototype.texture);
        }

        glDrawArraysInstanced(GL_TRIANGLE_FAN, 0, 10, id2n_instances.at(g_id));
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glBindVertexArray(0);
    }
}

void BuildingLayer::createBuildingLayout(sf::Vector2i size, int corner_size, int g_id)
{

    auto &b_contour = id2squares[g_id].vertices;
    b_contour.resize(10);
    Building b({0, 0}, {6, 6});
    b.n = size.x;
    b.m = size.y;
    b.corner_size = corner_size;
    b.intitializeEdges2({0, 0}, {5, 5});
    b_contour.at(0).pos = {0, 0};
    int i = 1;
    for (auto &edge : b.edges)
    {
        b_contour.at(i).pos = asFloat(edge.from) / 10.f;
        b_contour.at(i).uv_coord = (b_contour.at(i).pos + sf::Vector2f(1, 1)) / 2.f;
        b_contour.at(i).color = sf::Color::Red;
        i++;
    }
    b_contour.at(9) = b_contour.at(1);
}

BuildingLayer::BuildingLayer(BuildingManager& bm) : p_building_manager(&bm)
{

    id2shader[GraphicsID::BUILDING1] = {s_vertexShaderSource_instanced, s_framgentShaderSourceVoronoi, "Voronoi"};
    id2shader[GraphicsID::BUILDING2] = {s_vertexShaderSource_instanced, s_framgentShaderSourceNoise, "Noise"};

    for (int i = 0; i < 5000; ++i)
    {
        free_graph_inds.insert(i);
    }

    id2squares[0] = (Square({0, 0}, {1, 1}, {255, 0, 0}, 10));
    id2squares[1] = (Square({0, 0}, {1, 1}, {255, 0, 0}, 10));

    const auto &building_data = p_building_manager->getData();
    for(const auto& [id, data] : building_data){
        createBuildingLayout(data.size, data.corner_size, data.graphical_id);
    }

    for (int g_id = 0; g_id < static_cast<int>(GraphicsID::COUNT); ++g_id)
    {
        auto &instanceVBO = id2instanceVBO.at(g_id);
        auto &transforms = id2transforms.at(g_id);
        auto &prototype = id2squares.at(g_id);

        glGenBuffers(1, &instanceVBO);
        glBindVertexArray(prototype.quadVAO2);
        glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(InstancedData), transforms.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        //! create attribute pointers based on id maybe?
        glEnableVertexAttribArray(3);
        glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
        glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void *)0);
        glEnableVertexAttribArray(4);
        glVertexAttribPointer(4, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void *)(1 * sizeof(float)));
        glEnableVertexAttribArray(5);
        glVertexAttribPointer(5, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void *)(3 * sizeof(float)));
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        glVertexAttribDivisor(3, 1);
        glVertexAttribDivisor(4, 1);
        glVertexAttribDivisor(5, 1);
        glBindVertexArray(0);
    }
}
void MapGridLayer::updateFromMap()
{

    const auto &tiles = p_map_grid->tiles;
    const auto cell_size = p_map_grid->cell_size_;
    const auto n_cells = p_map_grid->n_cells_;

    id2transforms2[0].clear();

    for (int j = 1; j < n_cells.y - 1; ++j)
    {
        for (int i = 1; i < n_cells.x - 1; ++i)
        {
            const auto cell_index = j * n_cells.x + i;
            const sf::Vector2f position = {i * cell_size.x, j * cell_size.y};
            if (tiles.at(cell_index) == MapGrid::TileType::WALL)
            {
                id2transforms2[0].push_back({position, {0, 0}, sf::Color::Blue});
            }
            else
            {
                id2transforms2[0].push_back({position, {0, 0}, sf::Color::Transparent});
            }
            id2n_instances.at(0)++;
        }
    }

    glBindBuffer(GL_ARRAY_BUFFER, id2instanceVBO[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(InstancedDataTiles) * id2transforms2[0].size(), id2transforms2[0].data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

MapGridLayer::MapGridLayer()
{

    id2shader[GraphicsID::TILE1] = {s_vertexShaderSource_instanced, s_framgentShaderSourceVoronoi, "Voronoi"};
    id2shader[GraphicsID::TILE2] = {s_vertexShaderSource_instanced, s_framgentShaderSourceNoise, "Noise"};

    for (int i = 0; i < 5000; ++i)
    {
        free_graph_inds.insert(i);
    }

    id2squares.resize(5);

    id2squares[0] = (Square({0, 0}, {Geometry::CELL_SIZE, Geometry::CELL_SIZE}, {255, 0, 0}, 6));
    for (auto &vertex : id2squares[0].vertices)
    {
        vertex.pos *= static_cast<float>(Geometry::CELL_SIZE);
        vertex.pos += sf::Vector2f(Geometry::CELL_SIZE / 2.f, Geometry::CELL_SIZE / 2.f);
    }

    id2squares[1] = (Square({0, 0}, {1, 1}, {255, 0, 0}, 6));
    id2squares[1].vertices.resize(3);
    id2squares[1].vertices[0].pos = {0, 0};
    id2squares[1].vertices[1].pos = {0, 1};
    id2squares[1].vertices[2].pos = {1, 0};

    // // id2squares[1].vertices.resize(3);
    // id2squares[1].vertices[0].pos = {0,0};
    // id2squares[1].vertices[1].pos = {0,1};
    // id2squares[1].vertices[2].pos = {1,0};

    for (int g_id = 0; g_id < static_cast<int>(GraphicsID::COUNT); ++g_id)
    {
        auto &instanceVBO = id2instanceVBO.at(g_id);
        auto &transforms = id2transforms2.at(g_id);
        auto &prototype = id2squares.at(g_id);

        glGenBuffers(1, &instanceVBO);
        glBindVertexArray(prototype.quadVAO2);
        glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(InstancedDataTiles), transforms.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        glEnableVertexAttribArray(3);
        glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
        glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void *)0);
        glEnableVertexAttribArray(4);
        glVertexAttribPointer(4, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void *)(2 * sizeof(float)));
        glEnableVertexAttribArray(5);
        glVertexAttribPointer(5, 4, GL_UNSIGNED_BYTE, GL_FALSE, 5 * sizeof(float), (void *)(4 * sizeof(float)));
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        glVertexAttribDivisor(3, 1);
        glVertexAttribDivisor(4, 1);
        glVertexAttribDivisor(5, 1);
        glBindVertexArray(0);
    }
}

void MapGridLayer::draw(sf::RenderWindow &window)
{

    auto &view = window.view;
    view.calcMatrix();
    frame_i++;
    for (int g_id = 0; g_id < static_cast<int>(GraphicsID::COUNT); ++g_id)
    {

        if (id2n_instances.at(g_id) == 0)
        {
            continue;
        }
        auto &prototype = id2squares.at(g_id);
        auto &transforms = id2transforms2.at(g_id);

        auto &shader = id2shader.at(static_cast<GraphicsID>(g_id));
        tile_shader.use();
        tile_shader.setMat4("view", view.matrix);
        tile_shader.setFloat("u_time", frame_i);

        glBindVertexArray(prototype.quadVAO2);
        if (prototype.texture)
        {
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, prototype.texture);
        }

        glDrawArraysInstanced(GL_TRIANGLE_FAN, 0, 6, id2n_instances.at(g_id));
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glBindVertexArray(0);
    }
}

void makeBuilding(BuildingLayer &scene, Building &building)
{

    //! take one free graph index and activate it
    Graph *p_g;
    if (!scene.free_graph_inds.empty())
    {
        auto new_graph_ind_it = scene.free_graph_inds.begin();
        p_g = &scene.graphs.at(*new_graph_ind_it);

        scene.ind_in_active_graph_inds.at(*new_graph_ind_it) = scene.active_graph_inds.size();
        scene.active_graph_inds.push_back(*new_graph_ind_it);
        scene.free_graph_inds.erase(new_graph_ind_it);
    }
    else
    {
        throw std::runtime_error("ran out of graph slots!");
    }
    auto &g = *p_g;

    auto *body_square = new GraphicalEntity(scene.id2squares.at(building.graphics_id));
    body_square->id = building.graphics_id;

    Node *root = new Node();

    root->entity = body_square;
    g.root = root;

    auto parent_id = body_square->id;

    body_square->instance_id = scene.id2n_instances.at(parent_id);
    scene.id2n_instances.at(parent_id)++;

    building.instance_id = scene.id2n_instances.at(parent_id);
    building.p_graph = p_g;

    body_square->s = &scene.id2squares.at(parent_id);

    body_square->s->trans2.trans = building.calcCenter();
    body_square->s->trans2.scale = {5 * 2, 5 * 2};
    body_square->s->trans2.angle = 0;

    //! copy square initial transform into transform buffer
    scene.id2transforms.at(parent_id).at(body_square->instance_id) = body_square->s->trans2;

    body_square->transform = &(scene.id2transforms.at(parent_id).at(body_square->instance_id));

    scene.id2graphical_entities.at(parent_id).at(body_square->instance_id) = body_square;

    scene.id_instance2graph.at(parent_id).at(body_square->instance_id) = {&g, root};
}

void BuildingLayer::addBuilding(Building &b)
{
    makeBuilding(*this, b);
}

void BuildingLayer::removeBuilding(Building &b)
{
    destroyInstanceOf(b.graphics_id, b.instance_id);
}



UnitLayer::UnitLayer() : prototype({0, 0}, {1, 1}, {255, 0, 0})
{
    

    id2shader[0] = {s_vertexShaderSource_unit, s_framgentShaderSource, "unit"};

    glGenBuffers(1, &instanceVBO);
    glBindVertexArray(prototype.quadVAO2);
    glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(InstancedDataUnit), transforms.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    //! create attribute pointers based on id maybe?
    glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 1, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void *)(1 * sizeof(float)));
    glEnableVertexAttribArray(5);
    glVertexAttribPointer(5, 2, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void *)(2 * sizeof(float)));
    glEnableVertexAttribArray(6);
    glVertexAttribPointer(6, 2, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void *)(4 * sizeof(float)));
    glEnableVertexAttribArray(7);
    glVertexAttribPointer(7, 4, GL_UNSIGNED_BYTE, GL_FALSE, 7 * sizeof(float), (void *)(6 * sizeof(float)));
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glVertexAttribDivisor(3, 1);
    glVertexAttribDivisor(4, 1);
    glVertexAttribDivisor(5, 1);
    glVertexAttribDivisor(6, 1);
    glVertexAttribDivisor(7, 1);
    glBindVertexArray(0);
    
}

void UnitLayer::draw(GLint target, View &view)
{
    view.calcMatrix();

    if (n_instances == 0){ return; }
  
    id2shader.at(0).use();
    id2shader.at(0).setMat4("view", view.matrix);
    

    glBindVertexArray(prototype.quadVAO2);
    if (prototype.texture)
    {
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, prototype.texture);
    }

    glDrawArraysInstanced(GL_TRIANGLE_FAN, 0, 6, n_instances);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindVertexArray(0);
    
}

void createFramebuffer(GLuint &fbo, GLuint &fbtex)
{
    glGenTextures(1, &fbtex);
    glBindTexture(GL_TEXTURE_2D, fbtex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, 192, 108, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo);
    glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fbtex, 0);
    {
        GLenum status;
        status = glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER);
        switch (status)
        {
        case GL_FRAMEBUFFER_COMPLETE:
            break;
        case GL_FRAMEBUFFER_UNSUPPORTED:
            std::cerr << "Error: unsupported framebuffer format" << std::endl;
            exit(0);
        default:
            std::cerr << "Error: invalid framebuffer config" << std::endl;
            exit(0);
        }
    }
}

VisionLayer::VisionLayer()
{

    createFramebuffer(fbo[0], tex[0]);
    createFramebuffer(fbo[1], tex[1]);

    for (auto &v : tex_square.vertices)
    {
        v.pos *= 2.0f;
    }

    glGenVertexArrays(1, &revealed_fow_vertices.quadVAO);
    glBindVertexArray(revealed_fow_vertices.quadVAO);
    glGenBuffers(1, &revealed_fow_vertices.quadVBO);
    glBindBuffer(GL_ARRAY_BUFFER, revealed_fow_vertices.quadVBO);
    glBufferData(GL_ARRAY_BUFFER, 5 * sizeof(float) * revealed_fow_vertices.getVertexCount(), &revealed_fow_vertices[0], GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_TRUE, 5 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, 5 * sizeof(float), (void *)(2 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_TRUE, 5 * sizeof(float), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    glGenVertexArrays(1, &fow_vertices.quadVAO);
    glBindVertexArray(fow_vertices.quadVAO);
    glGenBuffers(1, &fow_vertices.quadVBO);
    glBindBuffer(GL_ARRAY_BUFFER, fow_vertices.quadVBO);
    glBufferData(GL_ARRAY_BUFFER, 5 * sizeof(float) * fow_vertices.getVertexCount(), &fow_vertices[0], GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_TRUE, 5 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, 5 * sizeof(float), (void *)(2 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_TRUE, 5 * sizeof(float), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void VisionLayer::drawFOW(sf::RenderWindow &window, sf::VertexArray &fow_verticess)
{

    const auto height0 = 1920;
    const auto width0 = 1080;
    const auto zoom_factor = 3.f;
    const auto old_view_matrix = window.view.matrix;

    window.view.zoom(zoom_factor);
    fow_shader.use();
    fow_shader.setMat4("view", window.view.matrix);
    fow_shader.setMat4("transform", glm::mat4(1));

    glBindVertexArray(fow_verticess.quadVAO);

    glBindFramebuffer(GL_FRAMEBUFFER, fbo[0]);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(0.0, 0.0, 192, 108);

    glDrawArrays(GL_TRIANGLES, 0, fow_verticess.getVertexCount());

    glBindVertexArray(tex_square.quadVAO2);
    glBindBuffer(GL_ARRAY_BUFFER, tex_square.quadVBO2);
    glBufferData(GL_ARRAY_BUFFER, 5 * sizeof(float) * tex_square.vertices.size(), tex_square.vertices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    const auto n_passes = 1;
    for (int pass_ind = 0; pass_ind < n_passes; ++pass_ind)
    {
        glBindFramebuffer(GL_FRAMEBUFFER, fbo[(pass_ind + 1) % 2]);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        gauss_horiz.use();
        gauss_horiz.setMat4("view", glm::mat4(1));
        gauss_horiz.setMat4("transform", glm::mat4(1));

        glBindTexture(GL_TEXTURE_2D, tex[pass_ind % 2]);
        glDrawArrays(GL_TRIANGLE_FAN, 0, 6);

        glBindFramebuffer(GL_FRAMEBUFFER, fbo[(pass_ind % 2)]);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        gauss_vert.use();
        gauss_vert.setMat4("view", glm::mat4(1));
        gauss_vert.setMat4("transform", glm::mat4(1));

        glBindTexture(GL_TEXTURE_2D, tex[(pass_ind + 1) % 2]);
        glDrawArrays(GL_TRIANGLE_FAN, 0, 6);
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0.0, 0.0, 1920, 1080);

    auto mat = glm::mat4(1);
    downsample.use();
    downsample.setMat4("view", glm::scale(mat, glm::vec3(zoom_factor, zoom_factor, 1.f)));
    downsample.setMat4("transform", glm::mat4(1));

    glBindTexture(GL_TEXTURE_2D, tex[(n_passes + 1) % 2]);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 6);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindVertexArray(0);

    // window.view.matrix = old_view_matrix;
    window.view.zoom(1.f / zoom_factor);
}

void VisionLayer::draw2(sf::RenderWindow &window)
{
    window.view.calcMatrix();

    const auto selected_player_ind = p_vs->selected_player_ind;

    auto &vf = p_vs->player2vision_field_.at(selected_player_ind);

    drawFOW(window, fow_vertices);

}

void VisionLayer::setup()
{

    auto &vs = *p_vs;
    const auto selected_player_ind = vs.selected_player_ind;

    auto &vf = vs.player2vision_field_.at(selected_player_ind);
    fow_vertices.clear();
    fow_vertices.setPrimitiveType(sf::Quads);
    revealed_fow_vertices.clear();
    revealed_fow_vertices.setPrimitiveType(sf::Quads);
    int n_revealed_vertices = 0;
    int n_fow_vertices = 0;

    auto &stripes_ = vf.stripes_;
    auto &revealed_stripes_ = vf.revealed_stripes_;

    for (int stripe_ind = 0; stripe_ind < FOW::N_STRIPES; ++stripe_ind)
    {
        n_revealed_vertices += 6 * (revealed_stripes_.at(stripe_ind).size() + 1);
        n_fow_vertices += 6 * (stripes_.at(stripe_ind).size() + 1);
    }

    revealed_fow_vertices.resize(n_revealed_vertices);
    fow_vertices.resize(n_fow_vertices);

    int last_ind_s = 0;
    int last_ind_rs = 0;
    for (int stripe_ind = 0; stripe_ind < FOW::N_STRIPES; ++stripe_ind)
    {
        const float y_pos = stripe_ind * vs.dy_;
        vs.drawStripe(stripes_.at(stripe_ind), y_pos, fow_vertices, {vs.grey_color}, last_ind_s);
        vs.drawStripe(revealed_stripes_.at(stripe_ind), y_pos, revealed_fow_vertices, sf::Color::Black, last_ind_rs);
    }

    glBindBuffer(GL_ARRAY_BUFFER, revealed_fow_vertices.quadVBO);
    glBufferData(GL_ARRAY_BUFFER, 5 * sizeof(float) * revealed_fow_vertices.getVertexCount(), &revealed_fow_vertices[0], GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, fow_vertices.quadVBO);
    glBufferData(GL_ARRAY_BUFFER, 5 * sizeof(float) * fow_vertices.getVertexCount(), &fow_vertices[0], GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}
