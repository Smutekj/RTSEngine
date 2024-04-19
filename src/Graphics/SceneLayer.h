#pragma once

#include <memory>
#include <set>
#include <queue>
#include <unordered_map>

#include "../core.h"
#include "../ShaderNames.h"
#include "../BuildingManager.h"
#include "../Utils/GayVector.h"


#include "Shader.h"
#include "RenderWindow.hpp"
#include "Texture.hpp"
#include "VertexArray.hpp"


struct InstancedData
{
    float angle = 0.f;
    sf::Vector2f scale = {1,1};
    sf::Vector2f trans = {0, 0};
};

struct InstancedDataUnit
{
    float angle = 0.f;
    float scale = 1.f;
    sf::Vector2f trans = {0, 0};
    sf::Vector2f tex_coord = {0,0};
    sf::Color color = sf::Color::Red;
};

struct InstancedDataTiles
{
    sf::Vector2f trans = {0, 0};
    sf::Vector2f tex_coord = {0,0};
    sf::Color color = sf::Color::Transparent;
};

struct Transform
{
    sf::Vector2f mTranslate = {0, 0};
    sf::Vector2f mScale = {1, 1};
    float mAngle = 0;

    glm::mat4 matrix = glm::mat4(1.0f);

    void calcMatrix()
    {
        matrix = glm::mat4(1.0f);
        matrix = glm::translate(matrix, glm::vec3(mTranslate.x, mTranslate.y, 0));
        matrix = glm::scale(matrix, glm::vec3(mScale.x, mScale.y, 0));
        matrix = glm::rotate(matrix, glm::radians(mAngle), glm::vec3(0.0f, 0.0f, 1.0f));
    }

    void setTranslate(sf::Vector2f translate)
    {
        mTranslate = translate;
        calcMatrix();
    }

    void setScale(sf::Vector2f scale)
    {
        mScale = scale;
        calcMatrix();
    }

    void setRotatation(float angle)
    {
        mAngle = angle;
        calcMatrix();
    }

    Transform getInverse() const;

    sf::Vector2f transformPoint(const sf::Vector2f r) const;
};

struct VertexData
{
    sf::Vector2f pos;
    sf::Color color;
    sf::Vector2f uv_coord;
    VertexData(sf::Vector2f crd, sf::Color col, sf::Vector2f uv) : pos(crd), color(col), uv_coord(uv) {}
    VertexData() = default;
};

struct Square
{
    sf::Vector2f r_center;
    sf::Vector2f size;

    std::unique_ptr<sf::Texture> p_texture;
    GLuint texture = 0;

    Transform trans;
    InstancedData trans2;

    Shader shader = {s_vertexShaderSourceViewTransform, s_framgentShaderSource};

    GLenum primitive_type = GL_TRIANGLE_FAN;
private:
    int n_vertices = 6;

public:
    // The fullscreen quad's FBO
    GLuint quadVAO2, quadVBO2;
    Square(sf::Vector2f pos = {0, 0}, sf::Vector2f size = {1, 1},
           sf::Color color = sf::Color::Transparent,
           int n_vertices = 6)
        : r_center(pos), size(size), n_vertices(n_vertices), vertices(n_vertices)
    {

        trans.setTranslate(pos);
        trans.setScale(size);

        trans2.trans = pos;
        trans2.scale = size;
        vertices[0].pos = {0.0, 0.0};
        vertices[1].pos = {0.5, -0.5};
        vertices[2].pos = {0.5, 0.5};
        vertices[3].pos = {-0.5, 0.5};
        vertices[4].pos = {-0.5, -0.5};
        vertices[5].pos = {0.5, -0.5};
        for (int vert_ind = 0; vert_ind < n_vertices; ++vert_ind)
        {
            vertices[vert_ind].color = color;
        }

        vertices[0].uv_coord = {0.5, 0.5};
        vertices[1].uv_coord = {1.0, 0};
        vertices[2].uv_coord = {1.0, 1.0};
        vertices[3].uv_coord = {0, 1.0};
        vertices[4].uv_coord = {0, 0};
        vertices[5].uv_coord = {1.0, 0};

        glGenVertexArrays(1, &quadVAO2);
        glBindVertexArray(quadVAO2);
        glGenBuffers(1, &quadVBO2);
        glBindBuffer(GL_ARRAY_BUFFER, quadVBO2);
        glBufferData(GL_ARRAY_BUFFER, 5 * sizeof(float) * vertices.size(), vertices.data(), GL_DYNAMIC_DRAW);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_TRUE, 5 * sizeof(float), (void *)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, 5 * sizeof(float), (void *)(2 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_TRUE, 5 * sizeof(float), (void *)(3 * sizeof(float)));
        glEnableVertexAttribArray(2);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }

    int frame = 0;
    void update()
    {
        frame++;
        sf::Vector2f dir = {-1, 1};
        for (int v_ind = 1; v_ind < 6; ++v_ind)
        {
            auto &v = vertices[v_ind];
            // v.pos -= dir * 0.005f * (std::sin((float)(frame + v_ind*5)/20.f));
            auto old_dir = dir;
            // dir.x = -old_dir.y;
            // dir.y = old_dir.x;
        }
    }

    void move(sf::Vector2f by)
    {
        r_center += by;
        trans.setTranslate(r_center);
    }

    void rotate(float by)
    {
        trans.mAngle += by;
        trans.setRotatation(trans.mAngle);
    }

    void draw(sf::RenderWindow &window)
    {
        glBindVertexArray(quadVAO2);

        // __glewBindFramebuffer(GL_DRAW_FRAMEBUFFER, target);
        // glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        // glClearColor(0.0, 1.f, 1.0f, 1.0f);
        if (texture)
        {
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, texture);
        }

        shader.use();
        shader.setMat4("view", window.view.matrix);
        shader.setMat4("transform", trans.matrix);

        // glBindBuffer(GL_ARRAY_BUFFER, quadVBO2);
        // glBufferData(GL_ARRAY_BUFFER, 5 * sizeof(float) * vertices.size(), vertices.data(), GL_DYNAMIC_DRAW);
        // glBindBuffer(GL_ARRAY_BUFFER, 0);

        glDrawArrays(primitive_type, 0, n_vertices);

        glBindVertexArray(0);
        glBindTexture(GL_TEXTURE_2D, 0);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    }

    void loadTextureFromFile(int width, int height, std::string filename)
    {
        p_texture = std::make_unique<sf::Texture>(width, height, filename);
        texture = p_texture->textureID;
    }

    unsigned char *getTexture()
    {
        return p_texture->getData();
    }

public:
    std::vector<VertexData> vertices;
};

enum class GraphicsID
{
    AGENT = 0,
    PLANET,
    MOON,
    COUNT_GRAPHICAL_IDS
};

constexpr auto N_UNIQUE_GRAPHICAL_ENTITES = static_cast<int>(GraphicsID::COUNT_GRAPHICAL_IDS);

template <int N_VERTS = 4>
class GMesh
{
    std::array<VertexData, N_VERTS> vertices;
};

struct GraphicalEntity
{

    InstancedData *transform = nullptr;
    Square *s;
    int id;
    int instance_id;
    GLint texture;
    int frame_i = 0;

    GraphicalEntity(Square &sq)
        : s(&sq)
    {
    }
};

struct GraphManager
{
};

struct Mover
{
    u_int16_t frame = 0;
    virtual void move(InstancedData &transform) = 0;
};

struct CircleMover : Mover
{

    float radius = 5;
    float angular_frequency = (2.0*M_PIf);

    virtual void move(InstancedData &transform)
    {
        const auto t = static_cast<float>(frame) / 1200.;
        transform.trans.x = radius * std::cos(angular_frequency * t);
        transform.trans.y = radius * std::sin(angular_frequency * t);
        frame = frame % 2400;
    }
};

struct LineMover : Mover
{

    float radius = 10;
    float angular_frequency = (M_2_PIf / 1200.f);
    sf::Vector2f dir = {1, 0};

    virtual void move(InstancedData &transform)
    {
        const auto t = static_cast<float>(frame);
        const auto alpha = std::cos(angular_frequency * t);
        transform.trans.x = dir.x * radius * alpha;
        transform.trans.y = dir.y * radius * alpha;
        transform.scale.x = (alpha + 1.5);
    }
};

template <typename MoverType>
struct GraphEdge
{
    MoverType mover;
};

struct Node;

struct GraphEdgeSimple
{
    Node *p_child = nullptr;
    Mover *p_mover = nullptr;
    GraphEdgeSimple(Node *node, Mover *mover = nullptr) : p_child(node), p_mover(mover) {}
};

struct Node
{
    GraphicalEntity *entity = nullptr;
    Node *parent = nullptr;
    std::vector<GraphEdgeSimple> edges;
};

struct Graph
{

    std::vector<Node> nodes;
    Node *root;

    Graph() = default;

    Graph(GraphicalEntity *root) : nodes(1)
    {
        nodes[0].entity = root;
    }

    void addChildTo(Node *parent, Node *child)
    {
        parent->edges.emplace_back(child, nullptr);
        child->parent = parent;
    }

    void removeChildNode(Node *node)
    {
        auto &children = node->parent->edges;
        auto child_it = std::find_if(children.begin(), children.end(), [&node](auto &edge)
                                     { return edge.p_child == node; });
        assert(child_it != children.end());
        children.erase(child_it);
        node->parent = nullptr;
    }

    void deleteGraph()
    {
        root = nullptr;
    }
};

constexpr int N_MAX_INSTANCES = 20000;
constexpr int N_MAX_GRAPHS = 20000;

struct SquareScene
{
    std::vector<std::vector<int>> graph_node2children;

    std::vector<Square> id2squares;
    // ShaderManager shader_manager;

    std::vector<std::array<InstancedData, N_MAX_INSTANCES>> id2transforms;
    std::vector<int> id2n_instances;

    std::vector<std::array<GraphicalEntity *, N_MAX_INSTANCES>> id2graphical_entities;
    std::vector<std::array<std::pair<Graph *, Node *>, N_MAX_INSTANCES>> id_instance2graph;


    std::array<Graph, N_MAX_GRAPHS> graphs;
    std::set<int> free_graph_inds;
    std::vector<int> active_graph_inds;
    std::array<int, N_MAX_GRAPHS> ind_in_active_graph_inds;
    std::array<std::unique_ptr<Mover>, 2> movers;

    Shader shader_instanced = {s_vertexShaderSource_instanced, s_framgentShaderSource};

    std::vector<unsigned int> id2instanceVBO;

    SquareScene();
    void createInstanceOf(u_int g_id);

    void connectParentAndChild(Node *parent, Node *child);
    void removeNodeFromInstances(const std::vector<int> &instance_inds, u_int g_id);
    void addChildToInstances(const std::vector<int> &instance_inds, u_int g_id, GraphicalEntity *child);
    void destroyInstanceOf(u_int g_id, int instance_ind);

    void initialize();

    void draw(GLint target, View &view);

    void update();
};

enum class SceneLayerID : int
{
    UNITS = 0,
    BUILDINGS,
    TERRAIN,
    PROJECTILES,
    EFFECTS,
    COUNT
};

struct GameScene
{
    std::vector<std::unique_ptr<SquareScene>> layers;
    GameScene()
    {
        for (int layer_id = 0; layer_id < static_cast<int>(SceneLayerID::COUNT); ++layer_id)
        {
            layers.emplace_back(std::make_unique<SquareScene>());
        }
    }
};


struct Building;

enum class ShaderType{
    VORONOI,
    INSTANCED,
    BASIC
};



struct BuildingLayer : public SquareScene
{

    enum class GraphicsID : int
    {
        BUILDING1 = 0,
        BUILDING2,
        COUNT
    };

    Shader test_building_shader = {s_vertexShaderSource_instanced, s_framgentShaderSourceVoronoi};
    u_int64_t frame_i = 0;

    std::unordered_map<GraphicsID, Shader> id2shader;
    BuildingManager* p_building_manager = nullptr;

    BuildingLayer(BuildingManager& bm);

    void addBuilding(Building &b);
    void removeBuilding(Building &b);
    virtual void draw(sf::RenderWindow &window);

    private:
    void createBuildingLayout(sf::Vector2i size, int corner_size, int g_id);
};

class MapGrid;

struct MapGridLayer : public SquareScene
{

    MapGrid* p_map_grid;

    enum class GraphicsID : int
    {
        TILE1 = 0,
        TILE2,
        COUNT
    };


    std::array<std::vector<InstancedDataTiles>, static_cast<int>(GraphicsID::COUNT)> id2transforms2;

    Shader tile_shader = {s_vertexShaderTiles, s_framgentShaderSource};
    u_int64_t frame_i = 0;

    std::unordered_map<GraphicsID, Shader> id2shader;

    MapGridLayer();

    void updateFromMap();
    virtual void draw(sf::RenderWindow &window);
};


struct UnitLayer{

    std::unordered_map<int, Shader> id2shader;

    enum class GraphicsID : int
    {
        UNIT1 = 0,
        UNIT2,
        COUNT
    };

    std::array<InstancedDataUnit, N_MAX_INSTANCES> transforms;
    GLuint instanceVBO; 
    int n_instances = 0;
    Square prototype;

    UnitLayer();

    void initialize(){

        // glGenBuffers(1, &instanceVBO);
        glBindVertexArray(prototype.quadVAO2);
        glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(InstancedDataUnit) * n_instances, transforms.data(), GL_DYNAMIC_DRAW);
        // glBindBuffer(GL_ARRAY_BUFFER, 0);
        // glBindVertexArray(0);

        // glGenBuffers(1, &instanceVBO);
        glBindVertexArray(prototype.quadVAO2);
        glBindBuffer(GL_ARRAY_BUFFER, prototype.quadVBO2);
        glBufferData(GL_ARRAY_BUFFER, 7 * sizeof(float) * prototype.vertices.size(), prototype.vertices.data(), GL_DYNAMIC_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    
    }
    void draw(GLint target, View& view);

    void removeInstance(int instance_ind){

        n_instances--;
        // transforms.at(instance_ind) = transforms.at(n_instances);

    }

};

class VisionSystem;
namespace sf{
    class RenderWindow;
};



struct VisionLayer : public SquareScene{

    VisionSystem* p_vs;

    Shader fow_shader = {s_vertexShaderSourceViewTransform, s_framgentShaderSourceVision,  "fow"};
    Shader gauss_horiz = {s_vertexShaderSourceViewTransform, s_framgentShaderSourceGaussHoriz,  "gauss_horiz"};
    Shader gauss_vert = {s_vertexShaderSourceViewTransform, s_framgentShaderSourceGaussVert,  "gauss_vert"};
    Shader downsample = {s_vertexShaderSourceViewTransform, s_framgentShaderSourceDownSample,  "downsampler"};

    GLuint fbo[2];
    GLuint tex[2];

    Square tex_square;

    sf::VertexArray fow_vertices;          //! this holds vertices for drawing
    sf::VertexArray revealed_fow_vertices; //! this holds vertices for drawing


    VisionLayer();

    void setup();

    virtual void draw(GLint target, View& view){}
    void draw2(sf::RenderWindow& window);
    void drawFOW(sf::RenderWindow& window, sf::VertexArray& fow_vertices);
};

enum class AtomName : int
{
    H = 0,
    He,
    Li,
    Be,
    B,
    C,
    N,
    O,
    F,
    Ne,
    Na,
    Mg,
    Al
};

// std::map<AtomName, std::string> symbol2string = {
//     {AtomName::H, "H"},
//     {AtomName::C, "C"},
//     {AtomName::N, "N"},
//     {AtomName::O, "O"},
//     {AtomName::F, "F"}
// };

// struct Atom
// {
//     Text2 name;
//     AtomName symbol = AtomName::C;
// };

// struct Bond
// {
//     int ind_i = -1;
//     int ind_j = -1;
//     sf::RectangleShape line;
// };

// struct Molecule : sf::Transformable
// {
//     std::vector<Bond> bonds;
//     std::vector<Atom> atoms;
//     std::vector<std::vector<Bond>> atom2bonds;


//     void addAtom(sf::Vector2f pos, const AtomName a)
//     {
//         Atom new_atom;
//         new_atom.name.setPosition(pos);
//         new_atom.name.setString(symbol2string.at(a));
//         atoms.push_back(new_atom);
//         atom2bonds.emplace_back();
//     }

//     void addBond(const Bond &b)
//     {
//         assert(b.ind_i < atoms.size() && b.ind_i >= 0 &&
//                b.ind_j >= 0 && b.ind_i < atoms.size());

//         atom2bonds.at(b.ind_i).push_back(b);
//         atom2bonds.at(b.ind_j).push_back(b);
//     }

//     void computeBondPoistions()
//     {
//         const auto n_atoms = atoms.size();
//         for (int atom_ind = 0; atom_ind < n_atoms; atom_ind++)
//         {
//         }
//     }

//     void draw(sf::RenderWindow &window)
//     {
//         int atom_ind = 0;
//         for(auto& atom : atoms){
            
//             auto& bonds = atom2bonds.at(atom_ind);
//             atom_ind++;
//             for(auto& bond : bonds){
//                 const auto first_atom = atoms.at(bond.ind_j);
//                 const auto second_atom = atoms.at(bond.ind_i);
//                 if(first_atom.symbol == AtomName::H || second_atom.symbol == AtomName::H){
//                     bond.line.draw(window);
//                 }
                
//             }
//             if(atom.symbol != AtomName::C){
//                 atom.name.draw(window);
//             }
//         }

//     }
// };

// Molecule createEthanol(sf::Vector2f pos, float angle, float scale)
// {

//     Molecule ethanol;

//     ethanol.setPosition(pos);
//     ethanol.setRotation(angle);
//     ethanol.setScale(scale, scale);
//     ethanol.addAtom(pos, AtomName::C);

//     // ethanol.addAtom(pos - {5, 0}, AtomName::C);
//     // ethanol.addAtom(pos + {2, 3}, AtomName::O);
//     // ethanol.addAtom( - {5, 0}, AtomName::H);

//     return ethanol;
// }

// void fillHydrogens(){

// }