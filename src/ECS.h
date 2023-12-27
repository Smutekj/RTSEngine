


enum ComponentID{
    NS,
    PHYSICS,
    VISION,
    ATTACK,
    HEALTH,
    MAX_COMPONENTS
};


struct Entity{


    std::array<bool, ComponentID::MAX_COMPONENTS> component_id; //! or maybe just int?
    int ind;

    Entity(int components){
        for(int comp_id = 0; comp_id < MAX_COMPONENTS; comp_id++){
            component_id[comp_id] = (((components >> comp_id) & 1) == 1);
        }
    }

    bool hasComponent(ComponentID comp)const{
        return component_id[comp];
    }


};


struct VisionComponent{
    sf::Vector2f r;
    float radius_sq;
    u_int_16_t player_ind = 0;
};

class IComponentArray
{

protected:
    std::vector<int> component2entity_ind_;

public:
	virtual ~IComponentArray() = default;
	virtual void removeComponent(int ind) = 0;

};


template<typename CompType>
struct ComponentArray : IComponentArray{

    std::vector<CompType> components_;
    ComponentID id;

    void add(const Entity& entity, const CompType& c){
        if(entity.hasComponent(id)){
            entity.compvec_ind[id] = components_.size();
            components_.emplace_back(c);
        }
    }

    void entityDestroyed(Entity entity) {
        if(entity.hasComponent(id)){
            removeComponent(entity.compvec_ind.at(id));
        }
    }

    int removeComponent(int ind_in_components){
        components_.at(ind_in_components) = components_.back();
        components_.pop_back();
        int swapped_entity_ind = component2entity_ind_.back();
        component2entity_ind_.at(ind_in_components) = component2entity_ind_.back();
        component2entity_ind_.pop_back(); 
        return component2entity_ind_.at(ind_in_components);
    }

};

class System2 {


    std::vector<int> entity2compvec_ind_;
protected:
    std::shared_ptr<IComponentArray> p_comps_;
public:
    ComponentID id;

    System2(ComponentID id) : id(id)
    {   
        
    }

    template <typename Component>
    int addComponent(const Entity& e, Component& c){
        p_comps_->add(e, c);
        return 0;
    }

    void remove(const Entity& e){
        p_comps_->removeComponent(e.ind);
        entity2compvec_ind_.at(e.ind) = entity2compvec_ind_.back();
        entity2compvec_ind_.pop_back();
    }

    virtual void update() = 0;
};


typedef ComponentArray<VisionComponent> VisionArray;

class VisionSystem : System2{

public:
    template <typename Component>
    int addComponent(Component) {
        auto& comp_array = static_cast<VisionArray&>(*p_comps_);
        comp_array.components_.emplace_back(vc);
    }

    virtual void update(){
        
        auto& comp_array = static_cast<VisionArray&>(*p_comps_);
        
        for(VisionComponent& comp : comp_array.components_){
            //...
        }
    } 
};

struct Assemblage{
    Entity entity; //! what will be assembled
    // std::vector<> initial_data;    //! stores values of respective components    
};

struct LocationComponent{
    sf::Vector2f r;
    float orientaion;
};

struct MotionComponent{
    sf::Vector2f v_impulse;
    sf::Vector2f v_inertia;
};


struct PhysicsComponent{
    float radius;
    float mass;
};

struct AttackComponent{
    float range;
    float dmg;
};

struct NeighbourSearchComponent{
    sf::Vector2f r;
    float r_max_sq;
};

struct PathFinderComponent{
    LocationComponent* loc;
    float radius;
    sf::Vector2f target;
    sf::Vector2f next_target;
    sf::Vector2f portal;
    sf::Vector2f next_portal;
    sf::Vector2f path_end;
};


static std::vector<sf::Sprite> sprites;

struct GraphicsComponent{
    sf::Vector2f r;
    int sprite_ind;
};



class ECSystem{

    std::vector<Entity> entities_;
    std::vector<int> free_entity_spots;


    std::vector<std::shared_ptr<System2>> systems2_;

    std::vector<Entity> to_add_;
    std::vector<int> to_remove_;
    
    void addEntity(int components){
        to_add_.emplace_back(components);
    }

    void removeEntity(int entity_id){
        const auto& entity = entities_.at(entity_id);
        to_remove_.push_back(entity_id);
    }    

    void update(){
        for(auto system : systems2_){
            system->update();
        }
        removeQueriedEntities();
        addQueriedEntities();
    }

    private:
    void addQueriedEntities(){
        for(auto system: systems2_){
            
            for(const auto& new_entity : to_add_){


                if(new_entity.hasComponent(system->id)){
                    // const auto swapped_entity_ind =  system->addComponent(new_entity, );
                    // entities_.at(swapped_entity_ind).compvec_ind.at(system->id) = old_pos_in_compovec;
                };
            }    
        }
        to_add_.clear();
    }

    void removeQueriedEntities(){
        for(auto system: systems2_){
            for(const auto entity_ind : to_remove_){
                auto& entity = entities_.at(entity_ind);
                if(entity.hasComponent(system->id)){
                    system->remove(entity);
                };
            }    
        }
        to_remove_.clear();
    }

};

