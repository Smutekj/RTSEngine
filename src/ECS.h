#pragma once

#include <memory>
#include <typeindex>
#include <typeinfo>
#include <type_traits>
#include <iostream>
#include <numeric>
#include <unordered_map>
#include <queue>

#include "core.h"
#include "Geometry.h"
#include "Systems/Components.h"
#include "Utils/Grid.h"
#include "Utils/GayVector.h"

constexpr float dt = 1. / 60.f;
constexpr int N_MAX_ENTITIES = 10000;




// struct Observatory{

//     std::vector<std::unique_ptr<Observer>> observers_;

//     void update(){
//         for(auto& observer : observers_){
    
//         }
//     }

// };

// struct Observer

//! \struct maintains a table for entity -> component mappings; 
struct BookKeeper
{

    std::array<int, N_MAX_ENTITIES> entity2compvec_ind_;

};

struct Entity
{

    int ind;
    std::array<bool, MAX_COMPONENTS> component_id; //! or maybe just int?


    Entity(int components)
    {
        for (int comp_id = 0; comp_id < MAX_COMPONENTS; comp_id++)
        {
            component_id[comp_id] = (((components >> comp_id) & 1) == 1);
        }
        sizeof(*this);
    }
    Entity(int components, int ind) : ind(ind)
    {
        for (int comp_id = 0; comp_id < MAX_COMPONENTS; comp_id++)
        {
            component_id[comp_id] = (((components >> comp_id) & 1) == 1);
        }
    }
    ~Entity() = default;

    bool hasComponent(ComponentID comp) const
    {
        return component_id[comp];
    }
};

inline int components2Signature(std::vector<ComponentID> &comps)
{

    int signature = 0;
    for (const int &comp : comps)
    {
        signature += comp;
    }
    return signature;
}

inline std::vector<ComponentID> signature2Components(int signature)
{
    std::vector<ComponentID> components;
    for (int id = 0; id < MAX_COMPONENTS; ++id)
    {
        if (((signature >> id) & 1) == 1)
        {
            components.push_back(static_cast<ComponentID>(id));
        }
    }
    return components;
}

class IComponentArray
{

protected:
    std::vector<int> component2entity_ind_;

public:
    virtual ~IComponentArray() = default;
    virtual void removeComponent(int ind) = 0;
};

template <typename CompType>
struct ComponentArray : IComponentArray
{

    std::vector<CompType> components_;

    int add(CompType &c)
    {
        components_.emplace_back(c);
        return components_.size() - 1;
    }

    void entityDestroyed(int compvec_ind)
    {
        removeComponent(compvec_ind);
    }

    void removeComponent(int compvec_ind)
    {
        components_.at(compvec_ind) = components_.back();
        components_.pop_back();
    }
};

struct SharedData
{
    TransformComponent transform;
    MoveState state = MoveState::STANDING;
    float size = RHARD;
    int player_ind = 0;
    HealthComponent health;
    sf::Vector2f acc = {0,0};
    sf::Vector2f target = {-1, -1};
};

class System2
{

protected:
    std::vector<int> compvec_ind2entity_ind_;
    std::array<int, N_MAX_ENTITIES> entity2compvec_ind_;

public:
    std::shared_ptr<IComponentArray> p_comps_;
    ComponentID id;
    int n_components = 0;

    System2(ComponentID id) : id(id)
    {
        entity2compvec_ind_.fill(-1);
    }

public:
    template <typename CompType>
    void addComponent(const Entity &e, const CompType &comp)
    {
        auto &components = static_cast<ComponentArray<CompType> &>(*p_comps_.get()).components_;
        components.push_back(comp);
        n_components++;

        compvec_ind2entity_ind_.push_back(e.ind);
        entity2compvec_ind_.at(e.ind) = compvec_ind2entity_ind_.size() - 1;
    }

    template <ComponentID id, typename CompType>
    const CompType &getComponent(const Entity &e) const
    {
        const auto &components = static_cast<ComponentArray<CompType> &>(*p_comps_.get()).components_;
        return components.at(entity2compvec_ind_.at(e.ind));
    }

    template <typename CompType>
    void setLastComponent(const Entity &e, CompType &c)
    {
        auto &components = static_cast<ComponentArray<CompType> &>(*p_comps_.get()).components_;
        components.back() = c;
    }

    template <typename CompType>
    void setComponent(const Entity &e, CompType &c)
    {
        auto &components = static_cast<ComponentArray<CompType> &>(*p_comps_.get()).components_;
        components.at(entity2compvec_ind_.at(e.ind)) = c;
    }

    void remove(const Entity &e)
    {
        // assert(e.hasComponent(id));

        auto removed_comp_ind = entity2compvec_ind_.at(e.ind);
        if(removed_comp_ind == -1){ //! entity does not have component in this system!
            return;
        }
        p_comps_->removeComponent(removed_comp_ind);

        auto changed_entity_ind = compvec_ind2entity_ind_.back();
        compvec_ind2entity_ind_.at(removed_comp_ind) = changed_entity_ind;
        compvec_ind2entity_ind_.pop_back();

        entity2compvec_ind_.at(changed_entity_ind) = removed_comp_ind;
        entity2compvec_ind_.at(e.ind) = -1;
        n_components--;
    }

    template <class CompType>
    void updateSharedTransforms(const std::array<SharedData, N_MAX_ENTITIES> &new_data, const std::vector<Entity> &active_entity_inds)
    {
        static_assert(component2id(CompType()) == id);
        auto &comps = static_cast<ComponentArray<CompType> &>(*p_comps_.get()).components_;
        for (const auto ent : active_entity_inds)
        {
            const auto compvec_ind = entity2compvec_ind_.at(ent.ind);
            // comps.at(compvec_ind).transform = new_data.at(ent_ind).transform;
            // comps.at(compvec_ind).state = new_data.at(ent_ind).state;
        }
    }

    //! \brief should tell RenderModule how to render stuff
    // virtual void draw() = 0;
    virtual void update() = 0;
    virtual void updateSharedData(const std::array<SharedData, N_MAX_ENTITIES> &new_data, const std::vector<Entity> &active_entity_inds) = 0;
    virtual void communicate(std::array<SharedData, N_MAX_ENTITIES> &entity2shared_data) const = 0;
};

class IndHeap
{

    std::vector<int> data_;
    // std::priority_queue<int> pq;
public:
    IndHeap()
    {
        // pq = {data_.begin(), data_.end()};
        std::make_heap(data_.begin(), data_.end(), std::greater<>{});
    }
    IndHeap(size_t n_inds)
        : data_(n_inds)
    {
        // pq = {data_.begin(), data_.end()};
        std::iota(data_.begin(), data_.end(), 0);
        std::make_heap(data_.begin(), data_.end(), std::greater<>{});
    }

    void push(int num)
    {
        data_.push_back(num);
        std::push_heap(data_.begin(), data_.end(), std::greater<>{});
    }
    void pop()
    {
        std::pop_heap(data_.begin(), data_.end(), std::greater<>{});
        data_.pop_back();
    }
    int top()
    {
        return data_.front();
    }
};


template <ComponentID ID>
constexpr ComponentID component2id(const Component<ID> &comp)
{
    return ID;
}

struct TimeData
{

    float max_time = 0;
    float avg_time = 0;
    const int n_frames = 1000;

    std::vector<float> times;
    int first = 0;

    TimeData()
        : times(n_frames, 0) {}

    void addTime(float t)
    {
        avg_time = (avg_time * n_frames + t - times[first]) / n_frames;
        times[first] = t;
        first++;
        if (first >= n_frames)
        {
            first = 0;
        }
        max_time = std::max(t, max_time);
    }
};

struct TimeData2
{

    float max_time = 0;
    float avg_time = 0;
    float avg_time_sq = 0;
    float error_est_avg_time = 0;
    unsigned long long n_frames = 0;
    u_int_32_t block_size_;
    std::vector<float> times;
    int first = 0;
    unsigned long long n_blocks = 0;

    TimeData2(int block_size = 200)
        : block_size_(block_size), times(block_size, 0) {}

    void addTime(float t)
    {
        n_frames++;
        times[first] = t;
        first++;
        if (n_frames % block_size_ == 0)
        {
            const auto new_block_avg_time = std::accumulate(times.begin(), times.end(), 0) / block_size_;
            avg_time = (avg_time * n_blocks + new_block_avg_time) / (n_blocks + 1);
            avg_time_sq = (avg_time_sq * n_blocks + (new_block_avg_time * new_block_avg_time)) / (n_blocks + 1);
            std::fill(times.begin(), times.end(), 0);
            first = 0;
            n_blocks++;
        }
        max_time = std::max(t, max_time);
    }
};

class Edges;
struct ECSystem
{

    std::array<std::unique_ptr<System2>, MAX_COMPONENTS> systems2_;
    std::array<std::unique_ptr<TimeData2>, MAX_COMPONENTS> system_time_statistics_;
    std::array<std::string, MAX_COMPONENTS> system2name_;

    struct InitEntityData
    {
        Entity entity;
        int ind_in_initializer;
    };

    std::vector<Entity> to_add_;
    std::vector<Entity> to_remove_;

    std::unordered_map<ComponentID, std::string> comp_id2name_;
    std::unordered_map<std::type_index, int> comp_type_index2comp_id;
    int n_unique_components = 0;

    std::array<std::array<int, MAX_COMPONENTS>, N_MAX_ENTITIES> entity2component_inds;
    std::vector<int> active_entity_inds_;
    std::vector<Entity> active_entities_;
    std::array<int, N_MAX_ENTITIES> entity2ind_in_active_inds_;
    
    GayVectorI<N_MAX_ENTITIES> active_entities2_;

public:
    std::array<SharedData, N_MAX_ENTITIES> entity2shared_data;
    IndHeap free_entity_inds_;

    ECSystem(Edges &e);

    template <typename SystemType>
    SystemType &getSystem(ComponentID id)
    {
        return static_cast<SystemType &>(*systems2_.at(id));
    }

    template <class... Args>
    void initializeEntity(const Args &...components)
    {
        Entity new_entity(0);
        new_entity.ind = free_entity_inds_.top();
        free_entity_inds_.pop();
        active_entity_inds_.push_back(new_entity.ind);
        entity2ind_in_active_inds_.at(new_entity.ind) = active_entity_inds_.size() - 1;

        ([&components, &new_entity, this] 
        {
            //! Folded expressions... not really sure how this works tbh.
            //! this iterates over components and does the thing in curly brackets ... (black magic!)
            auto comp_id = component2id(components);
            new_entity.component_id.at(comp_id) = true;
            auto& system = systems2_.at(comp_id);
            system->addComponent(new_entity, components);
        }
         (), ...);

        

        active_entities_.push_back(new_entity);
    }

    void registerComponent(ComponentID new_comp_id, std::string name)
    {
        comp_id2name_[new_comp_id] = name;
        n_unique_components++;
    }

    void removeEntity(Entity ent)
    {
        for (auto &system : systems2_)
        {
            system->remove(ent);
            entity2component_inds.at(ent.ind).fill(-1);
        }
        
        entity2ind_in_active_inds_.at(active_entities_.back().ind) = entity2ind_in_active_inds_.at(ent.ind);
        active_entities_.at(entity2ind_in_active_inds_.at(ent.ind)) = active_entities_.back();
        active_entities_.pop_back();
        active_entity_inds_.at(entity2ind_in_active_inds_.at(ent.ind)) = active_entity_inds_.back();
        active_entity_inds_.pop_back();
        
        entity2ind_in_active_inds_.at(ent.ind) = -1;
        
        free_entity_inds_.push(ent.ind);
    }

    void update();

    // void draw(sf::RenderWindow &window)
    // {
    //     for (auto &system : systems2_)
    //     {
    //         system->draw(window);
    //     }
    // }

    void setMoveState(MoveState state, const std::vector<int>& selection){
        for(auto entity_ind : selection){
            entity2shared_data.at(entity_ind).state = state;
        }
    }

private:
    void removeQueuedEntities()
    {

    }
};

struct IComponentFactory
{
};

template <typename CompType>
struct ComponentFactory : IComponentFactory
{

    std::unique_ptr<CompType> blue_print;

    template <ComponentID id>
    CompType create()
    {
        return CompType();
    }
};




struct InteractionData
{
    sf::Vector2f dr;
    sf::Vector2f dv;
    float r_collision;
    float mass;
    BoidInd second = -1;
    MoveState state;
    u_int8_t player_ind;
    int n_neighbours;
};


