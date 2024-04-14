
#pragma once 

#include "../core.h"
#include <array>
#include <vector>



template <typename Type, int MAX_ENTITIES>
struct GayVector{
    std::array<int ,MAX_ENTITIES> entity2ind_in_vec;
    std::vector<Type> data;
    std::vector<int> vec2entity_ind;

    public:

        void removeEnt(int entity_ind);
        void remove(int vec_ind);
        void insert(Type datum, int entity_ind);
        size_t size(){
            return data.size();
        }
        void clear(){
            data.clear();
            for(auto entity_ind : vec2entity_ind){
                entity2ind_in_vec.at(entity_ind) = -1;
            }
            vec2entity_ind.clear();
        }


};


template <typename DataType, int MAX_ENTITIES>
void GayVector<DataType, MAX_ENTITIES>::removeEnt(int entity_ind)
{
    auto vec_ind = entity2ind_in_vec.at(entity_ind);
    auto &last_entity = vec2entity_ind.back();

    data.at(vec_ind) = data.back();
    data.pop_back();

    vec2entity_ind.at(vec_ind) = vec2entity_ind.back();
    vec2entity_ind.pop_back();

    entity2ind_in_vec.at(last_entity) = vec_ind;
    entity2ind_in_vec.at(entity_ind) = -1;
}

template <typename DataType, int MAX_ENTITIES>
void GayVector<DataType, MAX_ENTITIES>::remove(int vec_ind)
{
    auto entity_ind = vec2entity_ind.at(vec_ind);
    auto &last_entity = vec2entity_ind.back();

    data.at(vec_ind) = data.back();
    data.pop_back();

    vec2entity_ind.at(vec_ind) = vec2entity_ind.back();
    vec2entity_ind.pop_back();

    entity2ind_in_vec.at(last_entity) = vec_ind;
    entity2ind_in_vec.at(entity_ind) = -1;
}

template <typename DataType, int MAX_ENTITIES>
void GayVector<DataType, MAX_ENTITIES>::insert(DataType datum, int entity_ind)
{
    assert(entity_ind < N_MAX_NAVIGABLE_BOIDS);
    const auto new_vec_ind = data.size();

    data.push_back(datum);
    vec2entity_ind.push_back(entity_ind);
    entity2ind_in_vec.at(entity_ind) = new_vec_ind;
}


template <int MAX_ENTITIES>
struct GayVectorI{
    std::array<int ,MAX_ENTITIES> entity2ind_in_vec;
    std::vector<int> data;

    public:
        GayVectorI(){
            entity2ind_in_vec.fill(-1);
        }

        void removeEnt(int entity_ind);
        void remove(int vec_ind);
        void insert(int ind, int entity_ind);
        size_t size(){
            return data.size();
        }
        void clear(){
            data.clear();
            for(auto entity_ind : data){
                entity2ind_in_vec.at(entity_ind) = -1;
            }
        }

};



template <int MAX_ENTITIES>
void GayVectorI<MAX_ENTITIES>::removeEnt(int entity_ind)
{
    auto vec_ind = entity2ind_in_vec.at(entity_ind);
    auto &last_entity = data.back();
    if(vec_ind == -1){return;}
    data.at(vec_ind) = data.back();
    data.pop_back();

    entity2ind_in_vec.at(last_entity) = vec_ind;
    entity2ind_in_vec.at(entity_ind) = -1;
}

template <int MAX_ENTITIES>
void GayVectorI<MAX_ENTITIES>::remove(int vec_ind)
{
    auto entity_ind = data.at(vec_ind);
    auto &last_entity = data.back();

    data.at(vec_ind) = data.back();
    data.pop_back();

    entity2ind_in_vec.at(last_entity) = vec_ind;
    entity2ind_in_vec.at(entity_ind) = -1;
}

template <int MAX_ENTITIES>
void GayVectorI<MAX_ENTITIES>::insert(int datum, int entity_ind)
{
    assert(entity_ind < N_MAX_NAVIGABLE_BOIDS);
    const auto new_vec_ind = data.size();

    data.push_back(datum);
    entity2ind_in_vec.at(entity_ind) = new_vec_ind;
}
