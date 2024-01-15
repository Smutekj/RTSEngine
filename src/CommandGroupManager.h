#pragma once
#include <set>
#include <unordered_set>
#include "core.h"

//! \class knows which agents belong to which command group
//! \class
struct CommandGroupManager
{
    //! Stuff for differnt groups sharing same command
    std::vector<int> boidind2_command_group_;
    std::set<int> inactive_group_inds = {0};

    std::vector<std::unordered_set<int>> group2_indices_;
    std::vector<std::unordered_set<int>> group2_finished_indices_;
    std::vector<float> group2finished_surface_area_;

    CommandGroupManager() : boidind2_command_group_(N_MAX_NAVIGABLE_BOIDS, -1) {}

    void removeFromGroup(const BoidInd ind)
    {

        const auto command_group_ind = boidind2_command_group_[ind];
        if (command_group_ind != -1)
        {
            group2_finished_indices_.at(command_group_ind).insert(ind);
            group2_indices_.at(command_group_ind).erase(ind);
            group2finished_surface_area_.at(command_group_ind) += M_PIf * RHARD * RHARD;

            if (group2_indices_.at(command_group_ind).size() == 0)
            {
                inactive_group_inds.insert(command_group_ind);
                n_active_groups--;
                updateInactives();
            }
        }
    }

    float getFinishedAreaOf(int comp_ind)
    {
        const auto group_ind = boidind2_command_group_.at(comp_ind);

        if (group_ind == -1)
        {
            return 0;
        }
        return group2finished_surface_area_.at(group_ind);
    }

    bool isSameGroup(const BoidInd i1, const BoidInd i2)
    {
        return boidind2_command_group_[i1] == boidind2_command_group_[i2];
    }

    void addGroup(const std::vector<BoidInd> &selection)
    {

        assert(inactive_group_inds.size() > 0);
        const int new_group_ind = *(inactive_group_inds.begin());
        inactive_group_inds.erase(new_group_ind);
        n_active_groups++;
        for (const auto selected : selection)
        {
            const auto old_group_ind = boidind2_command_group_.at(selected);
            assert(old_group_ind != new_group_ind);
            boidind2_command_group_.at(selected) = new_group_ind;
            if (old_group_ind != -1)
            {
                group2_finished_indices_.at(old_group_ind).erase(selected);
                group2finished_surface_area_.at(old_group_ind) -= M_PIf * RHARD * RHARD;
                group2_indices_.at(old_group_ind).erase(selected);
                boidind2_command_group_.at(selected) = new_group_ind;
            }
        }

        if (new_group_ind == group2_indices_.size()) //! new group does not exist yet
        {
            assert(inactive_group_inds.size() == 0);
            group2_indices_.emplace_back(selection.begin(), selection.end());
            group2_finished_indices_.emplace_back();
            group2finished_surface_area_.emplace_back(0);
        }
        else
        {
            assert(inactive_group_inds.size() != 0);
            group2_indices_.at(new_group_ind) = {selection.begin(), selection.end()};
            group2_finished_indices_.at(new_group_ind) = {};
            group2finished_surface_area_.at(new_group_ind) = 0;
        }

        if (inactive_group_inds.size() == 0)
        {
            int next_inactive_group_ind = group2_indices_.size();
            inactive_group_inds.insert(next_inactive_group_ind);
        }
    }

    void updateInactives()
    {
        for (int group_ind = 0; group_ind < group2_indices_.size(); group_ind++)
        {

            if (group2_indices_.at(group_ind).size() == 0) //! when group is empty
            {
                for (auto finished_ind : group2_finished_indices_.at(group_ind))
                {
                    boidind2_command_group_[finished_ind] = -1;
                }
                group2_finished_indices_.at(group_ind).clear();
                group2finished_surface_area_.at(group_ind) = 0;
            }
        }
    }

private:
    int n_active_groups = 0;
    int next_group_ind = 0;
};
