#ifndef BOIDS_COMMANDMANAGER_H
#define BOIDS_COMMANDMANAGER_H

#include "core.h"
#include <memory>
#include <deque>

class Command{
    std::vector<int> selection_; 
    std::vector<const sf::Vector2f>& r_coords;

    virtual void issue() const;
    virtual bool isFinished()const;
};

class MoveCommand : Command{

    sf::Vector2f r_target;


};

class AttackMoveCommand : Command{

    
    sf::Vector2f r_target;
};

class AttackUnitCommand : Command{

    
    sf::Vector2f r_target;
};

class AttackUnitCommand : Command{

    int target_boid_ind;
    sf::Vector2f r_target;

    
};


#include "FogOfWar.h"

class CommandManager{

    std::deque<std::shared_ptr<Command>> commands_;

    

};






#endif // BOIDS_COMMANDMANAGER_H