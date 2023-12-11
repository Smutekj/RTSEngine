#ifndef BOIDS_SETTINGS_H
#define BOIDS_SETTINGS_H

#include <string>

struct Settings {

    virtual ~Settings() = default;
    virtual void toggleOption(int option_ind) = 0;
    virtual void setValue(int value_ind, float new_value) = 0;
    virtual float getValue(int value_ind) = 0;
    virtual const std::string& getNameOf(int value_ind) const = 0;
    virtual const int getCount() const = 0;
};

#endif