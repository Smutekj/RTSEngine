#include "Settings.h"

FogOfWarSettings::FogOfWarSettings()
{
    option_names_.at(Options::FOGOFWAR) = "Show/hide fow";
    option_names_.at(Options::REVEAL) = "Show/hide map";
    options_.fill(true);

    value_names_[Values::ALPHA] = "alpha";
    values_.fill(1);
}

void FogOfWarSettings::toggleOption(int o)
{
    options_.at(o) = !options_.at(o);
}

void FogOfWarSettings::setValue(int v, float new_value)
{
    values_.at(v) = new_value;
}
float FogOfWarSettings::getValue(int v)
{
    return values_.at(v);
}

bool FogOfWarSettings::hasAttribute(Options o) const { return options_[o]; }

const std::string &FogOfWarSettings::getNameOfOption(int o) const
{
    return option_names_.at(o);
}
const std::string &FogOfWarSettings::getNameOfValue(int o) const
{
    return value_names_.at(o);
}

const int FogOfWarSettings::getCountValues() const { return COUNT_VALS; }
const int FogOfWarSettings::getCountOptions() const { return COUNT_OPTS; }

SeekSystemSettings::SeekSystemSettings()
{

    value_names_[Values::FINISH_ANGLE] = "finish angle";
    value_names_[Values::FINISH_RADIUS] = "finish radius";
    value_names_[Values::FINISH_N_STEPS] = "finish delay";
    value_names_[Values::FINISH_CONE_LEN] = "cone len";

    values_[Values::FINISH_ANGLE] = 36;
    values_[Values::FINISH_RADIUS] = 50;
    values_[Values::FINISH_N_STEPS] = 1;
    values_[Values::FINISH_CONE_LEN] = 10;
}

void SeekSystemSettings::toggleOption(int o)
{
    options_.at(o) = !options_.at(o);
}

void SeekSystemSettings::setValue(int v, float new_value)
{
    values_.at(v) = new_value;
}
float SeekSystemSettings::getValue(int v)
{
    return values_.at(v);
}

bool SeekSystemSettings::hasAttribute(Options o) const { return options_[o]; }

const std::string &SeekSystemSettings::getNameOfOption(int o) const
{
    return option_names_.at(o);
}
const std::string &SeekSystemSettings::getNameOfValue(int o) const
{
    return value_names_.at(o);
}

const int SeekSystemSettings::getCountValues() const { return COUNT_VALS; }
const int SeekSystemSettings::getCountOptions() const { return COUNT_OPTS; }