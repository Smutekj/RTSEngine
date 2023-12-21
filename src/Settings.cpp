#include "Settings.h"


FogOfWarSettings::FogOfWarSettings() {
    option_names_.at(Options::FOGOFWAR) = "Show/hide fow";
    option_names_.at(Options::REVEAL) = "Show/hide map";
    options_.fill(true);

    value_names_[Values::ALPHA] = "alpha";        
    values_.fill(1);
}

    void FogOfWarSettings::toggleOption(int o)  {
    options_.at(o) = !options_.at(o);
}

    void FogOfWarSettings::setValue(int v, float new_value)  {
    values_.at(v) = new_value;
}
    float FogOfWarSettings::getValue(int v)  {
    return values_.at(v);
}

bool FogOfWarSettings::hasAttribute(Options o) const { return options_[o]; }

    const std::string& FogOfWarSettings::getNameOfOption(int o) const  {
        return option_names_.at(o);
    }
    const std::string& FogOfWarSettings::getNameOfValue(int o) const  {
        return value_names_.at(o);
    }

    const int FogOfWarSettings::getCountValues() const  { return COUNT_VALS; }
    const int FogOfWarSettings::getCountOptions() const  { return COUNT_OPTS; }