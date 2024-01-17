#ifndef BOIDS_SETTINGS_H
#define BOIDS_SETTINGS_H

#include <string>
#include <array>

struct Settings {

    // virtual ~Settings() = default;
    virtual void toggleOption(int option_ind) = 0;
    virtual void setValue(int value_ind, float new_value) = 0;
    virtual float getValue(int value_ind) = 0;
    virtual const std::string& getNameOfOption(int option_ind) const = 0;
    virtual const std::string& getNameOfValue(int value_ind) const = 0;
    virtual const int getCountOptions() const = 0;
    virtual const int getCountValues() const = 0;
};


struct FogOfWarSettings : Settings {
    enum Options {
        REVEAL,
        FOGOFWAR,
        COUNT_OPTS,
    };

    enum Values {
        ALPHA,
        COUNT_VALS,
    };

    std::array<bool, COUNT_OPTS> options_;
    std::array<float, COUNT_VALS> values_;
    std::array<std::string, COUNT_OPTS> option_names_;
    std::array<std::string, COUNT_VALS> value_names_;

    FogOfWarSettings();
    ~FogOfWarSettings() = default;
    virtual void toggleOption(int o) override;

    virtual void setValue(int v, float new_value) override ;
    virtual float getValue(int v) override;

    bool hasAttribute(Options o) const;

    virtual const std::string& getNameOfOption(int o) const override;
    virtual const std::string& getNameOfValue(int o) const override;

    virtual const int getCountValues() const override;
    virtual const int getCountOptions() const override;
};



struct SeekSystemSettings : Settings {
    enum Options {
        SHOW_PATH,
        SHOW_PORTALS,
        SHOW_FUNNEL,
        COUNT_OPTS,
    };

    enum Values {
        FINISH_ANGLE,
        FINISH_RADIUS,
        FINISH_N_STEPS,
        FINISH_CONE_LEN,
        COUNT_VALS
    };

    std::array<bool, COUNT_OPTS> options_;
    std::array<float, COUNT_VALS> values_;
    std::array<std::string, COUNT_OPTS> option_names_;
    std::array<std::string, COUNT_VALS> value_names_;

    SeekSystemSettings();
    ~SeekSystemSettings() = default;
    virtual void toggleOption(int o) override;

    virtual void setValue(int v, float new_value) override ;
    virtual float getValue(int v) override;

    bool hasAttribute(Options o) const;

    virtual const std::string& getNameOfOption(int o) const override;
    virtual const std::string& getNameOfValue(int o) const override;

    virtual const int getCountValues() const override;
    virtual const int getCountOptions() const override;
};



#endif