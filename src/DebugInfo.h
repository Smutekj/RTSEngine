#ifndef DEBUGINFO_H
#define DEBUGINFO_H


#include <deque>

#include "core.h"
#include "Settings.h"
#include "Triangulation.h"


class Grid;
class BoidControler;
class Triangle;
class BoidWorld;

struct DebugInfoSettings2 : Settings {
    enum Options { DRAW_TRIANGLES = 0, DRAW_TRI_INDS, DRAW_VERT_INDS, DRAW_BUILDING_GRID, COUNT };

    enum Values {};
    std::array<bool, COUNT> options_;
    std::array<std::string, COUNT> option_names_;

    DebugInfoSettings2() {
        options_.fill(false);
        option_names_[Options::DRAW_TRIANGLES] = "triangles";
        option_names_[Options::DRAW_TRI_INDS] = "tri inds";
        option_names_[Options::DRAW_VERT_INDS] = "vert inds";
        option_names_[Options::DRAW_BUILDING_GRID] = "grid";
    }

    virtual void toggleOption(int o) override { options_[o] = !options_[o]; }

    virtual void setValue(int o, float new_value) override {}
    virtual float getValue(int o) override { return -1; }

    bool hasAttribute(Options o) const { return options_[o]; }


    virtual const std::string& getNameOfOption(int o) const override { return option_names_[o];}
    virtual const std::string& getNameOfValue(int o) const override { return option_names_.at(0);}
        virtual const int getCountValues() const override { return 0; }
    virtual const int getCountOptions() const override { return COUNT; }
};


//! \class draws stuff useful for debugging like:
//! \class triangulation: triangles, and indices
//! \class building grid
//! \class fps
class DebugInfo {


    std::vector<sf::RectangleShape> triangulation_lines_;
    int last_n_triangles_ = 0;

    DebugInfoSettings2 settings_;
    sf::Font font;
    sf::Clock clock;
    float last_time = 0;
    float n_frames_since_last = 0;
    sf::Text fps;
    sf::Text tri_ind_text;
    sf::Text n_active_boids;
    sf::Text n_triangles;
    sf::Text n_vertices;

    sf::View ui_view_;
    sf::RectangleShape boid_info_boundary_;

  public:
    DebugInfo();

    DebugInfoSettings2& getSettings() { return settings_; }

    void draw(sf::RenderWindow& window, const float fps_value, const Triangulation& cdt,
              const Grid& grid);
    void drawFunnel(sf::RenderWindow& window, const std::vector<std::pair<sf::Vector2f, sf::Vector2f>>& funnel);
    void drawPath(sf::RenderWindow& window, const std::deque<sf::Vector2f>& path);
    void drawBoidInfo(sf::RenderWindow& window, const int n_active);

    void onTriangulationUpdate(const Triangulation& cdt);

    friend Triangulation;

  private:
    void drawGrid(sf::RenderWindow& window, const Grid& grid);
    void drawEdge(sf::RenderWindow& window, sf::Vector2i from, sf::Vector2i to, sf::Color color = sf::Color::Green);

    void drawTriangles(sf::RenderWindow& window, const std::vector<Triangle>& triangles,
                       const std::vector<Vertex>& vertices, const Triangulation& cdt);

    void drawCircumCircles(sf::RenderWindow& window, const std::vector<Triangle>& triangles,
                           const std::vector<Vertex>& vertices);
    void drawTriangleInds(sf::RenderWindow& window, const std::vector<Triangle>& triangles,
                          const std::vector<Vertex>& vertices);

    void drawVertInds(sf::RenderWindow& window, const std::vector<Triangle>& triangles,
                      const std::vector<Vertex>& vertices);

    void drawCDT(sf::RenderWindow& window, const Triangulation& cdt);

    void drawFPS(sf::RenderWindow& window, const float fps_value);

    // void drawBoidControler(sf::RenderWindow& window, const BoidControler& controler);
};

#endif // DEBUGINFO_H
