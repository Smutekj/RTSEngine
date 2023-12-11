#include "DebugInfo.h"
#include "Triangulation.h"
#include "Grid.h"
#include "BoidControler.h"
#include <filesystem>

DebugInfo::DebugInfo() {
    const std::string font_path{std::filesystem::current_path()};
    if (!font.loadFromFile(font_path + "/../Resources/arial.ttf")) {
        throw std::runtime_error("no font file at " + font_path);
    }

    n_active_boids.setFillColor(sf::Color::Green);
    n_active_boids.setFont(font);
    n_active_boids.setCharacterSize(15);
    n_active_boids.setScale(0.7, 0.7);
    n_active_boids.setPosition(0.f, 30.f);

    fps.setFillColor(sf::Color::Red);
    fps.setFont(font);
    fps.setCharacterSize(15);
    fps.setScale(0.7, 0.7);
    fps.setPosition(0, 0);

    n_triangles.setFillColor(sf::Color::Green);
    n_triangles.setFont(font);
    n_triangles.setCharacterSize(15);
    n_triangles.setScale(0.7, 0.7);
    n_triangles.setPosition(0, 15);

    ui_view_.setViewport(sf::FloatRect(0.9f, 0.0f, 0.1f, 0.2f));
    ui_view_.setCenter(50, 50);
    ui_view_.setSize(100, 100);

    tri_ind_text.setFillColor(sf::Color::Green);
    tri_ind_text.setFont(font);
    tri_ind_text.setCharacterSize(24);
    tri_ind_text.scale(0.5f, 0.5f);

    boid_info_boundary_.setSize(ui_view_.getSize());
    boid_info_boundary_.setOutlineThickness(5);
    boid_info_boundary_.setOutlineColor(sf::Color::Black);
    boid_info_boundary_.setFillColor({0, 1, 1, 69});
    boid_info_boundary_.setPosition(-75, 0);
}

void DebugInfo::drawBoidControler(sf::RenderWindow& window, const BoidControler& controler) {

    sf::Text n_active_boids;

    const auto& world = controler.world_;

    sf::RectangleShape arrow1;
    arrow1.setSize({1, 20});
    arrow1.setFillColor(sf::Color::Black);
}

void DebugInfo::draw(sf::RenderWindow& window, const float fps, const int n_active_boids, const Triangulation& cdt,
                     const Grid& grid, BoidControler& controler) {
    drawCDT(window, cdt);
    if (settings_.hasAttribute(DebugInfoSettings2::Options::DRAW_BUILDING_GRID)) {
        drawGrid(window, grid);
    }
    drawBoidInfo(window, n_active_boids);
    drawFPS(window, fps);
}

void DebugInfo::drawFunnel(sf::RenderWindow& window, const std::vector<std::pair<sf::Vector2f, sf::Vector2f>>& funnel) {
    sf::RectangleShape line_right;
    sf::RectangleShape line_left;
    line_right.setFillColor(sf::Color::Black);
    line_left.setFillColor(sf::Color::Red);

    for (int i = 1; i < funnel.size(); ++i) {
        const auto dr_right = funnel[i].first - funnel[i - 1].first;
        const auto dr_left = funnel[i].second - funnel[i - 1].second;
        const auto drr_norm = std::sqrt(dot(dr_right, dr_right));
        const auto drl_norm = std::sqrt(dot(dr_left, dr_left));
        const auto angle_right =
            180.f / (M_PIf)*std::acos(dot(dr_right / drr_norm, {0, 1})) * (2.f * (dr_right.x < 0.f) - 1.f);
        const auto angle_left =
            180.f / (M_PIf)*std::acos(dot(dr_left / drl_norm, {0, 1})) * (2.f * (dr_left.x < 0.f) - 1.f);
        line_right.setPosition(funnel[i - 1].first);
        line_left.setPosition(funnel[i - 1].second);
        line_right.setSize({3, drr_norm});
        line_left.setSize({3, drl_norm});
        line_right.setRotation(angle_right);
        line_left.setRotation(angle_left);
        window.draw(line_right);
        window.draw(line_left);
    }
}

void DebugInfo::drawGrid(sf::RenderWindow& window, const Grid& grid) {
    sf::RectangleShape line;
    line.setFillColor(sf::Color::Blue);

    int n = grid.n_cells_.x;
    int m = grid.n_cells_.y;
    auto dx = grid.cell_size_.x;
    auto dy = grid.cell_size_.y;
    for (int i = 0; i < n + 1; ++i) {
        const auto x_coord = dx * i;
        const auto y_coord = 0.f;
        const sf::Vector2f r0 = {x_coord, y_coord};
        const sf::Vector2f r1 = {x_coord, m * dy};
        const auto dr = r1 - r0;
        const auto dr_norm = std::sqrt(dot(dr, dr));
        const auto angle = 180.f / (M_PIf)*std::acos(dot(dr / dr_norm, {0, 1})) * (2.f * (dr.x < 0.f) - 1.f);
        line.setPosition(r0);
        line.setSize({1, dr_norm});
        line.setRotation(angle);
        window.draw(line);
    }
    for (int i = 0; i < m + 1; ++i) {
        const auto x_coord = 0.f;
        const auto y_coord = i * dy;
        const sf::Vector2f r0 = {x_coord, y_coord};
        const sf::Vector2f r1 = {n * dx, y_coord};
        const auto dr = r1 - r0;
        const auto dr_norm = std::sqrt(dot(dr, dr));
        const auto angle = 180.f / (M_PIf)*std::acos(dot(dr / dr_norm, {0, 1})) * (2.f * (dr.x < 0.f) - 1.f);
        line.setPosition(r0);
        line.setSize({1, dr_norm});
        line.setRotation(angle);
        window.draw(line);
    }
}

void DebugInfo::drawEdge(sf::RenderWindow& window, sf::Vector2i from, sf::Vector2i to, sf::Color color) {
    sf::RectangleShape line;
    line.setFillColor(color);
    sf::Vector2f t = sf::Vector2f(to - from);
    auto l = std::sqrt(dot(t, t));
    t /= l;

    line.setSize({0.5f, l});
    line.setPosition(from.x, from.y);
    auto angle = 180.f / (M_PIf)*std::acos(dot(t, {0, 1})) * (2.f * (t.x < 0.f) - 1.f);
    line.setRotation(angle);
    window.draw(line);
}

void DebugInfo::drawTriangles(sf::RenderWindow& window, const std::vector<Triangle>& triangles,
                              const std::vector<Vertex>& vertices, const Triangulation& cdt) {

    if (triangles.size() != last_n_triangles_) { //! if the triangulation got updated
        onTriangulationUpdate(cdt);
    }

    const auto current_view = window.getView();
    auto is_in_view = [&](const sf::Vector2f& r) {
        const auto top_left = current_view.getCenter() - current_view.getSize() / 2.f;
        const auto bottom_right = current_view.getCenter() + current_view.getSize() / 2.f;
        return r.x <= bottom_right.x and r.x >= top_left.x and r.y <= bottom_right.y and r.y >= top_left.y;
    };

    for (const auto& line : triangulation_lines_) {
        if (is_in_view(line.getPosition()) or is_in_view(line.getPosition() + line.getSize())) {
            window.draw(line);
        }
    }
}
void DebugInfo::drawTriangleInds(sf::RenderWindow& window, const std::vector<Triangle>& triangles,
                                 const std::vector<Vertex>& vertices) {

    sf::ConvexShape sf_triangle;
    TriInd tri_ind = 0;
    for (auto& triangle : triangles) {
        auto v0 = triangle.verts[0];
        auto v1 = triangle.verts[1];
        auto v2 = triangle.verts[2];

        auto v_t = (v0 + v1 + v2) / 3;
        tri_ind_text.setPosition(sf::Vector2f(v_t));
        tri_ind_text.setString(std::to_string(tri_ind));
        tri_ind++;
        window.draw(tri_ind_text);
    }
}

void DebugInfo::drawVertInds(sf::RenderWindow& window, const std::vector<Triangle>& triangles,
                             const std::vector<Vertex>& vertices) {

    sf::ConvexShape sf_triangle;
    VertInd vert_ind = 0;
    tri_ind_text.setFillColor(sf::Color::Magenta);
    for (auto& vertex : vertices) {
        tri_ind_text.setPosition(sf::Vector2f(vertex));
        tri_ind_text.setString(std::to_string(vert_ind));
        vert_ind++;
        window.draw(tri_ind_text);
    }
}

void DebugInfo::drawCDT(sf::RenderWindow& window, const Triangulation& cdt) {

    const auto old_view = window.getView();
    window.setView(ui_view_);
    n_triangles.setString("# triangles: " + std::to_string(cdt.triangles_.size()));
    window.draw(n_triangles);
    window.setView(old_view);

    const auto& triangles = cdt.triangles_;
    if (settings_.hasAttribute(DebugInfoSettings2::Options::DRAW_TRIANGLES)) {
        drawTriangles(window, triangles, cdt.vertices_, cdt);
    }
    if (settings_.hasAttribute(DebugInfoSettings2::Options::DRAW_TRI_INDS)) {
        drawTriangleInds(window, triangles, cdt.vertices_);
    }
    if (settings_.hasAttribute(DebugInfoSettings2::Options::DRAW_VERT_INDS)) {
        drawVertInds(window, triangles, cdt.vertices_);
    }
}

void DebugInfo::onTriangulationUpdate(const Triangulation& cdt) {
    const auto& vertices = cdt.vertices_;

    triangulation_lines_.reserve(cdt.triangles_.size());
    triangulation_lines_.clear();

    sf::RectangleShape line;
    line.setFillColor(sf::Color::Green);

    for (const auto& tri : cdt.triangles_) {
        for (int k = 0; k < 3; ++k) {
            tri.is_constrained[k] ? line.setFillColor(sf::Color::Red) : line.setFillColor(sf::Color::Red);
            line.setFillColor(sf::Color::Green);
            const auto v1 = asFloat(tri.verts[k]);
            const auto v2 = asFloat(tri.verts[next(k)]);
            const auto v = v2 - v1;
            line.setPosition(v1);
            line.setSize({0.5, norm(v2 - v1)});
            line.setRotation(angle(v) - 90);
            triangulation_lines_.push_back(line);
        }
    }
    last_n_triangles_ = cdt.triangles_.size();
}

void DebugInfo::drawFPS(sf::RenderWindow& window, const float fps_value) {

    const auto old_view = window.getView();
    window.setView(ui_view_);
    n_frames_since_last++;
    float current_time = clock.getElapsedTime().asSeconds();
    auto dt = (current_time - last_time);
    if (dt > 1.0) {
        last_time = 0;
        clock.restart();
        // int fps_value = std::floor(n_frames_since_last / dt);
        fps.setString("fps: " + std::to_string(fps_value));
        n_frames_since_last = 0;
    }
    // std::cout << "current time is: " << current_time << "\n";
    window.draw(fps);
    window.setView(old_view);
}

void DebugInfo::drawBoidInfo(sf::RenderWindow& window, const int n_active) {

    n_active_boids.setString("n_boids: " + std::to_string(n_active));

    const auto old_view = window.getView();
    window.setView(ui_view_);
    window.draw(n_active_boids);
    window.setView(old_view);

    // sf::RectangleShape arrow1;
    // arrow1.setSize({1, 20});
    // arrow1.setFillColor(sf::Color::Green);

    // for (int i : world.active_inds) {
    //     sf::Vector2f v = world.velocities_[i];
    //     auto r = world.boid_inds2draw_data_[i].circle.getRadius();
    //     if (std::sqrt(dot(v, v)) != 0) {
    //         v /= std::sqrt(dot(v, v));
    //         auto angle = 180.f / (M_PIf)*std::acos(dot(v, {0, 1})) * (2.f * (v.x < 0.f) - 1.f);
    //         arrow1.setRotation(angle);
    //         arrow1.setPosition(world.r_coords_[i]);
    //         arrow1.move(r + 0.5, r);
    //         //            window.draw(arrow1);
    //     }
    //     arrow1.setPosition(world.r_coords_[i]);
    //     //        arrow1.move(r + 0.5, r);
    //     arrow1.setRotation(bc.orientation_[i] - 90);
    //     arrow1.setFillColor(sf::Color::Magenta);
    //     window.draw(arrow1);
    // }
}

void DebugInfo::drawPath(sf::RenderWindow& window, const std::deque<sf::Vector2f>& path) {
    sf::RectangleShape line;
    line.setSize({3, 50});
    line.setFillColor(sf::Color::Cyan);
    sf::CircleShape node;
    node.setRadius(3.f);
    node.setFillColor(sf::Color::Cyan);

    for (int i = 1; i < path.size(); ++i) {
        const auto dr = path.at(i) - path.at(i - 1);
        const auto dr_norm = std::sqrt(dot(dr, dr));
        const auto angle = 180.f / (M_PIf)*std::acos(dot(dr / dr_norm, {0, 1})) * (2.f * (dr.x < 0.f) - 1.f);
        line.setPosition(path.at(i - 1));
        line.setSize({3, dr_norm});
        line.setRotation(angle);

        node.setPosition(path.at(i));

        window.draw(line);
        window.draw(node);
    }
}
