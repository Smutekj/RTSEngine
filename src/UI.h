#ifndef BOIDS_UI_H
#define BOIDS_UI_H

#include <filesystem>
#include "core.h"
#include "Settings.h"

class Game;
class DebugInfo;
class FogOfWarV2;
class BoidControler;

class ClickableBox {

  protected:
    bool has_been_clicked_ = false;
    sf::RectangleShape bounding_box_;
    sf::Text info_text_;

  public:
    ClickableBox(sf::Vector2f position, sf::Vector2f size, std::string text = "");
    bool contains(const sf::Vector2f& r);
};

class Widget : public ClickableBox {
  protected:
    Settings* p_controled_settings_;
    int controled_ind_in_settings = -1;
    bool is_active = false;

  public:
    Widget(sf::Vector2f position, sf::Vector2f size, const std::string& text, int controled_ind, Settings* p_settings)
        : ClickableBox(position, size, text)
        , p_controled_settings_(p_settings)
        , controled_ind_in_settings(controled_ind) {}
    virtual void onClick(sf::RenderWindow& window){};
    virtual void draw(sf::RenderWindow& window) = 0;
    virtual void onRelease(sf::RenderWindow& window) = 0;
    virtual void onMouseButtonHold(sf::RenderWindow& window) = 0;
    virtual void onKeyRelease() = 0;
    virtual void onKeyPress(const sf::Keyboard::Key& pressed_key) = 0;
};

class Slider2 : public Widget {

    float min_value_ = 0;
    float max_value_ = 1;

    sf::RectangleShape slider_rectangle_;
    sf::Text value_text_;

  public:
    Slider2(sf::Vector2f center, sf::Vector2f size, std::string text, const sf::Font& font, int ind_in_settings,
            Settings* controled_settings, float min_value = 0, float max_value = 1);

    ~Slider2() = default;

    virtual void onRelease(sf::RenderWindow& window) override;

    virtual void onKeyRelease() override;
    virtual void onKeyPress(const sf::Keyboard::Key& pressed_key) override;

    virtual void onClick(sf::RenderWindow& window) override;

    virtual void draw(sf::RenderWindow& window) override ;

    virtual void onMouseButtonHold(sf::RenderWindow& window) override;
};

class Button2 : public Widget {

    bool is_active = false;

  public:
    Button2(sf::Vector2f center, sf::Vector2f size, std::string text, const sf::Font& font, int ind_in_settings,
            Settings* controled_settings);

    Button2(const Button2& b);

    virtual void draw(sf::RenderWindow& window) override;
    virtual void onClick(sf::RenderWindow& window) override;
    virtual void onRelease(sf::RenderWindow& window) override;

    virtual void onMouseButtonHold(sf::RenderWindow& window) override;
    virtual void onKeyRelease() override;
    virtual void onKeyPress(const sf::Keyboard::Key& pressed_key) override;
};

#include <iostream>

class NumberField : public Widget {

    int frames_since_last_seen_;
    int max_n_digits_ = 10;
    sf::RectangleShape insertion_point_line_;
    sf::Text value_text_;
    std::string old_value_;
    bool key_pressed_ = false;
    bool number_has_point = true;

    float min_value_ = -MAXFLOAT;
    float max_value_ = MAXFLOAT;

  public:
    NumberField(sf::Vector2f center, sf::Vector2f size, std::string text, const sf::Font& font, int ind_in_settings,
                Settings* controled_settings, float min_value = 0, float max_value = 1);

    virtual void onMouseButtonHold(sf::RenderWindow& window) override;

    virtual void onClick(sf::RenderWindow& window) override;

    virtual void onRelease(sf::RenderWindow& window) override;

    void update();
    virtual void onKeyRelease() override ;

    virtual void onKeyPress(const sf::Keyboard::Key& pressed_key) override;

    virtual void draw(sf::RenderWindow& window) override;
};

template <class WidgetType> class PopUpWindow : public Widget {

    std::vector<std::unique_ptr<Widget>> widgets_;

    sf::View view_;
    bool is_opened = false;
    sf::RectangleShape opened_window_box_;

  public:
    PopUpWindow(sf::Vector2f center, sf::Vector2f button_size, std::string text, const sf::Font& font,
                Settings* settings)
        : Widget(center, button_size, text, 0, settings) {

        view_.setViewport(sf::FloatRect(0.0f, 0.5f, 0.1f, 0.4f));

        const auto n_settings = settings->getCountValues();
        const auto n_options = settings->getCountOptions();

        sf::Vector2f window_dimension = {button_size.x, button_size.y * (n_settings+n_options)};
        view_.setSize(window_dimension);
        view_.setCenter(window_dimension / 2.f);

        sf::Vector2f upper_left_pos = {0, 0};
        for (int i = 0; i < n_settings; ++i) {
            std::unique_ptr<Widget> new_widget(
                new WidgetType(upper_left_pos, button_size, settings->getNameOfValue(i), font, i, settings));
            widgets_.push_back(std::move(new_widget));
            upper_left_pos.y += button_size.y;
        }

        for (int i = 0; i < n_options; ++i) { //! options need always buttons
            std::unique_ptr<Button2> new_widget(
                new Button2(upper_left_pos, button_size, settings->getNameOfOption(i), font, i, settings));
            widgets_.push_back(std::move(new_widget));
            upper_left_pos.y += button_size.y;
        }

        const auto scale = 0.25 / 8. * n_settings;

        info_text_.setFillColor(sf::Color::Black);
        info_text_.setPosition(bounding_box_.getPosition().x, bounding_box_.getPosition().y);
        info_text_.setScale(0.25, 0.25);
        info_text_.setString(text);
        info_text_.setFont(font);

        bounding_box_.setOutlineThickness(1);
        bounding_box_.setOutlineColor(sf::Color::Black);
        bounding_box_.setFillColor({0, 0, 1, 69});
        opened_window_box_.setFillColor({0, 0, 1, 69});
        opened_window_box_.setOutlineColor(sf::Color::Black);
        opened_window_box_.setOutlineThickness(1);
        opened_window_box_.setSize({100.f, 100.f});
        opened_window_box_.setPosition(view_.getCenter() - view_.getSize() / 2.0f);
    }

    virtual void draw(sf::RenderWindow& window) override {
        if (is_opened) {
            const auto old_view = window.getView();
            window.setView(view_);
            for (auto& widget : widgets_) {
                widget->draw(window);
            }
            window.draw(opened_window_box_);
            window.setView(old_view);
            window.draw(bounding_box_);
        } else {
            window.draw(bounding_box_);
            window.draw(info_text_);
        }
    }

    virtual void onClick(sf::RenderWindow& window) override {
        const auto mouse_coords = window.mapPixelToCoords(sf::Mouse::getPosition(window), view_);

        const auto lb = opened_window_box_.getLocalBounds();
        const auto gb = opened_window_box_.getGlobalBounds();
        const auto old_view = window.getView();

        if (is_opened) {
            window.setView(view_);
            if (gb.contains(mouse_coords)) {
                for (auto& widget : widgets_) {
                    widget->onClick(window);
                }
            }
            window.setView(old_view);
        }
    }

    virtual void onMouseButtonHold(sf::RenderWindow& window) override {
        const auto mouse_coords = window.mapPixelToCoords(sf::Mouse::getPosition(window), view_);

        const auto old_view = window.getView();

        if (is_opened) {
            window.setView(view_);
            if (opened_window_box_.getGlobalBounds().contains(mouse_coords)) {
                for (auto& widget : widgets_) {
                    widget->onMouseButtonHold(window);
                }
            }
            window.setView(old_view);
        }
    }

    virtual void onRelease(sf::RenderWindow& window) override {

        const auto old_view = window.getView();

        const auto lb = opened_window_box_.getLocalBounds();
        const auto gb = opened_window_box_.getGlobalBounds();

        const auto mouse_coords = window.mapPixelToCoords(sf::Mouse::getPosition());
        const auto mouse_coords2 = window.mapPixelToCoords(sf::Mouse::getPosition(window));
        const auto mouse_coords3 = window.mapPixelToCoords(sf::Mouse::getPosition(window), view_);
        if (is_opened) {
            for (auto& widget : widgets_) {
                window.setView(view_);
                widget->onRelease(window);
            }
        }
        if (bounding_box_.getGlobalBounds().contains(mouse_coords2)) {
            is_opened = !is_opened;
        }
        window.setView(old_view);
    }

    virtual void onKeyPress(const sf::Keyboard::Key& pressed_key) override {
        for (auto& widget : widgets_) {
            widget->onKeyPress(pressed_key);
        }
        if (pressed_key == sf::Keyboard::Escape) {
            is_opened = false;
        }
    }

    virtual void onKeyRelease() override {
        for (auto& widget : widgets_) {
            widget->onKeyRelease();
        }
    }
};

class UI {

    sf::Vector2f mouse_coords_on_click_;
    std::vector<std::unique_ptr<Widget>> widgets_;

    sf::RectangleShape control_panel_boundary_;

    sf::Font font;
    sf::Vector2f button_size_ = {50, 10};

    sf::View ui_view_;

  public:
    UI(Game& game, FogOfWarV2& fow, DebugInfo& dbg, BoidControler& bc);

    void draw(sf::RenderWindow& window);

    void onRelease(sf::RenderWindow& window);

    void onMouseHold(sf::RenderWindow& window);

    void onKeyRelease(sf::RenderWindow& window);

    void onKeyPress(const sf::Keyboard::Key& pressed_key);

    void onClick(sf::RenderWindow& window);
};

#endif // BOIDS_UI_H
