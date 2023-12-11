#include "UI.h"


    ClickableBox::ClickableBox(sf::Vector2f position, sf::Vector2f size, std::string text)
        : bounding_box_(size) {
        bounding_box_.setPosition(position);
        bounding_box_.setOutlineColor(sf::Color::Black);
        bounding_box_.setOutlineThickness(1);
        info_text_.setString(text);
        info_text_.setPosition(bounding_box_.getPosition());
        info_text_.setFillColor(sf::Color::Black);
        info_text_.setScale(0.25, 0.25);
        //        info_text_.setCharacterSize(
    }

    bool ClickableBox::contains(const sf::Vector2f& r) { return bounding_box_.getGlobalBounds().contains(r); }


    Slider2::Slider2(sf::Vector2f center, sf::Vector2f size, std::string text, const sf::Font& font, int ind_in_settings,
            Settings* controled_settings, float min_value, float max_value)
        : Widget(center, size, text, ind_in_settings, controled_settings)
        , min_value_(min_value)
        , max_value_(max_value) {

        slider_rectangle_.setSize({size.y / 3.f, size.y});
        slider_rectangle_.setPosition(bounding_box_.getPosition().x + bounding_box_.getSize().x,
                                      bounding_box_.getPosition().y);

        slider_rectangle_.setOutlineColor(sf::Color::Blue);
        slider_rectangle_.setFillColor(sf::Color::Blue);

        value_text_.setFillColor(sf::Color::Black);
        value_text_.setScale(0.25, 0.25);
        value_text_.setPosition(bounding_box_.getPosition().x + bounding_box_.getSize().x * 1.15f,
                                bounding_box_.getPosition().y - slider_rectangle_.getSize().y);
        value_text_.setString("1.000");
        value_text_.setFont(font);
        info_text_.setFillColor(sf::Color::Black);
        info_text_.setPosition(bounding_box_.getPosition().x, bounding_box_.getPosition().y);
        info_text_.setScale(0.25, 0.25);
        info_text_.setString(p_controled_settings_->getNameOf(ind_in_settings));
        info_text_.setFont(font);
    }


     void Slider2::onRelease(sf::RenderWindow& window)  { is_active = false; }

     void Slider2::onKeyRelease() {};
     void Slider2::onKeyPress(const sf::Keyboard::Key& pressed_key) {};
     void Slider2::onClick(sf::RenderWindow& window)  {
        const auto mouse_coords = window.mapPixelToCoords(sf::Mouse::getPosition(window));

        const auto slider_thingy_bounds = slider_rectangle_.getGlobalBounds();
        const auto slider_bounds = bounding_box_.getGlobalBounds();
        if (slider_bounds.contains(mouse_coords)) {
            is_active = true;

            const auto slider_box_size = bounding_box_.getSize().x;
            auto current_value_ = (mouse_coords.x - bounding_box_.getPosition().x) / slider_box_size;
            if (current_value_ > 1) {
                current_value_ = 1;
            }
            if (current_value_ < 0) {
                current_value_ = 0;
            }

            slider_rectangle_.setPosition(bounding_box_.getPosition().x + current_value_ * slider_box_size,
                                          slider_rectangle_.getPosition().y);

            p_controled_settings_->setValue(controled_ind_in_settings, current_value_);
            std::string value_string = std::to_string(current_value_);
            value_string.resize(5); // = value_string.begin() + 4;
            value_text_.setString(value_string);
        }
    }

     void Slider2::draw(sf::RenderWindow& window)  {
        window.draw(bounding_box_);
        window.draw(slider_rectangle_);
        window.draw(value_text_);
        window.draw(info_text_);
    }

    void Slider2::onMouseButtonHold(sf::RenderWindow& window) {
        const auto mouse_coords = window.mapPixelToCoords(sf::Mouse::getPosition(window));

        if (is_active and bounding_box_.getGlobalBounds().contains(mouse_coords)) {
            const auto slider_box_size = bounding_box_.getSize().x;
            auto current_value_ = (mouse_coords.x - bounding_box_.getPosition().x) / slider_box_size;
            if (current_value_ > 1) {
                current_value_ = 1;
            }
            if (current_value_ < 0) {
                current_value_ = 0;
            }
            slider_rectangle_.setPosition(bounding_box_.getPosition().x + current_value_ * slider_box_size,
                                          slider_rectangle_.getPosition().y);

            p_controled_settings_->setValue(controled_ind_in_settings, current_value_);

            std::string value_string = std::to_string(current_value_);
            value_string.resize(5); // = value_string.begin() + 4;
            value_text_.setString(value_string);
        }
    }


Button2::Button2(sf::Vector2f center, sf::Vector2f size, std::string text, const sf::Font& font, int ind_in_settings,
        Settings* controled_settings)
    : Widget(center, size, text, ind_in_settings, controled_settings) {
    info_text_.setFont(font);
}

Button2::Button2(const Button2& b)
    : Widget(b.bounding_box_.getPosition(), b.bounding_box_.getSize(), b.info_text_.getString(),
                b.controled_ind_in_settings, b.p_controled_settings_) {}

void Button2::draw(sf::RenderWindow& window) {
    window.draw(bounding_box_);
    window.draw(info_text_);
}

void Button2::onClick(sf::RenderWindow& window) {
    const auto mouse_coords = window.mapPixelToCoords(sf::Mouse::getPosition(window));
    if (contains(mouse_coords)) {
        has_been_clicked_ = true;
    }
}
void Button2::onRelease(sf::RenderWindow& window)  {
    const auto mouse_coords = window.mapPixelToCoords(sf::Mouse::getPosition(window));
    if (has_been_clicked_ and contains(mouse_coords)) {
        p_controled_settings_->toggleOption(controled_ind_in_settings);
    }
}

void Button2::onMouseButtonHold(sf::RenderWindow& window) {}
void Button2::onKeyRelease() {};
void Button2::onKeyPress(const sf::Keyboard::Key& pressed_key){};



    NumberField::NumberField(sf::Vector2f center, sf::Vector2f size, std::string text, const sf::Font& font, int ind_in_settings,
                Settings* controled_settings, float min_value, float max_value)
        : Widget(center, size, text, ind_in_settings, controled_settings)
        , min_value_(min_value)
        , max_value_(max_value) {

        value_text_.setPosition(bounding_box_.getPosition());
        std::string value = std::to_string(controled_settings->getValue(ind_in_settings));
        value.resize(5);
        value_text_.setString(value);
        value_text_.setFillColor(sf::Color::Black);
        value_text_.scale(0.25, 0.25);
        value_text_.setFont(font);
        info_text_.setFont(font);

        const auto character_size = value_text_.getCharacterSize() * value_text_.getScale().x * 0.5f;
        insertion_point_line_.setFillColor(sf::Color::Transparent);
        insertion_point_line_.setPosition(bounding_box_.getPosition().x + character_size * value.length(),
                                          bounding_box_.getPosition().y);
        insertion_point_line_.setSize({size.y / 3.0f, size.y * 0.95f});
    }

    void NumberField::onMouseButtonHold(sf::RenderWindow& window) {}

    void NumberField::onClick(sf::RenderWindow& window) {
        const auto mouse_coords = window.mapPixelToCoords(sf::Mouse::getPosition(window));
        const auto field_bounds = bounding_box_.getGlobalBounds();
        if (field_bounds.contains(mouse_coords)) {
            has_been_clicked_ = true;
        } else {
            if (is_active) {
                has_been_clicked_ = false;
                value_text_.setString(old_value_);
                is_active = false;
            }
        }
    }

    void NumberField::onRelease(sf::RenderWindow& window) {
        const auto mouse_coords = window.mapPixelToCoords(sf::Mouse::getPosition(window));
        const auto field_bounds = bounding_box_.getGlobalBounds();
        if (field_bounds.contains(mouse_coords) and has_been_clicked_) {
            has_been_clicked_ = false;
            is_active = true;
            old_value_ = value_text_.getString();
        }
    }

    void NumberField::update() {
        if (is_active) {
            frames_since_last_seen_++;
            if (frames_since_last_seen_ > 30) {
                auto current_color = insertion_point_line_.getFillColor();
                current_color.a = 255 - current_color.a;
                insertion_point_line_.setFillColor(current_color);
                frames_since_last_seen_ = 0;
            }
        }
    }

    void NumberField::onKeyRelease() { key_pressed_ = false; }

    void NumberField::onKeyPress(const sf::Keyboard::Key& pressed_key) {
        if (is_active and !key_pressed_) {
            key_pressed_ = true;

            std::string current_string = value_text_.getString();
            bool too_many_digits = current_string.size() > max_n_digits_;

            float character_size = value_text_.getCharacterSize() * value_text_.getScale().x;
            if (pressed_key >= sf::Keyboard::Num0 and pressed_key <= sf::Keyboard::Num9 and !too_many_digits) {
                current_string += static_cast<char>(pressed_key - sf::Keyboard::Num0 + '0');
            }
            if (pressed_key >= sf::Keyboard::Numpad0 and pressed_key <= sf::Keyboard::Numpad9 and !too_many_digits) {
                current_string += static_cast<char>(pressed_key - sf::Keyboard::Numpad0 + '0');
            }
            if ((pressed_key == sf::Keyboard::Period or pressed_key == sf::Keyboard::Comma) and !number_has_point and
                !too_many_digits) {
                current_string += '.';
                number_has_point = true;
            }
            if (pressed_key == sf::Keyboard::BackSpace) {
                if (current_string.size() > 0) {
                    if (static_cast<char>(current_string[current_string.size() - 1]) == '.') {
                        number_has_point = false;
                    }
                    current_string.resize(current_string.size() - 1);
                    character_size *= -1;
                }
            }
            insertion_point_line_.move({character_size, 0});
            value_text_.setString(current_string);

            if (pressed_key == sf::Keyboard::Enter) {
                p_controled_settings_->setValue(controled_ind_in_settings, std::stof(current_string));
                is_active = false;
            }
            if (pressed_key == sf::Keyboard::Escape) {
                value_text_.setString(old_value_);
                is_active = false;
            }
        }
    }

    void NumberField::draw(sf::RenderWindow& window) {
        update();
        window.draw(bounding_box_);
        //        window.draw(insertion_point_line_);
        window.draw(value_text_);
    }


UI::UI(Game& game, DebugInfo& dbg, BoidControler& bc) {
    const std::string font_path{std::filesystem::current_path()};
    if (!font.loadFromFile(font_path + "/../Resources/arial.ttf")) {
        throw std::runtime_error("no font file at " + font_path);
    }

    sf::Vector2f center;
    auto& dbg_settings = dbg.getSettings();
    typedef DebugInfoSettings2::Options Options;

    auto& bc_settings = bc.getSettings();

    sf::Vector2f window_size = {button_size_.x, button_size_.y * 20.f};
    ui_view_.setViewport(sf::FloatRect(0.0f, 0.0f, 0.1f, 1.0f));
    ui_view_.setSize(window_size);
    ui_view_.setCenter(window_size / 2.f);

    const sf::Vector2f upper_left_pos1 = {0, 0};
    const sf::Vector2f upper_left_pos2 = {0, 1.0f * button_size_.y};
    const sf::Vector2f upper_left_pos3 = {0, 2.0f * button_size_.y};

    std::unique_ptr<Widget> popup_window(
        new PopUpWindow<Button2>(upper_left_pos1, button_size_, "Triangulation", font, &dbg_settings));
    std::unique_ptr<Widget> popup_window2(
        new PopUpWindow<Slider2>(upper_left_pos2, button_size_, "Behaviour", font, &bc_settings));
    std::unique_ptr<Widget> popup_window3(
        new PopUpWindow<NumberField>(upper_left_pos3, button_size_, "UnitCreator", font, &game.uc_settings_));

    widgets_.push_back(std::move(popup_window));
    widgets_.push_back(std::move(popup_window2));
    widgets_.push_back(std::move(popup_window3));

    const auto ui_view_upper_left = ui_view_.getCenter() - ui_view_.getSize() / 2.0f;

    control_panel_boundary_.setSize(ui_view_.getSize());
    control_panel_boundary_.setOutlineThickness(1);
    control_panel_boundary_.setOutlineColor(sf::Color::Black);
    control_panel_boundary_.setFillColor({0, 0, 1, 69});
    control_panel_boundary_.setPosition(ui_view_upper_left);
}

void UI::draw(sf::RenderWindow& window) {

    auto old_view = window.getView();
    window.setView(ui_view_);
    for (auto& widget : widgets_) {
        widget->draw(window);
    }
    window.draw(control_panel_boundary_);
    window.setView(old_view);
}

void UI::onRelease(sf::RenderWindow& window) {
    auto old_view = window.getView();
    window.setView(ui_view_);
    const auto current_mouse_coords = window.mapPixelToCoords(sf::Mouse::getPosition(window));
    for (auto& widget : widgets_) {
        widget->onRelease(window);
    }
    window.setView(old_view);
}

void UI::onMouseHold(sf::RenderWindow& window) {
    auto old_view = window.getView();
    window.setView(ui_view_);
    const auto current_mouse_coords = window.mapPixelToCoords(sf::Mouse::getPosition(window));
    for (auto& widget : widgets_) {
        widget->onMouseButtonHold(window);
    }
    window.setView(old_view);
}

void UI::onKeyRelease(sf::RenderWindow& window) {
    for (auto& widget : widgets_) {
        widget->onKeyRelease();
    }
}

void UI::onKeyPress(const sf::Keyboard::Key& pressed_key) {
    for (auto& widget : widgets_) {
        widget->onKeyPress(pressed_key);
    }
}

void UI::onClick(sf::RenderWindow& window) {
    auto old_view = window.getView();
    window.setView(ui_view_);
    mouse_coords_on_click_ = window.mapPixelToCoords(sf::Mouse::getPosition(window));
    for (auto& widget : widgets_) {
        widget->onClick(window);
    }
    window.setView(old_view);
}
