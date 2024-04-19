#pragma once 


#include "Rect.hpp"
#include "RenderTarget.hpp"

#include <iostream>
#include <vector>

class GLFWwindow;



namespace sf{


struct Window{
    Window(sf::Vector2i size) : size(size){}
    sf::Vector2i size;
    GLFWwindow* handle;
};


////////////////////////////////////////////////////////////
/// \brief Defines a system event and its parameters
///
////////////////////////////////////////////////////////////
struct Event
{

    struct SizeEvent
    {
        unsigned int width;  //!< New width, in pixels
        unsigned int height; //!< New height, in pixels
    };

    struct KeyEvent
    {
        int               code;     //!< Code of the key that has been pressed
        int               scancode; //!< Physical code of the key that has been pressed
        int               mods;      //!< Is the Alt key pressed?
    };

    struct TextEvent
    {
        u_int32_t unicode; //!< UTF-32 Unicode value of the character
    };

    struct MouseMoveEvent
    {
        int x; //!< X position of the mouse pointer, relative to the left of the owner window
        int y; //!< Y position of the mouse pointer, relative to the top of the owner window
    };

    struct MouseButtonEvent
    {
        int         button; //!< Code of the button that has been pressed
        double           x;      //!< X position of the mouse pointer, relative to the left of the owner window
        double           y;      //!< Y position of the mouse pointer, relative to the top of the owner window
    };

    struct MouseWheelScrollEvent
    {
        float        delta; //!< Wheel offset (positive is up/left, negative is down/right). High-precision mice may use non-integral offsets.
        int          x; //!< X position of the mouse pointer, relative to the left of the owner window
        int          y; //!< Y position of the mouse pointer, relative to the top of the owner window
    };


    enum EventType
    {
        Closed,                 //!< The window requested to be closed (no data)
        Resized,                //!< The window was resized (data in event.size)
        LostFocus,              //!< The window lost the focus (no data)
        GainedFocus,            //!< The window gained the focus (no data)
        TextEntered,            //!< A character was entered (data in event.text)
        KeyPressed,             //!< A key was pressed (data in event.key)
        KeyReleased,            //!< A key was released (data in event.key)
        MouseWheelScrolled,     //!< The mouse wheel was scrolled (data in event.mouseWheelScroll)
        MouseButtonPressed,     //!< A mouse button was pressed (data in event.mouseButton)
        MouseButtonReleased,    //!< A mouse button was released (data in event.mouseButton)
        MouseMoved,             //!< The mouse cursor moved (data in event.mouseMove)
        MouseWheelMoved,             //!< The mouse cursor moved (data in event.mouseMove)
        MouseEntered,           //!< The mouse cursor entered the area of the window (no data)
        MouseLeft,              //!< The mouse cursor left the area of the window (no data)
        Count //!< Keep last -- the total number of event types
    };

    EventType type{}; //!< Type of the event

    union
    {
        SizeEvent      size;      //!< Size event parameters (Event::Resized)
        KeyEvent       key;       //!< Key event parameters (Event::KeyPressed, Event::KeyReleased)
        TextEvent      text;      //!< Text event parameters (Event::TextEntered)
        MouseMoveEvent mouseMove; //!< Mouse move event parameters (Event::MouseMoved)
        MouseButtonEvent mouseButton; //!< Mouse button event parameters (Event::MouseButtonPressed, Event::MouseButtonReleased)
        MouseWheelScrollEvent mouseWheelScroll; //!< Mouse wheel event parameters (Event::MouseWheelScrolled)
    };
};

struct RenderWindow : RenderTarget, Window{
    
    View view;
    FloatRect view_port;
    bool mouse_was_pressed = false;
    bool key_was_pressed = false;

    std::vector<Event> events;

    RenderWindow(sf::Vector2i size);
    void draw();
    Vector2f mapPixelToCoords(const Vector2i& point) const;

    ////////////////////////////////////////////////////////////
    Vector2f mapPixelToCoords(const Vector2i& point, const View& view) const;

    ////////////////////////////////////////////////////////////
    IntRect getViewport(const View& view) const;
 
    bool pollEvent(sf::Event& event){
        if(events.empty()){
            return 0;
        }
        event = events.back();
        events.pop_back();
        return 1;
    }

};
}
