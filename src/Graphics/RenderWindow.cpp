#include "RenderWindow.hpp"
#include <iostream> 


#include "../include/glad/glad.h"
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>


// #include <SFML/Window/Window.hpp> // important to be included first (conflict with None)
// #include <SFML/Window/Unix/InputImpl.hpp>
// #include <SFML/Window/Unix/Display.hpp>
#include <SFML/System/Err.hpp>
#include <X11/Xlib.h>
#include <X11/keysym.h>


void initializeEventCallbacks(sf::RenderWindow& window){
    glfwSetWindowUserPointer(window.handle, &window);

        auto func = [](GLFWwindow* window, int key, int scancode, int action, int mods)
        { 
            sf::Event event;
            
            auto w = static_cast<sf::RenderWindow*>(glfwGetWindowUserPointer(window));
            auto view_l = &w->view;
            if(action == GLFW_PRESS){
                event.type = sf::Event::EventType::KeyPressed;
                event.key.code = key;
                event.key.scancode = scancode;
                w->events.push_back(event);
            }
            if(action == GLFW_RELEASE){
                event.type = sf::Event::EventType::KeyReleased;
                event.key.code = key;
                event.key.scancode = scancode;
                w->events.push_back(event);
            }
            // if (glfwGetKey(window, GLFW_KEY_LEFT))
            // {
            //     view_l->move({0.5f, 0.f});
            // }
            // else if (glfwGetKey(window, GLFW_KEY_RIGHT))
            // {
            //     view_l->move({-0.5f, 0.f});
            // }
            // else if (glfwGetKey(window, GLFW_KEY_DOWN))
            // {
            //     view_l->move({0.f, 1.0f});
            // }
            // else if (glfwGetKey(window, GLFW_KEY_UP))
            // {
            //     view_l->move({0.f, -1.0f});
            // }
            // else if (glfwGetKey(window, GLFW_KEY_DELETE)){
            //     // scene.destroyInstanceOf(0, 0);
            // }   
 
        };
        glfwSetKeyCallback(window.handle, func);

        auto func_mouse = [](GLFWwindow* window, int button, int action, int mods)
        {
            sf::Event event;
            auto w  = static_cast<sf::RenderWindow*>(glfwGetWindowUserPointer(window));
            if(action == GLFW_PRESS)
                event.type = sf::Event::EventType::MouseButtonPressed;
                event.mouseButton.button = button;
                glfwGetCursorPos(window, static_cast<double*>(&event.mouseButton.x), static_cast<double*>(&event.mouseButton.y));
                w->events.push_back(event);
            if(action == GLFW_RELEASE){
                event.type = sf::Event::EventType::MouseButtonReleased;
                event.mouseButton.button = button;
                glfwGetCursorPos(window, static_cast<double*>(&event.mouseButton.x), static_cast<double*>(&event.mouseButton.y));
                w->events.push_back(event);
            }
            if(action == GLFW_RAW_MOUSE_MOTION){
                event.type = sf::Event::EventType::MouseMoved;
                event.mouseButton.button = button;
                glfwGetCursorPos(window, static_cast<double*>(&event.mouseButton.x), static_cast<double*>(&event.mouseButton.y));
                w->events.push_back(event);
            }
        };
        glfwSetMouseButtonCallback(window.handle, func_mouse);

        auto func_scroll = [](GLFWwindow* window, double xoffset, double yoffset)
        {
            sf::Event event;
            auto w  = static_cast<sf::RenderWindow*>(glfwGetWindowUserPointer(window));
            event.type = sf::Event::EventType::MouseWheelMoved;
            event.mouseWheelScroll.x = xoffset;
            event.mouseWheelScroll.y = yoffset;
            event.mouseWheelScroll.delta = yoffset;
            w->events.push_back(event);
        };
        glfwSetScrollCallback(window.handle, func_scroll);
        

}

namespace sf{

    RenderWindow::RenderWindow(sf::Vector2i size) : events(0){
        
        this->size = size;
        view.calcMatrix();


        glfwInit();
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    
        handle = glfwCreateWindow(size.x, size.y, "LearnOpenGL", glfwGetPrimaryMonitor(), NULL); //NULL, NULL); //

        if (handle == NULL)
        {
            glfwTerminate();
            throw std::runtime_error("Failed to create GLFW window");
        }
        glfwMakeContextCurrent(handle);
        
        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
        {
            throw std::runtime_error("Failed to initialize GLAD");
        }    

        // glEnable(GL_CULL_FACE);
        glEnable(GL_BLEND);  
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);  


        initializeEventCallbacks(*this);

    }

    void RenderWindow::draw(){
        
    }    

    Vector2f RenderWindow::mapPixelToCoords(const Vector2i& point) const
    {
        return mapPixelToCoords(point, view);
    }

    ////////////////////////////////////////////////////////////
    Vector2f RenderWindow::mapPixelToCoords(const Vector2i& point, const View& view) const
    {
        // First, convert from viewport coordinates to homogeneous coordinates
        Vector2f normalized;
        sf::IntRect viewport = getViewport(view);
        viewport.height = 1080;
        viewport.width = 1920;
        const auto alpha = 1080.f/1920.f;

        normalized.x = -1.0f + 2.0f*(point.x - viewport.left) / viewport.width;
        normalized.y =  1.0f - 2.0f*(point.y - viewport.top)  / viewport.height;

        // Then transform by the inverse of the view matrix
        Vector2f result = normalized;
        auto inv_view = glm::inverse(view.matrix);
        glm::vec4 result2 = inv_view * glm::vec4(result.x, result.y, 0.f, 1.f);

        // result.x *= view.width /2.0f;
        // result.y *= view.height/2.0f;
        // result.x += view.r_center.x;
        // result.y += view.r_center.y ;
        // result.y *= alpha;

        return {result2.x, result2.y};
    }


    ////////////////////////////////////////////////////////////
    IntRect RenderWindow::getViewport(const View& view) const
    {
        float width  = static_cast<float>(size.x);
        float height = static_cast<float>(size.y);
        const FloatRect& viewport = {0.0, 0.0, 1.0,1.0}; //view.getViewport();

        return IntRect(static_cast<int>(width  * viewport.left),
                    static_cast<int>(height * viewport.top),
                    static_cast<int>(width  * viewport.width),
                    static_cast<int>(height * viewport.height));
    }


}
