#include "RenderTarget.hpp"


#include "../include/glad/glad.h"
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>


   void View::calcMatrix()
    {
        matrix = glm::mat4(1.0f);
        matrix = glm::scale(matrix, glm::vec3(2. / width , -2. / height, 1));
        matrix = glm::translate(matrix, glm::vec3(-r_center.x, -r_center.y, 0));
    }

    void View::zoom(float scale)
    {
        height *= scale;
        width *= scale;
        calcMatrix();
    }


    void View::move(float x, float y)
    {
        move({x,y});
    }

    void View::move(sf::Vector2f delta_r)
    {
        r_center -= delta_r;
        calcMatrix();
    }




namespace sf
{

}