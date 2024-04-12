#pragma once


#include "../Utils/Vector.hpp"


#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/matrix_transform_2d.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "../Geometry.h"


struct View
{
    sf::Vector2f r_center = {Geometry::BOX[0]/2, Geometry::BOX[1]/2};
    float width = Geometry::BOX[0];
    float height = 1080./1920.f*Geometry::BOX[1];

    glm::mat4 matrix = glm::mat4(1.0f);

    void calcMatrix();

    void zoom(float scale);

    void move(sf::Vector2f delta_r);
    void move(float x, float y);
};



namespace sf
{

    struct RenderTarget{

        View view;
        virtual void draw() = 0;
    };
}
