////////////////////////////////////////////////////////////
//
// SFML - Simple and Fast Multimedia Library
// Copyright (C) 2007-2018 Laurent Gomila (laurent@sfml-dev.org)
//
// This software is provided 'as-is', without any express or implied warranty.
// In no event will the authors be held liable for any damages arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it freely,
// subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented;
//    you must not claim that you wrote the original software.
//    If you use this software in a product, an acknowledgment
//    in the product documentation would be appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such,
//    and must not be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source distribution.
//
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// Headers
////////////////////////////////////////////////////////////
#include "RectangleShape.hpp"
#include <cmath>


namespace sf
{
////////////////////////////////////////////////////////////
RectangleShape::RectangleShape(const Vector2f& size){
    setSize(size);

    m_vertices.resize(6);

    m_vertices[0].position = {0.0, 0.0};
    m_vertices[1].position = {0.5, -0.5};
    m_vertices[2].position = {0.5, 0.5};
    m_vertices[3].position = {-0.5, 0.5};
    m_vertices[4].position = {-0.5, -0.5};
    m_vertices[5].position = {0.5, -0.5};

    m_vertices[0].texCoords = { 0.5,  0.5};
    m_vertices[1].texCoords = { 1.0, 0.0};
    m_vertices[2].texCoords = { 1.0,  1.0};
    m_vertices[3].texCoords = {0.0,  1.0};
    m_vertices[4].texCoords = {0.0, 0.0};
    m_vertices[5].texCoords = { 1.0, 0.0};

    glGenVertexArrays(1, &m_vertices.quadVAO);
    glBindVertexArray(m_vertices.quadVAO);
    glGenBuffers(1, &m_vertices.quadVBO);
    glBindBuffer(GL_ARRAY_BUFFER, m_vertices.quadVBO);
    glBufferData(GL_ARRAY_BUFFER, 5 * sizeof(float) * m_vertices.getVertexCount(), &(m_vertices[0]), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_TRUE, 5 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, 5 * sizeof(float), (void *)(2 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_TRUE, 5 * sizeof(float), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);    

    m_vertices.primitive_type = GL_TRIANGLE_FAN;

}

RectangleShape::~RectangleShape(){
}

////////////////////////////////////////////////////////////
void RectangleShape::setSize(const Vector2f& size)
{
    m_size = size;
    trans.setScale(m_size);
    // m_vertices.setScale(m_size);
    // update();
}


////////////////////////////////////////////////////////////
const Vector2f& RectangleShape::getSize() const
{
    return m_size;
}


////////////////////////////////////////////////////////////
std::size_t RectangleShape::getPointCount() const
{
    return 4;
}




////////////////////////////////////////////////////////////
Vector2f RectangleShape::getPoint(std::size_t index) const
{
    switch (index)
    {
        default:
        case 0: return Vector2f(0, 0);
        case 1: return Vector2f(m_size.x, 0);
        case 2: return Vector2f(m_size.x, m_size.y);
        case 3: return Vector2f(0, m_size.y);
    }
}

void RectangleShape::draw(sf::RenderWindow& window){
    m_vertices.draw(window, trans);
}


    // void RectangleShape::setFillColor(sf::Color color){
    //     for(int i = 0; i < 6; ++i){
    //         m_vertices[i].color = color;
    //     }
    // }

} // namespace sf
