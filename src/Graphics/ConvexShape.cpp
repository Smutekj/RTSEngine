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
#include "ConvexShape.hpp"
#include "RenderTexture.hpp"

namespace sf
{
////////////////////////////////////////////////////////////
ConvexShape::ConvexShape(std::size_t pointCount)
{
    setPointCount(pointCount);
    m_vertices.primitive_type = GL_TRIANGLE_FAN;
}


////////////////////////////////////////////////////////////
void ConvexShape::setPointCount(std::size_t count)
{
    m_vertices.resize(count+2);
    m_points.resize(count);
    update();
    initialize();
}


////////////////////////////////////////////////////////////
std::size_t ConvexShape::getPointCount() const
{
    return m_points.size();
}


////////////////////////////////////////////////////////////
void ConvexShape::setPoint(std::size_t index, const Vector2f& point)
{
    m_points[index] = point;
    m_vertices[index+1].position = point;

    update();
}

void ConvexShape::update(){
    const auto n_points = getPointCount();

    sf::Vector2f center = {0,0};
    for(int i = 0; i < n_points; ++i){
        center += m_vertices[i].position/static_cast<float>(n_points);
    }
    // m_vertices.resize(n_points + 2);
    m_vertices[0].position = center;
    m_vertices[n_points + 2 -1] = m_vertices[1];
}

////////////////////////////////////////////////////////////
Vector2f ConvexShape::getPoint(std::size_t index) const
{
    return vertices[index].position;
}

void ConvexShape::draw(sf::RenderTexture& texture){
    m_vertices.draw(texture, trans);
}
void ConvexShape::draw(sf::RenderWindow& window){
    m_vertices.draw(window, trans);
}


void ConvexShape::initialize(){
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

} // namespace sf

