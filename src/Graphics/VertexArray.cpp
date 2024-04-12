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
#include "VertexArray.hpp"
#include "RenderWindow.hpp"
#include "RenderTexture.hpp"


#include "../include/glad/glad.h"
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>



namespace sf
{

void Transformable::setRotation(float angle){
    trans.setRotatation(angle);
}
void Transformable::setPosition(sf::Vector2f pos){
    trans.setTranslate(pos);
}
void Transformable::setPosition(float x, float y){
    trans.setTranslate({x,y});
}
void Transformable::setScale(sf::Vector2f scale){
    trans.setScale(scale);
}
void Transformable::setScale(float x, float y){
    trans.setScale({x, y});
}
sf::Vector2f Transformable::getPosition()const{
    return trans.mTranslate;
}


////////////////////////////////////////////////////////////
VertexArray::VertexArray() :
m_vertices     (),
m_primitiveType(Points)
{

    
}
////////////////////////////////////////////////////////////
VertexArray::VertexArray(PrimitiveType type, std::size_t vertexCount) :
m_vertices     (vertexCount),
m_primitiveType(type)
{
}


////////////////////////////////////////////////////////////
std::size_t VertexArray::getVertexCount() const
{
    return m_vertices.size();
}


////////////////////////////////////////////////////////////
Vertex& VertexArray::operator [](std::size_t index)
{
    return m_vertices[index];
}


////////////////////////////////////////////////////////////
const Vertex& VertexArray::operator [](std::size_t index) const
{
    return m_vertices[index];
}


////////////////////////////////////////////////////////////
void VertexArray::clear()
{
    m_vertices.clear();
}


////////////////////////////////////////////////////////////
void VertexArray::resize(std::size_t vertexCount)
{
    m_vertices.resize(vertexCount);
}


////////////////////////////////////////////////////////////
void VertexArray::append(const Vertex& vertex)
{
    m_vertices.push_back(vertex);
}


////////////////////////////////////////////////////////////
void VertexArray::setPrimitiveType(PrimitiveType type)
{
    m_primitiveType = type;
}


////////////////////////////////////////////////////////////
PrimitiveType VertexArray::getPrimitiveType() const
{
    return m_primitiveType;
}


////////////////////////////////////////////////////////////
FloatRect VertexArray::getBounds() const
{
    if (!m_vertices.empty())
    {
        float left   = m_vertices[0].position.x;
        float top    = m_vertices[0].position.y;
        float right  = m_vertices[0].position.x;
        float bottom = m_vertices[0].position.y;

        for (std::size_t i = 1; i < m_vertices.size(); ++i)
        {
            Vector2f position = m_vertices[i].position;

            // Update left and right
            if (position.x < left)
                left = position.x;
            else if (position.x > right)
                right = position.x;

            // Update top and bottom
            if (position.y < top)
                top = position.y;
            else if (position.y > bottom)
                bottom = position.y;
        }

        return FloatRect(left, top, right - left, bottom - top);
    }
    else
    {
        // Array is empty
        return FloatRect();
    }
}

void VertexArray::draw(RenderWindow& window)
{
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

    glBindVertexArray(quadVAO);

    // __glewBindFramebuffer(GL_DRAW_FRAMEBUFFER, target);
    // glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // glClearColor(0.0, 1.f, 1.0f, 1.0f);
    // if (texture)
    // {
    //     glActiveTexture(GL_TEXTURE0);
    //     glBindTexture(GL_TEXTURE_2D, texture);
    // }


    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(sf::Vertex) * m_vertices.size(), m_vertices.data(), GL_DYNAMIC_DRAW);
         

    glDrawArrays(primitive_type, 0, m_vertices.size());
    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    
}



void VertexArray::draw(RenderWindow& window, const Transform& trans)
{

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glBindVertexArray(quadVAO);

    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(sf::Vertex) * m_vertices.size(), m_vertices.data(), GL_DYNAMIC_DRAW);
         
    if(texture){
        glBindTexture(GL_TEXTURE_2D, *texture);
        shader_textured.use();
        shader_textured.setMat4("view", window.view.matrix);
        shader_textured.setMat4("transform", trans.matrix);
    }else{
        shader.use();
        shader.setMat4("view", window.view.matrix);
        shader.setMat4("transform", trans.matrix);
    }


    glDrawArrays(primitive_type, 0, m_vertices.size());
    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    
}


void VertexArray::draw(RenderTexture& target, const Transform& trans)
{

    
    glBindFramebuffer(GL_FRAMEBUFFER, target.framebuffer_handle);
    glViewport(0,0, target.texture.size.x, target.texture.size.y);

    glBindVertexArray(quadVAO);

    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(sf::Vertex) * m_vertices.size(), m_vertices.data(), GL_DYNAMIC_DRAW);

    target.view.calcMatrix();
    shader.use();
    shader.setMat4("view", target.view.matrix);
    shader.setMat4("transform", trans.matrix);

    glDrawArrays(primitive_type, 0, m_vertices.size());
    glViewport(0,0, 1920, 1080);

    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    
}

} // namespace sf


