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

#ifndef SFML_CONVEXSHAPE_HPP
#define SFML_CONVEXSHAPE_HPP

////////////////////////////////////////////////////////////
// Headers
////////////////////////////////////////////////////////////
#include "Shape.hpp"
#include <vector>


namespace sf
{
////////////////////////////////////////////////////////////
/// \brief Specialized shape representing a convex polygon
///
////////////////////////////////////////////////////////////
class ConvexShape : public Shape
{
public:


    explicit ConvexShape(std::size_t pointCount = 0);


    void setPointCount(std::size_t count);


    virtual std::size_t getPointCount() const;

    void setPoint(std::size_t index, const Vector2f& point);


    virtual Vector2f getPoint(std::size_t index) const;

    // void drawOn(sf::RenderTexture& texture);
    void draw(sf::RenderTexture& texture);
    void draw(sf::RenderWindow& texture);

    void initialize();
    void update();

public:

    sf::VertexArray vertices;
    std::vector<Vector2f> m_points; ///< Points composing the convex polygon
};

} // namespace sf


#endif // SFML_CONVEXSHAPE_HPP


////////////////////////////////////////////////////////////
/// \class sf::ConvexShape
/// \ingroup graphics
///
/// This class inherits all the functions of sf::Transformable
/// (position, rotation, scale, bounds, ...) as well as the
/// functions of sf::Shape (outline, color, texture, ...).
///
/// It is important to keep in mind that a convex shape must
/// always be... convex, otherwise it may not be drawn correctly.
/// Moreover, the points must be defined in order; using a random
/// order would result in an incorrect shape.
///
/// Usage example:
/// \code
/// sf::ConvexShape polygon;
/// polygon.setPointCount(3);
/// polygon.setPoint(0, sf::Vector2f(0, 0));
/// polygon.setPoint(1, sf::Vector2f(0, 10));
/// polygon.setPoint(2, sf::Vector2f(25, 5));
/// polygon.setOutlineColor(sf::Color::Red);
/// polygon.setOutlineThickness(5);
/// polygon.setPosition(10, 20);
/// ...
/// window.draw(polygon);
/// \endcode
///
/// \see sf::Shape, sf::RectangleShape, sf::CircleShape
///
////////////////////////////////////////////////////////////
