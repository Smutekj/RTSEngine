#include "Text.hpp"

#include "Font.hpp"
#include "RenderWindow.hpp"
// #include "Texture.hpp"
// #include "Shader.h"

// #include <algorithm>
// #include <utility>

// #include <cmath>
// #include <cstddef>
// #include <cstdint>

// namespace
// {
//     // Add an underline or strikethrough line to the vertex array
//     void addLine(sf::VertexArray &vertices,
//                  float lineLength,
//                  float lineTop,
//                  const sf::Color &color,
//                  float offset,
//                  float thickness,
//                  float outlineThickness = 0)
//     {
//         const float top = std::floor(lineTop + offset - (thickness / 2) + 0.5f);
//         const float bottom = top + std::floor(thickness + 0.5f);

//         vertices.append(sf::Vertex(sf::Vector2f(-outlineThickness, top - outlineThickness), color, sf::Vector2f(1, 1)));
//         vertices.append(
//             sf::Vertex(sf::Vector2f(lineLength + outlineThickness, top - outlineThickness), color, sf::Vector2f(1, 1)));
//         vertices.append(sf::Vertex(sf::Vector2f(-outlineThickness, bottom + outlineThickness), color, sf::Vector2f(1, 1)));
//         vertices.append(sf::Vertex(sf::Vector2f(-outlineThickness, bottom + outlineThickness), color, sf::Vector2f(1, 1)));
//         vertices.append(
//             sf::Vertex(sf::Vector2f(lineLength + outlineThickness, top - outlineThickness), color, sf::Vector2f(1, 1)));
//         vertices.append(
//             sf::Vertex(sf::Vector2f(lineLength + outlineThickness, bottom + outlineThickness), color, sf::Vector2f(1, 1)));
//     }

//     // Add a glyph quad to the vertex array
//     void addGlyphQuad(sf::VertexArray &vertices, sf::Vector2f position, const sf::Color &color, const sf::Glyph &glyph, float italicShear)
//     {
//         const float padding = 1.0;

//         const float left = glyph.bounds.left - padding;
//         const float top = glyph.bounds.top - padding;
//         const float right = glyph.bounds.left + glyph.bounds.width + padding;
//         const float bottom = glyph.bounds.top + glyph.bounds.height + padding;

//         const float u1 = static_cast<float>(glyph.textureRect.left) - padding;
//         const float v1 = static_cast<float>(glyph.textureRect.top) - padding;
//         const float u2 = static_cast<float>(glyph.textureRect.left + glyph.textureRect.width) + padding;
//         const float v2 = static_cast<float>(glyph.textureRect.top + glyph.textureRect.height) + padding;

//         vertices.append(
//             sf::Vertex(sf::Vector2f(position.x + left - italicShear * top, position.y + top), color, sf::Vector2f(u1, v1)));
//         vertices.append(
//             sf::Vertex(sf::Vector2f(position.x + right - italicShear * top, position.y + top), color, sf::Vector2f(u2, v1)));
//         vertices.append(
//             sf::Vertex(sf::Vector2f(position.x + left - italicShear * bottom, position.y + bottom), color, sf::Vector2f(u1, v2)));
//         vertices.append(
//             sf::Vertex(sf::Vector2f(position.x + left - italicShear * bottom, position.y + bottom), color, sf::Vector2f(u1, v2)));
//         vertices.append(
//             sf::Vertex(sf::Vector2f(position.x + right - italicShear * top, position.y + top), color, sf::Vector2f(u2, v1)));
//         vertices.append(sf::Vertex(sf::Vector2f(position.x + right - italicShear * bottom, position.y + bottom),
//                                    color,
//                                    sf::Vector2f(u2, v2)));
//     }
// } // namespace

// namespace sf
// {
//     ////////////////////////////////////////////////////////////
//     Text::Text(const Font &font, std::string string, unsigned int characterSize) : m_string(std::move(string)),
//                                                                                    m_font(&font),
//                                                                                    m_characterSize(characterSize)
//     {
//     }

//     ////////////////////////////////////////////////////////////
//     Text::Text()
//     {
//     }

//     ////////////////////////////////////////////////////////////
//     void Text::setString(const std::string &string)
//     {
//         if (m_string != string)
//         {
//             m_string = string;
//             m_geometryNeedUpdate = true;
//         }
//     }

//     ////////////////////////////////////////////////////////////
//     void Text::setFont(const Font &font)
//     {
//         if (m_font != &font)
//         {
//             m_font = &font;
//             m_geometryNeedUpdate = true;
//         }
//     }

//     ////////////////////////////////////////////////////////////
//     void Text::setCharacterSize(unsigned int size)
//     {
//         if (m_characterSize != size)
//         {
//             m_characterSize = size;
//             m_geometryNeedUpdate = true;
//         }
//     }

//     ////////////////////////////////////////////////////////////
//     void Text::setLetterSpacing(float spacingFactor)
//     {
//         if (m_letterSpacingFactor != spacingFactor)
//         {
//             m_letterSpacingFactor = spacingFactor;
//             m_geometryNeedUpdate = true;
//         }
//     }

//     ////////////////////////////////////////////////////////////
//     void Text::setLineSpacing(float spacingFactor)
//     {
//         if (m_lineSpacingFactor != spacingFactor)
//         {
//             m_lineSpacingFactor = spacingFactor;
//             m_geometryNeedUpdate = true;
//         }
//     }

//     ////////////////////////////////////////////////////////////
//     void Text::setStyle(std::uint32_t style)
//     {
//         if (m_style != style)
//         {
//             m_style = style;
//             m_geometryNeedUpdate = true;
//         }
//     }

//     ////////////////////////////////////////////////////////////
//     void Text::setFillColor(const Color &color)
//     {
//         if (color != m_fillColor)
//         {
//             m_fillColor = color;

//             // Change vertex colors directly, no need to update whole geometry
//             // (if geometry is updated anyway, we can skip this step)
//             if (!m_geometryNeedUpdate)
//             {
//                 for (std::size_t i = 0; i < m_vertices.getVertexCount(); ++i)
//                     m_vertices[i].color = m_fillColor;
//             }
//         }
//     }

//     ////////////////////////////////////////////////////////////
//     void Text::setOutlineColor(const Color &color)
//     {
//         if (color != m_outlineColor)
//         {
//             m_outlineColor = color;

//             // Change vertex colors directly, no need to update whole geometry
//             // (if geometry is updated anyway, we can skip this step)
//             if (!m_geometryNeedUpdate)
//             {
//                 for (std::size_t i = 0; i < m_outlineVertices.getVertexCount(); ++i)
//                     m_outlineVertices[i].color = m_outlineColor;
//             }
//         }
//     }

//     ////////////////////////////////////////////////////////////
//     void Text::setOutlineThickness(float thickness)
//     {
//         if (thickness != m_outlineThickness)
//         {
//             m_outlineThickness = thickness;
//             m_geometryNeedUpdate = true;
//         }
//     }

//     ////////////////////////////////////////////////////////////
//     const std::string &Text::getString() const
//     {
//         return m_string;
//     }

//     ////////////////////////////////////////////////////////////
//     const Font &Text::getFont() const
//     {
//         return *m_font;
//     }

//     ////////////////////////////////////////////////////////////
//     unsigned int Text::getCharacterSize() const
//     {
//         return m_characterSize;
//     }

//     ////////////////////////////////////////////////////////////
//     float Text::getLetterSpacing() const
//     {
//         return m_letterSpacingFactor;
//     }

//     ////////////////////////////////////////////////////////////
//     float Text::getLineSpacing() const
//     {
//         return m_lineSpacingFactor;
//     }

//     ////////////////////////////////////////////////////////////
//     std::uint32_t Text::getStyle() const
//     {
//         return m_style;
//     }

//     ////////////////////////////////////////////////////////////
//     const Color &Text::getFillColor() const
//     {
//         return m_fillColor;
//     }

//     ////////////////////////////////////////////////////////////
//     const Color &Text::getOutlineColor() const
//     {
//         return m_outlineColor;
//     }

//     ////////////////////////////////////////////////////////////
//     float Text::getOutlineThickness() const
//     {
//         return m_outlineThickness;
//     }

//     ////////////////////////////////////////////////////////////
//     Vector2f Text::findCharacterPos(std::size_t index) const
//     {
//         // Adjust the index if it's out of range
//         if (index > m_string.length())
//             index = m_string.length();

//         // Precompute the variables needed by the algorithm
//         const bool isBold = m_style & Bold;
//         float whitespaceWidth = m_font->getGlyph(U' ', m_characterSize, isBold).advance;
//         const float letterSpacing = (whitespaceWidth / 3.f) * (m_letterSpacingFactor - 1.f);
//         whitespaceWidth += letterSpacing;
//         const float lineSpacing = m_font->getLineSpacing(m_characterSize) * m_lineSpacingFactor;

//         // Compute the position
//         Vector2f position;
//         std::uint32_t prevChar = 0;
//         for (std::size_t i = 0; i < index; ++i)
//         {
//             const std::uint32_t curChar = m_string[i];

//             // Apply the kerning offset
//             position.x += m_font->getKerning(prevChar, curChar, m_characterSize, isBold);
//             prevChar = curChar;

//             // Handle special characters
//             switch (curChar)
//             {
//             case U' ':
//                 position.x += whitespaceWidth;
//                 continue;
//             case U'\t':
//                 position.x += whitespaceWidth * 4;
//                 continue;
//             case U'\n':
//                 position.y += lineSpacing;
//                 position.x = 0;
//                 continue;
//             }

//             // For regular characters, add the advance offset of the glyph
//             position.x += m_font->getGlyph(curChar, m_characterSize, isBold).advance + letterSpacing;
//         }

//         // Transform the position to global coordinates
//         position += trans.mTranslate;

//         return position;
//     }

//     ////////////////////////////////////////////////////////////
//     FloatRect Text::getLocalBounds() const
//     {
//         ensureGeometryUpdate();

//         return m_bounds;
//     }

//     ////////////////////////////////////////////////////////////
//     FloatRect Text::getGlobalBounds() const
//     {
//         return {};
//         // return getTransform().transformRect(getLocalBounds());
//     }

//     ////////////////////////////////////////////////////////////
//     void Text::draw(RenderWindow &target) const
//     {
//         ensureGeometryUpdate();
//         m_vertices.draw(target);
//     }

//     ////////////////////////////////////////////////////////////
//     void Text::ensureGeometryUpdate() const
//     {
//         // Do nothing, if geometry has not changed and the font texture has not changed
//         if (!m_geometryNeedUpdate && m_font->getTexture(m_characterSize).m_cacheID == m_fontTextureId)
//             return;

//         // Save the current fonts texture id
//         m_fontTextureId = m_font->getTexture(m_characterSize).m_cacheID;

//         // Mark geometry as updated
//         m_geometryNeedUpdate = false;

//         // Clear the previous geometry
//         m_vertices.clear();
//         m_outlineVertices.clear();
//         m_bounds = FloatRect();

//         // No text: nothing to draw
//         if (m_string.empty())
//             return;

//         // Compute values related to the text style
//         const bool isBold = m_style & Bold;
//         const bool isUnderlined = m_style & Underlined;
//         const bool isStrikeThrough = m_style & StrikeThrough;
//         const float italicShear = 0.0f; //(m_style & Italic) ? sf::degrees(12).asRadians() : 0.f;
//         const float underlineOffset = m_font->getUnderlinePosition(m_characterSize);
//         const float underlineThickness = m_font->getUnderlineThickness(m_characterSize);

//         // Compute the location of the strike through dynamically
//         // We use the center point of the lowercase 'x' glyph as the reference
//         // We reuse the underline thickness as the thickness of the strike through as well
//         const float strikeThroughOffset = m_font->getGlyph(U'x', m_characterSize, isBold).bounds.getCenter().y;

//         // Precompute the variables needed by the algorithm
//         float whitespaceWidth = m_font->getGlyph(U' ', m_characterSize, isBold).advance;
//         const float letterSpacing = (whitespaceWidth / 3.f) * (m_letterSpacingFactor - 1.f);
//         whitespaceWidth += letterSpacing;
//         const float lineSpacing = m_font->getLineSpacing(m_characterSize) * m_lineSpacingFactor;
//         float x = 0.f;
//         auto y = static_cast<float>(m_characterSize);

//         // Create one quad for each character
//         auto minX = static_cast<float>(m_characterSize);
//         auto minY = static_cast<float>(m_characterSize);
//         float maxX = 0.f;
//         float maxY = 0.f;
//         std::uint32_t prevChar = 0;
//         for (const std::uint32_t curChar : m_string)
//         {
//             // Skip the \r char to avoid weird graphical issues
//             if (curChar == U'\r')
//                 continue;

//             // Apply the kerning offset
//             x += m_font->getKerning(prevChar, curChar, m_characterSize, isBold);

//             // If we're using the underlined style and there's a new line, draw a line
//             if (isUnderlined && (curChar == U'\n' && prevChar != U'\n'))
//             {
//                 addLine(m_vertices, x, y, m_fillColor, underlineOffset, underlineThickness);

//                 if (m_outlineThickness != 0)
//                     addLine(m_outlineVertices, x, y, m_outlineColor, underlineOffset, underlineThickness, m_outlineThickness);
//             }

//             // If we're using the strike through style and there's a new line, draw a line across all characters
//             if (isStrikeThrough && (curChar == U'\n' && prevChar != U'\n'))
//             {
//                 addLine(m_vertices, x, y, m_fillColor, strikeThroughOffset, underlineThickness);

//                 if (m_outlineThickness != 0)
//                     addLine(m_outlineVertices, x, y, m_outlineColor, strikeThroughOffset, underlineThickness, m_outlineThickness);
//             }

//             prevChar = curChar;

//             // Handle special characters
//             if ((curChar == U' ') || (curChar == U'\n') || (curChar == U'\t'))
//             {
//                 // Update the current bounds (min coordinates)
//                 minX = std::min(minX, x);
//                 minY = std::min(minY, y);

//                 switch (curChar)
//                 {
//                 case U' ':
//                     x += whitespaceWidth;
//                     break;
//                 case U'\t':
//                     x += whitespaceWidth * 4;
//                     break;
//                 case U'\n':
//                     y += lineSpacing;
//                     x = 0;
//                     break;
//                 }

//                 // Update the current bounds (max coordinates)
//                 maxX = std::max(maxX, x);
//                 maxY = std::max(maxY, y);

//                 // Next glyph, no need to create a quad for whitespace
//                 continue;
//             }

//             // Apply the outline
//             if (m_outlineThickness != 0)
//             {
//                 const Glyph &glyph = m_font->getGlyph(curChar, m_characterSize, isBold, m_outlineThickness);

//                 // Add the outline glyph to the vertices
//                 addGlyphQuad(m_outlineVertices, Vector2f(x, y), m_outlineColor, glyph, italicShear);
//             }

//             // Extract the current glyph's description
//             const Glyph &glyph = m_font->getGlyph(curChar, m_characterSize, isBold);

//             // Add the glyph to the vertices
//             addGlyphQuad(m_vertices, Vector2f(x, y), m_fillColor, glyph, italicShear);

//             // Update the current bounds
//             const float left = glyph.bounds.left;
//             const float top = glyph.bounds.top;
//             const float right = glyph.bounds.left + glyph.bounds.width;
//             const float bottom = glyph.bounds.top + glyph.bounds.height;

//             minX = std::min(minX, x + left - italicShear * bottom);
//             maxX = std::max(maxX, x + right - italicShear * top);
//             minY = std::min(minY, y + top);
//             maxY = std::max(maxY, y + bottom);

//             // Advance to the next character
//             x += glyph.advance + letterSpacing;
//         }

//         // If we're using outline, update the current bounds
//         if (m_outlineThickness != 0)
//         {
//             const float outline = std::abs(std::ceil(m_outlineThickness));
//             minX -= outline;
//             maxX += outline;
//             minY -= outline;
//             maxY += outline;
//         }

//         // If we're using the underlined style, add the last line
//         if (isUnderlined && (x > 0))
//         {
//             addLine(m_vertices, x, y, m_fillColor, underlineOffset, underlineThickness);

//             if (m_outlineThickness != 0)
//                 addLine(m_outlineVertices, x, y, m_outlineColor, underlineOffset, underlineThickness, m_outlineThickness);
//         }

//         // If we're using the strike through style, add the last line across all characters
//         if (isStrikeThrough && (x > 0))
//         {
//             addLine(m_vertices, x, y, m_fillColor, strikeThroughOffset, underlineThickness);

//             if (m_outlineThickness != 0)
//                 addLine(m_outlineVertices, x, y, m_outlineColor, strikeThroughOffset, underlineThickness, m_outlineThickness);
//         }

//         // Update the bounding rectangle
//         m_bounds.left = minX;
//         m_bounds.top = minY;
//         m_bounds.width = maxX - minX;
//         m_bounds.height = maxY - minY;
//     }

// } // namespace sf

Text2::Text2()
{
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 6 * 4, NULL, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

}

void Text2::setFont(Font2 &font)
{
    p_font = &font;
}

void Text2::draw(sf::RenderWindow &window)
{
    // activate corresponding render state
    shader.use();
    shader.setVec3("textColor", glm::vec3(0.5, 0.8f, 0.2f));//glm::vec3(m_fillColor.r, m_fillColor.g, m_fillColor.b)/255.f);
    shader.setMat4("projection", window.view.matrix * trans.matrix);

    glActiveTexture(GL_TEXTURE0);
    glBindVertexArray(VAO);

    // iterate through all characters
    const auto scale = trans.mScale.x * 0.5f;
    auto x = 0;//trans.mTranslate.x;
    auto y = 0;//trans.mTranslate.y;

    std::string::const_iterator c;
    auto text = m_string;
    for (c = text.begin(); c != text.end(); c++)
    {
        Character ch = p_font->characters[*c];

        float xpos = x + ch.Bearing.x * scale;
        float ypos = y - (ch.Size.y - ch.Bearing.y) * scale;
        // ypos *= -1;

        float w = ch.Size.x * scale;
        float h = ch.Size.y * scale;
        // update VBO for each character
        float vertices[6][4] = {
            {xpos, ypos - h, 0.0f, 0.0f},
            {xpos, ypos, 0.0f, 1.0f},
            {xpos + w, ypos, 1.0f, 1.0f},

            {xpos, ypos - h, 0.0f, 0.0f},
            {xpos + w, ypos, 1.0f, 1.0f},
            {xpos + w, ypos - h, 1.0f, 0.0f}};
        // render glyph texture over quad
        glBindTexture(GL_TEXTURE_2D, ch.textureID);
        // update content of VBO memory
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        // render quad
        glDrawArrays(GL_TRIANGLES, 0, 6);
        // now advance cursors for next glyph (note that advance is number of 1/64 pixels)
        x += (ch.Advance >> 6) * scale; // bitshift by 6 to get value in pixels (2^6 = 64)
    }
    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void Text2::drawOnScreen(sf::RenderWindow& window)
{

    glm::mat4 projection = glm::ortho(0.f, (float)window.size.x, 0.f, (float)window.size.y);
    // activate corresponding render state
    shader.use();
    shader.setVec3("textColor", glm::vec3(0.5, 0.8f, 0.2f));//glm::vec3(m_fillColor.r, m_fillColor.g, m_fillColor.b)/255.f);
    shader.setMat4("projection",  projection);

    glActiveTexture(GL_TEXTURE0);
    glBindVertexArray(VAO);

    // iterate through all characters
    const auto scale = trans.mScale.x;
    auto x = trans.mTranslate.x;
    auto y = trans.mTranslate.y;

    std::string::const_iterator c;
    auto text = m_string;
    for (c = text.begin(); c != text.end(); c++)
    {
        Character ch = p_font->characters[*c];

        float xpos = x + ch.Bearing.x * scale;
        float ypos = y - (ch.Size.y - ch.Bearing.y) * scale;

        float w = ch.Size.x * scale;
        float h = ch.Size.y * scale;
        // update VBO for each character
        float vertices[6][4] = {
            {xpos, ypos + h, 0.0f, 0.0f},
            {xpos, ypos, 0.0f, 1.0f},
            {xpos + w, ypos, 1.0f, 1.0f},

            {xpos, ypos + h, 0.0f, 0.0f},
            {xpos + w, ypos, 1.0f, 1.0f},
            {xpos + w, ypos + h, 1.0f, 0.0f}};
        // render glyph texture over quad
        glBindTexture(GL_TEXTURE_2D, ch.textureID);
        // update content of VBO memory
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        // render quad
        glDrawArrays(GL_TRIANGLES, 0, 6);
        // now advance cursors for next glyph (note that advance is number of 1/64 pixels)
        x += (ch.Advance >> 6) * scale; // bitshift by 6 to get value in pixels (2^6 = 64)
    }
    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, 0);
}



    void Text2::setFillColor(const sf::Color& color){
        m_fillColor = color;
    }

    void Text2::setString(const std::string string){
        m_string = string;
    }


