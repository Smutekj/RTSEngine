
#pragma once

#include <string>
#include <cstddef>
#include <cstdint>
#include <map>

#include "../Utils/Vector.hpp"
#include "../ShaderNames.h"

#include "Rect.hpp"
#include "Color.hpp"
#include "VertexArray.hpp"
// #include "Shader.h"

// namespace sf
// {
// class Font;
// class RenderTarget;



// ////////////////////////////////////////////////////////////
// /// \brief Graphical text that can be drawn to a render target
// ///
// ////////////////////////////////////////////////////////////
// class Text : public Transformable
// {
// public:
//     ////////////////////////////////////////////////////////////
//     /// \brief Enumeration of the string drawing styles
//     ///
//     ////////////////////////////////////////////////////////////
//     enum Style
//     {
//         Regular       = 0,      //!< Regular characters, no style
//         Bold          = 1 << 0, //!< Bold characters
//         Italic        = 1 << 1, //!< Italic characters
//         Underlined    = 1 << 2, //!< Underlined characters
//         StrikeThrough = 1 << 3  //!< Strike through characters
//     };

//     ////////////////////////////////////////////////////////////
//     /// \brief Construct the text from a string, font and size
//     ///
//     /// Note that if the used font is a bitmap font, it is not
//     /// scalable, thus not all requested sizes will be available
//     /// to use. This needs to be taken into consideration when
//     /// setting the character size. If you need to display text
//     /// of a certain size, make sure the corresponding bitmap
//     /// font that supports that size is used.
//     ///
//     /// \param string         Text assigned to the string
//     /// \param font           Font used to draw the string
//     /// \param characterSize  Base size of characters, in pixels
//     ///
//     ////////////////////////////////////////////////////////////
//     Text();

//     Text(const Font& font, std::string string = "", unsigned int characterSize = 30);

//     ////////////////////////////////////////////////////////////
//     /// \brief Disallow construction from a temporary font
//     ///
//     ////////////////////////////////////////////////////////////
//     Text(Font&& font, std::string string = "", unsigned int characterSize = 30) = delete;


//     void setString(const std::string& string);

//     void setFont(const Font& font);

//     void setFont(Font&& font) = delete;

//     void setCharacterSize(unsigned int size);

//     void setLineSpacing(float spacingFactor);

//     void setLetterSpacing(float spacingFactor);
//     void setStyle(std::uint32_t style);

//     void setFillColor(const Color& color);

//     void setOutlineColor(const Color& color);

//     void setOutlineThickness(float thickness);

//     const std::string& getString() const;

//     const Font& getFont() const;

//     unsigned int getCharacterSize() const;

//     float getLetterSpacing() const;
//     float getLineSpacing() const;

//     std::uint32_t getStyle() const;

//     const Color& getFillColor() const;

//     const Color& getOutlineColor() const;

//     float getOutlineThickness() const;

//     Vector2f findCharacterPos(std::size_t index) const;


//     FloatRect getLocalBounds() const;

//     FloatRect getGlobalBounds() const;

//     void draw(RenderWindow& target) const ;
// private:
//     ////////////////////////////////////////////////////////////



//     ////////////////////////////////////////////////////////////
//     /// \brief Make sure the text's geometry is updated
//     ///
//     /// All the attributes related to rendering are cached, such
//     /// that the geometry is only updated when necessary.
//     ///
//     ////////////////////////////////////////////////////////////
//     void ensureGeometryUpdate() const;

//     ////////////////////////////////////////////////////////////
//     // Member data
//     ////////////////////////////////////////////////////////////
//     std::string           m_string;                                    //!< String to display
//     const Font*           m_font{};                                    //!< Font used to display the string
//     unsigned int          m_characterSize{30};                         //!< Base size of characters, in pixels
//     float                 m_letterSpacingFactor{1.f};                  //!< Spacing factor between letters
//     float                 m_lineSpacingFactor{1.f};                    //!< Spacing factor between lines
//     std::uint32_t         m_style{Regular};                            //!< Text style (see Style enum)
//     Color                 m_fillColor{Color::White};                   //!< Text fill color
//     Color                 m_outlineColor{Color::Black};                //!< Text outline color
//     float                 m_outlineThickness{0.f};                     //!< Thickness of the text's outline
//     mutable VertexArray   m_vertices{PrimitiveType::Triangles};        //!< Vertex array containing the fill geometry
//     mutable VertexArray   m_outlineVertices{PrimitiveType::Triangles}; //!< Vertex array containing the outline geometry
//     mutable FloatRect     m_bounds;               //!< Bounding rectangle of the text (in local coordinates)
//     mutable bool          m_geometryNeedUpdate{}; //!< Does the geometry need to be recomputed?
//     mutable std::uint64_t m_fontTextureId{};      //!< The font texture id
//     GLuint                VAO;
//     GLuint                VBO;
// };


// } // namespace sf


class Font2;


//! This does basically the same thing as sf::Text but worse 
struct Text2 : sf::Transformable{

    Text2();
    void setFont(Font2& font);

    void draw(sf::RenderWindow& window);

    //! draw directly on screen (not on world map)
    void drawOnScreen(sf::RenderWindow& window);

    void setFillColor(const sf::Color& color);

    void setString(const std::string string);

// private:
    // bool needsUpdate();

    std::string           m_string;                                    //!< String to display
    unsigned int          m_characterSize{30};                         //!< Base size of characters, in pixels
    float                 m_letterSpacingFactor{1.f};                  //!< Spacing factor between letters
    float                 m_lineSpacingFactor{1.f};                    //!< Spacing factor between lines
    sf::Color             m_fillColor{sf::Color::Black};                   //!< Text fill color
    mutable sf::FloatRect     m_bounds;               //!< Bounding rectangle of the text (in local coordinates)
    mutable bool          m_geometryNeedUpdate{}; //!< Does the geometry need to be recomputed?
    Font2*                p_font;
    Shader                shader = {s_vertexShaderText, s_fragShaderText};
    GLuint                VAO;
    GLuint                VBO;
};


