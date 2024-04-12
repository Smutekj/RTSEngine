#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <map>
#include <unordered_map>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <ft2build.h>
#include FT_FREETYPE_H
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/matrix_transform_2d.hpp>
#include <glm/gtc/type_ptr.hpp>


#include "../Utils/Vector.hpp"

#include "Rect.hpp"
#include "Texture.hpp"





struct Character {
    unsigned int textureID;  // ID handle of the glyph texture
    glm::ivec2   Size;       // Size of glyph
    glm::ivec2   Bearing;    // Offset from baseline to left/top of glyph
    unsigned int Advance;    // Offset to advance to next glyph
};



    struct Font2
    {

        std::map<char, Character> characters;

        Font2()
        {

            FT_Library ft;
            FT_Face face;
            if (FT_Init_FreeType(&ft))
            {
                throw std::runtime_error("ERROR:::FREETYPE: Could not init FreeType Library");
            }

            if (FT_New_Face(ft, "/home/smutekj/projects/RTSEngineWithOpenGL/arial.ttf", 0, &face))
            {
                throw std::runtime_error("ERROR::FREETYPE: Failed to load font");
            }
            FT_Set_Pixel_Sizes(face, 0, 48);

            ///! INITIALIZE CHARACTERS
            glPixelStorei(GL_UNPACK_ALIGNMENT, 1); // disable byte-alignment restriction

            for (unsigned char c = 0; c < 128; c++)
            {
                // load character glyph
                if (FT_Load_Char(face, c, FT_LOAD_RENDER))
                {
                    std::cout << "ERROR::FREETYTPE: Failed to load Glyph" << std::endl;
                    continue;
                }
                // generate texture
                unsigned int texture;
                glGenTextures(1, &texture);
                glBindTexture(GL_TEXTURE_2D, texture);
                glTexImage2D(
                    GL_TEXTURE_2D,
                    0,
                    GL_RED,
                    face->glyph->bitmap.width,
                    face->glyph->bitmap.rows,
                    0,
                    GL_RED,
                    GL_UNSIGNED_BYTE,
                    face->glyph->bitmap.buffer);
                // set texture options
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                // now store character for later use
                Character character = {
                    texture,
                    glm::ivec2(face->glyph->bitmap.width, face->glyph->bitmap.rows),
                    glm::ivec2(face->glyph->bitmap_left, face->glyph->bitmap_top),
                    static_cast<unsigned int>(face->glyph->advance.x)};
                characters.insert(std::pair<char, Character>(c, character));
            }

            glBindTexture(GL_TEXTURE_2D, 0);
            
            FT_Done_Face(face);
            FT_Done_FreeType(ft);
        
        
        }

    private:

    };
