#include "Texture.hpp"
#include "Image.hpp"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>


#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"


namespace sf{
Texture::Texture(int width, int height, std::string filename) : width(width), height(height)
    {
        glGenTextures(1, &textureID);
        glBindTexture(GL_TEXTURE_2D, textureID);  
        if (!filename.empty())
        {
            if(!loadFromFile(filename));  
                throw std::runtime_error("no file found!");
            
            // data = stbi_load(filename.c_str(), &width, &height, &nrChannels, 0);
            if (data)
            {
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
                glGenerateMipmap(GL_TEXTURE_2D);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);     
           }
            else
            {
                throw std::runtime_error("texture at filename: " + filename + " not found!\n");
            }
        }else{
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
            // glGenerateMipmap(GL_TEXTURE_2D);
        }
        
    }

////////////////////////////////////////////////////////////
bool Texture::loadFromFile(const std::filesystem::path& filename, const IntRect& area)
{
    Image image;
    return image.loadFromFile(filename) && loadFromImage(image, area);
}

////////////////////////////////////////////////////////////
bool Texture::loadFromFile(const std::filesystem::path& filename)
{
    return loadFromFile(filename, {0, 0, width, height});
}

bool Texture::create(sf::Vector2i size){

// Check if texture parameters are valid before creating it
    if ((size.x == 0) || (size.y == 0))
    {
        // err() << "Failed to create texture, invalid size (" << size.x << "x" << size.y << ")" << std::endl;
        return false;
    }

    // const TransientContextLock lock;

    // Make sure that extensions are initialized
    // priv::ensureExtensionsInit();

    // Compute the internal texture dimensions depending on NPOT textures support
    // const Vector2u actualSize(getValidSize(size.x), getValidSize(size.y));

    // Check the maximum texture size
    // const unsigned int maxSize = getMaximumSize();
    // if ((actualSize.x > maxSize) || (actualSize.y > maxSize))
    // {
    //     err() << "Failed to create texture, its internal size is too high "
    //           << "(" << actualSize.x << "x" << actualSize.y << ", "
    //           << "maximum is " << maxSize << "x" << maxSize << ")" << std::endl;
    //     return false;
    // }

    // All the validity checks passed, we can store the new texture settings
    this->size          = size;
    // m_actualSize    = actualSize;
    // m_pixelsFlipped = false;
    // m_fboAttachment = false;

    // Create the OpenGL texture if it doesn't exist yet
    if (!texture_handle)
    {
        GLuint texture;
        glGenTextures(1, &texture);
        texture_handle = texture;
    }

    // Make sure that the current texture binding will be preserved
    // const priv::TextureSaver save;

    // static const bool textureEdgeClamp = GLEXT_texture_edge_clamp || GLEXT_GL_VERSION_1_2 ||
    //                                      Context::isExtensionAvailable("GL_EXT_texture_edge_clamp");

    // if (!m_isRepeated && !textureEdgeClamp)
    // {
    //     static bool warned = false;

    //     if (!warned)
    //     {
    //         err() << "OpenGL extension SGIS_texture_edge_clamp unavailable" << '\n'
    //               << "Artifacts may occur along texture edges" << '\n'
    //               << "Ensure that hardware acceleration is enabled if available" << std::endl;

    //         warned = true;
    //     }
    // }

    // static const bool textureSrgb = GLEXT_texture_sRGB;

//     if (m_sRgb && !textureSrgb)
//     {
//         static bool warned = false;

//         if (!warned)
//         {
// #ifndef SFML_OPENGL_ES
//             err() << "OpenGL extension EXT_texture_sRGB unavailable" << '\n';
// #else
//             err() << "OpenGL ES extension EXT_sRGB unavailable" << '\n';
// #endif
//             err() << "Automatic sRGB to linear conversion disabled" << std::endl;

//             warned = true;
//         }

//         m_sRgb = false;
//     }

    // const GLint textureWrapParam = m_isRepeated ? GL_REPEAT : GL_CLAMP_TO_EDGE;

    // Initialize the texture
    glBindTexture(GL_TEXTURE_2D, texture_handle);
    glTexImage2D(GL_TEXTURE_2D,
                         0,
                         GL_RGB, width, height,
                         0,
                         GL_RGBA,
                         GL_UNSIGNED_BYTE,
                         nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, is_smooth ? GL_LINEAR : GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, is_smooth ? GL_LINEAR : GL_NEAREST);
    // m_cacheId = TextureImpl::getUniqueId();

    // m_hasMipmap = false;

    return true;

    }

    sf::Vector2i Texture::getSize()const{
        return {width, height};
    }

    void Texture::loadTextureFromFile(std::string filename)
    {
        data = stbi_load(filename.c_str(), &width, &height, &nrChannels, 0);
        if (data)
        {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
            glGenerateMipmap(GL_TEXTURE_2D);
        }
        else
        {
            std::cout << "texture at filename: " << filename << " not found!\n";
        }
    }


////////////////////////////////////////////////////////////
void Texture::update(const Image& image)
{
    // Update the whole texture
    update(image.getPixelsPtr(), image.getSize(), {0, 0});
}


bool Texture::create(int x, int y){
    return create({x, y});
}

////////////////////////////////////////////////////////////
void Texture::update(const Image& image, const Vector2u& dest)
{
    update(image.getPixelsPtr(), image.getSize(), dest);
}

bool Texture::loadFromImage(const Image& image)
{
    return loadFromImage(image, {0, 0, size.x, size.y});
}

bool Texture::loadFromImage(const Image& image, const IntRect& area)
{
    // Retrieve the image size
    const auto [width, height] = Vector2i(image.getSize());

    // Load the entire image if the source area is either empty or contains the whole image
    if (area.width == 0 || (area.height == 0) ||
        ((area.left <= 0) && (area.top <= 0) && (area.width >= width) && (area.height >= height)))
    {
        // Load the entire image
        if (create(image.m_size.x, image.m_size.y))
        {
            update(image);

            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        // Load a sub-area of the image

        // Adjust the rectangle to the size of the image
        IntRect rectangle = area;
        if (rectangle.left < 0)
            rectangle.left = 0;
        if (rectangle.top < 0)
            rectangle.top = 0;
        if (rectangle.left + rectangle.width > width)
            rectangle.width = width - rectangle.left;
        if (rectangle.top + rectangle.height > height)
            rectangle.height = height - rectangle.top;

        // Create the texture and upload the pixels
        if (create(rectangle.getSize().x, rectangle.getSize().y))
        {
            // const TransientContextLock lock;

            // Make sure that the current texture binding will be preserved
            // const priv::TextureSaver save;

            // Copy the pixels to the texture, row by row
            const std::uint8_t* pixels = image.getPixelsPtr() + 4 * (rectangle.left + (width * rectangle.top));
            glBindTexture(GL_TEXTURE_2D, texture_handle);
            for (int i = 0; i < rectangle.height; ++i)
            {
                glTexSubImage2D(GL_TEXTURE_2D, 0, 0, i, rectangle.width, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
                pixels += 4 * width;
            }

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, is_smooth ? GL_LINEAR : GL_NEAREST);
            // m_hasMipmap = false;

            // Force an OpenGL flush, so that the texture will appear updated
            // in all contexts immediately (solves problems in multi-threaded apps)
            glFlush();

            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}


    unsigned char * Texture::getData()
    {
        return data;
    }


    void Texture::swap(Texture& new_texture){
        std::swap(quadVAO2, new_texture.quadVAO2);
        std::swap(texture_handle, new_texture.texture_handle);
        std::swap(data, new_texture.data);
        std::swap(textureID, new_texture.textureID);
        std::swap(size, new_texture.size);
        std::swap(m_cacheID, new_texture.m_cacheID);
        
    }


////////////////////////////////////////////////////////////
void Texture::update(const std::uint8_t* pixels, const Vector2u& size, const Vector2u& dest)
{
    // assert(dest.x + size.x <= m_size.x && "Destination x coordinate is outside of texture");
    // assert(dest.y + size.y <= m_size.y && "Destination y coordinate is outside of texture");

    if (pixels && textureID)
    {
        // const TransientContextLock lock;

        // Make sure that the current texture binding will be preserved
        // const priv::TextureSaver save;

        // Copy pixels from the given array to the texture
        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexSubImage2D(GL_TEXTURE_2D,
                                0,
                                static_cast<GLint>(dest.x),
                                static_cast<GLint>(dest.y),
                                static_cast<GLsizei>(size.x),
                                static_cast<GLsizei>(size.y),
                                GL_RGBA,
                                GL_UNSIGNED_BYTE,
                                pixels);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, is_smooth ? GL_LINEAR : GL_NEAREST);
        // m_hasMipmap     = false;
        // m_pixelsFlipped = false;
        // m_cacheId       = 0;

        // Force an OpenGL flush, so that the texture data will appear updated
        // in all contexts immediately (solves problems in multi-threaded apps)
        glFlush();
    }
}



}
