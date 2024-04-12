#pragma once

#include <string>
#include <iostream>
#include <filesystem>

#include "../Utils/Vector.hpp"
#include "../include/glad/glad.h"

#include "Rect.hpp"

namespace sf
{
    // struct Texture{
    //     GLuint texture_handle;
    //     sf::Vector2i size;

    //     sf::Vector2i getSize()const;
    // };

    class Image;

struct Texture
{
    int width, height, nrChannels;

    GLuint quadVAO2;
    u_int64_t  m_cacheID;       ///< Unique number that identifies the texture to the render target's cache
    GLuint texture_handle;
    sf::Vector2i size = {0,0};
    unsigned int textureID;
    bool is_smooth = true;

    Texture(int width, int height, std::string filename = "");
    Texture(){}

    bool create(int width, int height);
    bool create(sf::Vector2i size);

    sf::Vector2i getSize()const;

    void loadTextureFromFile(std::string filename);
    bool loadFromFile(const std::filesystem::path& filename, const IntRect& area);
    bool loadFromFile(const std::filesystem::path& filename);
    bool loadFromImage(const Image& image, const IntRect& area);
    bool loadFromImage(const Image& image);
    unsigned char *getData();

    void swap(sf::Texture& other);

    void  update(const Image& image, const Vector2u& dest);
    void  update(const Image& image);
    
    void update(const std::uint8_t* pixels, const Vector2u& size, const Vector2u& dest);
private:
    unsigned char *data = nullptr;
};



}
