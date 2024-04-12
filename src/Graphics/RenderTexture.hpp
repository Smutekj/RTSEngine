#pragma once

#include "../include/glad/glad.h"
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include "RenderTarget.hpp"
#include "Texture.hpp"

namespace sf
{
    struct RenderTexture : RenderTarget
    {
        GLuint framebuffer_handle ;
        GLuint texture_handle ;
        sf::Texture texture;

        RenderTexture();

        void clear();
        void display();

        void draw();

        int create(int n_x_pixels = 800, int n_y_pixels = 600);

        GLuint getTextureHandle();
    private:
        void createFramebuffer(GLuint &fbo, GLuint &fbtex, sf::Vector2i  size);
    };
}


