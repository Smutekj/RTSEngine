#include "RenderTexture.hpp"
#include "VertexArray.hpp"

#include "../include/glad/glad.h"
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

namespace sf{

RenderTexture::RenderTexture()
{

}

void RenderTexture::clear(){
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_handle);
    glClearColor(1.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //glViewport(0,0,texture.size.x, texture.size.y);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void RenderTexture::display(){
    
}

void RenderTexture::draw()
{

}

int RenderTexture::create(int n_x_pixels, int n_y_pixels)
{
    createFramebuffer(framebuffer_handle, texture_handle, {n_x_pixels, n_y_pixels});
    return 1;
}

GLuint RenderTexture::getTextureHandle(){
    return texture_handle;
}

void RenderTexture::createFramebuffer(GLuint &fbo, GLuint &fbtex, sf::Vector2i  size)
{
    texture.size = size;
    glGenTextures(1, &fbtex);
    glBindTexture(GL_TEXTURE_2D, fbtex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, size.x, size.y, 0, GL_BGR, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo);
    glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fbtex, 0);
    {
        GLenum status;
        status = glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER);
        switch (status)
        {
        case GL_FRAMEBUFFER_COMPLETE:
            break;
        case GL_FRAMEBUFFER_UNSUPPORTED:
            // std::cerr << "Error: unsupported framebuffer format" << std::endl;
            exit(0);
        default:
            // std::cerr << "Error: invalid framebuffer config" << std::endl;
            exit(0);
        }
    }
}

};// !namespace sf