////////////////////////////////////////////////////////////
//
// SFML - Simple and Fast Multimedia Library
// Copyright (C) 2007-2024 Laurent Gomila (laurent@sfml-dev.org)
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
#include "Image.hpp"

// #include <SFML/System/Err.hpp>
#include "InputStream.hpp"
#include "Utils.hpp"


#include <algorithm>
#include <iomanip>
#include <memory>
#include <ostream>
#include <string>

#include <cassert>
#include <cstring>

#include "stb_image.h"
#include "stb_image_write.h"



namespace
{
    // stb_image callbacks that operate on a sf::InputStream
    int read(void* user, char* data, int size)
    {
        auto& stream = *static_cast<sf::InputStream*>(user);
        return static_cast<int>(stream.read(data, size));
    }

    void skip(void* user, int size)
    {
        auto& stream = *static_cast<sf::InputStream*>(user);
        if (stream.seek(stream.tell() + size) == -1)
            throw std::runtime_error("Failed to seek image loader input stream\n");
    }

    int eof(void* user)
    {
        auto& stream = *static_cast<sf::InputStream*>(user);
        return stream.tell() >= stream.getSize();
    }

    // stb_image callback for constructing a buffer
    void bufferFromCallback(void* context, void* data, int size)
    {
        const auto* source = static_cast<std::uint8_t*>(data);
        auto*       dest   = static_cast<std::vector<std::uint8_t>*>(context);
        std::copy(source, source + size, std::back_inserter(*dest));
    }

    // Deleter for STB pointers
    struct StbDeleter
    {
        void operator()(stbi_uc* image) const
        {
            stbi_image_free(image);
        }
    };
    using StbPtr = std::unique_ptr<stbi_uc, StbDeleter>;
} // namespace


namespace sf
{
    ////////////////////////////////////////////////////////////
    void Image::create(const Vector2u& size, const Color& color)
    {
        if (size.x && size.y)
        {
            // Create a new pixel buffer first for exception safety's sake
            std::vector<std::uint8_t> newPixels(static_cast<std::size_t>(size.x) * static_cast<std::size_t>(size.y) * 4);

            // Fill it with the specified color
            std::uint8_t* ptr = newPixels.data();
            std::uint8_t* end = ptr + newPixels.size();
            while (ptr < end)
            {
                *ptr++ = color.r;
                *ptr++ = color.g;
                *ptr++ = color.b;
                *ptr++ = color.a;
            }

            // Commit the new pixel buffer
            m_pixels.swap(newPixels);

            // Assign the new size
            m_size = size;
        }
        else
        {
            // Dump the pixel buffer
            std::vector<std::uint8_t>().swap(m_pixels);

            // Assign the new size
            m_size = {};
        }
    }


    ////////////////////////////////////////////////////////////
    void Image::create(const Vector2u& size, const std::uint8_t* pixels)
    {
        if (pixels && size.x && size.y)
        {
            // Create a new pixel buffer first for exception safety's sake
            std::vector<std::uint8_t> newPixels(pixels, pixels + size.x * size.y * 4);

            // Commit the new pixel buffer
            m_pixels.swap(newPixels);

            // Assign the new size
            m_size = size;
        }
        else
        {
            // Dump the pixel buffer
            std::vector<std::uint8_t>().swap(m_pixels);

            // Assign the new size
            m_size = {};
        }
    }


////////////////////////////////////////////////////////////
bool Image::loadFromFile(const std::filesystem::path& filename)
{
#ifndef SFML_SYSTEM_ANDROID

    // Clear the array (just in case)
    m_pixels.clear();

    // Load the image and get a pointer to the pixels in memory
    int        width    = 0;
    int        height   = 0;
    int        channels = 0;
    const auto ptr      = StbPtr(stbi_load(filename.string().c_str(), &width, &height, &channels, STBI_rgb_alpha));

    if (ptr)
    {
        // Assign the image properties
        m_size = Vector2u(Vector2i(width, height));

        // Copy the loaded pixels to the pixel buffer
        m_pixels.assign(ptr.get(), ptr.get() + width * height * 4);

        return true;
    }
    else
    {
    //     // Error, failed to load the image
    //     err() << "Failed to load image\n"
    //           << formatDebugPathInfo(filename) << "\nReason: " << stbi_failure_reason() << std::endl;

        return false;
    }

#else

    priv::ResourceStream stream(filename);
    return loadFromStream(stream);

#endif
}


////////////////////////////////////////////////////////////
bool Image::loadFromMemory(const void* data, std::size_t size)
{
    // Check input parameters
    if (data && size)
    {
        // Clear the array (just in case)
        m_pixels.clear();

        // Load the image and get a pointer to the pixels in memory
        int         width    = 0;
        int         height   = 0;
        int         channels = 0;
        const auto* buffer   = static_cast<const unsigned char*>(data);
        const auto  ptr      = StbPtr(
            stbi_load_from_memory(buffer, static_cast<int>(size), &width, &height, &channels, STBI_rgb_alpha));

        if (ptr)
        {
            // Assign the image properties
            m_size = Vector2u(Vector2i(width, height));

            // Copy the loaded pixels to the pixel buffer
            m_pixels.assign(ptr.get(), ptr.get() + width * height * 4);

            return true;
        }
        else
        {
            // Error, failed to load the image
            // err() << "Failed to load image from memory. Reason: " << stbi_failure_reason() << std::endl;

            return false;
        }
    }
    else
    {
        // err() << "Failed to load image from memory, no data provided" << std::endl;
        return false;
    }
}


////////////////////////////////////////////////////////////
bool Image::loadFromStream(InputStream& stream)
{
    // Clear the array (just in case)
    m_pixels.clear();

    // Make sure that the stream's reading position is at the beginning
    if (stream.seek(0) == -1)
    {
        // err() << "Failed to seek image stream" << std::endl;
        return false;
    }

    // Setup the stb_image callbacks
    stbi_io_callbacks callbacks;
    callbacks.read = read;
    callbacks.skip = skip;
    callbacks.eof  = eof;

    // Load the image and get a pointer to the pixels in memory
    int        width    = 0;
    int        height   = 0;
    int        channels = 0;
    const auto ptr = StbPtr(stbi_load_from_callbacks(&callbacks, &stream, &width, &height, &channels, STBI_rgb_alpha));

    if (ptr)
    {
        // Assign the image properties
        m_size = Vector2u(Vector2i(width, height));

        // Copy the loaded pixels to the pixel buffer
        m_pixels.assign(ptr.get(), ptr.get() + width * height * 4);

        return true;
    }
    else
    {
        // Error, failed to load the image
        throw std::runtime_error("Failed to load image from stream. Reason: " + std::string(stbi_failure_reason()));
        return false;
    }
}


////////////////////////////////////////////////////////////
bool Image::saveToFile(const std::filesystem::path& filename) const
{
    // Make sure the image is not empty
    if (!m_pixels.empty() && m_size.x > 0 && m_size.y > 0)
    {
        // Deduce the image type from its extension

        // Extract the extension
        const std::filesystem::path extension     = filename.extension();
        const Vector2i              convertedSize = Vector2i(m_size);

        if (extension == ".bmp")
        {
            // BMP format
            if (stbi_write_bmp(filename.string().c_str(), convertedSize.x, convertedSize.y, 4, m_pixels.data()))
                return true;
        }
        else if (extension == ".tga")
        {
            // TGA format
            if (stbi_write_tga(filename.string().c_str(), convertedSize.x, convertedSize.y, 4, m_pixels.data()))
                return true;
        }
        else if (extension == ".png")
        {
            // PNG format
            if (stbi_write_png(filename.string().c_str(), convertedSize.x, convertedSize.y, 4, m_pixels.data(), 0))
                return true;
        }
        else if (extension == ".jpg" || extension == ".jpeg")
        {
            // JPG format
            if (stbi_write_jpg(filename.string().c_str(), convertedSize.x, convertedSize.y, 4, m_pixels.data(), 90))
                return true;
        }
        else
        {
           throw std::runtime_error( "Image file extension " + extension.string() + " not supported\n");
        }
    }

    // err() << "Failed to save image\n" << formatDebugPathInfo(filename) << std::endl;
    return false;
}


////////////////////////////////////////////////////////////
std::optional<std::vector<std::uint8_t>> Image::saveToMemory(std::string_view format) const
{
    // Make sure the image is not empty
    if (!m_pixels.empty() && m_size.x > 0 && m_size.y > 0)
    {
        // Choose function based on format
        const std::string specified     = toLower(std::string(format));
        const Vector2i    convertedSize = Vector2i(m_size);

        std::vector<std::uint8_t> buffer;

        if (specified == "bmp")
        {
            // BMP format
            if (stbi_write_bmp_to_func(bufferFromCallback, &buffer, convertedSize.x, convertedSize.y, 4, m_pixels.data()))
                return buffer;
        }
        else if (specified == "tga")
        {
            // TGA format
            if (stbi_write_tga_to_func(bufferFromCallback, &buffer, convertedSize.x, convertedSize.y, 4, m_pixels.data()))
                return buffer;
        }
        else if (specified == "png")
        {
            // PNG format
            if (stbi_write_png_to_func(bufferFromCallback, &buffer, convertedSize.x, convertedSize.y, 4, m_pixels.data(), 0))
                return buffer;
        }
        else if (specified == "jpg" || specified == "jpeg")
        {
            // JPG format
            if (stbi_write_jpg_to_func(bufferFromCallback, &buffer, convertedSize.x, convertedSize.y, 4, m_pixels.data(), 90))
                return buffer;
        }
    }

    // err() << "Failed to save image with format " << std::quoted(format) << std::endl;
    return std::nullopt;
}


////////////////////////////////////////////////////////////
Vector2u Image::getSize() const
{
    return m_size;
}


////////////////////////////////////////////////////////////
void Image::createMaskFromColor(const Color& color, std::uint8_t alpha)
{
    // Make sure that the image is not empty
    if (!m_pixels.empty())
    {
        // Replace the alpha of the pixels that match the transparent color
        std::uint8_t* ptr = m_pixels.data();
        std::uint8_t* end = ptr + m_pixels.size();
        while (ptr < end)
        {
            if ((ptr[0] == color.r) && (ptr[1] == color.g) && (ptr[2] == color.b) && (ptr[3] == color.a))
                ptr[3] = alpha;
            ptr += 4;
        }
    }
}


////////////////////////////////////////////////////////////
[[nodiscard]] bool Image::copy(const Image& source, const Vector2u& dest, const IntRect& sourceRect, bool applyAlpha)
{
    // Make sure that both images are valid
    if (source.m_size.x == 0 || source.m_size.y == 0 || m_size.x == 0 || m_size.y == 0)
        return false;

    // Make sure the sourceRect components are non-negative before casting them to unsigned values
    if (sourceRect.left < 0 || sourceRect.top < 0 || sourceRect.width < 0 || sourceRect.height < 0)
        return false;

    Rect<unsigned int> srcRect(sourceRect);

    // Use the whole source image as srcRect if the provided source rectangle is empty
    if (srcRect.width == 0 || srcRect.height == 0)
    {
        srcRect = Rect<unsigned int>({0, 0}, source.m_size);
    }
    // Otherwise make sure the provided source rectangle fits into the source image
    else
    {
        // Checking the bottom right corner is enough because
        // left and top are non-negative and width and height are positive.
        if (source.m_size.x < srcRect.left + srcRect.width || source.m_size.y < srcRect.top + srcRect.height)
            return false;
    }

    // Make sure the destination position is within this image bounds
    if (m_size.x <= dest.x || m_size.y <= dest.y)
        return false;

    // Then find the valid size of the destination rectangle
    const Vector2u dstSize(std::min(m_size.x - dest.x, srcRect.width), std::min(m_size.y - dest.y, srcRect.height));

    // Precompute as much as possible
    const std::size_t  pitch     = static_cast<std::size_t>(dstSize.x) * 4;
    const unsigned int srcStride = source.m_size.x * 4;
    const unsigned int dstStride = m_size.x * 4;

    const std::uint8_t* srcPixels = source.m_pixels.data() + (srcRect.left + srcRect.top * source.m_size.x) * 4;
    std::uint8_t*       dstPixels = m_pixels.data() + (dest.x + dest.y * m_size.x) * 4;

    // Copy the pixels
    if (applyAlpha)
    {
        // Interpolation using alpha values, pixel by pixel (slower)
        for (unsigned int i = 0; i < dstSize.y; ++i)
        {
            for (unsigned int j = 0; j < dstSize.x; ++j)
            {
                // Get a direct pointer to the components of the current pixel
                const std::uint8_t* src = srcPixels + j * 4;
                std::uint8_t*       dst = dstPixels + j * 4;

                // Interpolate RGBA components using the alpha values of the destination and source pixels
                const std::uint8_t srcAlpha = src[3];
                const std::uint8_t dstAlpha = dst[3];
                const auto outAlpha = static_cast<std::uint8_t>(srcAlpha + dstAlpha - srcAlpha * dstAlpha / 255);

                dst[3] = outAlpha;

                if (outAlpha)
                    for (int k = 0; k < 3; k++)
                        dst[k] = static_cast<std::uint8_t>((src[k] * srcAlpha + dst[k] * (outAlpha - srcAlpha)) / outAlpha);
                else
                    for (int k = 0; k < 3; k++)
                        dst[k] = src[k];
            }

            srcPixels += srcStride;
            dstPixels += dstStride;
        }
    }
    else
    {
        // Optimized copy ignoring alpha values, row by row (faster)
        for (unsigned int i = 0; i < dstSize.y; ++i)
        {
            std::memcpy(dstPixels, srcPixels, pitch);
            srcPixels += srcStride;
            dstPixels += dstStride;
        }
    }

    return true;
}


////////////////////////////////////////////////////////////
void Image::setPixel(const Vector2u& coords, const Color& color)
{
    const auto index = (coords.x + coords.y * m_size.x) * 4;
    assert(index < m_pixels.size() && "Image::setPixel() cannot access out of bounds pixel");
    std::uint8_t* pixel = &m_pixels[index];
    *pixel++            = color.r;
    *pixel++            = color.g;
    *pixel++            = color.b;
    *pixel++            = color.a;
}


////////////////////////////////////////////////////////////
Color Image::getPixel(const Vector2u& coords) const
{
    const auto index = (coords.x + coords.y * m_size.x) * 4;
    assert(index < m_pixels.size() && "Image::getPixel() cannot access out of bounds pixel");
    const std::uint8_t* pixel = &m_pixels[index];
    return {pixel[0], pixel[1], pixel[2], pixel[3]};
}


////////////////////////////////////////////////////////////
const std::uint8_t* Image::getPixelsPtr() const
{
    if (!m_pixels.empty())
    {
        return m_pixels.data();
    }
    else
    {
        // err() << "Trying to access the pixels of an empty image" << std::endl;
        return nullptr;
    }
}


////////////////////////////////////////////////////////////
void Image::flipHorizontally()
{
    if (!m_pixels.empty())
    {
        const std::size_t rowSize = m_size.x * 4;

        for (std::size_t y = 0; y < m_size.y; ++y)
        {
            auto left = m_pixels.begin() + static_cast<std::vector<std::uint8_t>::iterator::difference_type>(y * rowSize);
            auto right = m_pixels.begin() +
                         static_cast<std::vector<std::uint8_t>::iterator::difference_type>((y + 1) * rowSize - 4);

            for (std::size_t x = 0; x < m_size.x / 2; ++x)
            {
                std::swap_ranges(left, left + 4, right);

                left += 4;
                right -= 4;
            }
        }
    }
}


////////////////////////////////////////////////////////////
void Image::flipVertically()
{
    if (!m_pixels.empty())
    {
        const auto rowSize = static_cast<std::vector<std::uint8_t>::iterator::difference_type>(m_size.x * 4);

        auto top    = m_pixels.begin();
        auto bottom = m_pixels.end() - rowSize;

        for (std::size_t y = 0; y < m_size.y / 2; ++y)
        {
            std::swap_ranges(top, top + rowSize, bottom);

            top += rowSize;
            bottom -= rowSize;
        }
    }
}

} // namespace sf