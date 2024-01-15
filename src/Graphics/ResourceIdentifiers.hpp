#ifndef BOOK_RESOURCEIDENTIFIERS_HPP
#define BOOK_RESOURCEIDENTIFIERS_HPP


// Forward declaration of SFML classes
namespace sf {
class Texture;
class Font;
class Shader;
class SoundBuffer;
} // namespace sf

namespace Textures {
enum ID {
    Entities,
    Jungle,
    TitleScreen,
    Buttons,
    Explosion,
    Particle,
    FinishLine,
};
}

namespace Shaders {
enum ID {
    BrightnessPass,
    DownSamplePass,
    GaussianBlurPass,
    AddPass,
};
}

namespace Fonts {
enum ID {
    Main,
};
}

namespace Sounds {
enum ID {
    Pop,
};
}

// Forward declaration and a few type definitions
template <typename Resource, typename Identifier> class ResourceHolder;

typedef ResourceHolder<sf::Texture, Textures::ID> TextureHolder;
typedef ResourceHolder<sf::Font, Fonts::ID> FontHolder;
typedef ResourceHolder<sf::Shader, Shaders::ID> ShaderHolder;
typedef ResourceHolder<sf::SoundBuffer, Sounds::ID> SoundHolder;

#endif // BOOK_RESOURCEIDENTIFIERS_HPP
