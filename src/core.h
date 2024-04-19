#include <cmath>
#include <cstdint>
#include <cassert>
#include <array>
#include <chrono>
#include <numbers>
#include <vector>
#include <algorithm>

#include "Utils/Vector.hpp"
#include "Graphics/Color.hpp"
// #include "SFML/Graphics.hpp"

// #define and && //! for reasons, I have yet to understand, some compilers/versions of c++ do not have and :(
// #define or ||  //! I like using these so I gotta define them myself
// #define xor ^

#ifndef M_PIf
#define M_PIf std::numbers::pi_v<float>
#endif 
#ifndef MAXFLOAT
#define MAXFLOAT  3.402823466e+38;
#endif 


//! OMP THREADS
#ifndef NUM_OMP_INTERACTION_THREADS
#define NUM_OMP_INTERACTION_THREADS 6
#endif
#ifndef NUM_OMP_FOW_THREADS
#define NUM_OMP_FOW_THREADS 6
#endif
#ifndef NUM_OMP_NS_THREADS
#define NUM_OMP_NS_THREADS 6
#endif
#ifndef NUM_OMP_WALLS_THREADS
#define NUM_OMP_WALLS_THREADS 6
#endif


typedef uint32_t u_int_32_t;
typedef uint16_t u_int_16_t;

#pragma once

constexpr int N_MAX_NAVIGABLE_BOIDS = 5000;
constexpr int N_MAX_NEIGHBOURS = 500;
constexpr int  MAX_N_AGENTS_IN_PHYSICS_CELLS = 500;


const std::array<sf::Vector2f, 8> t_vectors = {sf::Vector2f{1.f, 0.f}, sf::Vector2f{1.f / M_SQRT2, 1.f / M_SQRT2},
                                               sf::Vector2f{0, 1},     sf::Vector2f{-1 / M_SQRT2, 1 / M_SQRT2},
                                               sf::Vector2f{-1, 0},    sf::Vector2f{-1 / M_SQRT2, -1 / M_SQRT2},
                                               sf::Vector2f{0, -1},    sf::Vector2f{1 / M_SQRT2, -1 / M_SQRT2}};

typedef sf::Vector2<int> Vertex;
typedef uint32_t TriInd;
typedef uint32_t VertInd;
typedef int BoidInd;

constexpr int N_PLAYERS = 2;
constexpr int N_MAX_THREADS = 12; //! how to find this at compile time?

struct EdgeVInd {
    VertInd from = -1;
    VertInd to = -1;

    EdgeVInd(VertInd from, VertInd to)
        : from(from)
        , to(to) {}
    EdgeVInd() = default;

    bool operator==(const EdgeVInd& e) const {
        return (from == e.from and to == e.to) or (from == e.to and to == e.from);
    }
};

enum Orientation { RIGHTLEFT = 0, RIGHTDOWNLEFTUP = 1, DOWNUP = 2, LEFTDOWNRIGHTUP = 3 };

enum Direction : int {
    RIGHT = 0,
    RIGHTDOWN = 1,
    DOWN = 2,
    LEFTDOWN = 3,
    LEFT = 4,
    LEFTUP = 5,
    UP = 6,
    RIGHTUP = 7,
};

inline Orientation direction2Orientation(Direction dir) { return Orientation(dir % 4); }

constexpr float RAVOID = 10;
constexpr float RSCATTER = 50;
constexpr float RHARD = 3;
constexpr float RFLOCK = 50;
constexpr float RALLIGN = 50;

template <typename T> inline float dot(const T& a, const T& b) { return a.x * b.x + a.y * b.y; }
template <typename T> inline float dot(const T&& a, const T&& b) { return a.x * b.x + a.y * b.y; }

template <typename T> inline float norm2(const T& a) { return dot(a, a); }
template <typename T> inline float norm(const T& a) { return std::sqrt(norm2(a)); }

template <typename T> inline float dist2(const T& a, const T& b) { return dot(a - b, a - b); }

template <typename T> inline float dist2(const T&& a, const T&& b) { return dot(a - b, a - b); }

template <typename T> inline float dist(const T& a, const T& b) { return std::sqrt(dist2(a, b)); }
template <typename T> inline float dist(const T&& a, const T&& b) { return std::sqrt(dist2(a, b)); }

constexpr float TOLERANCE = 0.001f;
template <typename T> inline bool vequal(const T& a, const T& b) { return dist2(a, b) < TOLERANCE; }

template <int NBINS> struct AcosTable {

    std::array<float, NBINS> dot_2_angle_;

    AcosTable() {
        for (int bin = 0; bin < NBINS; ++bin) {
            const float dot_value = 2 * static_cast<float>(bin) / NBINS - 1;
            dot_2_angle_[bin] = 180.f / (M_PIf)*std::acos(dot_value);
        }
    }

    template <typename T> float angleBetween(const T& v1, const T& v2) const {
        const auto dot_value = dot(v2, v1) / norm(v1) / norm(v2) + 1.0f;
        int bin = std::floor(dot_value / 2.f * NBINS);
        //        assert(bin >= 0 and bin<=NBINS);
        if (bin >= NBINS) {
            bin = NBINS - 1;
        }
        if (bin < 0) {
            bin = 0;
        }
        return dot_2_angle_[bin] * (2.f * ((v1.x * v2.y - v2.x * v1.y) > 0) - 1.f);
    }

    template <typename T> float angle(const T& v) const {
        const auto dot_value = dot(v, {1, 0}) / norm(v) + 1.0f;
        int bin = std::floor(dot_value / 2.f * NBINS);
        //        assert(bin >= 0 and bin<=NBINS);
        if (bin >= NBINS) {
            bin = NBINS - 1;
        }
        if (bin < 0) {
            bin = 0;
        }
        return dot_2_angle_[bin] * (2.f * (v.y > 0) - 1.f);
    }
};

template <typename T>
inline float angleBetween(const T& v1,
                          const T& v2) { //! I should just tabelate this, because this is stupidely expensive
    return 180.f / (M_PIf)*std::acos(dot(v2, v1) / norm(v1) / norm(v2)) *
           (2.f * ((v1.x * v2.y - v2.x * v1.y) > 0) - 1.f);
}

template <typename T>
inline float angleBetweenNormed(const T& v1,
                                const T& v2) { //! same as angleBetween but assumes that vectors are normed to save time
    return 180.f / (M_PIf)*std::acos(dot(v2, v1)) * (2.f * ((v1.x * v2.y - v2.x * v1.y) > 0) - 1.f);
}

template <typename T> inline float angle(const T& v) {
    return 180. / M_PI * std::acos(dot(v, {1, 0}) / norm(v)) * (2 * (v.y > 0) - 1);
}

template <typename VectorType>
inline VectorType calcTriangleCOM(const VectorType& v1, const VectorType& v2, const VectorType& v3) {
    return (v1 + v2 + v3) / 3;
}



template <typename VectorType>
inline float calcTrianglesDistance(VectorType v11, VectorType v12, VectorType v13, VectorType v21, VectorType v22,
                                   VectorType v23) {
    VectorType vt1 = calcTriangleCOM(v11, v12, v13);
    VectorType vt2 = calcTriangleCOM(v21, v22, v23);
    return std::sqrt(dist2(vt1, vt2));
}

inline sf::Vector2f asFloat(const sf::Vector2i& r) { return static_cast<sf::Vector2f>(r); }
inline sf::Vector2f asFloat(const sf::Vector2<u_int16_t>& r) { return static_cast<sf::Vector2f>(r); }
//
// inline sf::Vector2f asFloat(const Vertex& r){
//    return static_cast<sf::Vector2f> (r);
//}

template <typename VectorType> inline VectorType inDirection(const VectorType&& v, const VectorType&& dir) {
    return (dir / dot(dir, dir)) * dot(v, dir);
}

inline bool isInRect(sf::Vector2f r, sf::Vector2f lower_left, sf::Vector2f upper_right) {
    bool is_in_x = r.x > std::min(lower_left.x, upper_right.x) && r.x <= std::max(lower_left.x, upper_right.x);
    bool is_in_y = r.y > std::min(lower_left.y, upper_right.y) && r.y <= std::max(lower_left.y, upper_right.y);
    return is_in_x && is_in_y;
}

template <typename VectorType> struct EdgeT {
    VectorType from;
    sf::Vector2f t = {0.f, 0.f};
    float l = 0;

    EdgeT(const VectorType& v1, const VectorType& v2)
        : from(v1) {
        const auto t_new = static_cast<sf::Vector2f>(v2 - v1);
        l = norm(t_new);
        t = t_new / l;
    }
    EdgeT() = default;
    sf::Vector2f to() const { return static_cast<sf::Vector2f>(from) + t * l; }
};

struct Edgef {
    sf::Vector2f from;
    sf::Vector2f t = {0.f, 0.f};
    float l = 0;

    Edgef(const sf::Vector2f& v1, const sf::Vector2f& v2)
        : from(v1) {
        const auto t_new = static_cast<sf::Vector2f>(v2 - v1);
        l = norm(t_new);
        t = t_new / l;
    }
    Edgef(const Vertex& v1, const Vertex& v2)
        : Edgef(asFloat(v1), asFloat(v2)) {}
    Edgef() = default;
    sf::Vector2f to() const { return from + t * l; }
};

inline float distanceFromSegment(const sf::Vector2f& r, const Edgef& segment) {
    const auto dr = r - segment.from;
    const auto alpha = dot(dr, segment.t);
    const sf::Vector2f n = {segment.t.y, -segment.t.x};

    if (0 <= alpha and alpha < segment.l) {
        return std::abs(dot(dr, n));
    }
    if (alpha >= segment.l) {
        return dist(r, segment.to());
    }
    return dist(r, segment.from);
}

inline float distanceOfSegments(const Edgef& s1, const Edgef& s2) {
    return std::min({distanceFromSegment(s1.from, s2), distanceFromSegment(s1.to(), s2),
                     distanceFromSegment(s2.from, s1), distanceFromSegment(s2.to(), s1)});
}

struct EdgeI {
    Vertex from;
    Vertex t;

    EdgeI() = default;
    EdgeI(const Vertex& v1, const Vertex& v2)
        : from(v1)
        , t(v2 - v1) {}

    //    EdgeI(const EdgeI& e) : from(e.from), t(e.t) {}

    float length() const { return norm(t); }
    Vertex to() const { return from + t; }
    bool operator==(const EdgeI& e) const { return e.from == from and e.t == t; }
};

typedef EdgeT<sf::Vector2i> Edge;
// typedef EdgeT<sf::Vector2f> Edgef;

template <typename VectorType> inline float dist2Edge(const sf::Vector2f& r, const EdgeT<VectorType>& e) {

    sf::Vector2f n_wall = {e.t.y, -e.t.x};
    auto dr = r - static_cast<sf::Vector2f>(e.from);
    auto normal_dist2 = dot(dr, n_wall) * dot(dr, n_wall);
    auto dist2_to_v1 = dist2(r, e.from);
    auto dist2_to_v2 = dist2(r, e.to);
    return std::min(dist2_to_v1, std::min(normal_dist2, dist2_to_v2));
}

template <typename EdgeType> inline sf::Vector2f closestPointOnEdge(const sf::Vector2f& r, const EdgeType& e) {

    auto dr = r - static_cast<sf::Vector2f>(e.from);
    sf::Vector2f point_on_edge;
    const auto position_on_edge = dot(dr, e.t);
    if (position_on_edge < e.l and position_on_edge > 0) {
        point_on_edge = static_cast<sf::Vector2f>(e.from) + e.t * position_on_edge;
    } else if (position_on_edge > e.l) {
        point_on_edge = e.to();
    } else {
        point_on_edge = static_cast<sf::Vector2f>(e.from);
    }

    return point_on_edge;
}

inline sf::Vector2f randPoint(const float& x_min, const float& x_max, const float& y_min, const float&  y_max){
    float&& x = static_cast<float>(rand()) / RAND_MAX * (x_max - x_min) + x_min;
    float&& y = static_cast<float>(rand()) / RAND_MAX * (y_max - y_min) + y_min;
    return {x, y};
}

inline sf::Vector2f randPoint(const sf::Vector2f& r_upper_left, const sf::Vector2f& size){
    return randPoint(r_upper_left.x, r_upper_left.x + size.x, r_upper_left.y, r_upper_left.y + size.y);
}




enum class MoveState {
    MOVING,
    STANDING,
    HOLDING
};