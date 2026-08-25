#ifndef UTIL_H
#define UTIL_H 

#include <cassert>
#include <ostream>

#include "raylib.h"
#include "raymath.h"

// vector util functions
std::ostream& operator<<(std::ostream& os, const Vector2& v); 

float   dot_product           (const Vector2& a, const Vector2& b);
Vector2 middle                (Vector2 const& p1, Vector2 const& p2);
float   vector_angle          (const Vector2& a, const Vector2& b);
float   perpendicular_distance(const Vector2& p, 
                               const Vector2& x0, const Vector2& x1);


enum Quadrant {
    TopLeft,
    TopRight,
    BottomLeft,
    BottomRight
};


struct Box {
    static constexpr float inf = std::numeric_limits<float>::infinity();

    Vector2 min{inf, inf};
    Vector2 max{-inf, -inf};

    Box() = default;
    Box(Vector2 _min, Vector2 _max);

    bool is_empty() const;

    bool contains(const Vector2& vec) const;

    float width() const;

    float height() const;

    Vector2 dimensions() const;

    // Given in TL, TR, BL, BR order
    std::array<Box, 4> quadrants() const;
    Box get_quadrant(Quadrant q);

    bool operator==(const Box& other) const;

    // box union.
    Box& operator|=(const Box& other);
    Box operator|(const Box& other) const;

    // box union with vector
    Box& operator|=(const Vector2& other);
    Box operator|(const Vector2& other) const;

    // box intersection
    Box& operator&=(const Box& other);
    Box operator&(const Box& other) const;
};

// equivelant to foldr (|) boxes or foldr (|) vectors
template<typename Iterator>
Box bounding_box(Iterator begin, Iterator end) { 
    Box out;
    for (auto it = begin; it != end; ++it) {
        out |= *it;
    }
    return out;
}


#endif
