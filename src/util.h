#ifndef UTIL_H
#define UTIL_H 

#include <cassert>
#include <ostream>

#include "raylib.h"
#include "raymath.h"


std::ostream& operator<<(std::ostream& os, const Vector2& v); 

float dot_product(const Vector2& a, const Vector2& b);

Vector2 middle(Vector2 const& p1, Vector2 const& p2);


float vector_angle(const Vector2& a, const Vector2& b);


float 
perpendicular_distance(const Vector2& p, const Vector2& x0, const Vector2& x1);


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
    Box(Vector2 _min, Vector2 _max) :
        min(_min),
        max(_max)
    {}

    bool is_empty() const {
        return min.x >= max.x
            || min.y >= max.y;
    }


    bool contains(const Vector2& vec) const {
        return 
            min.x <= vec.x 
            && vec.x < max.x
            && min.y <= vec.y
            && vec.y < max.y;
    }

    float width() const {
        return max.x - min.x;
    }

    float height() const {
        return max.y - min.y;
    }


    Vector2 dimensions() const {
        return max - min;
    }

    std::array<Box, 4> // TL TR BL BR
    quadrants() const {
        Vector2 mid = middle(min, max);

        return {
            Box(min, mid), 
            Box({mid.x, min.y}, {max.x, mid.y}), 
            Box({min.x, mid.y}, {mid.x, max.y}), 
            Box(mid, max)
        };
    }


    Box get_quadrant(Quadrant q) {
        Vector2 mid = middle(min, max);
        switch (q) {
            case TopLeft:
                return Box(min, mid);
            case TopRight:
                return Box({mid.x, min.y}, {max.x, mid.y});
            case BottomLeft:
                return Box({min.x, mid.y}, {mid.x, max.y});
            case BottomRight:
                return Box(mid, max);
        }
    }

    Quadrant which_quadrant(Vector2 pos) {
        Vector2 mid = middle(min, max);

        for (Quadrant q : {TopLeft, TopRight, BottomLeft, BottomRight}) {
            if (get_quadrant(q).contains(pos)) return q;
        }
    }

    bool operator==(const Box& other) const {
        return other.min == min && other.max == max;
    }

    // box union.
    Box& operator|=(const Box& other) {
        min = {
            std::min(min.x, other.min.x),
            std::min(min.y, other.min.y)
        };
        max = {
            std::max(max.x, other.max.x),
            std::max(max.y, other.max.y)
        };

        return *this;
    }

    Box operator|(const Box& other) const {
        Box out = *this;
        out |= other;
        return out;
    }

    // box union with vector
    Box& operator|=(const Vector2& other) {
        Box boxed = Box(other, other);
        *this |= boxed;
        return *this;
    }

    Box operator|(const Vector2& other) const {
        Box out = *this;
        out |= other;
        return out;
    }


    // box intersection
    Box& operator&=(const Box& other) {
        min = {
            std::max(min.x, other.min.x),
            std::max(min.y, other.min.y)
        };
        max = {
            std::min(max.x, other.max.x),
            std::min(max.y, other.max.y)
        };

        return *this;
    }

    Box operator&(const Box& other) const {
        Box out = *this;
        out &= other;
        return out;
    }
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
