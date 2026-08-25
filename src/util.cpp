#include "util.h"

std::ostream& operator<<(std::ostream& os, const Vector2& v) {
    return os << "(" << v.x << "," << v.y << ")";
}


float dot_product(const Vector2& a, const Vector2& b) {
    return a.x*b.x + a.y*b.y;
}


Vector2 middle(Vector2 const& p1, Vector2 const& p2) {
    return (p1 + p2)/2.0;
}


float vector_angle(const Vector2& a, const Vector2& b) {
    float dot = dot_product(a, b);
    float det = a.x*b.y - a.y*b.x;

    return atan2(det, dot);
}


float 
perpendicular_distance(const Vector2& p, const Vector2& x0, const Vector2& x1) {
    Vector2 d = x1 - x0;

    float l2 = dot_product(d, d); // 0 line length
    if (l2 == 0.0) {
        Vector2 res = x1 - p;
        float dx = res.x;
        float dy = res.y;
        return std::hypot(dx, dy);
    }

    return
        std::abs(d.y * p.x - d.x * p.y + x1.x * x0.y - x1.y * x0.x) / std::sqrt(l2);
}


Box::Box(Vector2 _min, Vector2 _max) :
    min(_min),
    max(_max)
{}

bool Box::is_empty() const {
    return min.x >= max.x
        || min.y >= max.y;
}


bool Box::contains(const Vector2& vec) const {
    return 
        min.x <= vec.x 
        && vec.x < max.x
        && min.y <= vec.y
        && vec.y < max.y;
}

float Box::width() const {
    return max.x - min.x;
}

float Box::height() const {
    return max.y - min.y;
}


Vector2 Box::dimensions() const {
    return max - min;
}

std::array<Box, 4> // TL TR BL BR
Box::quadrants() const {
    Vector2 mid = middle(min, max);

    return {
        Box(min, mid), 
        Box({mid.x, min.y}, {max.x, mid.y}), 
        Box({min.x, mid.y}, {mid.x, max.y}), 
        Box(mid, max)
    };
}


Box Box::get_quadrant(Quadrant q) {
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

Quadrant Box::which_quadrant(Vector2 pos) {
    Vector2 mid = middle(min, max);

    for (Quadrant q : {TopLeft, TopRight, BottomLeft, BottomRight}) {
        if (get_quadrant(q).contains(pos)) return q;
    }
}

bool Box::operator==(const Box& other) const {
    return other.min == min && other.max == max;
}

// box union.
Box& Box::operator|=(const Box& other) {
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

Box Box::operator|(const Box& other) const {
    Box out = *this;
    out |= other;
    return out;
}

// box union with vector
Box& Box::operator|=(const Vector2& other) {
    Box boxed = Box(other, other);
    *this |= boxed;
    return *this;
}

Box Box::operator|(const Vector2& other) const {
    Box out = *this;
    out |= other;
    return out;
}


// box intersection
Box& Box::operator&=(const Box& other) {
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

Box Box::operator&(const Box& other) const {
    Box out = *this;
    out &= other;
    return out;
}

