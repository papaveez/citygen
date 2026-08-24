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
