#ifndef RENDERER_H
#define RENDERER_H

#include "raylib.h"

#include "../generation/generator.h"
#include "config.h"

struct RenderContext {
    int width, height;
    Camera2D camera = {0};
    bool is_window = false;
    bool is_drawing = false;
    bool is_2d_mode = false;
    bool camera_locked = false;
    DVector2 mouse_world_pos = {0.0, 0.0};
    Box<double> viewport = Box(
        {0,0},
        DVector2{
            static_cast<double>(width), 
            static_cast<double>(height)
        }
    );


    RenderContext(int w, int h) : width(w), height(h) {}
};


class Renderer {
private:
    void draw_vector_line(
        const Vector2& vec,
        const Vector2& world_pos,
        Color col
    ) const;

    void draw_roads_2d(RoadType road_type, Eigenfield eigenfield) const;

protected:
    RenderContext& ctx_;
    TensorField*   tf_ptr_;
    RoadGenerator* generator_ptr_;

    Renderer(RenderContext& ctx, TensorField* tf_ptr, RoadGenerator* gen_ptr);

    std::unordered_map<RoadType, RoadStyle> road_styles_ 
        = default_road_styles;

    void render_tensorfield() const;
    void render_map_2d() const;
};


#endif
