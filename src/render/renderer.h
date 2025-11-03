#ifndef RENDERER_H
#define RENDERER_H

#include "raylib.h"

#include "../generation/generator.h"
#include "config.h"
#include <memory>

enum class RenderState {
    None,
    Window,
    Drawing,
    Mode2D
};

struct RenderContext {
    int width, height;
    const char* window_title;

    Camera2D camera = {0};
    RenderState state = RenderState::None;
    bool camera_locked = false;
    DVector2 mouse_world_pos = {0.0, 0.0};
    Box<double> viewport = Box(
        {0,0},
        DVector2{
            static_cast<double>(width), 
            static_cast<double>(height)
        }
    );

    RenderContext(int w, int h, const char* title) :
        width(w),
        height(h),
        window_title(title)
    {}

    void init_window() {
        assert(state == RenderState::None);

        InitWindow(width, height, window_title);
        state = RenderState::Window;
    }

    void close_window() {
        assert (state == RenderState::Window);
        CloseWindow();
        state = RenderState::None;
    }

    void begin_drawing() {
        assert(state == RenderState::Window);
        BeginDrawing();
        state = RenderState::Drawing;
    }

    void end_drawing() {
        assert(state == RenderState::Drawing);
        EndDrawing();
        state = RenderState::Window;
    }

    void begin_mode_2d () {
        assert(state == RenderState::Drawing);
        BeginMode2D(camera);
        state = RenderState::Mode2D;
    }

    void end_mode_2d() {
        assert(state == RenderState::Mode2D);
        EndMode2D();
        state = RenderState::Drawing;
    }
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
    std::shared_ptr<TensorField> tf_ptr_;
    RoadGenerator generator;

    Renderer(RenderContext& ctx);

    std::unordered_map<RoadType, RoadStyle> road_styles_ 
        = default_road_styles;

    void render_tensorfield() const;
    void render_map_2d() const;
};


#endif
