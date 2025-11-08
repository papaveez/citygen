#ifndef RENDERER_H
#define RENDERER_H

#include "styles.h"
#include "../generation/generator.h"


enum class RenderState {
    None,
    Window,
    Drawing,
    Mode2D
};


struct Renderer {
    int width, height;
    Box<float> screen_dims = {Vector2{0, 0}, Vector2{(float)width, (float)height}};
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

    Renderer(int w, int h, const char* window_title);
    ~Renderer();

    void init_window();
    void close_window();
    void begin_drawing();
    void end_drawing();
    void begin_mode_2d ();
    void end_mode_2d();


    void main_loop();
};


class Component {
protected:
    // empty definitions
    virtual void render_impl(Renderer* ren) {};
    virtual void render_2d_impl(Renderer* ren) {};

public:
    void render(Renderer* ren);
    void render_2d(Renderer* ren);
};


class TensorFieldView : public Component {
private:
    TensorField* tf_;
    FieldStyle style_;

    void draw_eigen_line(Renderer* ren, const Vector2& vec, 
        const Vector2& world_pos, Color col) const;
public:
    TensorFieldView(TensorField* tf_ptr);
    void set_style(FieldStyle s);
    void render_impl(Renderer* ren) override;
};





class MapView : public Component {
private:
    RoadGenerator* gen_;
    const RoadStyle* styles_;

    void draw_road_2d(const RoadHandle& handle,
            const RoadStyle& style) const;
public:
    MapView(RoadGenerator* gen, const RoadStyle* styles);

    void render_2d_impl(Renderer* ren) override;
};

#endif
