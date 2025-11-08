#include "renderer.h"

#include <cstdint>

#include "raymath.h"


// Renderer 

void Renderer::init_window() {
    assert(state == RenderState::None);

    InitWindow(width, height, window_title);
    state = RenderState::Window;
}


void Renderer::close_window() {
    assert (state == RenderState::Window);
    CloseWindow();
    state = RenderState::None;
}


void Renderer::begin_drawing() {
    assert(state == RenderState::Window);
    BeginDrawing();
    state = RenderState::Drawing;
}


void Renderer::end_drawing() {
    assert(state == RenderState::Drawing);
    EndDrawing();
    state = RenderState::Window;
}


void Renderer::begin_mode_2d () {
    assert(state == RenderState::Drawing);
    BeginMode2D(camera);
    state = RenderState::Mode2D;
}


void Renderer::end_mode_2d() {
    assert(state == RenderState::Mode2D);
    EndMode2D();
    state = RenderState::Drawing;
}


void Renderer::main_loop() {
    mouse_world_pos = GetScreenToWorld2D(
        GetMousePosition(), 
        camera
    );

    viewport = Box(
        DVector2(GetScreenToWorld2D(screen_dims.min, camera)),
        DVector2(GetScreenToWorld2D(screen_dims.max, camera))
    );


    float wheel = GetMouseWheelMove();

    // camera pan
    if (IsMouseButtonDown(MOUSE_LEFT_BUTTON) && !camera_locked) {
        Vector2 delta = GetMouseDelta();
        delta = delta/camera.zoom*-0.5f;
        camera.target = camera.target+delta;
    }


    // camera scroll
    if (wheel != 0) {
        camera.offset = GetMousePosition();
        camera.target = mouse_world_pos;

        float scale = 0.2f*wheel;
        camera.zoom = 
            Clamp(expf(logf(camera.zoom)+scale), 0.125f, 64.0f);
    }
}


Renderer::Renderer(int w, int h, const char* window_title) :
    width(w), height(h), window_title(window_title)
{
    init_window();
    // SetTargetFPS(60);
    camera.zoom = 1.0f;
}


Renderer::~Renderer() {
    close_window();
}


// Component

void Component::render(Renderer* ren) {
    assert(ren->state == RenderState::Drawing);
    render_impl(ren);
}


void Component::render_2d(Renderer* ren) {
    assert(ren->state == RenderState::Mode2D);
    render_2d_impl(ren);
}


// TensorFieldView

void TensorFieldView::draw_eigen_line(Renderer* ren, const Vector2& vec,
        const Vector2& world_pos, Color col) const
{
    float l = std::hypot(vec.x, vec.y);

    if (l == 0) return;

    Vector2 offset = vec*style_.line_scale/l/ren->camera.zoom;

    Vector2 pts[2] = {
        GetWorldToScreen2D(world_pos - offset, ren->camera),
        GetWorldToScreen2D(world_pos + offset, ren->camera)
    };

    DrawSplineLinear(pts, 2, style_.line_thick, col);
}


TensorFieldView::TensorFieldView(
        TensorField* tf_ptr) : 
    tf_(tf_ptr)
{}


void TensorFieldView::set_style(FieldStyle s) {
    style_ = s;
}


void TensorFieldView::render_impl(Renderer* ren) {
    for (float i=0; i<ren->width; i+=style_.granularity) {
        for (float j=0; j<ren->height; j+=style_.granularity) {

            Vector2 world_pos = 
                GetScreenToWorld2D(Vector2{i,j}, ren->camera);

            Tensor t = tf_->sample(world_pos);

            draw_eigen_line(
                ren,
                t.get_major_eigenvector(),
                world_pos,
                style_.major_col
            );

            draw_eigen_line(
                ren,
                t.get_minor_eigenvector(),
                world_pos,
                style_.minor_col
            );


            DrawCircle(i, j, 1, style_.degen_col);
        }
    }
}


// MapView
void MapView::draw_road_2d(const RoadHandle& handle,
    const RoadStyle& style) const 
{
    auto [len, data] = gen_->get_road_points(handle);
    // draw road outline
    DrawSplineLinear(
        data,
        len, 
        style.outline_width+style.width,
        style.outline_col
    );


    // draw road
    DrawSplineLinear(
        data, 
        len, 
        style.width, 
        style.col
    );

#ifdef DRAW_NODES
    Color col = eigenfield == Eigenfield::major() ? renderConfig.major_col 
                                                  : renderConfig.minor_col;
    for (int i=0; i<len; ++i) {
        DrawCircleV(data[i], 1, col);
        if (i>0) {
            DrawLineV(data[i-1], data[i], col);
        }
    }
#endif
}


MapView::MapView(RoadGenerator* gen, const RoadStyle* styles) :
    gen_(gen), styles_(styles) {}

void MapView::render_2d_impl(Renderer* ren) {
    if (styles_ == nullptr) return;

    const auto& num_road_type = gen_->road_type_count();
    
    Eigenfield efs[2] = {Eigenfield::major(), Eigenfield::minor()};

    RoadHandle hand{0, 0, 0};

    for (int road_type = num_road_type-1; road_type>=0; --road_type) {
        const RoadStyle& style = styles_[road_type];

        hand.road_type = road_type;

        for (Eigenfield ef : efs) {
            hand.eigenfield = ef;

            std::uint32_t count = gen_->road_count(road_type, ef);
            for (std::uint32_t idx=0; idx<count; ++idx) {
                hand.idx = idx;

                draw_road_2d(hand, style);
            }
        }
    }
}

