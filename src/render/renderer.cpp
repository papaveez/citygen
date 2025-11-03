#include "renderer.h"

#include "config.h"
#include "raylib.h"
#include "raymath.h"


// #define DRAW_NODES

void 
Renderer::draw_vector_line(const Vector2& vec, const Vector2& world_pos,
    Color col) const
{
    float l = std::hypot(vec.x, vec.y);

    if (l == 0) return;

    Vector2 offset = vec*renderConfig.field_line_scale/l/ctx_.camera.zoom;

    Vector2 pts[2] = {
        GetWorldToScreen2D(world_pos - offset, ctx_.camera),
        GetWorldToScreen2D(world_pos + offset, ctx_.camera)
    };

    DrawSplineLinear(pts, 2, renderConfig.field_line_thick, col);
}



void Renderer::draw_roads_2d(RoadType road_type, Eigenfield eigenfield) const {
    const RoadStyle& style = road_styles_.at(road_type);

    std::uint32_t num_roads = generator.road_count(road_type, eigenfield);

    for (std::uint32_t idx=0; idx<num_roads; ++idx) {
        RoadHandle handle = {
            idx,
            road_type,
            eigenfield
        };

        auto [len, data] = generator.get_road_points(handle);

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
}

Renderer::Renderer(RenderContext& ctx) :
    ctx_(ctx),
    tf_ptr_(std::make_shared<TensorField>()),
    generator(RoadGenerator(tf_ptr_, params, ctx_.viewport))
{}


void Renderer::render_tensorfield() const {
    assert(ctx_.state == RenderState::Drawing);

    const float& granularity = renderConfig.field_granularity;

    for (float i=0; i<ctx_.width; i+=granularity) {
        for (float j=0; j<ctx_.height; j+=granularity) {

            Vector2 world_pos = 
                GetScreenToWorld2D(Vector2{i,j}, ctx_.camera);

            Tensor t = tf_ptr_->sample(world_pos);

            draw_vector_line(
                t.get_major_eigenvector(),
                world_pos,
                renderConfig.major_col
            );

            draw_vector_line(
                t.get_minor_eigenvector(),
                world_pos,
                renderConfig.minor_col
            );


            DrawCircle(i, j, 1, renderConfig.degenerate_col);
        }
    }
}


void Renderer::render_map_2d() const {
    assert(ctx_.state == RenderState::Mode2D);

    const std::vector<RoadType>& road_types = 
        generator.get_road_types();

    for (int i=road_types.size()-1; i>=0; --i) {
        draw_roads_2d(road_types[i], Eigenfield::major());
        draw_roads_2d(road_types[i], Eigenfield::minor());
    }
}
