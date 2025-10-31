#include "ui.h"

#include <cstdint>
#include <limits>

#include "generation/integrator.h"
#include "generation/road_storage.h"
#include "raylib.h"
#include "raygui.h"
#include "raymath.h"
#include "types.h"

// #define DRAW_NODES

static constexpr float f_epsilon = std::numeric_limits<float>::epsilon();

bool Renderer::mouse_in_viewport() {
    return ctx_.viewport.contains(ctx_.mouse_world_pos)
        && !Box<double>(
                { 0, 0 },
                {
                    uiConfig.icon_size + 2*uiConfig.icon_padding, 
                    static_cast<double>(ctx_.height)
                }
            ).contains(ctx_.mouse_world_pos);
}



void 
Renderer::draw_vector_line( 
                 const Vector2& vec, const Vector2& world_pos,
                 Color col) const
{
    float l = std::hypot(vec.x, vec.y);

    if (l <= f_epsilon)
        return;

    Vector2 offset = vec*uiConfig.lineScale/l/ctx_.camera.zoom;

    Vector2 pts[2] = {
        GetWorldToScreen2D(world_pos - offset, ctx_.camera),
        GetWorldToScreen2D(world_pos + offset, ctx_.camera)
    };



    DrawSplineLinear(pts, 2, 2.0f, col);
}



void Renderer::render_tensorfield() const {
    assert(ctx_.is_drawing);
    assert(!ctx_.is_2d_mode);

    for (float i=0;i<ctx_.width;i+=uiConfig.granularity) {
        for (float j=0;j<ctx_.height;j+=uiConfig.granularity) {
            // map to world coordinates

            Vector2 world_pos = GetScreenToWorld2D((Vector2) {i, j}, ctx_.camera);

            Tensor t = tf_ptr_->sample(world_pos);
            DVector2 major_eigen = t.get_major_eigenvector();
            DVector2 minor_eigen = t.get_minor_eigenvector();

            // draw cross
            draw_vector_line(major_eigen, world_pos, RED);
            draw_vector_line(minor_eigen, world_pos, DARKBLUE);

            DrawCircle(i, j, 1, BLUE);
        }
    } 
}

void Renderer::editor() {
    assert(ctx_.is_drawing);
    assert(ctx_.is_2d_mode);

    if(!mouse_in_viewport()) {
        radial_edit_.initialised = false;
        grid_edit_.initialised = false;
    }

    EditorTool* edit;
    switch (brush_) {
        case GridBrush:
            edit = &grid_edit_;
            break;
        case RadialBrush:
            edit = &radial_edit_;
            break;
        default:
            return;
    }

    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        if (!edit->initialised) {
            edit->centre = ctx_.mouse_world_pos;
            edit->initialised = true;
        }

        DVector2 offset = ctx_.mouse_world_pos - edit->centre;
        float rad = std::hypot(offset.x, offset.y);

        DrawCircleLinesV(edit->centre, rad, RED);
        if (edit->draw_spoke) {
            DrawLineV(edit->centre, ctx_.mouse_world_pos, RED);
        }
    } else if (edit->initialised) {
        handle_brush_release();
        edit->initialised = false;
    } 
}


void Renderer::handle_brush_release() {
    if (brush_ == GridBrush) {
        double theta = vector_angle({1, 0}, ctx_.mouse_world_pos);
        DVector2 diff = ctx_.mouse_world_pos - grid_edit_.centre;
        double rad = std::hypot(diff.x, diff.y);

        tf_ptr_->add_basis_field(std::make_unique<Grid>(
            theta,
            grid_edit_.centre,
            rad,
            grid_edit_.decay
        ));
    } else if (brush_ == RadialBrush) {
        DVector2 diff = ctx_.mouse_world_pos - radial_edit_.centre;
        double rad = std::hypot(diff.x, diff.y);

        tf_ptr_->add_basis_field(std::make_unique<Radial>(
            radial_edit_.centre,
rad,
            radial_edit_.decay
        ));
    }
}


void Renderer::draw_streamlines(RoadType road_type, Eigenfield ef) const {
    // const std::vector<Streamline>& sls = generator_ptr_->get_streamlines(road, dir);

    bool draw_joins = IsKeyDown(KEY_J);

    const RoadStyle& style = road_styles_.at(road_type);

    std::uint32_t num_roads = generator_ptr_->road_count(road_type, ef);

    for (std::uint32_t idx=0; idx<num_roads; ++idx) {
        RoadHandle handle = {
            idx,
            road_type,
            ef
        };

        bool highlight = draw_joins & generator_ptr_->is_connective_road(handle);


        auto [len, data] = generator_ptr_->get_road_points(handle);

        DrawSplineLinear(
            data, 
            len,
            style.outline_width+style.width,
            style.outline_colour
        );

        DrawSplineLinear(
            data,
            len,
            style.width,
            // style.colour
            highlight ? RED : style.colour
        );

#ifdef DRAW_NODES
        Color col = ef == Major ? RED : BLUE;
        for (int i=0; i < len; ++i) {
            DrawCircleV(data[i], 1, col);
            if (i>0) {
                DrawLineV(data[i-1], data[i], col);
            }
        }
#endif

    }

}

void Renderer::render_map() {
    assert(ctx_.is_drawing);
    assert(ctx_.is_2d_mode);

    if (!generated_) {
        generator_ptr_->reset(ctx_.viewport);
        generator_ptr_->generate();
        generated_ = true;
    }


    const std::vector<RoadType>& road_types = generator_ptr_->get_road_types();
    for (int i=road_types.size()-1; i>=0; --i) {
        draw_streamlines(road_types[i], Major);
        draw_streamlines(road_types[i], Minor);
    }
}

void Renderer::render_generating_popup() const {
    Vector2 mid = {ctx_.width/2.0f, ctx_.height/2.0f};

    Rectangle b = {
        mid.x - uiConfig.modalWidth/2.0f,
        mid.y - uiConfig.modalHeight/2.0f,
        uiConfig.modalWidth,
        uiConfig.modalHeight
    };

    GuiButton(b, NULL);
    const char* text = "Generating...";

    int text_width = MeasureText(text, 40);
    DrawText(text, mid.x-text_width/2.0f, mid.y-20, 40, BLACK);

}



void Renderer::render_hud() {
     DrawRectangle(
            0, 
            0, 
            uiConfig.icon_size + 2*uiConfig.icon_padding,
            ctx_.height, 
            LIGHTGRAY
        );

    
     std::list<Tool>& tools = mode_ == FieldEditor ? fieldEditorTools : mapTools;

     int i=0;
     for (Tool t : tools) {
        Rectangle button = {
            uiConfig.icon_padding, 
            (float) (uiConfig.y +(uiConfig.icon_size+ uiConfig.icon_padding)*i), 
            uiConfig.icon_size, 
            uiConfig.icon_size
        };

        bool clicked = GuiButton(button, NULL);
        if (clicked) {
            handle_tool_click(t);
        }

        if (t == brush_) {
            DrawRectangleLinesEx(button, 2, RED);
        }

        GuiDrawIcon(
            toolIcons[t],
            (int) button.x,
            (int) button.y,
            3,
            BLACK
        );

        ++i;
    }
}

void Renderer::handle_tool_click(const Tool& t) {
    bool will_generate;
    if (t <= 1) {
        brush_ = t;
    } else if (t == GenerateMap) {
        step_mode_ = false;
        mode_ = Map;
        will_generate = !generated_;
    } else if (t == BackToEditor) {
        step_mode_ = false;
        generated_ = false;
        road_idx_ = 0;
        mode_ = FieldEditor;
    } else if (t == Regenerate) {
        generated_ = false;
        will_generate = true;
    }

    if (!generated_ && will_generate) {
        render_generating_popup();
    }
}


void Renderer::reset_field_editor() {
    tf_ptr_->clear();
    tf_ptr_->add_basis_field(
        std::make_unique<Grid>(0, DVector2{0,0})
    );
}

Renderer::Renderer(
        RenderContext& ctx, 
        TensorField* tf_ptr, 
        RoadGenerator* gen_ptr
    ) : 
    ctx_(ctx),
    tf_ptr_(tf_ptr),
    generator_ptr_(gen_ptr)
{
    reset_field_editor();
}



#ifdef STORAGE_TEST 
void Renderer::test_spatial() {
    assert(ctx_.is_drawing);
    assert(ctx_.is_2d_mode);

    if (IsKeyDown(KEY_SPACE)) {
        Color col = BLACK;

        std::list<NodeHandle> majors = 
            generator_ptr_->nearby_points(ctx_.mouse_world_pos, 100, Eigenfield(Major));
        
        std::list<NodeHandle> minors =
            generator_ptr_->nearby_points(ctx_.mouse_world_pos, 100, Eigenfield(Minor));



        int a = majors.size();
        int b = minors.size();
        if (a && b ) {
            col = GREEN;
        } else if (a) {
            col = RED;
        } else if (b) {
            col= BLUE;
        }

        DrawCircleLinesV(ctx_.mouse_world_pos, 100, col);

        DrawTextEx(GetFontDefault(), TextFormat("[%i, %i]", a, b),
                ctx_.mouse_world_pos+DVector2{ -44, -24 }, 20, 2, BLACK);
    }

    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        if (points_.size()) {
            if (points_.size() > 1) {
                int n = points_.size();
                Vector2* pts = new Vector2[n];

                int i = 0;
                for (const DVector2& v : points_) {
                    pts[i] = v;
                    ++i;
                }

                DrawSplineLinear(pts, n, 3.0f, dir_== Major ? RED : BLUE);
            }

            DVector2 diff = points_.back() - ctx_.mouse_world_pos;
            if (dot_product(diff, diff) < 1000.0) return;
        }

        points_.push_back(ctx_.mouse_world_pos);

        
    } else if (points_.size()) {
        generator_ptr_->push_road(points_, Main, dir_);
        points_ = {};
    } else if (IsKeyPressed(KEY_D)) {
        dir_ = flip(dir_);
    }
}

void Renderer::test_draw_spatial(qnode_id head_ptr) {
    if (head_ptr == NullQNode) return;

    QuadNode node = generator_ptr_->qnodes_[head_ptr];

    Box<double> bbox = node.bbox;
    Vector2 pos = bbox.min;

    Color col = BLACK;

    ef_mask maj = Eigenfield(Major).mask();
    ef_mask min = Eigenfield(Minor).mask();


    if (node.eigenfields & maj && node.eigenfields & min) {
        col = GREEN;
    } else if (node.eigenfields & maj) {
        col = RED;
    } else if (node.eigenfields & min) {
        col = BLUE;
    }

    Vector2 dims = bbox.max - bbox.min;

    Rectangle rect = {pos.x, pos.y, dims.x, dims.y};

    DrawRectangleLinesEx(rect, 2.0f, col);

    if (generator_ptr_->is_leaf(head_ptr)) {
        for (const NodeHandle& hd: node.data) {
            DVector2 pos = generator_ptr_->get_pos(hd);
            Eigenfield ef = hd.road_handle.eigenfield;

            DrawCircleV(pos, 1.0, ef == Major ? RED : BLUE);
        }
    }
    else {
        for (auto q : {TopLeft, TopRight, BottomLeft, BottomRight}) {
            test_draw_spatial(node.children[q]);
        }
    }
}
#endif

void Renderer::main_loop() {
    assert(ctx_.is_drawing);

    
    ClearBackground(RAYWHITE);
    #ifdef STORAGE_TEST 
        BeginMode2D(ctx_.camera); ctx_.is_2d_mode = true; {
            test_spatial();
            test_draw_spatial(generator_ptr_->root_);
        } EndMode2D(); ctx_.is_2d_mode = false;
        // draw current dir
        
        auto text = dir_ == Major ? "MAJOR" : "MINOR";
        Color col = dir_ == Major ? RED : BLUE;
        DrawText(text, ctx_.width-200, 10, 30, col);
    #else
    if (mode_ == FieldEditor) render_tensorfield();
    BeginMode2D(ctx_.camera); ctx_.is_2d_mode = true; {
        if (mode_ == Map) render_map();
        editor();
    } EndMode2D(); ctx_.is_2d_mode = false;
    render_hud();
    // DrawTextEx(
    //     GetFontDefault(), 
    //     TextFormat(
    //         "[%f, %f]", 
    //         ctx_.mouse_world_pos.x, 
    //         ctx_.mouse_world_pos.y
    //     ), 
    //     GetMousePosition()+(Vector2){ -44, -24 }, 
    //     20,
    //     2,
    //     BLACK
    // );

    #endif
}
