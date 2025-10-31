#include "ui.h"
#include "raylib.h"
#include <cmath>


bool UI::mouse_in_editor() const {
    return ctx_.viewport.contains(ctx_.mouse_world_pos)
        && !sidebar_bbox.contains(Vector2(ctx_.mouse_world_pos));
}

void UI::handle_tool_click(Tool t) {
    // state == FieldEditor
    selected_tool_ = {};
    brush_handler_ = {};
    
    if (state_ == FieldEditor) {
        if (t == EnterMap) {
            state_ = Map;
        } else if (t == RadialBrush) {
            selected_tool_ = t;
            brush_handler_ = [this]() {handle_radial_brush();};
        } else if (t == GridBrush) {
            selected_tool_ = t;
            brush_handler_ = [this]() {handle_grid_brush();};
        }
    } else if (state_ == Map) {
        if (t == Generate) {
            generator_ptr_->reset(ctx_.viewport);
            generator_ptr_->generate();
        } else if (t == BackToEditor) {
            state_ = FieldEditor;
        }
    }
}

void UI::handle_radial_brush() {
    if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
        if (!mouse_in_editor()) {
            radial_edit_.initialised = false;
            return;
        }

        if (!radial_edit_.initialised)  {
            radial_edit_.initialised = true;
            radial_edit_.centre = ctx_.mouse_world_pos;
        }

        DVector2 diff = radial_edit_.centre - ctx_.mouse_world_pos;
        radial_edit_.radius = std::hypot(diff.x, diff.y);

        DrawCircleLinesV(radial_edit_.centre, radial_edit_.radius, RED);
    } else if (radial_edit_.initialised && mouse_in_editor()) {
        tf_ptr_->add_radial(Radial(
            radial_edit_.centre,
            radial_edit_.radius,
            radial_edit_.decay
        ));
        radial_edit_.initialised = false;
    } else {
        radial_edit_.initialised = false;
    }
}


void UI::handle_grid_brush() {
    if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
        if (!mouse_in_editor()) {
            grid_edit_.initialised = false;
            return;
        }
        
        if (!grid_edit_.initialised) {
            grid_edit_.initialised = true;
            grid_edit_.centre = ctx_.mouse_world_pos;
        }

        DVector2 diff = ctx_.mouse_world_pos - grid_edit_.centre;
        grid_edit_.radius = std::hypot(diff.x, diff.y);
        grid_edit_.angle = vector_angle({1,0}, diff);


        DrawCircleLinesV(grid_edit_.centre, grid_edit_.radius, RED);
        DrawLineV(grid_edit_.centre, ctx_.mouse_world_pos, RED);
    } else if (grid_edit_.initialised && mouse_in_editor()) {
        tf_ptr_->add_grid(Grid(
            grid_edit_.angle,
            grid_edit_.centre,
            grid_edit_.radius,
            grid_edit_.decay
        ));
        grid_edit_.initialised = false;
    } else {
        grid_edit_.initialised = false;
    }
}


void UI::reset_tensorfield() {
    tf_ptr_->clear();

    tf_ptr_->add_grid(Grid(0, DVector2{0,0}));
}


void UI::render_sidebar() {
    assert(ctx_.is_drawing);
    assert(!ctx_.is_2d_mode);
    // DrawRectangle(
    //     0,
    //     0, 
    //     uiConfig.icon_size + 2*uiConfig.icon_padding, 
    //     ctx_.height, 
    //     LIGHTGRAY
    // );
    DrawRectangleV(
        sidebar_bbox.min,
        sidebar_bbox.max-sidebar_bbox.min,
        uiConfig.sidebar_colour
    );


    Rectangle button_template {
        uiConfig.icon_padding,
        0,
        uiConfig.icon_size,
        uiConfig.icon_size
    };

    int i=0;

    for (Tool t : tools_[state_]) {
        Rectangle button = button_template;
        button.y = static_cast<float>(
            uiConfig.icon_y_offset + i*(uiConfig.icon_size + uiConfig.icon_padding)
        );

        bool clicked = GuiButton(button, NULL);
        if (clicked) {
            handle_tool_click(t);
        }

        if (selected_tool_.has_value() && selected_tool_.value() == t) {
            DrawRectangleLinesEx(button, 2, RED);
        }


        GuiDrawIcon(
            toolIcons[t], 
            button.x, 
            button.y,
            uiConfig.icon_pixel_size,
            BLACK
        );

        ++i;
    }
}


void UI::render_field_artifacts_2d() {
    assert(ctx_.is_drawing);
    assert(ctx_.is_2d_mode);
    for (auto& [field_type, field] : tf_ptr_->basis_fields) {
        if (field->get_size())
            DrawCircleLinesV(field->get_centre(), field->get_size(), BLACK);

        
        DrawCircleV(field->get_centre(), 3, RED);
    }
}


UI::UI(RenderContext& ctx, TensorField* tf_ptr, RoadGenerator* gen_ptr) 
    : Renderer(ctx, tf_ptr, gen_ptr) 
{
    reset_tensorfield();
}


void UI::main_loop() {
    assert(ctx_.is_drawing);

    ClearBackground(RAYWHITE);
    if (state_ == FieldEditor) render_tensorfield();

    BeginMode2D(ctx_.camera); ctx_.is_2d_mode = true; {
        if (state_ == Map) render_map_2d();
        else if (state_ == FieldEditor) render_field_artifacts_2d();

    } EndMode2D(); ctx_.is_2d_mode = false;
    render_sidebar();
    if (brush_handler_.has_value()) 
        brush_handler_.value()();
}
