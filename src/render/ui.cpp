#include "ui.h"
#include "config.h"
#include "raylib.h"
#include "raymath.h"
#include <cmath>
#include <cstddef>


bool UI::mouse_in_editor() const {
    return ctx_.viewport.contains(ctx_.mouse_world_pos)
        && !toolbar_bbox_.contains(ctx_.mouse_world_pos)
        && !(selected_basis_field_.has_value() && editor_pane_bbox_.contains(ctx_.mouse_world_pos));
}


void UI::handle_tool_click(Tool t) {
    selected_tool_ = {};
    brush_handler_ = {};
    
    if (t == EnterMap) {
        state_ = Map;
    } else if (t == RadialBrush) {
        selected_tool_ = t;
        brush_handler_ = [this]() {handle_radial_brush();};
    } else if (t == GridBrush) {
        selected_tool_ = t;
        brush_handler_ = [this]() {handle_grid_brush();};
    } else if (t == Generate) {
        generator_ptr_->reset(ctx_.viewport);
        generator_ptr_->generate();
    } else if (t == BackToEditor) {
        state_ = FieldEditor;
    }
}

void UI::handle_radial_brush() {
    if (IsMouseButtonDown(MOUSE_RIGHT_BUTTON)) {
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

        DrawCircleLinesV(
            GetWorldToScreen2D(radial_edit_.centre, ctx_.camera),
            radial_edit_.radius*ctx_.camera.zoom, 
            RED
        );

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
    if (IsMouseButtonDown(MOUSE_RIGHT_BUTTON)) {
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


        Vector2 screen_centre = GetWorldToScreen2D(grid_edit_.centre, ctx_.camera);

        DrawCircleLinesV(screen_centre, grid_edit_.radius*ctx_.camera.zoom, RED);
        DrawLineV(screen_centre, GetMousePosition(), RED);

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


void UI::render_toolbar() {
    assert(ctx_.is_drawing);
    assert(!ctx_.is_2d_mode);

    DrawRectangleV(
        toolbar_bbox_.min,
        toolbar_bbox_.dimensions(),
        uiConfig.toolbar_col
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

        if (selected_tool_.has_value() && 
            selected_tool_.value() == t) 
        {
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


void UI::render_field_artifacts() {
    assert(ctx_.is_drawing);
    assert(!ctx_.is_2d_mode);

    ctx_.camera_locked = false;

    bool mouse_down = IsMouseButtonDown(MOUSE_LEFT_BUTTON);
    bool mouse_click = IsMouseButtonDown(MOUSE_LEFT_BUTTON);

    for (size_t i=0;i<tf_ptr_->basis_fields.size(); i++) {
        auto& [field_type, field] = tf_ptr_->basis_fields[i];

        bool is_dragged = dragged_basis_field_.value_or(-1) == i;
        bool is_selected = selected_basis_field_.value_or(-1) == i;

        // draw bounding circle
        if (field->get_size()) {
            DrawCircleLinesV(
                GetWorldToScreen2D(field->get_centre(), ctx_.camera),
                field->get_size()*ctx_.camera.zoom,
                BLACK
            );
        }

        DVector2 centre = field->get_centre();

        
        DrawCircleV(
            GetWorldToScreen2D(centre, ctx_.camera),
            uiConfig.basis_field_selector_rad,
            is_selected ? uiConfig.basis_field_selected_col
                        : uiConfig.basis_field_centre_col
        );

        DVector2 offset = DVector2{1,1}*uiConfig.basis_field_selector_rad;

        Box<double> bounds = {
            centre - offset,
            centre + offset
        };

        Box<double> hover_bounds = {
            centre - offset*2,
            centre + offset*2
        };

        if (is_dragged && mouse_down) {
            ctx_.camera_locked |= true;
            field->set_centre(ctx_.mouse_world_pos);
        } else if (is_dragged) {
            dragged_basis_field_ = {}; // unset dragged field
        }

        if (mouse_click && bounds.contains(ctx_.mouse_world_pos)) {
            dragged_basis_field_ = {};
            ctx_.camera_locked = false;
            selected_basis_field_ = i;
        }
    }
}


void UI::render_editor_pane() {
    assert(ctx_.is_drawing);
    assert(!ctx_.is_2d_mode);
    assert(selected_basis_field_.has_value());

    size_t basis_id = selected_basis_field_.value();
    auto& [field_type, field] = tf_ptr_->basis_fields[basis_id];

    DrawRectangleV(
        editor_pane_bbox_.min,
        editor_pane_bbox_.dimensions(),
        uiConfig.editor_pane_col
    );

    Rectangle close_pane = {
        static_cast<float>(editor_pane_bbox_.max.x - uiConfig.icon_size),
        static_cast<float>(editor_pane_bbox_.min.y),
        uiConfig.icon_size,
        uiConfig.icon_size
    };

    if (GuiButton(close_pane, NULL)) {
        selected_basis_field_ = {};
        return;
    }

    GuiDrawIcon(ICON_CROSS, close_pane.x, close_pane.y, uiConfig.icon_pixel_size, BLACK);

    // TODO: not using external font
    DrawTextEx(
        fonts.normal,
        TextFormat("Field <ID #%i> : %s", basis_id, basis_field_string(field_type)),
        editor_pane_bbox_.min,
        0.8*static_cast<float>(fonts.normal.baseSize),
        2,
        BLACK
    );

    switch (field_type) {
        case BasisFieldType::Grid:
            draw_grid_properties(basis_id);
        case BasisFieldType::Radial:
            draw_radial_properties(basis_id);
    }
}


void UI::draw_grid_properties(size_t field_id) {

}


void UI::draw_radial_properties(size_t field_id) {

}


UI::UI(RenderContext& ctx, TensorField* tf_ptr, RoadGenerator* gen_ptr) 
    : Renderer(ctx, tf_ptr, gen_ptr) 
{
    reset_tensorfield();
}


void UI::main_loop() {
    ctx_.mouse_world_pos= GetScreenToWorld2D(
        GetMousePosition(), 
        ctx_.camera
    );

    ctx_.viewport = Box(
        DVector2(GetScreenToWorld2D(screen_dims.min, ctx_.camera)),
        DVector2(GetScreenToWorld2D(screen_dims.max, ctx_.camera))
    );

    float wheel = GetMouseWheelMove();

    // camera pan
    if (IsMouseButtonDown(MOUSE_LEFT_BUTTON) && !ctx_.camera_locked) {
        Vector2 delta = GetMouseDelta();
        delta = delta*-1.0f/ctx_.camera.zoom;
        ctx_.camera.target = ctx_.camera.target+delta;
    }


    // camera scroll
    if (wheel != 0) {
        ctx_.camera.offset = GetMousePosition();
        ctx_.camera.target = ctx_.mouse_world_pos;

        float scale = 0.2f*wheel;
        ctx_.camera.zoom = 
            Clamp(expf(logf(ctx_.camera.zoom)+scale), 0.125f, 64.0f);
    }

    
    BeginDrawing(); ctx_.is_drawing = true; {
        assert(ctx_.is_drawing);

        ClearBackground(RAYWHITE);
        if (state_ == FieldEditor) {
            render_tensorfield();
            render_field_artifacts();
            if (selected_basis_field_.has_value()) {
                render_editor_pane();
            }
        }

        BeginMode2D(ctx_.camera); ctx_.is_2d_mode = true; {
            if (state_ == Map) render_map_2d();

        } EndMode2D(); ctx_.is_2d_mode = false;

        render_toolbar();
        if (brush_handler_.has_value()) 
            brush_handler_.value()();

        DrawFPS(0, 0);

    } EndDrawing(); ctx_.is_drawing = false;
}
