#ifndef UI_H
#define UI_H

#include "renderer.h"
#include <cstddef>
#include <memory>

enum Tool {
    GridBrush,
    RadialBrush,
    EnterMap,
    BackToEditor,
    Generate,
    ToolCount
};


static constexpr int toolIcons[ToolCount] = {
    ICON_BOX_GRID,
    ICON_GEAR_EX,
    ICON_PLAYER_PLAY,
    ICON_UNDO_FILL,
    ICON_RESTART
};


enum UIState {
    FieldEditor,
    Map,
    UIStateCount
};

static std::array<std::list<Tool>, UIStateCount> tools = {
    std::list<Tool>{GridBrush, RadialBrush, EnterMap}, // FieldEditor
    std::list<Tool>{BackToEditor, Generate}  // Map
};

class UI : public Renderer  {
private:
    struct BrushHandler {
        std::function<size_t()> spawn;
        std::function<void(size_t)> edit;
        std::function<void(size_t)> cancel;
    };

    Box<int> toolbar_bbox_ {
        IVector2{0,0},
        IVector2{uiConfig.icon_size + 2*uiConfig.icon_padding, ctx_.height}
    };
    Box<int> editor_pane_bbox_ {
        IVector2 {ctx_.width - uiConfig.editor_pane_width, 0}, IVector2 { ctx_.width, ctx_.height }
    };

    UIState state_ = FieldEditor;

    std::optional<size_t> selected_basis_field_;
    std::optional<size_t> dragged_basis_field_;
    std::optional<Tool> selected_tool_;
    std::optional<size_t> new_basis_field_;

    BrushHandler radial_handler_ = {
        [this]() {
            size_t id = tf_ptr_->size();
            tf_ptr_->add_radial(Radial(ctx_.mouse_world_pos, 0, 0.4));
            return id;
        },
        [this](size_t id) {
            DVector2 diff = ctx_.mouse_world_pos - tf_ptr_->get_centre(id);
            tf_ptr_->set_size(id, diff.mag());
        },
        [this](size_t id) {
            tf_ptr_->erase(id);
        }
    };

    BrushHandler grid_handler_ = {
        [this]() {
            size_t id = tf_ptr_->size();
            tf_ptr_->add_grid(Grid(0, ctx_.mouse_world_pos, 0, 0.4));
            return id;
        },
        [this](size_t id) {
            DVector2 diff = ctx_.mouse_world_pos  - tf_ptr_->get_centre(id);

            double angle = vector_angle({1, 0}, diff);

            tf_ptr_->set_size(id, diff.mag());
            tf_ptr_->visit_if<Grid>(id, [&angle](Grid& g) -> void {
                g.set_theta(angle);
            });

        },
        [this](size_t id) {
            tf_ptr_->erase(id);
        }
    };

    std::optional<std::function<void()>> brush_handler_;

    bool mouse_in_editor() const;
    
    void handle_brush(const BrushHandler& handler);

    void handle_tool_click(Tool t);

    void reset_tensorfield();


    void render_toolbar();
    void render_field_artifacts();
    void render_editor_pane();
        void draw_grid_properties  (size_t field_id);
        void draw_radial_properties(size_t field_id);

public:
    UI (RenderContext& ctx);
    void main_loop();
};

#endif
