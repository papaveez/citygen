#ifndef UI_H
#define UI_H

#include "renderer.h"

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


class UI : public Renderer  {
private:
    std::array<std::list<Tool>, UIStateCount> tools_ = {
        std::list<Tool>{GridBrush, RadialBrush, EnterMap}, // FieldEditor
        std::list<Tool>{BackToEditor, Generate}  // Map
    };

    std::optional<Tool> selected_tool_;
    std::optional<std::function<void()>> brush_handler_;

    // optional id of the basis field being dragged around.

    Box<int> toolbar_bbox_ {
        IVector2{0,0},
        IVector2{
            uiConfig.icon_size + 2*uiConfig.icon_padding,
            ctx_.height
        }
    };


    std::optional<size_t> selected_basis_field_;
    std::optional<size_t> dragged_basis_field_;
    Box<int> editor_pane_bbox_ {
        IVector2 {
            ctx_.width - uiConfig.editor_pane_width,
            0
        },
        IVector2 { ctx_.width, ctx_.height }
    };

    struct {
        DVector2 centre {0,0};
        double radius = 0;
        double decay = 0.4;
        bool initialised = false;
    } radial_edit_;

    struct {
        DVector2 centre {0,0};
        double radius = 0;
        double angle = 0;
        double decay = 0.4;
        bool initialised = false;
    } grid_edit_;

    bool mouse_in_editor() const;
    
    void handle_radial_brush();
    void handle_grid_brush();

    void handle_tool_click(Tool t);

    void reset_tensorfield();

    UIState state_ = FieldEditor;

    void render_toolbar();
    void render_field_artifacts();
    void render_editor_pane();
        void draw_grid_properties  (size_t field_id);
        void draw_radial_properties(size_t field_id);

public:
    UI (RenderContext& ctx, TensorField* tf_ptr, RoadGenerator* gen_ptr);
    void main_loop();
};

#endif
