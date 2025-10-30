#ifndef UI_H
#define UI_H

#include <cassert>

#include "generation/road_storage.h"
#include "raylib.h"
#include "raygui.h"

#include "generation/tensor_field.h"
#include "generation/generator.h"

#include "const.h"

struct RenderContext {
    int width, height;
    Camera2D camera;
    bool is_drawing;
    bool is_2d_mode;
    DVector2 mouse_world_pos;
    Box<double> viewport;
};

static constexpr struct {
    int icon_size = 48;
    int icon_padding = 6;
    int y = 20;
    float lineScale = 10.0f;
    int granularity = 26;
    float modalWidth = 270.0f;
    float modalHeight = 90.0f;
} uiConfig;


struct RoadStyle {
    Color colour;
    Color outline_colour;
    float width;
    float outline_width;

    static RoadStyle default_main() {
        return {
            {250, 224, 98, 255},
            {238, 199, 132, 255},
            10.0f,
            2.0f
        };
    }

    static RoadStyle default_highstreet() {
        return {
            {252,252,224, 255},
            {240,210,152, 255},
            8.0f,
            2.0f
        };
    }

    static RoadStyle default_sidestreet() {
        return {
            {255,255,255, 255},
            {215, 208, 198},
            6.0f,
            1.0f
        };
    }
};


enum UIMode {
    FieldEditor,
    Map
};


enum Tool: int {
    GridBrush,
    RadialBrush,
    GenerateMap,
    BackToEditor,
    Regenerate,
    ToolCount
};


static std::list<Tool> fieldEditorTools{GridBrush, RadialBrush, GenerateMap};
static std::list<Tool> mapTools{BackToEditor, Regenerate};


static constexpr int toolIcons[ToolCount] = {
    ICON_BOX_GRID,
    ICON_GEAR_EX,
    ICON_PLAYER_PLAY,
    ICON_UNDO_FILL,
    ICON_RESTART
};


struct EditorTool {
    DVector2 centre = {0.0, 0.0};
    bool initialised = false;
    bool draw_spoke = false;
    double decay = 2.0;

    EditorTool(bool spoke) : draw_spoke(spoke) {}
};


class Renderer {
private:
    RenderContext& ctx_;
    TensorField* tf_ptr_;
    RoadGenerator* generator_ptr_;

    std::unordered_map<RoadType, RoadStyle> road_styles_ = {
        {Main, RoadStyle::default_main()},
        {HighStreet, RoadStyle::default_highstreet()},
        {SideStreet, RoadStyle::default_sidestreet()}
    };


    UIMode mode_ = FieldEditor;
    Tool brush_ = GridBrush;

    EditorTool radial_edit_ = EditorTool(false);
    EditorTool grid_edit_   = EditorTool(true);

    bool generated_;
    bool step_mode_ = false;
    int road_idx_ = 0;


    bool mouse_in_viewport();
    
    void draw_vector_line(const Vector2& vec, const Vector2& world_pos, Color col) const;
    void render_tensorfield() const;

    void render_generating_popup() const;

    void draw_streamlines(RoadType road, Eigenfield ef) const;
    void render_map();

    void editor();
    void handle_brush_release();

    void render_hud();
    void handle_tool_click(const Tool&);
    void reset_field_editor();

    #ifdef STORAGE_TEST 
    std::list<DVector2> points_;

    void test_spatial();
    void test_draw_spatial(qnode_id head_ptr);
    #endif



public:
    Renderer(RenderContext& ctx, TensorField* tf_ptr, RoadGenerator* gen_ptr);
    void main_loop();
};

#endif

