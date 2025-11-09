#ifndef APP_H
#define APP_H


#include <cstddef>
#include <optional>

#include "renderer.h"

#include "raygui.h"


enum AppState {
    Editor,
    Map,
    AppStateCount
};


struct Tool {
    int icon;
    std::function<void()> on_click;
};


class ToolBar : public Component {
private:
    std::optional<int> selected_;
    ToolBarStyle style_;
    Box<double> bbox_;
    std::vector<Tool>* tools_ = nullptr;

public:
    ToolBar(int window_height);
    void set_style(ToolBarStyle s);
    void set_tools(std::vector<Tool>* new_tools);
    void render_impl(Renderer* ren);
};


class App  {
private:
    Renderer ren_;
    TensorField field_;
    GeneratorParameters* params_;
    RoadGenerator gen_;

    AppState app_state_;

    std::array<std::vector<Tool>, AppStateCount> tools {
        // FieldEditor
        std::vector<Tool>{
            Tool{ICON_PLAYER_PLAY,  [this]() {set_state(Map);}}
        },
        std::vector<Tool>{
            Tool{ICON_RESTART,      [this]() {run_generator();}},
            Tool{ICON_UNDO_FILL,    [this]() {set_state(Editor);}}
        }
    };

    void run_generator();
    void reset_tensorfield();

    void set_state(AppState state);

    ToolBar toolbar;
    MapView map_view;
    TensorFieldView field_view;

    // bool mouse_in_editor() const;


public:
    App (int w, int h, const char* window_title, 
        GeneratorParameters* params, size_t road_type_count);
    void main_loop();
    void go();
};

#endif
