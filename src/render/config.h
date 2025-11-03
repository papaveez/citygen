#ifndef CONFIG_H
#define CONFIG_H

#include "raylib.h"
#include "raygui.h"

#include "../generation/generator.h"
#include "../types.h"

#define SCREEN_WIDTH 1920
#define SCREEN_HEIGHT 1080


static Box<double> screen_dims(
    {0.0, 0.0},
    {
        static_cast<double>(SCREEN_WIDTH),
        static_cast<double>(SCREEN_HEIGHT)
    }
);



static std::unordered_map<RoadType, GeneratorParameters> params = {
    {SideStreet, 
        GeneratorParameters(300, 1970,  20.0,  15.0, 5.0, 1.0,  40.0, 0.1, 0.5, 10.0)
    },
    {HighStreet, 
        GeneratorParameters(300, 3020, 100.0,  30.0, 8.0, 1.0, 200.0, 0.1, 0.5, 10.0)
    },
    {Main,       
        GeneratorParameters(300, 1900, 400.0, 200.0, 10.0, 1.0, 500.0, 0.1, 0.5, 10.0)
    }
};


struct CustomFonts {
    Font normal;
    // Font 
    CustomFonts(Font&& normal) : normal(std::move(normal)) {}

    ~CustomFonts() {
        UnloadFont(normal);
    }
};

static CustomFonts fonts(
    LoadFontEx("./assets/font/computer_modern/cmunrm.ttf", 32, 0, 700)
);

static constexpr struct {
    // granularity of tensorfield sample points
    int field_granularity  = 26;
    // vector lines
    float field_line_scale = 10.0f;
    float field_line_thick = 2.0f;
    Color major_col = RED;
    Color minor_col = BLUE;
    Color degenerate_col = BLACK;
} renderConfig;


static constexpr struct {
// toolbar
    int icon_size = 48;
    int icon_padding = 6;
    int icon_y_offset = 20;
    int icon_pixel_size = 3;
    Color toolbar_col = LIGHTGRAY;

// modals
    float modal_width = 270.0f;
    float modal_height = 90.0f;

// basis field centre point selector
    int basis_field_selector_rad = 9;
    Color basis_field_centre_col = DARKBLUE;
    Color basis_field_selected_col = RED;

// basis field editor pane
    int editor_pane_width = 300;
    Color editor_pane_col = LIGHTGRAY;
} uiConfig;


struct RoadStyle {
    Color col;
    Color outline_col;
    float width;
    float outline_width;
};


static std::unordered_map<RoadType, RoadStyle> 
default_road_styles = {
    {Main,       { {250, 224,  98, 255}, {238, 199, 132, 255}, 10.0f, 2.0f} },
    {HighStreet, { {252, 252, 224, 255}, {240, 210, 152, 255},  8.0f, 2.0f} },
    {SideStreet, { {255, 255, 255, 255}, {215, 208, 198, 255},  6.0f, 1.0f} }
};



#endif
