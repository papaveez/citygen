#ifndef STYLES_H
#define STYLES_H

#include "raylib.h"

struct FieldStyle {
    int granularity  = 26;
    float line_scale = 10.0f;
    float line_thick = 2.0f;
    Color major_col  = RED;
    Color minor_col  = BLUE;
    Color degen_col  = BLACK;
};

struct RoadStyle {
    Color col;
    Color outline_col;
    float width;
    float outline_width;
};

static constexpr RoadStyle main_road_style {
    Color {250, 224,  98, 255}, 
    Color {238, 199, 132, 255}, 
    10.0f,
    2.0f
};

static constexpr RoadStyle high_street_style {
    Color {252, 252, 224, 255},
    Color {240, 210, 152, 255},
    8.0f,
    2.0f,
};

static constexpr RoadStyle side_street_style {
    Color {255, 255, 255, 255}, 
    Color {215, 208, 198, 255}, 
    6.0f,
    1.0f
};


static constexpr RoadStyle default_styles[3] = {main_road_style, high_street_style, side_street_style};

struct ToolBarStyle {
    Color col = LIGHTGRAY;
    float width = 60;
    int icon_pixel_size = 3;
    float icon_padding = 6;
    float icon_size = 48;
    float y_begin = 20;
};

#endif
