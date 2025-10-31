#ifndef CONFIG_H
#define CONFIG_H


#include "raylib.h"
#include "raygui.h"

#include "../generation/road_storage.h"


static constexpr struct {
    int field_granularity  = 26;
    float field_line_scale = 10.0f;
    float field_line_thick = 2.0f;
    Color major_colour = RED;
    Color minor_colour = BLUE;
    Color degenerate_colour = BLACK;
} renderConfig;

static constexpr struct {
    int icon_size = 48;
    int icon_padding = 6;
    int icon_y_offset = 20;
    int icon_pixel_size = 3;
    float modal_width = 270.0f;
    float modal_height = 90.0f;
    Color sidebar_colour = LIGHTGRAY;
} uiConfig;


// road styles
struct RoadStyle {
    Color colour;
    Color outline_colour;
    float width;
    float outline_width;
};

static std::unordered_map<RoadType, RoadStyle> default_road_styles = {
    {Main,       { {250, 224,  98, 255}, {238, 199, 132, 255}, 10.0f, 2.0f} },
    {HighStreet, { {252, 252, 224, 255}, {240, 210, 152, 255},  8.0f, 2.0f} },
    {SideStreet, { {255, 255, 255, 255}, {215, 208, 198, 255},  6.0f, 1.0f} }
};



#endif
