#include <cassert>

#include "raylib.h"
#include "rlgl.h"

#define RAYGUI_IMPLEMENTATION
#include "render/ui.h"


int main(int argc, char** argv)
{
    RenderContext ctx(SCREEN_WIDTH, SCREEN_HEIGHT, "MapGen");
    ctx.camera.zoom = 1.0f;

    UI ui(ctx);

    ctx.init_window(); {
        SetTargetFPS(60);
        while (!WindowShouldClose())
        {
            ui.main_loop();
        }
    } ctx.close_window();
    

    return 0;
}

