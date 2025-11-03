#include <cassert>

#include "raylib.h"
#include "rlgl.h"
#include "types.h"

#define RAYGUI_IMPLEMENTATION

#include "generation/generator.h"
#include "generation/tensor_field.h"

#include "render/ui.h"


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


int main(int argc, char** argv)
{
    RenderContext ctx(SCREEN_WIDTH, SCREEN_HEIGHT, "MapGen");

    TensorField tf;
    std::unique_ptr<NumericalFieldIntegrator> itg = std::make_unique<RK4>(&tf);

    RoadGenerator generator = RoadGenerator(
            itg,
            params,
            ctx.viewport
    );

    UI ui(ctx, &tf, &generator);

    ctx.init_window(); {
        ctx.camera.zoom = 1.0f;

        SetTargetFPS(60);
        while (!WindowShouldClose())
        {
            ui.main_loop();
        }
    } ctx.close_window();
    

    return 0;
}

