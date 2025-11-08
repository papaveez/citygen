#define RAYGUI_IMPLEMENTATION

#include "render/app.h"

#define SCREEN_WIDTH 1920
#define SCREEN_HEIGHT 1080

int main(int argc, char** argv)
{
    constexpr size_t num_roads = 3;

    GeneratorParameters defualt_params[num_roads] = {
        GeneratorParameters(300, 1900, 400.0, 200.0, 10.0, 1.0, 500.0, 0.1, 0.5, 10.0),
        GeneratorParameters(300, 3020, 100.0,  30.0, 8.0, 1.0, 200.0, 0.1, 0.5, 10.0),
        GeneratorParameters(300, 1970,  20.0,  15.0, 5.0, 1.0,  40.0, 0.1, 0.5, 10.0)
    };


    App app(SCREEN_WIDTH, SCREEN_HEIGHT, "CityGen", defualt_params, num_roads);
    app.go();


    return 0;
}

