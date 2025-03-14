#include "fullpower_effect.h"

struct fullpower_effect
{
    unsigned width;
    unsigned height;
};

typedef struct fullpower_effect fullpower_effect_t;

fullpower_effect_t be;

static void fullpower_effect_init(unsigned width, unsigned height)
{
    be.width = width;
    be.height = height;
}

static led_t white = { 255, 255, 255 };

static void fullpower_effect_render_frame(led_t *leds)
{
    for (int i = 0; i < be.width * be.height; i++)
        leds[i] = white;
}

effect_t fullpower_effect =
{
    .init = fullpower_effect_init,
    .render_frame = fullpower_effect_render_frame,
};
