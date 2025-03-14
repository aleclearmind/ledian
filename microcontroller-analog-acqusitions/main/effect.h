#ifndef EFFECT_H
#define EFFECT_H

#include <stdint.h>

struct led
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

typedef struct led led_t;

struct effect
{
    void (*init)(unsigned width, unsigned height);
    void (*render_frame)(led_t *leds);
};

typedef struct effect effect_t;

#endif
