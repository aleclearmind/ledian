#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "config.h"
#include "ws2812_spidev.h"

ws2812_spidev_t ledian;

#define LED_LUMINOSITY              1

const led_t red =     { .r = LED_LUMINOSITY, .g = 0,              .b = 0              };
const led_t green =   { .r = 0,              .g = LED_LUMINOSITY, .b = 0              };
const led_t blue =    { .r = 0,              .g = 0,              .b = LED_LUMINOSITY };
const led_t yellow =  { .r = LED_LUMINOSITY, .g = LED_LUMINOSITY, .b = 0              };
const led_t magenta = { .r = LED_LUMINOSITY, .g = 0,              .b = LED_LUMINOSITY };
const led_t cyan =    { .r = 0,              .g = LED_LUMINOSITY, .b = LED_LUMINOSITY };
const led_t white =   { .r = LED_LUMINOSITY, .g = LED_LUMINOSITY, .b = LED_LUMINOSITY };
const led_t black =   { .r = 0,              .g = 0,              .b = 0              };

const led_t led_colors[] =
{
    { .r = LED_LUMINOSITY, .g = 0,              .b = 0              },
    { .r = 0,              .g = LED_LUMINOSITY, .b = 0              },
    { .r = 0,              .g = 0,              .b = LED_LUMINOSITY },
    { .r = LED_LUMINOSITY, .g = LED_LUMINOSITY, .b = 0              },
    { .r = LED_LUMINOSITY, .g = 0,              .b = LED_LUMINOSITY },
    { .r = 0,              .g = LED_LUMINOSITY, .b = LED_LUMINOSITY },
    { .r = LED_LUMINOSITY, .g = LED_LUMINOSITY, .b = LED_LUMINOSITY },
};
#define LED_COLORS_SIZE     (sizeof(led_colors) / sizeof(led_t))

static void set_led_colors(ws2812_spidev_t *l)
{
    static unsigned color;

    for (int x = 0; x < LEDS_HEIGHT; x++) {
        for (int y = 0; y < LEDS_HEIGHT; y++) {
            l->leds[x][y] = led_colors[color];
            color = (color + 1) % LED_COLORS_SIZE;
        }
    }
}

int main(int argc, char *argv[])
{
    int step = 0;
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = 5 * 1000000;

    init_led_panels(&ledian);

    for (;;) {
//        nanosleep(&ts, NULL);

//	if ((step++ % 30) == 0)
            set_led_colors(&ledian);

        update_led_panels(&ledian);
    }
}
