#ifndef WS2812_H
#define WS2812_H

#include <stdint.h>

struct led
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

typedef struct led led_t;

struct ws2812_spidev
{
    led_t leds[LEDS_WIDTH][LEDS_HEIGHT];
    uint8_t panels[LED_PANELS][LEDS_PER_PANEL * BYTES_PER_PIXEL];
    int fd_spidev[LED_PANELS];
};

typedef struct ws2812_spidev ws2812_spidev_t;

void init_led_panels(ws2812_spidev_t *l);

void update_led_panels(ws2812_spidev_t *l);

#endif