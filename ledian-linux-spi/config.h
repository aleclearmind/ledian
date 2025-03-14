#ifndef LEDIAN_H
#define LEDIAN_H

#define LEDS_WIDTH                      64
#define LEDS_HEIGHT                     64

#define BYTES_PER_PIXEL                 9
#define BYTES_PER_EACH_COLOR            3

#define LED_PANELS                      4
#define LEDS_PER_PANEL                  64
#define LEDS_SPI_BUFFER                 (LED_PANELS * LEDS_PER_PANEL * BYTES_PER_PIXEL)

#define LEDS_REFRESH_FREQUENCY          5

#endif
