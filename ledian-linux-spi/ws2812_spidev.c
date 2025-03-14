#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/ioctl.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include "config.h"
#include "ws2812_spidev.h"

#define SPI_BIT_1           (0b110)
#define SPI_BIT_0           (0b100)

#define LED_SPI_MODE        (0)
#define LED_SPI_FREQ        (3 * 800 * 1000)
#define LED_SPI_BITS        (8)

#define BIT(n)             (1UL << (n))

/*
 * FIXME try to avoid using this auxiliary variable, twice size of the pixel buffer
 */
static uint8_t panels[LED_PANELS][LEDS_PER_PANEL * BYTES_PER_PIXEL];

static void set_bits(uint8_t *buf, uint8_t data)
{
    uint32_t spi_data = 0;

    for (unsigned i = 0; i < 8; i++) {
        if (data & BIT(i))
            spi_data |= SPI_BIT_1 << (3 * i);
        else
            spi_data |= SPI_BIT_0 << (3 * i);
    }

    buf[2] = (spi_data >> 0) & 0xff;
    buf[1] = (spi_data >> 8) & 0xff;
    buf[0] = (spi_data >> 16) & 0xff;

//    printf("Converting 0x%02x to 0x%08lx\n", data, spi_data);
//    printf("Writing buf 0x%02x 0x%02x 0x%02x\n", buf[0], buf[1], buf[2]);
}

static void set_pixel(uint8_t *buffer, uint32_t index, led_t led)
{
    uint32_t start = index * BYTES_PER_PIXEL;

    memset(buffer + start, 0, BYTES_PER_PIXEL);

    set_bits(buffer + start, led.g);
    set_bits(buffer + start + BYTES_PER_EACH_COLOR, led.r);
    set_bits(buffer + start + BYTES_PER_EACH_COLOR * 2, led.b);
}

void set_led_pixed(int x, int y, led_t l)
{
    int x_p, y_p, index, panel;

    if (x > (LEDS_WIDTH / 2)) {
        if (y > (LEDS_HEIGHT / 2)) {
            x_p = x - (LEDS_HEIGHT / 2);
            y_p = y - (LEDS_WIDTH / 2);
            panel = 3;
        } else {
            x_p = x;
            y_p = y - (LEDS_WIDTH / 2);
            panel = 2;
        }
    } else {
        if (y > (LEDS_HEIGHT / 2)) {
            x_p = x - (LEDS_HEIGHT / 2);
            y_p = y;
            panel = 1;
        } else {
            x_p = x;
            y_p = y;
            panel = 0;
        }
    }

    index = (LEDS_WIDTH / 2) * y_p + x_p;

    set_pixel(panels[panel], index, l);
}

static led_t get_led_color(ws2812_spidev_t *l, int panel, int index)
{
    int x_start, y_start, x, y;

    switch (panel)
    {
    default:
    case 0:
        x_start = 0;
        y_start = 0;
        break;

    case 1:
        x_start = LEDS_WIDTH / 2;
        y_start = 0;
        break;

    case 2:
        x_start = 0;
        y_start = LEDS_HEIGHT / 2;
        break;

    case 3:
        x_start = LEDS_WIDTH / 2;
        y_start = LEDS_HEIGHT / 2;
        break;
    }

    x = index % (LEDS_WIDTH / 2);
    y = index / (LEDS_WIDTH / 2);

    if ((y % 2) == 1)
        x = (LEDS_WIDTH / 2) - 1 - x;

//    printf("Getting LED panel: %d, index: %d -> x: %d, x_start: %d, y: %d, y_start: %d\n", panel, index, x, x_start, y, y_start);

    return l->leds[x + x_start][y + y_start];
}

int open_spidev(const char *spidev)
{
    int fd = open(spidev, O_RDWR);
    if (fd < 0) {
        printf("Failed to open spidev '%s'\n", spidev);
        return -1;
    }

    unsigned mode = 0;
    int ret = ioctl(fd, SPI_IOC_WR_MODE32, &mode);
    if (ret == -1) {
        printf("Failed to set write mode to 0x%08x\n", mode);
        goto fail;
    }

    unsigned bits = LED_SPI_BITS;
    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret == -1) {
        printf("Failed to set write bits to %u\n", bits);
        goto fail;
    }

    ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if (ret == -1) {
        printf("Failed to set read bits to %u\n", bits);
        goto fail;
    }

    unsigned speed = LED_SPI_FREQ;
    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret == -1) {
        printf("Failed to set max write speed to %u Hz\n", speed);
        goto fail;
    }

    ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if (ret == -1) {
        printf("Failed to set max read speed to %u Hz\n", speed);
        goto fail;
    }

    return fd;

fail:
    close(fd);
    return -1;
}

const char *spidev_panels[LED_PANELS] =
{
    "/dev/spidev0.0",
    "/dev/spidev3.0",
    "/dev/spidev4.0",
    "/dev/spidev5.0",
};

void init_led_panels(ws2812_spidev_t *l)
{
    for (int i = 0; i < LED_PANELS; i++) {
        int ret = l->fd_spidev[i] = open_spidev(spidev_panels[i]);
        if (ret == -1) {
            printf("Failed to initalized spidev '%s' for panel %d\n", spidev_panels[i], i);
        } else {
            printf("Initalized spidev '%s' for panel %d\n", spidev_panels[i], i);
        }
    }
}

void update_led_panels(ws2812_spidev_t *l)
{
    uint32_t leds = LEDS_PER_PANEL;
    uint32_t leds_bytes = leds * BYTES_PER_PIXEL;
    uint32_t leds_panels_bits = leds_bytes * 8;

    for (unsigned p = 0; p < LED_PANELS; p++) {
        for (unsigned d = 0; d < LEDS_PER_PANEL; d++) {
            led_t c = get_led_color(l, p, d);
            set_pixel(l->panels[p], d, c);
        }
    }

    for (int i = 0; i < LED_PANELS; i++) {
        struct spi_ioc_transfer tr;

        memset(&tr, 0, sizeof(tr));

        tr.tx_buf = (void *)l->panels[0];
        tr.len = leds_bytes;
        tr.speed_hz = LED_SPI_FREQ;
        tr.bits_per_word = LED_SPI_BITS;

//        printf("Preparing SPI transaction of %d bytes for panel %d\n", tr.len, i);

        int ret = ioctl(l->fd_spidev[i], SPI_IOC_MESSAGE(1), &tr);
        if (ret == -1)
            printf("Failed to send SPI transaction, err: %d\n", errno);
    }
}
