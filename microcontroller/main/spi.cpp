#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#define __GLIBC_USE
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "Common.h"

static const char *TAG = "Ledian";

/*
 * WS281x LEDs work using an input signal at 800kHz, bit 0 is represented
 * with this wave form
 *
 *  |
 *  |---.
 *  |   |
 *  +---|______---
 *  |
 *
 * while bit 1 is represented with this wave
 *
 *  |
 *  |------.
 *  |      |
 *  +------|___---
 *  |
 *
 * using SPI with a clock frequency three times the nominal we can
 * use it to transmit 0b100 in case of 0 and 0b110 in cas of 1
 *
 * each LED has 3 bytes of R, G, B colors, therefore each pixel is
 * represented with 9 bytes (72bits)
 *
 * using SPI in half duplex with 4 output lines we can send data to
 * 4 LED panels at once, each line will carry information for a 
 * LED strip
 *
 * data is packed in this way
 *                              1 byte
 * .---------------------------------------------------------------.
 * | IO0_0 | IO1_0 | IO2_0 | IO3_0 | IO0_1 | IO1_1 | IO2_1 | IO3_1 |
 * '---------------------------------------------------------------'
 *
 */
#define BYTES_PER_PIXEL                 9
#define BYTES_PER_EACH_COLOR            3

#define LED_PANELS                      4
#define LEDS_PER_PANEL                  480
#define LEDS_SPI_BUFFER                 (LED_PANELS * LEDS_PER_PANEL * BYTES_PER_PIXEL)

#define LED_PANELS_NTCS                 4
#define PSU_12V_UNITS                   2

/*
 * GPIOs definition for SPI using GPIO matrix, more flexibility
 * rather than IOMux and no need to high speeds
 */
#define GPIO_SPI_CLK                    (gpio_num_t) 8
#define GPIO_SPI_LED_CS                 (gpio_num_t) 13
#define GPIO_SPI_ADC1_CS                (gpio_num_t) 19
#define GPIO_SPI_ADC2_CS                (gpio_num_t) 20
#define GPIO_SPI_ADC3_CS                (gpio_num_t) 21
#define GPIO_SPI_IO0_MOSI               (gpio_num_t) 6
#define GPIO_SPI_IO1_MISO               (gpio_num_t) 7
#define GPIO_SPI_IO2                    (gpio_num_t) 18
#define GPIO_SPI_IO3                    (gpio_num_t) 9

/*
 * GPIOs for reading PSU good signals
 */
#define GPIO_PSU_GOOD_0                 (gpio_num_t) 5
#define GPIO_PSU_GOOD_1                 (gpio_num_t) 11

/*
 * GPIOs for I2C communication with BME280 and other possible
 * external sensors or devices
 */
#define GPIO_I2C_SDA                    (gpio_num_t) 3
#define GPIO_I2C_SCL                    (gpio_num_t) 2

/*
 * I2C addresses and bus frequencies of devices
 */
#define I2C_BME280_ADDRESS              0x76
#define I2C_BME280_BUS_FREQ             (400 * 1000)

/*
 * 2.4MHz frequency, WS281x uses a protocol that runs at 800kHz
 * each bit of the LED's protocol is encoded with 3 bits on SPI
 */
#define LED_SPI_FREQ                    (3 * 800 * 1000)
#define LED_SPI_QUEUE_SIZE              2

/*
 * MCP3208 - ADC maximum SPI speed is 2MHz at 5V, 1MHz at 2.7V
 * Input resistance of NTCs is as high as 4.7k, so use a rather
 * slow speed to allow a proper sampling time, maybe it can be
 * safely increased to an higher frequency
 * 
 * ADC VREF currently is 3.3v, the same as the VDD, in the final
 * setup a precise reference of 3/2.5/2.048V will be used
 */
#define ADC_SPI_FREQ                    (750 * 1000)
#define ADC_SPI_QUEUE_SIZE              10
#define ADC_VREF                        3.3f
#define ADC_NUM_ADCS                    3
#define ADC_NUM_CHANNELS                8
#define ADC_RESOLUTION_BITS             12
#define ADC_TOTAL_CHANNELS              (ADC_NUM_ADCS * ADC_NUM_CHANNELS)

#define ADC1_CHANNEL_0_GND              0
#define ADC1_CHANNEL_1_POTENTIOMETER    1
#define ADC1_CHANNEL_2_PHOTORESISTOR    2
#define ADC1_CHANNEL_3_VCC              3
#define ADC1_CHANNEL_4_GND              4
#define ADC1_CHANNEL_5_GND              5
#define ADC1_CHANNEL_6_GND              6
#define ADC1_CHANNEL_7_GND              7

#define ADC2_CHANNEL_0_GND              8
#define ADC2_CHANNEL_1_GND              9
#define ADC2_CHANNEL_2_GND              10
#define ADC2_CHANNEL_3_GND              11
#define ADC2_CHANNEL_4_GND              12
#define ADC2_CHANNEL_5_GND              13
#define ADC2_CHANNEL_6_GND              14
#define ADC2_CHANNEL_7_GND              15

#define ADC3_CHANNEL_0_GND              16
#define ADC3_CHANNEL_1_GND              17
#define ADC3_CHANNEL_2_GND              18
#define ADC3_CHANNEL_3_GND              19
#define ADC3_CHANNEL_4_GND              20
#define ADC3_CHANNEL_5_GND              21
#define ADC3_CHANNEL_6_GND              22
#define ADC3_CHANNEL_7_GND              23

/*
 * Temporary just for the developing board
 */
#define GPIO_STATIC_VS_ANIMATED         (gpio_num_t) 0

enum led_working_mode
{
    LED_WORKING_MODE_STATIC,
    LED_WORKING_MODE_ANIMATED,
};

typedef enum led_working_mode led_working_mode_e;

/* LEDIAN */

struct led
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

typedef struct led led_t;

struct ledian
{
    led_t leds[LED_PANELS][LEDS_PER_PANEL];
    uint8_t frame_buffer[LEDS_SPI_BUFFER];
    spi_host_device_t spi_host;
    spi_device_handle_t led_spi_handle;
    spi_device_handle_t adc_spi_handle[ADC_NUM_ADCS];
    i2c_master_bus_handle_t i2c_handle;
    i2c_master_dev_handle_t bme280_handle;
};

typedef struct ledian ledian_t;

struct ledian_sensor
{
    float panel_temp[LED_PANELS][LED_PANELS_NTCS];
    float psu_temp[PSU_12V_UNITS];
    float support_board_temp;
    float support_board_humidity;
    float support_board_pressure;
    bool psu_active[PSU_12V_UNITS];
};

typedef struct ledian_sensor ledian_sensor_t;

ledian_t ledian;
ledian_sensor_t ledian_sensor;

static int setup_spi(ledian_t *l)
{
    ESP_LOGI(TAG, "Setup SPI");

    l->spi_host = SPI2_HOST;

    const spi_bus_config_t spi_bus_cfg = {
        .data0_io_num = GPIO_SPI_IO0_MOSI,
        .data1_io_num = GPIO_SPI_IO1_MISO,
        .sclk_io_num = GPIO_SPI_CLK,
        .data2_io_num = GPIO_SPI_IO2,
        .data3_io_num = GPIO_SPI_IO3,
        .max_transfer_sz = LEDS_SPI_BUFFER,
        .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_QUAD | SPICOMMON_BUSFLAG_GPIO_PINS,
    };

    const spi_device_interface_config_t led_spi_dev_cfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .clock_source = SPI_CLK_SRC_DEFAULT,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = LED_SPI_FREQ,
        .spics_io_num = GPIO_SPI_LED_CS,
        .flags = SPI_DEVICE_HALFDUPLEX,
        .queue_size = LED_SPI_QUEUE_SIZE,
    };

    const spi_device_interface_config_t adc_spi_dev_cfg[ADC_NUM_ADCS] =
    {
        {
            .command_bits = 8,
            .address_bits = 0,
            .dummy_bits = 0,
            .mode = 0,
            .clock_source = SPI_CLK_SRC_DEFAULT,
            .cs_ena_pretrans = 0,
            .cs_ena_posttrans = 0,
            .clock_speed_hz = ADC_SPI_FREQ,
            .spics_io_num = GPIO_SPI_ADC1_CS,
            .flags = SPI_DEVICE_HALFDUPLEX,
            .queue_size = ADC_SPI_QUEUE_SIZE,
        },
        {
            .command_bits = 8,
            .address_bits = 0,
            .dummy_bits = 0,
            .mode = 0,
            .clock_source = SPI_CLK_SRC_DEFAULT,
            .cs_ena_pretrans = 0,
            .cs_ena_posttrans = 0,
            .clock_speed_hz = ADC_SPI_FREQ,
            .spics_io_num = GPIO_SPI_ADC2_CS,
            .flags = SPI_DEVICE_HALFDUPLEX,
            .queue_size = ADC_SPI_QUEUE_SIZE,
        },
        {
            .command_bits = 8,
            .address_bits = 0,
            .dummy_bits = 0,
            .mode = 0,
            .clock_source = SPI_CLK_SRC_DEFAULT,
            .cs_ena_pretrans = 0,
            .cs_ena_posttrans = 0,
            .clock_speed_hz = ADC_SPI_FREQ,
            .spics_io_num = GPIO_SPI_ADC3_CS,
            .flags = SPI_DEVICE_HALFDUPLEX,
            .queue_size = ADC_SPI_QUEUE_SIZE,
        },
    };

    esp_err_t err = spi_bus_initialize(l->spi_host, &spi_bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI line, err: %d", err);
        return -1;
    }

    err = spi_bus_add_device(l->spi_host, &led_spi_dev_cfg, &l->led_spi_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device to SPI line, err: %d", err);
        return -1;
    }
    ESP_LOGI(TAG, "Initialized LED SPI device");

    for (int i = 0; i < ADC_NUM_ADCS; i++) {
        err = spi_bus_add_device(l->spi_host, &adc_spi_dev_cfg[i], &l->adc_spi_handle[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add device to SPI line, err: %d", err);
            return -1;
        }
    }
    ESP_LOGI(TAG, "Initialized ADC SPI device");

    return 0;
}

static int setup_i2c(ledian_t *l)
{
    ESP_LOGI(TAG, "Setup I2C");

    const i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = -1,
        .sda_io_num = GPIO_I2C_SDA,
        .scl_io_num = GPIO_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags { .enable_internal_pullup = true },
    };

    esp_err_t err = i2c_new_master_bus(&i2c_mst_config, &l->i2c_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C line, err: %d", err);
        return -1;
    }

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_BME280_ADDRESS,
        .scl_speed_hz = I2C_BME280_BUS_FREQ,
    };

    err = i2c_master_bus_add_device(l->i2c_handle, &dev_cfg, &l->bme280_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add BME280 to I2C devices, err: %d", err);
        return -1;
    }

    return 0;
}

#define SPI_BIT_1           0b110
#define SPI_BIT_0           0b100

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

    ESP_LOGD(TAG, "Converting 0x%02x to 0x%08dx", (int) data, (int) spi_data);
    ESP_LOGD(TAG, "Writing buf 0x%02x 0x%02x 0x%02x", buf[0], buf[1], buf[2]);
}

static void set_pixel(uint8_t *buffer, uint32_t index, led_t led)
{
    uint32_t start = index * BYTES_PER_PIXEL;

    memset(buffer + start, 0, BYTES_PER_PIXEL);

    set_bits(buffer + start, led.g);
    set_bits(buffer + start + BYTES_PER_EACH_COLOR, led.r);
    set_bits(buffer + start + BYTES_PER_EACH_COLOR * 2, led.b);
}

static void setup_gpio(void)
{
    ESP_LOGI(TAG, "Setup GPIO");

    /*
     * For SPI IO lines and clock disable pullups and enable pulldowns
     * For CS of SPI devices disable pulldowns and enable pullups
     * Direction and other settings are perfomed by SPI setup
     */
    gpio_pullup_dis(GPIO_SPI_IO0_MOSI);
    gpio_pullup_dis(GPIO_SPI_IO1_MISO);
    gpio_pullup_dis(GPIO_SPI_IO2);
    gpio_pullup_dis(GPIO_SPI_IO3);

    gpio_pulldown_en(GPIO_SPI_IO0_MOSI);
    gpio_pulldown_en(GPIO_SPI_IO1_MISO);
    gpio_pulldown_en(GPIO_SPI_IO2);
    gpio_pulldown_en(GPIO_SPI_IO3);

    gpio_pullup_en(GPIO_SPI_ADC1_CS);
    gpio_pullup_en(GPIO_SPI_ADC2_CS);
    gpio_pullup_en(GPIO_SPI_ADC3_CS);
    gpio_pullup_en(GPIO_SPI_LED_CS);

    /*
     * Disable both pullup/pulldown for I2C lines, there's a strong external pullup
     */
    gpio_pullup_en(GPIO_I2C_SCL);
    gpio_pullup_en(GPIO_I2C_SDA);
    gpio_pulldown_dis(GPIO_I2C_SCL);
    gpio_pulldown_dis(GPIO_I2C_SDA);

    /*
     * Set PSU power good signals as input with no pullup/pulldown
     */
    gpio_set_direction(GPIO_PSU_GOOD_0, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_PSU_GOOD_1, GPIO_MODE_INPUT);
    gpio_pullup_dis(GPIO_PSU_GOOD_0);
    gpio_pullup_dis(GPIO_PSU_GOOD_0);
    gpio_pullup_dis(GPIO_PSU_GOOD_1);
    gpio_pullup_dis(GPIO_PSU_GOOD_1);

    gpio_set_direction(GPIO_STATIC_VS_ANIMATED, GPIO_MODE_INPUT);
    gpio_pullup_en(GPIO_STATIC_VS_ANIMATED);
}

static void create_buffer_for_4bit(uint8_t out[4], uint8_t l0, uint8_t l1, uint8_t l2, uint8_t l3)
{
    uint32_t temp = 0;
    uint8_t in[4] = { l0, l1, l2, l3 };

    for (int i = 0; i < 8; i++) {
        uint32_t bit_a = (l0 >> i) & 1;
        uint32_t bit_b = (l1 >> i) & 1;
        uint32_t bit_c = (l2 >> i) & 1;
        uint32_t bit_d = (l3 >> i) & 1;

        temp |= (bit_a << (i * 4));
        temp |= (bit_b << (i * 4 + 1));
        temp |= (bit_c << (i * 4 + 2));
        temp |= (bit_d << (i * 4 + 3));
    }

    out[0] = (temp >> 24)  & 0xff;
    out[1] = (temp >> 16)  & 0xff;
    out[2] = (temp >> 8) & 0xff;
    out[3] = (temp >> 0) & 0xff;

    ESP_LOGD(TAG, "Converting from 0x%02x%02x%02x%02x to 0x%02x%02x%02x%02x",
                in[0], in[1], in[2], in[3], out[0], out[1], out[2], out[3]);
}

#define LED_LUMINOSITY              1

const led_t red =     { .r = LED_LUMINOSITY, .g = 0,              .b = 0              };
const led_t green =   { .r = 0,              .g = LED_LUMINOSITY, .b = 0              };
const led_t blue =    { .r = 0,              .g = 0,              .b = LED_LUMINOSITY };
const led_t yellow =  { .r = LED_LUMINOSITY, .g = LED_LUMINOSITY, .b = 0              };
const led_t magenta = { .r = LED_LUMINOSITY, .g = 0,              .b = LED_LUMINOSITY };
const led_t cyan =    { .r = 0,              .g = LED_LUMINOSITY, .b = LED_LUMINOSITY };
const led_t white =   { .r = LED_LUMINOSITY, .g = LED_LUMINOSITY, .b = LED_LUMINOSITY };

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

float adc_voltage_values[ADC_TOTAL_CHANNELS];

led_working_mode_e get_working_mode(void)
{
    if (gpio_get_level(GPIO_STATIC_VS_ANIMATED))
        return LED_WORKING_MODE_STATIC;
    else
        return LED_WORKING_MODE_ANIMATED;
}

uint32_t get_time_time_wait_ms(void)
{
    return 200 * adc_voltage_values[ADC1_CHANNEL_1_POTENTIOMETER];
}

uint32_t get_led_luminosity(void)
{
    return 1 + (adc_voltage_values[ADC1_CHANNEL_2_PHOTORESISTOR] / ADC_VREF) * 254;
//    return 255;
//    return 10;
}

/*
 * FIXME try to avoid using this auxiliary variable, twice size of the pixel buffer
 */
uint8_t panels[LED_PANELS][LEDS_PER_PANEL * BYTES_PER_PIXEL];

uint32_t single_led;

static void update_led_panels(ledian_t *l, led_working_mode_e mode)
{
    static uint32_t iter;

    spi_transaction_t tx_conf;
    uint32_t leds = LEDS_PER_PANEL;
    uint32_t leds_bytes = leds * BYTES_PER_PIXEL;
    uint32_t leds_panels_bits = leds_bytes * 8;
#ifdef OLD
    for (unsigned p = 0; p < LED_PANELS; p++) {
        for (unsigned l = 0; l < leds; l++) {
            switch (mode)
            {
                default:
 //               case LED_WORKING_MODE_STATIC:
 //                   set_pixel(panels[p], l, white);
 //                   break;

 //               case LED_WORKING_MODE_ANIMATED:
#if 0
                    uint32_t i = (l + iter++/25) % LED_COLORS_SIZE;
                    led_t c = led_colors[i];

                    c.r *= get_led_luminosity();
                    c.g *= get_led_luminosity();
                    c.b *= get_led_luminosity();

                    set_pixel(panels[p], l, c);
#else
                    uint32_t i = (l + iter++) % LED_COLORS_SIZE;
                    led_t c = led_colors[i];
                    if (l == single_led) {
                        c.r *= get_led_luminosity();
                        c.g *= get_led_luminosity();
                        c.b *= get_led_luminosity();
                    } else {
                        c.r = 0;
                        c.g = 0;
                        c.b = 0;
                    }
                    set_pixel(panels[p], l, c);
#endif
                    break;
            }
        }
    }

    single_led = (single_led + 1) % 480;
#else
    const LEDArray &Array = LEDs.render();
    for (unsigned p = 0; p < Array.Strips.size(); p++) {
        for (unsigned l = 0; l < Array.Strips[p].LEDs.size(); l++) {
            led_t c = {
                .r = Array.Strips[p].LEDs[l].Red,
                .g = Array.Strips[p].LEDs[l].Green,
                .b = Array.Strips[p].LEDs[l].Blue
            };
            set_pixel(panels[p], l, c);
        }
    }
#endif

    memset(l->frame_buffer, 0, leds);
    uint8_t *out = &l->frame_buffer[0];
    for (int b = 0; b < BYTES_PER_PIXEL * leds; b++) {
        create_buffer_for_4bit(out, panels[0][b], panels[1][b], panels[2][b], panels[3][b]);
        out += 4;
    }

    memset(&tx_conf, 0, sizeof(tx_conf));
    tx_conf.length = leds_panels_bits * 4;
    tx_conf.tx_buffer = l->frame_buffer;
    tx_conf.flags = SPI_TRANS_MODE_QIO;

    ESP_LOGD(TAG, "Preparing SPI transaction of %d bytes", tx_conf.length);

    esp_err_t err = spi_device_transmit(l->led_spi_handle, &tx_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send SPI transaction, err: %d", err);
    }
}

static uint8_t get_addr_for_channel(uint8_t channel)
{
    return 0b01100000 | ((channel & 0b111) << 2);
}

static float convert_raw_to_voltage(uint16_t raw)
{
    return  (raw / (float)(1 << ADC_RESOLUTION_BITS)) * ADC_VREF;
}

static void adc_acquisition(ledian_t *l, float voltage_values[ADC_TOTAL_CHANNELS], int adc)
{
    spi_transaction_t tx_conf[ADC_NUM_CHANNELS];

    for (unsigned channel = 0; channel < ADC_NUM_CHANNELS; channel++) {
        memset(&tx_conf[channel], 0, sizeof(tx_conf[channel]));
        tx_conf[channel].rxlength = 16;
        tx_conf[channel].flags = SPI_TRANS_USE_RXDATA;

        tx_conf[channel].cmd = get_addr_for_channel(channel);
        esp_err_t err = spi_device_queue_trans(l->adc_spi_handle[adc], &tx_conf[channel], portMAX_DELAY);
        if (err != ESP_OK)
            ESP_LOGE(TAG, "Failed to send SPI transaction, err: %d", err);
    }

    for (unsigned channel = 0; channel < ADC_NUM_CHANNELS; channel++) {
        spi_transaction_t *result;
        esp_err_t err = spi_device_get_trans_result(l->adc_spi_handle[adc], &result, portMAX_DELAY);
        if (err != ESP_OK)
            ESP_LOGE(TAG, "Failed to receive data of SPI transaction, err: %d", err);

        uint16_t raw = result->rx_data[0] << 4 | result->rx_data[1] >> 4;
        float voltage = convert_raw_to_voltage(raw);

        ESP_LOGD(TAG, "Read data from ADC: 0x%02x 0x%02x 0x%02x 0x%02x",
                result->rx_data[0], result->rx_data[1], result->rx_data[2], result->rx_data[3]);

        voltage_values[adc * ADC_NUM_CHANNELS + channel] = voltage;
    }
}

static void adc_conversion(ledian_sensor_t *sensor, float voltage_values[ADC_TOTAL_CHANNELS])
{
    /* do the magic converting voltages to meaningful numbers */

    for (int channel = 0; channel < ADC_NUM_ADCS * ADC_NUM_CHANNELS; channel++) {
        ESP_LOGI(TAG, "Read voltage chan %02d -> %f", channel, voltage_values[channel]);
    }
}

static void i2c_test_transaction(ledian_t *l, ledian_sensor_t *sensor)
{
    uint8_t tx_data;

    tx_data = 0x00;

//    esp_err_t err = i2c_master_transmit(l->bme280_handle, &tx_data, sizeof(tx_data), pdMS_TO_TICKS(2));
//    if (err != ESP_OK) {
//        ESP_LOGE(TAG, "Failed to write to I2C device");
//    }

    for (int i = 0; i < 255; i++) {
        esp_err_t err = i2c_master_probe(l->i2c_handle, i, pdMS_TO_TICKS(10));
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Found device with address 0x%02x", i);
        }
    }
}

void renderer_main2(void *)
{
    led_working_mode_e mode;

    setup_spi(&ledian);

    // setup_i2c(&ledian);

    setup_gpio();

    ESP_LOGI(TAG, "Dumping GPIO configuration");
    gpio_dump_io_configuration(stdout, SOC_GPIO_VALID_GPIO_MASK);

    ESP_LOGI(TAG, "Start LED strip test");

    for (;;) {
        // mode = get_working_mode();
        mode = LED_WORKING_MODE_ANIMATED;

        update_led_panels(&ledian, mode);

        for (int adc = 0; adc < ADC_NUM_ADCS; adc++)
            adc_acquisition(&ledian, adc_voltage_values, adc);

        // adc_conversion(&ledian_sensor, adc_voltage_values);

        // i2c_test_transaction(&ledian, &ledian_sensor);

        uint32_t time_time_sleep_ms = get_time_time_wait_ms();

        //vTaskDelay(pdMS_TO_TICKS(time_time_sleep_ms));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
