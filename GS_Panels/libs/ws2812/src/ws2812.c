#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "ws2812.h"
#include "stdio.h"

#define WS2812_FREQ 800000      // 800kHz for WS2812
#define WS2812_PERIOD (1250)    // 1.25µs period in nanoseconds

// Define duty cycles for WS2812 "0" and "1" signals
#define WS2812_DUTY_CYCLE_0 30  // ~30% of the period
#define WS2812_DUTY_CYCLE_1 70  // ~70% of the period

static uint32_t ws2812_dma_channel;
static uint16_t* ws2812_data_buffer; // Buffer to store PWM duty cycles
static size_t data_buffer_size;

void ws2812_debug_pwm(uint gpio, bool level);

// Internal function to configure PWM for WS2812
static void ws2812_configure_pwm(uint gpio) {
	gpio_init(gpio);
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);

    // Configure the PWM frequency
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 1.0f); // No clock division
    pwm_config_set_wrap(&config, WS2812_PERIOD - 1);
    pwm_init(slice_num, &config, false); // Do not start PWM yet
}

// Initialize DMA for PWM transfer
static void ws2812_configure_dma(uint gpio) {
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    ws2812_dma_channel = dma_claim_unused_channel(true);
    dma_channel_config dma_cfg = dma_channel_get_default_config(ws2812_dma_channel);

    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_16); // 16-bit transfer
    channel_config_set_dreq(&dma_cfg, DREQ_PWM_WRAP0 + slice_num); // PWM DREQ based on slice

    dma_channel_configure(
        ws2812_dma_channel,
        &dma_cfg,
        &pwm_hw->slice[slice_num].cc,  // Destination is PWM compare register
        ws2812_data_buffer,            // Source is data buffer
        data_buffer_size,               // Length of buffer
        false                          // Do not start yet
    );
}

// Setup WS2812
bool ws2812_setup(ws2812_t *ws2812, uint8_t pin, uint8_t num) {
    ws2812->pin = pin;
    ws2812->num = num;
    ws2812->rgb = malloc(num * sizeof(struct rgb_s));

	// allocate the DMA buffer size
	data_buffer_size = (num * 24) + 2;
	ws2812_data_buffer = malloc(data_buffer_size); // 3 bytes per LED

    if (ws2812->rgb == NULL) return false;

    ws2812_configure_pwm(pin);
    ws2812_configure_dma(pin);
    return true;
}

void ws2812_set(ws2812_t *ws2812, uint8_t led, uint8_t r, uint8_t g, uint8_t b) {
    if (led >= ws2812->num) return;
    ws2812->rgb[led].r = r;
    ws2812->rgb[led].g = g;
    ws2812->rgb[led].b = b;
}

void ws2812_set_all(ws2812_t *ws2812, uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < ws2812->num; i++) {
        ws2812->rgb[i].r = r;
        ws2812->rgb[i].g = g;
        ws2812->rgb[i].b = b;
    }
}

// Set RGB values in the buffer with the corresponding duty cycles
static void ws2812_prepare_data(ws2812_t *ws2812) {
    uint16_t *pBuf = ws2812_data_buffer;
    uint8_t r, g, b;

    for (int i = 0; i < ws2812->num; i++) {
        g = ws2812->rgb[i].g;
        r = ws2812->rgb[i].r;
        b = ws2812->rgb[i].b;

        // Send green, red, and blue
        for (int8_t j = 7; j >= 0; j--) {
            *pBuf++ = ((g >> j) & 1) ? WS2812_DUTY_CYCLE_1 : WS2812_DUTY_CYCLE_0;
        }
        for (int8_t j = 7; j >= 0; j--) {
            *pBuf++ = ((r >> j) & 1) ? WS2812_DUTY_CYCLE_1 : WS2812_DUTY_CYCLE_0;
        }
        for (int8_t j = 7; j >= 0; j--) {
            *pBuf++ = ((b >> j) & 1) ? WS2812_DUTY_CYCLE_1 : WS2812_DUTY_CYCLE_0;
        }
    }

    // Ensure the last bits are low for reset
    for (int i = 0; i < 2; i++) {
        ws2812_data_buffer[data_buffer_size - 2 + i] = 0;
    }
}

// Start sending data using DMA
void ws2812_show(ws2812_t *ws2812) {
    ws2812_prepare_data(ws2812);
    uint slice_num = pwm_gpio_to_slice_num(ws2812->pin);

    // Start PWM
    pwm_set_enabled(slice_num, true);

    // Start DMA
	// Reconfigure the DMA channel with source buffer and length
    dma_channel_set_read_addr(ws2812_dma_channel, ws2812_data_buffer, false);
    dma_channel_set_trans_count(ws2812_dma_channel, data_buffer_size, false);
    dma_channel_start(ws2812_dma_channel);
}

// Debug function to set output continuously
void ws2812_debug_pwm(uint gpio, bool level) {

    ws2812_configure_pwm(gpio);
    uint slice_num = pwm_gpio_to_slice_num(gpio);

    // Set duty cycle to either full on or off
    pwm_set_gpio_level(gpio, level ? WS2812_PERIOD - 1 : 0);
    pwm_set_enabled(slice_num, true);

    while (1) {
        tight_loop_contents(); // Keep the CPU in a tight loop for continuous output
    }
}