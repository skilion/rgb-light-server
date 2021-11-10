#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>

#include "io.h"


static const char* TAG = "IO";
static const int used_pins[] = {16, 17, 18, 19, 21};
static const int input_pins[] = {21};
static const int LED_PINS[] = {16, 17, 18, 19};
static const int LEDC_FREQUENCY_HZ = 5000;

static void reset_pins();
static void ledc_timer_init();
static void ledc_channels_init();


void io_init()
{
	reset_pins();
	ledc_timer_init();
	ledc_channels_init();
}

static void reset_pins()
{
	ESP_LOGI(TAG, "reset_pins");
    for (int i = 0; i < sizeof(used_pins) / sizeof(used_pins[0]); i++) {
        gpio_reset_pin(used_pins[i]);
    }
    for (int i = 0; i < sizeof(input_pins) / sizeof(input_pins[0]); i++) {
        gpio_set_direction(input_pins[i], GPIO_MODE_INPUT);
    }
}

static void ledc_timer_init()
{
	ESP_LOGI(TAG, "ledc_timer_init");
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .freq_hz          = LEDC_FREQUENCY_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
}

static void ledc_channels_init()
{
	ESP_LOGI(TAG, "ledc_channels_init");
	for (int i = 0; i < sizeof(LED_PINS) / sizeof(LED_PINS[0]); i++) {
	    ledc_channel_config_t ledc_channel = {
	        .speed_mode     = LEDC_LOW_SPEED_MODE,
	        .channel        = LEDC_CHANNEL_0 + i,
	        .timer_sel      = LEDC_TIMER_0,
	        .intr_type      = LEDC_INTR_DISABLE,
	        .gpio_num       = LED_PINS[i],
	        .duty           = 0,
	        .hpoint         = 0
	    };
	    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
	}
}

void io_set_rgb(uint32_t rgb)
{
	for (int i = 0; i < 3; i++) {
		uint32_t value = (rgb >> (i * 8)) & 0xff;
		ESP_LOGI(TAG, "value %d = %x", i, value);
		ledc_channel_t channel = LEDC_CHANNEL_0 + i;
	    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, value));
	    // Update duty to apply the new value
	    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
	}
}