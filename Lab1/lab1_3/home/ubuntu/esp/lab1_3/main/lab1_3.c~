#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include "sdkconfig.h"

#define BLINK_GPIO 2
static uint8_t s_led_state = 0;
static led_strip_handle_t StripLED;

void BlinkStrip(){
     if(s_led_state){
        led_strip_set_pixel(StripLED, 0, 30, 30, 30);
	led_strip_refresh(StripLED);
     }
     else{
     	led_strip_clear(StripLED);
     }

}

void blink_task(void *pvParameter){
     // Look at esp-idf github under blink and main
     //esp_rom_gpio_pad_select_gpio(BLINK_GPIO);
 
     led_strip_config_t strip_config = {
     	 .strip_gpio_num = BLINK_GPIO,
	 .max_leds = 1,
	 .led_pixel_format = LED_PIXEL_FORMAT_GRB,
	 .led_model = LED_MODEL_WS2812,
	 .flags.invert_out = false,
     };
     led_strip_rmt_config_t rmt_config = {
     	 .resolution_hz = 10 * 1000 * 1000,
	 .flags.with_dma = false,
     };
     ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &StripLED));
     while(1){
     	  // NOTHIING
	  BlinkStrip();
	  s_led_state = !(s_led_state);
	  vTaskDelay(1000 / portTICK_PERIOD_MS);
     }
}

void app_main(void){
     xTaskCreate(&blink_task, "blink_task", 2048, NULL, 5, NULL); 
}
