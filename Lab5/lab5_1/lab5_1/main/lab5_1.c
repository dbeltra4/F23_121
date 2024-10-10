#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "math.h"

// Define statements, with assistance by ChatGPT
#define I2C_MASTER_SCL_IO 8 // GPIO for I2C Clock Line (SCL)
#define I2C_MASTER_SDA_IO 10 // GPIO for I2C Data Line (SDA)
#define I2C_MASTER_NUM  I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000 // I2C Clock Frequency

#define SHTC3_ADDR 0x70

#define SR04_TRIGGER_PIN 1
#define SR04_ECHO_PIN 0

void i2c_master_init(){
     i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
     };
     i2c_param_config(I2C_MASTER_NUM, &conf);
     i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void sr04_init(){
     gpio_config_t trig_io_conf = {
          .pin_bit_mask = (1ULL << SR04_TRIGGER_PIN),
     	  .mode = GPIO_MODE_OUTPUT,
	  .intr_type = GPIO_INTR_DISABLE,
     
     };
     gpio_config(&trig_io_conf);

     gpio_config_t echo_io_conf = {
          .pin_bit_mask = (1ULL << SR04_ECHO_PIN),
	    .mode = GPIO_MODE_INPUT,
	    .intr_type = GPIO_INTR_POSEDGE,
     };
     gpio_config(&echo_io_conf);
}

esp_err_t read_temperature(float* tempC){
          uint8_t addr[] = {0x7C, 0xA2};
          uint8_t data[6];
          i2c_cmd_handle_t cmd = i2c_cmd_link_create();
          i2c_master_start(cmd);
          i2c_master_write_byte(cmd, (SHTC3_ADDR << 1) | I2C_MASTER_WRITE, true);
          i2c_master_write(cmd, addr, 2, true);
          i2c_master_stop(cmd);

          esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
          i2c_cmd_link_delete(cmd);

          if(ret != ESP_OK){
             return false;
          }

          vTaskDelay(20 / portTICK_PERIOD_MS);


          cmd = i2c_cmd_link_create();
          i2c_master_start(cmd);
          i2c_master_write_byte(cmd, (SHTC3_ADDR << 1) | I2C_MASTER_READ, true);
          i2c_master_read(cmd, data, sizeof(data) - 1, I2C_MASTER_ACK); // subract the MSB
          i2c_master_read_byte(cmd, &data[sizeof(data) - 1], I2C_MASTER_NACK);
          i2c_master_stop(cmd);

          ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
          i2c_cmd_link_delete(cmd);

          if(ret != ESP_OK){
             return false;
          }

          uint16_t raw_temp = (data[0] << 8) | data[1];
          //uint16_t raw_humidity = (data[3] << 8) | data[4];                                                  

          *tempC = (float)raw_temp * 175.0 / 65535.0 - 45.0;
          //*humidity = 100.0 * ((float)raw_humidity / 65535.0);

          return ESP_OK;
}

void app_main(void){
     float tempC;

     // Initialize I2C and SR04
     i2c_master_init();
     sr04_init();

     while(1){
         read_temperature(&tempC);
         float tempF = (tempC * 9.0 / 5.0) + 32.0;
         ESP_LOGI("SHTC3", "Temperature: %.1f c (%.1f F)", tempC, tempF);

	 // Trigger & Echo logicsp
	 gpio_set_level(SR04_TRIGGER_PIN, 0);
	 esp_rom_delay_us(2);
	 gpio_set_level(SR04_TRIGGER_PIN, 1);
	 esp_rom_delay_us(10);
	 gpio_set_level(SR04_TRIGGER_PIN, 0);
	 
	 /*int32_t start = esp_timer_get_time();
	 while(gpio_get_level(SR04_ECHO_PIN) == 0) {}
	 start = esp_timer_get_time();

	 while(gpio_get_level(SR04_ECHO_PIN) == 1) {}
	 //int32_t end = esp_timer_get_time();*/

	 int64_t start = esp_timer_get_time();
	 //int64_t end = esp_timer_get_time();
	 
         
	 //printf("Start: %lld    End: %lld", start, end);
	 while(gpio_get_level(SR04_ECHO_PIN) == 0 && (esp_timer_get_time()-start) < 23000){}
	       //end = esp_timer_get_time();
	
	 start = esp_timer_get_time();
	 while(gpio_get_level(SR04_ECHO_PIN) == 1 && (esp_timer_get_time()-start) < 23000){}
	       //end = esp_timer_get_time();
	 

	 //printf("New Start: %lld   End: %lld", start, end);
	 float sTravel = (esp_timer_get_time() - start); 
	 printf("sTravel: %2.f\n", sTravel);

	 float dist_temp = (333.1 + 0.606 * tempC) * 100;
	 float dist = (sTravel / 1000000) * (dist_temp / 2); 
	 //uint32_t sound_travel = (esp_timer_get_time()-start);i

	 printf("Distance: %1.f cm at %.0fC\n", dist, tempC);
	 //printf("Sound Travel (us): %d us\n", (int)sound_travel);

	 vTaskDelay(1000 / portTICK_PERIOD_MS);
     
     }

}
