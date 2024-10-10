#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "math.h"

// Define statements, with assistance by ChatGPT
#define I2C_MASTER_SCL_IO 8 // GPIO for I2C Clock Line (SCL)
#define I2C_MASTER_SDA_IO 10 // GPIO for I2C Data Line (SDA)
#define I2C_MASTER_NUM 	I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000 // I2C Clock Frequency

#define SHTC3_ADDR 0x70 // I2C Address of SHTC3 sensor

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

esp_err_t read_temperature_and_humidity(float* tempC, float* humidity){
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

	  vTaskDelay(50 / portTICK_PERIOD_MS);


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
	  uint16_t raw_humidity = (data[3] << 8) | data[4];

	  *tempC = (float)raw_temp * 175.0 / 65535.0 - 45.0;
	  *humidity = 100.0 * ((float)raw_humidity / 65535.0);

	  return ESP_OK;
}

void app_main(void){
     float tempC, humidity;

     i2c_master_init();

     while(1){
         if(read_temperature_and_humidity(&tempC, &humidity) == ESP_OK){
	    float temperature_F = (tempC * 9.0 / 5.0) + 32.0;
	    int rTempC = round(tempC);
	    int rTempF = round(temperature_F);
	    int rHumid = round(humidity);
	    // Casting our float type vars to a int w/ rounding

	    printf("Temperature is %dC (or %dF) with a %d%% humidity\n", rTempC, rTempF, rHumid);
	 }
	 
	 else{
	    printf("Failed to read data from the sensor\n");
	 }
     	 vTaskDelay(2000 / portTICK_PERIOD_MS);
     
     }

}
