#include <stdio.h>
#include "math.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "DFRobot_LCD.h"
#define SHTC3_ADDR 0x70
#define I2C_MASTER_NUM  I2C_NUM_0

DFRobot_LCD lcd(16,2);

extern "C" {
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


	   void cTASK(void *pvP){
	        
		while(1){
		     char mander[10];
		     float tempC, humidity;
		     if(read_temperature_and_humidity(&tempC, &humidity) == ESP_OK){
		        lcd.setCursor(0,0);
			sprintf(mander, "Temp: %dC", (int)round(tempC));
			lcd.printstr(mander);
			lcd.setCursor(0,1);
			sprintf(mander, "Hum : %d%%", (int)round(humidity));
			lcd.printstr(mander);
		     
		     }
		     vTaskDelay(1000 / portTICK_PERIOD_MS);
		
		} 
	   }
	   
           void app_main(void){
                // Taken from Lab3 Documentation
                lcd.init();
                lcd.setRGB(0,255,0);
                xTaskCreate(&cTASK, "cTASK", 2048, NULL, 5, NULL);
            }
}
