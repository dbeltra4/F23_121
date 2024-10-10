#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "esp_timer.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 10
#define I2C_MASTER_SCL_IO 8
#define I2C_MASTER_FREQ_HZ 100000

#define ICM_ADDR 0x68 // I2C Address for ICM-42670-P 

// Gyro Data for X/Y values
#define GYRO_DATA_X1 0x11
#define GYRO_DATA_X0 0x12
#define GYRO_DATA_Y1 0x13
#define GYRO_DATA_Y0 0x14

// Acceleration Data for X/Y values
#define ACCEL_DATA_X1 0x0B
#define ACCEL_DATA_X0 0x0C
#define ACCEL_DATA_Y1 0x0D
#define ACCEL_DATA_Y0 0x0E
#define ACCEL_DATA_Z1 0x0F
#define ACCEL_DATA_Z0 0x10

#define PWR_MGMTO 0x1F
#define PWR_D_SETTING 0x0C

#define GYRO_CONFIG 0x20
#define GYRO_D_SETTING 0x6C

void i2c_master_init(){
     	i2c_config_t config = {
     	.mode = I2C_MODE_MASTER,
     	.sda_io_num = I2C_MASTER_SDA_IO,
     	.sda_pullup_en = GPIO_PULLUP_ENABLE,
     	.scl_io_num = I2C_MASTER_SCL_IO,
     	.scl_pullup_en = GPIO_PULLUP_ENABLE,
     	.master.clk_speed = I2C_MASTER_FREQ_HZ,
     	.clk_flags = 0,
     };

     i2c_param_config(I2C_MASTER_NUM, &config);
     i2c_driver_install(I2C_MASTER_NUM, config.mode, 0, 0, 0);
}

uint8_t readSensor(uint8_t writeVal){
     // code
     uint8_t accelData; 
     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
     i2c_master_start(cmd);
     i2c_master_write_byte(cmd, (ICM_ADDR << 1) | I2C_MASTER_WRITE, true);
     i2c_master_write_byte(cmd, writeVal, true);

     i2c_master_start(cmd); 
     i2c_master_write_byte(cmd, (ICM_ADDR << 1) | I2C_MASTER_READ, true);
     i2c_master_read_byte(cmd, &accelData, I2C_MASTER_NACK);

     i2c_master_stop(cmd);
     i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000)); 
     i2c_cmd_link_delete(cmd);
     
     return accelData;
}

int writeSensor(uint8_t writeVal, uint8_t accelData){
     // code
     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
     i2c_master_start(cmd);
     i2c_master_write_byte(cmd, (ICM_ADDR << 1) | I2C_MASTER_WRITE, true);
     i2c_master_write_byte(cmd, writeVal, true);
     i2c_master_write_byte(cmd, accelData, true);
     i2c_master_stop(cmd);
     esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
     if(ret != ESP_OK){
        ESP_LOGE("I2C_ERROR", "Write failed: %d", ret);
     }
     i2c_cmd_link_delete(cmd);
     return ret; 
}

void app_main(void){
     // code
     i2c_master_init();
     // Configure the Gyroscope by calling on writeSensor
     // This is the writing of specific values to particular mem locations
     esp_err_t error = writeSensor(PWR_MGMTO, PWR_D_SETTING);
     vTaskDelay(1000 / portTICK_PERIOD_MS);
     if(error != ESP_OK){
        ESP_LOGE("PWR ERROR", "PWR configuration failed!");
     }else{
     	ESP_LOGI("PWR SUCCESS", "PWR configuration successfull!");
     }
     // Let there be a delay after
     error = writeSensor(GYRO_CONFIG, GYRO_D_SETTING);
     vTaskDelay(1000/portTICK_PERIOD_MS);
     if(error != ESP_OK){
        ESP_LOGE("GYRO ERROR", "Gyroscope configuration failed!");
     }else{
     	ESP_LOGI("GYRO SUCCESS", "Gyroscope configuration successfull");
     }
     while(true){
	  int16_t xGyro, yGyro;
	  // call on readSensor, like this
	  xGyro = (readSensor(GYRO_DATA_X1) << 8) | readSensor(GYRO_DATA_X0);
	  yGyro = (readSensor(GYRO_DATA_Y1) << 8) | readSensor(GYRO_DATA_Y0);

	  // Since we are obtaining the values, we want ti implement the gyroscope sensitivity

	  // Print the coordinates
	  //ESP_LOGI("GYRO DATA", "Gyroscope x-axis data: %.1f", xGyro);
	  //ESP_LOGI("GYRO DATA", "Gyroscope y-axis data: %.1f", yGyro);

          if(xGyro > 2500 && yGyro < -2500){
	     ESP_LOGI("Gyroscope", "UP LEFT");
	  }
	  else if(xGyro > 2500 && yGyro > 2500){
	     ESP_LOGI("Gyroscope", "UP RIGHT");
	  }
	  else if(xGyro < -2500 && yGyro < -2500){
	     ESP_LOGI("Gyroscope", "DOWN LEFT");
	  }
	  else if(xGyro < -2500 && yGyro > 2500){
	     ESP_LOGI("Gyroscope", "DOWN RIGHT");
	  }
	  else if(xGyro < -2500){
	     ESP_LOGI("Gyroscope", "DOWN");
	  }
	  else if(xGyro > 2500){
	     ESP_LOGI("Gyroscope", "UP");     
	  }
	  else if(yGyro < -2500){
	     ESP_LOGI("Gyroscope", "LEFT");
	  }
	  else if(yGyro > 2500){
	     ESP_LOGI("Gyroscope", "RIGHT");
	  }
	  //printf("\n");
	  vTaskDelay(100/ portTICK_PERIOD_MS);
     }
}
