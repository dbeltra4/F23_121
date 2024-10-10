#include <stdio.h>
#include "DFRobot_LCD.h"

DFRobot_LCD lcd(16,2);

extern "C" {
	   void app_main(void){
		// Taken from Lab3 Documentation
		lcd.init();
		lcd.setRGB(0,255,0);
        	while(1){
	      		lcd.setCursor(0,0);
	      		lcd.printstr("Hello CSE121!");
	      		lcd.setCursor(0,1);
	      		lcd.printstr("Beltran");
        	}
	    }
}
