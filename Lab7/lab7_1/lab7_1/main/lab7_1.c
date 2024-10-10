#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "wiringPi.h"
#include <unistd.h>
#include <ctype.h>
#include <time.h>


#define LED_GPIO_PIN 26

// Morse Code Mapping to the English Alphabet

const char *MORSE_CODE[] = {
	  ".-",    // A
	  "-...",  // B
	  "-.-.",  // C
	  "-..",   // D
	  ".",	   // E
	  "..-.",  // F
	  "--.",   // G
	  "....",  // H
	  "..",	   // I
	  ".---",  // J
	  "-.-",   // K
	  ".-..",  // L
	  "--",	   // M
	  "-.",	   // N
	  "---",   // O
	  ".--.",  // P
	  "--.-",  // Q
	  ".-.",   // R
	  "...",   // S
	  "-",	   // T
	  "..-",   // U
	  "...-",  // V
	  ".--",   // W
	  "-..-",  // X
	  "-.--",  // Y
	  "--..",  // Z
	  "-----", // 0
          ".----", // 1
          "..---", // 2
          "...--", // 3
          "....-", // 4
          ".....", // 5 
          "-....", // 6 
          "--...", // 7
          "---..", // 8
          "----."  // 9

};

void transmit_morse_char(char C){
     // Do nothing
     C = toupper(C);

     if(C == ' '){
     	usleep(2000000);
     }else{
         int dex; // creates the index from the start of the Alphabet
	 if(isdigit(C)){
	    dex = C - '0' + 26; 
	 } else if(isalpha(C)){
	    dex = C - 'A';
	 }else{
	    return ;
	 }

	 if(dex >= 0 && dex < 36){ // Set's the boundary for the alphabet
	    const char *MORSE = MORSE_CODE[dex]; // Setting more to the library 
	    
	    while(*MORSE){
	          if(*MORSE == '.'){
		     digitalWrite(LED_GPIO_PIN, HIGH); 
		     usleep(500000);
		  }else if(*MORSE == '-'){
		     digitalWrite(LED_GPIO_PIN, HIGH); 
		     usleep(750000);
		  }
		  digitalWrite(LED_GPIO_PIN, LOW);
		  usleep(500000);
		  MORSE++;
	    }
	    usleep(1000000);
	 }
     
     }
}

void transmit_morse_message(const char *message){
     // Do nothing
     int i = 0; 
     while(message[i] != '\0'){
           transmit_morse_char(message[i]); 
	   i++; 
     }
}

int main(int argc, char *argv[]){
    if(argc != 3){
       printf("Usage: %s <num_of_times_to_print_message> <message>\n", argv[0]);
       return 1;        
    }

    if(wiringPiSetupGpio() == -1){
       fprintf(stderr, "Unable to initialize wiringPi\n"); 
       return -1;
    }

    pinMode(LED_GPIO_PIN, OUTPUT);

    int number_of_times = atoi(argv[1]);

    const char *message = argv[2]; 

    int i = 0;
    while(i < number_of_times){
    	  transmit_morse_message(message);
	  i++;
    }
    
    digitalWrite(LED_GPIO_PIN, LOW);

    return 0;
}
