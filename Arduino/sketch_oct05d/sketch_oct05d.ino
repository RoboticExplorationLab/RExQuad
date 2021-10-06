#include <Servo.h>

#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs

Servo ESC;

int potValue;  

void setup() {
    Serial.begin(9600);

    // Attach the ESC on pin 9
    ESC.attach(9, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // (pin, min pulse width, max pulse width in microseconds) 
    ESC.writeMicroseconds(1500);
    delay(7000);
}

void loop() {
  
    Serial.println("Enter PWM signal value 1148 to 1832, 1488 to stop");
    
    while (Serial.available() == 0);
    
    int val = Serial.parseInt(); 
    
    if(val < MIN_PULSE_LENGTH || val > MAX_PULSE_LENGTH) {
      Serial.print(val);
      Serial.println(" not valid");
    }
    else {
      Serial.print("Entered: "); Serial.println(val);
      ESC.writeMicroseconds(val); // Send signal to ESC.
    }
}
