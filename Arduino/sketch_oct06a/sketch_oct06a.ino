#include <Servo.h>
​
​
Servo ESC; // object to control the ESC
​
int maxthrot = 1832;
int minthrot = 1148; 
int voltpin = A1;
int readValue;
​
void setup() {
    // put your setup code here, to run once:
    ​
    pinMode(voltpin, INPUT);
    Serial.begin(9600);
    while(!Serial);
    
    ESC.attach(A2, maxthrot, minthrot);
    ​
    delay(2000);
    ​
    Serial.println("Enter anything to start calibration");
    while(!Serial.available());
    
    //calibration
    Serial.println("throttle up") ;
    ​
    ESC.writeMicroseconds(maxthrot);
    delay(5000);
    Serial.println("throttle down"); 
    ESC.writeMicroseconds(minthrot);
    delay(5000);
    ​
    delay(2000);
    ​
    //arming when there is a new calibration
    ESC.writeMicroseconds(maxthrot);
    //delay(2000);
    ESC.writeMicroseconds(minthrot);
    delay(2000);
    Serial.println("armed");
}
​
void loop() {
​
  ESC.writeMicroseconds(maxthrot-500);
​
}
