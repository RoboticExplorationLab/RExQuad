#include <Servo.h>

Servo ESC; // object to control the ESC

int maxthrot = 1832;
int minthrot = 1148; 

int voltpin = A1;
int readValue;
char data;

void setup() {
    // put your setup code here, to run once:
    pinMode(voltpin, INPUT);
    Serial.begin(9600);
    while(!Serial);
    
    ESC.attach(12, maxthrot, minthrot);
    //calibration
    Serial.println("Send anything to begin calibration");
    while(!Serial.available());
    
    Serial.println("throttle up") ;
    ESC.writeMicroseconds(maxthrot);
    delay(7000);
    Serial.println("throttle down"); 
    ESC.writeMicroseconds(minthrot);
    delay(9000);
}

void loop() {
   if (Serial.available()) {
        data = Serial.read();

        switch (data) {
            // 0
            case 48 : Serial.println("Sending minimum throttle");
                      ESC.writeMicroseconds(minthrot);
            break;

            // 1
            case 49 : Serial.println("Sending maximum throttle");
                      ESC.writeMicroseconds(maxthrot);
            break;

            // 2
            case 50 : Serial.print("Running test in 3");
                      delay(1000);
                      Serial.print(" 2");
                      delay(1000);
                      Serial.println(" 1...");
                      delay(1000);
                      test();
            break;
        }
    }
    
}
/**
 * Test function: send min throttle to max throttle to each ESC.
 */
void test()
{
    for (int i = minthrot; i <= maxthrot; i += 5) {
        Serial.print("Pulse length = ");
        Serial.println(i);
        
        ESC.writeMicroseconds(i);
        
        delay(200);
    }

    Serial.println("STOP");
    ESC.writeMicroseconds(minthrot);
}

/**
 * Displays instructions to user
 */
void displayInstructions()
{  
    Serial.println("READY - PLEASE SEND INSTRUCTIONS AS FOLLOWING :");
    Serial.println("\t0 : Send min throttle");
    Serial.println("\t1 : Send max throttle");
    Serial.println("\t2 : Run test function\n");
}
