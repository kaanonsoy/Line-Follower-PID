#include <QTRSensors.h>  // Library
QTRSensors lineSensor; // Define the sensor
const uint8_t SensorCount = 8;
unsigned int sensorValues[SensorCount];  

#define RIGHTIN1 12
#define RIGHTIN2 11
#define PWMRIGHT 10  
#define LEFTIN3 7
#define LEFTIN4 8
#define PWMLEFT 9  
#define LED 13  

float Kp = 0.04;  // Proportional coefficient
float Ki = 0.0002;  // Integral coefficient
float Kd = 0.48;   // Derivative coefficient

int previousError = 0;
unsigned int Speed = 180;
float integral = 0;

void setup() {
    Serial.begin(9600);
    lineSensor.setTypeRC(); // Set the sensor type as digital
    lineSensor.setSensorPins((const byte[]){2, 3, 4, 5, 6, A2, A1, A0}, SensorCount); // Define sensor pins
       
    pinMode(RIGHTIN1, OUTPUT);
    pinMode(RIGHTIN2, OUTPUT);
    pinMode(LEFTIN3, OUTPUT); // Define pin modes
    pinMode(LEFTIN4, OUTPUT);
    pinMode(LED, OUTPUT);
    
    digitalWrite(LED, HIGH); // Turn on LED during calibration
    for (int i = 0; i < 300; i++) { // Calibration process
        lineSensor.calibrate();
    }
    
    for (byte i = 0; i < 10; i++) { // LED blinks to indicate readiness
        digitalWrite(LED, HIGH);
        delay(200);
        digitalWrite(LED, LOW);
        delay(200);
    }
}

void loop() {
    unsigned int linePosition = lineSensor.readLineWhite(sensorValues); // Measure line position
    int error = 3500 - linePosition; // Calculate deviation from the center
    
    // Print sensor values
    for (uint8_t i = 0; i < SensorCount; i++) {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
    }
    Serial.println(linePosition); // Print sensor values
    
    if (sensorValues[0] >= 850 && sensorValues[1] >= 850 && sensorValues[2] >= 850 && sensorValues[3] >= 850 && 
        sensorValues[4] >= 850 && sensorValues[5] >= 850 && sensorValues[6] >= 850 && sensorValues[7] >= 850) {

        unsigned int linePosition = lineSensor.readLineWhite(sensorValues);
        int error = 3500 - linePosition;
        
        if (error > 0) { // Check if the last detected value is on the right or left
            rightMotorBackward(200); // Turn right
            leftMotorForward(200);  
        } else {
            rightMotorForward(200); // Turn left
            leftMotorBackward(200);
        }
    } else {
        // PID Control
        float P = Kp * error; // Proportional correction
        integral += error; // Integral calculation
        float I = Ki * integral; // Integral correction
        float D = Kd * (error - previousError); // Derivative correction
        previousError = error; // Record the error

        int PID_output = P + I + D; // Total PID value

        // Calculate PWM values
        int leftMotorPwmValue = Speed + PID_output;
        int rightMotorPwmValue = Speed - PID_output;

        // Constrain PWM values
        leftMotorPwmValue = constrain(leftMotorPwmValue, 0, 255);
        rightMotorPwmValue = constrain(rightMotorPwmValue, 0, 255);

        leftMotorForward(leftMotorPwmValue);
        rightMotorForward(rightMotorPwmValue);
    }
}

void leftMotorForward(byte PWMvalue) {
    digitalWrite(LEFTIN3, HIGH);
    digitalWrite(LEFTIN4, LOW);
    analogWrite(PWMLEFT, PWMvalue);
}

void leftMotorBackward(byte PWMvalue) {
    digitalWrite(LEFTIN3, LOW);
    digitalWrite(LEFTIN4, HIGH);
    analogWrite(PWMLEFT, PWMvalue);
}

void rightMotorForward(byte PWMvalue) {
    digitalWrite(RIGHTIN1, HIGH);
    digitalWrite(RIGHTIN2, LOW);
    analogWrite(PWMRIGHT, PWMvalue);
}

void rightMotorBackward(byte PWMvalue) {
    digitalWrite(RIGHTIN1, LOW);
    digitalWrite(RIGHTIN2, HIGH);
    analogWrite(PWMRIGHT, PWMvalue);
}
