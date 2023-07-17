#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

#define motorPin1 8
#define motorPin2 9
#define motorPin3 10
#define motorPin4 11


Adafruit_INA219 ina219;


//STEPPER INITIAL SETUP
AccelStepper stepper(AccelStepper::FULL4WIRE,motorPin1, motorPin3, motorPin2, motorPin4);
int startingPosition = 0;


//SENSOR INITIAL SETUP 
int base = 18; // Define pins
float freq = 1; // Reporting frequency Hz
int del = 2; // Delay after changing state of transistor

float current_mA = 0; // Sensor variables 
float voltage = 0;
float power_mW = 0;
float voltage_oc = 0;

// TIME TRACKING INITIAL SETUP
unsigned long last = 0;
unsigned long led_last = 0;
float t = 0;
int led = 2000;
bool state = false;

void setup(void) {
  // put your setup code here, to run once:
   Serial.begin(115200);

  // STEPPER MOTOR SETUP
  stepper.setMaxSpeed(1000);
  stepper.setSpeed(750);  
  stepper.setAcceleration(25);
  startingPosition = stepper.currentPosition();
  stepper.moveTo(2000);
  
  while (!Serial) {
      // will pause Zero, Leonardo, etc until serial console opens
      delay(1);
  }
 
  uint32_t currentFrequency;
    
  Serial.println("Hello!");
  
  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  //ina219.setCalibration_16V_400mA();
 
  Serial.println("Measuring voltage and current with INA219 ...");

    // SENSOR SETUP
//  pinMode(base, OUTPUT);
//  pinMode(2, OUTPUT);

//  last = millis();
//  led_last = millis();
  

}

void loop(void) {

    // STEPPER MOTOR OPERATION
    stepper.run();
    Serial.println(stepper.currentPosition());
    // If at the end of travel go to the other end
    if (stepper.currentPosition() == startingPosition)
      stepper.disableOutputs();

//    // TIME TRACKING
//    if (millis() - last > 1000.0/freq){
//    last = millis();
//    t = last/1000.0; // Time in seconds
//
//    // SENSOR OPERATION
//    // measure current
//    current_mA = ina219.getCurrent_mA();
//    // measure voltage
//    voltage = ina219.getBusVoltage_V();
//    voltage_oc = voltage*0.7;
//    
//
//    // *** END OF POWER COLLECTION 
//    // Max power (mW) is at roughly 0.7x the voltage
//    power_mW = (0.7*voltage)*current_mA;
//
//    // Format: Time, Voltage, Current, Estimated Power
//    Serial.print(t); Serial.print(" Seconds, "); 
//    Serial.print(voltage_oc); Serial.print(" Volt (Open circuit), "); 
//    Serial.print(voltage); Serial.print(" Volt (Maximum power), ");    
//    Serial.print(current_mA); Serial.print(" Amps, ");
//    Serial.println(power_mW); Serial.print(" Watts, ");

   float shuntvoltage = 0;
   float busvoltage = 0;
   float current_mA = 0;
   float loadvoltage = 0;
   float power_mW = 0;
 
   shuntvoltage = ina219.getShuntVoltage_mV();
   busvoltage = ina219.getBusVoltage_V();
   current_mA = ina219.getCurrent_mA();
   power_mW = ina219.getPower_mW();
   loadvoltage = busvoltage + (shuntvoltage / 1000);
  
   Serial.println("Bus Voltage:   "); 
   Serial.println(busvoltage); 
   Serial.println(" V");
   Serial.println("Shunt Voltage: "); 
   Serial.println(shuntvoltage); 
   Serial.println(" mV");
   Serial.println("Load Voltage:  "); 
   Serial.print(loadvoltage); 
   Serial.println(" V");
   Serial.print("Current:       "); 
   Serial.print(current_mA); Serial.println(" mA");
   Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
   Serial.println("");
 
   delay(2000);
//  }
}