// 0. INSTALL PACKAGES
#include <AccelStepper.h> // Package for programming stepper motor
#include <Wire.h> // Package for communicating through SDA and SCL
#include <Adafruit_INA219.h> // Package for programming INA219

// 1. DEFINITIONS
#define motorPin1 14 // Motor pin for stepper motor
#define motorPin2 27 // Motor pin for stepper motor
#define motorPin3 26 // Motor pin for stepper motor
#define motorPin4 25 // Motor pin for stepper motor

// Motor Setup
AccelStepper stepper(AccelStepper::FULL4WIRE, motorPin1, motorPin3, motorPin2, motorPin4); // Instantiating stepper motor
int startingPosition = 0; 
double optimal_angles[] = {-90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -87.6633278353471, -84.2661106613641, -80.89733310920033, -77.55156563727671, -74.2236607898597, -70.908772185975, -67.60237065826973, -64.30025872351993, -60.99858421897897, -57.693853622290945, -54.38294528340668, -51.063122531748185, -47.73204637931164, -44.3877873216574, -41.028835547780254, -37.65410871315806, -34.2629563169917, -30.85515966487738, -27.430926402477485, -23.990878682941208, -20.53603418601884, -17.067779440263894, -13.587835203892537, -10.098214020377517, -6.60117045881347, -3.0991449477908497, 0.4052975174854311, 3.909532204971062, 7.410941201753367, 10.90697977605977, 14.395240637342287, 17.87351431941664, 21.339844152151954, 24.7925746118845, 28.230392219829643, 31.652358557800255, 35.057935359821776, 38.44700198877301, 41.81986589787404, 45.17726689462972, 48.520376163877806, 51.85079106807788, 55.17052673324636, 58.48200535685959, 61.788044050574136, 65.09184186552835, 68.39696644999962, 71.70734056452415, 75.02722843128566, 78.36122162384815, 81.71422390792914, 85.09143412121767, 88.49832582633016, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90}; // Optimal angles for Waterloo, Canada on July 20th, 2023. Calculated using NREL data and sun.py
const int stepsPerRevolution = 2038; // Steps in 24BYJ48S stepper motor
const int stepsPerDegree = round(2038 / 360);


// Current Sensor Setup
Adafruit_INA219 ina219; // Instantiating INA219
int base = 33;   // Define base pins
float freq = 2; // Reporting frequency Hz
int del = 2;    // Delay after changing state of transistor
float current_mA = 0; // 
float voltage = 0;
float power_mW = 0;

// Time Tracking Setup
unsigned long last = 0;
unsigned long led_last = 0;
float t = 0;
int led = 2000;
bool state = false;

int initial_minute = 30;
int initial_hour = 10;
int elapsed_hour = 0;
int elapsed_minute = 0;
int current_hour = 0;
int current_minute = 0;

// Initial position setup
bool checked = false;

void setup()
{
  // 2. Calibrating devices
  delay(5000); // 5 second delay after setup to allow user to move panel to correct location
  Serial.begin(115200); // Defining serial rate

  // Stepper motor calibration
  startingPosition = stepper.currentPosition(); // Storing current position of stepper motor
  int initial_angle_index = (initial_hour * 4) + (initial_minute / 15); // Obtaining index for optimal angle based on time
  int initial_angle = optimal_angles[initial_angle_index]; // Ontaining angle from calculated index
  int initial_steps = round(initial_angle * stepsPerDegree);
  stepper.moveTo(initial_steps); // Move stepper to inital angle

  while (!Serial)
  {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }

  uint32_t currentFrequency; // Define variable for current frequency
  Serial.println("Hello!"); // Indicate power

  if (!ina219.begin())   // Initialize the INA219 to default, largest range (32V, 2A)
  {
    Serial.println("Failed to find INA219 chip"); 
    while (1)
    {
      delay(10);
    }
  }
  ina219.setCalibration_16V_400mA(); // Set INA219 to slightly lower 32V, 1A range (higher precision on amps):
  Serial.println("Measuring voltage and current with INA219 ...");
 
  // Sensor setup
  pinMode(base, OUTPUT);

  last = millis(); // Set time for last
  led_last = millis(); // Set time for LED

  stepper.setSpeed(120); // Set speed for stepper motor
  stepper.setMaxSpeed(300); // Set maximum speed for stepper motor
  stepper.setAcceleration(100);  // Set accelerator for stepper motor


void loop()
{ 

  // Get current time in elapsed hour and minutes
  elapsed_hour = round(t / 3600); 
  elapsed_minute = ((t / 3600 - round(t/3600)) * 60);
  current_hour = initial_hour + elapsed_hour;
  current_minute = initial_minute + elapsed_minute;

  // Get optimal angle of stepper motor from array, based on time
  int angle_index = (current_hour * 4) + (current_minute / 15);
  int optimal_angle = optimal_angles[angle_index];

  // Calculate the difference between the previous optimal angle and the new one
  int previous_optimal_angle = optimal_angles[angle_index - 1];
  int angleDifference = abs(optimal_angle - previous_optimal_angle);

  // Move the stepper motor by the angle difference
  int stepsToMove = angleDifference * stepsPerDegree;

  // Set checkers to only run movement every 15 mins
  if (current_minute % 15 != 0 && (current_minute != initial_minute) ) {
    checked = false;
  }
  if (!checked && (current_minute % 15 == 0) && (current_minute != initial_minute) && (stepper.distanceToGo() == 0)) {
    stepper.move(stepsToMove);
    checked = true;
  
  }

  // Run stepper motor
  stepper.run();

  // Time tracker
  if (millis() - last > 1000.0 / freq)
  {
    last = millis();
    t = last / 1000.0; // Time in seconds

    digitalWrite(base, HIGH);
    delay(10);

    // measure current
    current_mA = ina219.getCurrent_mA();
    digitalWrite(base, LOW);
    delay(10);

    // measure voltage
    voltage = ina219.getBusVoltage_V();

    // *** END OF POWER COLLECTION
    // Max power (mW) is at roughly 0.7x the voltage
    power_mW = (0.7 * voltage) * current_mA;

    // Format: Time, Voltage, Current, Estimated Power
    Serial.print(t);
    Serial.print(", ");
    Serial.print(voltage);
    Serial.print(", ");
    Serial.print(current_mA);
    Serial.print(", ");
    Serial.println(power_mW);
  }
  delay(100);
}
