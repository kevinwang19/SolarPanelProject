#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

#define motorPin1 8
#define motorPin2 9
#define motorPin3 10
#define motorPin4 11

Adafruit_INA219 ina219;

// STEPPER INITIAL SETUP
AccelStepper stepper(AccelStepper::FULL4WIRE, motorPin1, motorPin3, motorPin2, motorPin4);
int startingPosition = 0;
double optimal_angles[] = {-90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -87.6633278353471, -84.2661106613641, -80.89733310920033, -77.55156563727671, -74.2236607898597, -70.908772185975, -67.60237065826973, -64.30025872351993, -60.99858421897897, -57.693853622290945, -54.38294528340668, -51.063122531748185, -47.73204637931164, -44.3877873216574, -41.028835547780254, -37.65410871315806, -34.2629563169917, -30.85515966487738, -27.430926402477485, -23.990878682941208, -20.53603418601884, -17.067779440263894, -13.587835203892537, -10.098214020377517, -6.60117045881347, -3.0991449477908497, 0.4052975174854311, 3.909532204971062, 7.410941201753367, 10.90697977605977, 14.395240637342287, 17.87351431941664, 21.339844152151954, 24.7925746118845, 28.230392219829643, 31.652358557800255, 35.057935359821776, 38.44700198877301, 41.81986589787404, 45.17726689462972, 48.520376163877806, 51.85079106807788, 55.17052673324636, 58.48200535685959, 61.788044050574136, 65.09184186552835, 68.39696644999962, 71.70734056452415, 75.02722843128566, 78.36122162384815, 81.71422390792914, 85.09143412121767, 88.49832582633016, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90};

// SENSOR INITIAL SETUP
int base = 2;   // Define pins
float freq = 2; // Reporting frequency Hz
int del = 2;    // Delay after changing state of transistor

float current_mA = 0; // Sensor variables
float voltage = 0;
float power_mW = 0;

// TIME TRACKING INITIAL SETUP
unsigned long last = 0;
unsigned long led_last = 0;
float t = 0;
int led = 2000;
bool state = false;

void setup(void)
{
  // put your setup code here, to run once:
  Serial.begin(115200);

  // STEPPER MOTOR SETUP
  stepper.setMaxSpeed(1000);
  stepper.setSpeed(750);
  stepper.setAcceleration(25);
  startingPosition = stepper.currentPosition();
  stepper.moveTo(2000);

  while (!Serial)
  {
    // will pause Zero, Leonardo, etc until serial console opens
    delay(1);
  }

  uint32_t currentFrequency;

  Serial.println("Hello!");

  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  if (!ina219.begin())
  {
    Serial.println("Failed to find INA219 chip");
    while (1)
    {
      delay(10);
    }
  }
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  // ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  ina219.setCalibration_16V_400mA();

  Serial.println("Measuring voltage and current with INA219 ...");

  // SENSOR SETUP
  pinMode(base, OUTPUT);

  last = millis();
  led_last = millis();

  // TIME SETUP
  int hour; // 24 hour time
  int minute;
  Serial.println("Type the current hour (as an integer, 24 hour time):");
  while (Serial.available() == 0)
  {
  }
  hour = Serial.read();
  Serial.println("Type the current minute:");
  while (Serial.available() == 0)
  {
  }
  minute = Serial.read();
  Serial.println("Thank you! Now calibrating...");
  Serial.println(hour, minute);

  delay(5000);
}

void loop(void)
{

  // STEPPER MOTOR OPERATION
  stepper.run();
//  Serial.println(stepper.currentPosition());
  // If at the end of travel go to the other end
  if (stepper.currentPosition() == startingPosition)
    stepper.disableOutputs();

  // TIME TRACKING
  if (millis() - last > 1000.0 / freq)
  {
    last = millis();
    t = last / 1000.0; // Time in seconds

    digitalWrite(base, HIGH);
    delay(5);

    // SENSOR OPERATION
    // measure current
    current_mA = ina219.getCurrent_mA();
    Serial.println(digitalRead(base));

    digitalWrite(base, LOW);
    delay(5);

    Serial.println(digitalRead(base));

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
  delay(2000);
}
