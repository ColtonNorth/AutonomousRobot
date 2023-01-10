#include <Wire.h>
#include <VL53L1X.h>

// The number of ToF sensors
const uint8_t sensorCount = 2;

// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPins[sensorCount] = { 43, 42 };

VL53L1X sensors[sensorCount];

//Ultrasonic trig and echo pins
const int trigPin = 33;
const int echoPin = 32;

//Motor direction and enable pins (BL = Back Left).
int BLdir = 2;
int BLpwm = 3;
int BRdir = 4;
int BRpwm = 5;
int FLdir = 6;
int FLpwm = 7;
int FRdir = 8;
int FRpwm = 9;

//Line Following Sensor pins (LLS = LeftLeftSensor)
int LLS = 25;
int LMS = 24;
int LRS = 27;
int RLS = 26;
int RMS = 29;
int RRS = 28;

long duration;
int distance;

void setup()
{
  //Motor Setup 
  pinMode(BLdir, OUTPUT);
  pinMode(BLpwm, OUTPUT);
  pinMode(BRdir, OUTPUT);
  pinMode(BRpwm, OUTPUT);
  pinMode(FLdir, OUTPUT);
  pinMode(FLpwm, OUTPUT);
  pinMode(FRdir, OUTPUT);
  pinMode(FRpwm, OUTPUT);

  //Set all motor pins low on startup
  digitalWrite(BLdir, LOW);
  analogWrite(BLpwm, 0);
  digitalWrite(BRdir, LOW);
  analogWrite(BRpwm, 0);
  digitalWrite(FLdir, LOW);
  analogWrite(FLpwm, 0);
  digitalWrite(FRdir, LOW);
  analogWrite(FRpwm, 0);

  //Line Following Sensor pin setup
  pinMode(LLS, INPUT);
  pinMode(LMS, INPUT);
  pinMode(LRS, INPUT);
  pinMode(RLS, INPUT);
  pinMode(RMS, INPUT);
  pinMode(RRS, INPUT);

  //for sensor Ultrasonic
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //Time of Flight sensor startup
  while (!Serial) {}
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }

  // Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    digitalWrite(xshutPins[i], HIGH);
    delay(10);

    sensors[i].setTimeout(1);
    sensors[i].setDistanceMode(VL53L1X::Short);
    if (!sensors[i].init())
    {
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
      while (1);
    }

    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensors[i].setAddress(0x2A + i);
    sensors[i].startContinuous(50);
  }

  delay(2000);
  uint32_t wdp_ms = 2048; //8 seconds = 256 x 8 = 2048
  WDT_Enable(WDT, 0x2000 | wdp_ms | ( wdp_ms << 16 ));
}

void loop(){
  //Stops the motors and then checks the sensors in order given below.
  //The motors have to stop for the ToF sensors to be able to give readings
  //as they were drawing power away too much power.
  WDT_Restart(WDT); // restart watchdog timer at the beginning of every loop.
  stopMotors();
  delay(200);
  readLineFollowingSensors();
  readTimeOfFlightSensors();
  readUltrasonicSensor();
}

void readLineFollowingSensors()
{
  //If both left and right line following sensors detect the black line with any of their 3 sensors, reverse and turn right
  if(((digitalRead(LLS) == HIGH) || (digitalRead(LMS) == HIGH) || (digitalRead(LRS) == HIGH)) && ((digitalRead(RLS) == HIGH) || (digitalRead(RMS) == HIGH) || (digitalRead(RRS) == HIGH)))
  {
      Serial.println("Boundary detected ahead of robot");
      moveBackwards();
      delay(1000);
      rotateRight();
      delay(2000);
      stopMotors();
      delay(1000);
  }
  //If any of the left line following sensors detect the black line, turn right.
  else if((digitalRead(LLS) == HIGH) || (digitalRead(LMS) == HIGH) || (digitalRead(LLS) == HIGH))
  {
    Serial.println("Boundary detected left of robot");
    rotateRight();
    delay(1500);
    stopMotors();
    delay(1000);
  }
  //If any of the right line following sensors detect the black line, turn left.
  else if((digitalRead(RLS) == HIGH) || (digitalRead(RMS) == HIGH) || (digitalRead(RRS) == HIGH))
  {
    Serial.println("Boundary detected right of robot");
    rotateLeft();
    delay(1500);
    stopMotors();
    delay(1000);
  }
  else // There is no boundary detected so continue to next sensors.
  {
    Serial.println("No boundary detected");
  }
}

void readTimeOfFlightSensors()
{
  //For each sensor (we have 2) read their data and print it to the Serial Monitor. 
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    sensors[i].read();
    Serial.print("range: ");
    Serial.print(sensors[i].ranging_data.range_mm);
    Serial.print("\tstatus: ");
    Serial.print(VL53L1X::rangeStatusToString(sensors[i].ranging_data.range_status));
    Serial.print("\tpeak signal: ");
    Serial.print(sensors[i].ranging_data.peak_signal_count_rate_MCPS);
    Serial.print("\tambient: ");
    Serial.print(sensors[i].ranging_data.ambient_count_rate_MCPS);
    //Notify the serial monitor if a timeout occured with the sensor.
    if (sensors[i].timeoutOccurred()) { Serial.print(" TIMEOUT");}
    Serial.print('\t');
  }
  Serial.println();
  //If the left sensor is below the threshold, turn the robot right.
  if(sensors[0].ranging_data.range_mm < 190)
  {
    Serial.println("ToF Left sensor below threshold, turning right");
    rotateRight();
    delay(1500);
    stopMotors();
    delay(500);

  }
  //If the right sensor is below the threshold, turn the robot left.
  else if (sensors[1].ranging_data.range_mm < 190)
  {
    Serial.println("ToF right sensor below threshold, turning left");
    rotateLeft();
    delay(1500);
    stopMotors();
    delay(500);
  }
}

void readUltrasonicSensor()
{
  //Send the trigger signal
  digitalWrite(trigPin, LOW);
  delay(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(8);
  digitalWrite(trigPin, LOW);

  //measure the echo pulse and store the distance
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034/2;

  Serial.print("Ultrasonic Distance: ");
  Serial.println(distance);
  
  //If the value is below 30, we reverse and rotate right. Otherwise we move forwards
  //as nothing is in the way.
  if (distance < 20)
  {
    Serial.println("Ultrasonic sensor detected object ahead.");
    moveBackwards();
    delay(1000);
    rotateRight();
    delay(2000);
  }
  else
  {
    moveForwards();
    delay(300);
  }
}

void moveForwards()
{
  //Moves the robot forwards. The left wheels have lower duty cycles to account for drift.
  analogWrite(BLpwm, 190);
  digitalWrite(BLdir, LOW);
  analogWrite(BRpwm, 220);
  digitalWrite(BRdir, HIGH);
  analogWrite(FLpwm, 190);
  digitalWrite(FLdir, LOW);
  analogWrite(FRpwm, 220);
  digitalWrite(FRdir, HIGH);
}

void moveBackwards()
{
  //Moves the motors at the same speed while reversing the direction.
  analogWrite(BLpwm, 220);
  digitalWrite(BLdir, HIGH);
  analogWrite(BRpwm, 220);
  digitalWrite(BRdir, LOW);
  analogWrite(FLpwm, 220);
  digitalWrite(FLdir, HIGH);
  analogWrite(FRpwm, 220);
  digitalWrite(FRdir, LOW);
}

void rotateRight()
{
  //Rotates the robot left by having the left wheels rotate forwards,
  //while the right two wheels rotate backwards.
  analogWrite(BLpwm, 220);
  digitalWrite(BLdir, LOW);
  analogWrite(BRpwm, 220);
  digitalWrite(BRdir, LOW);
  analogWrite(FLpwm, 220);
  digitalWrite(FLdir, LOW);
  analogWrite(FRpwm, 220);
  digitalWrite(FRdir, LOW);
}

void rotateLeft()
{
  //Rotates the robot left by having the left wheels rotate backwards,
  //while the right two wheels rotate forwards.
  analogWrite(BLpwm, 220);
  digitalWrite(BLdir, HIGH);
  analogWrite(BRpwm, 220);
  digitalWrite(BRdir, HIGH);
  analogWrite(FLpwm, 220);
  digitalWrite(FLdir, HIGH);
  analogWrite(FRpwm, 220);
  digitalWrite(FRdir, HIGH);
}

void stopMotors()
{
  //Stops the motors from moving. Sets their directions to forward.
  analogWrite(BLpwm, 0);
  digitalWrite(BLdir, LOW);
  analogWrite(BRpwm, 0);
  digitalWrite(BRdir, HIGH);
  analogWrite(FLpwm, 0);
  digitalWrite(FLdir, LOW);
  analogWrite(FRpwm, 0);
  digitalWrite(FRdir, HIGH);
}

