/*Connect the wheel encoder signal to pin A7 in the Arduino.*/
volatile uint32_t CaptureCountA;
volatile boolean CaptureFlag;

const int trigPin = 33;
const int echoPin = 32;

int BLdir = 2;
int BLpwm = 3;
int BRdir = 4;
int BRpwm = 5;
int FLdir = 6;
int FLpwm = 7;
int FRdir = 8;
int FRpwm = 9;

long duration;
int distance;
void setup()
{
  
  pinMode(BLdir, OUTPUT);
  pinMode(BLpwm, OUTPUT);
  pinMode(BRdir, OUTPUT);
  pinMode(BRpwm, OUTPUT);
  pinMode(FLdir, OUTPUT);
  pinMode(FLpwm, OUTPUT);
  pinMode(FRdir, OUTPUT);
  pinMode(FRpwm, OUTPUT);

  //Set all pins low on startup
  digitalWrite(BLdir, LOW);
  digitalWrite(BLpwm, LOW);
  digitalWrite(BRdir, LOW);
  digitalWrite(BRpwm, LOW);
  digitalWrite(FLdir, LOW);
  digitalWrite(FLpwm, LOW);
  digitalWrite(FRdir, LOW);
  digitalWrite(FRpwm, LOW);

  //for sensor Ultrasonic
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
}

void loop(){
delay(500);

digitalWrite(trigPin, LOW);
delayMicroseconds(200);

digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);

duration = pulseIn(echoPin, HIGH);
distance = duration * 0.034/2;

Serial.print("Distance: ");
Serial.println(distance);

if (distance > 40)
{
  moveForwards();
}
else 
{
  moveBackwards();
  delay(1000);
  rotateRight();
  delay(2000);
}

}

void moveForwards()
{
  //Moving motors (forward)
  analogWrite(BLpwm, 220);
  digitalWrite(BLdir, LOW);
  analogWrite(BRpwm, 220);
  digitalWrite(BRdir, HIGH);
  analogWrite(FLpwm, 220);
  digitalWrite(FLdir, LOW);
  analogWrite(FRpwm, 220);
  digitalWrite(FRdir, HIGH);
}

void moveBackwards()
{
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
  analogWrite(BLpwm, 220);
  digitalWrite(BLdir, HIGH);
  analogWrite(BRpwm, 220);
  digitalWrite(BRdir, HIGH);
  analogWrite(FLpwm, 220);
  digitalWrite(FLdir, HIGH);
  analogWrite(FRpwm, 220);
  digitalWrite(FRdir, HIGH);
}
