/*Connect the wheel encoder signal to pin A7 in the Arduino.*/

//Currently this activates all of the motors at once to verify they are working properly.
//In reality, only the back two wheels will be activated when going straight. When turning,
//one of the front wheels will be activated as well to cause the robot to rotate.

volatile uint32_t CaptureCountA;
volatile boolean CaptureFlag;

const int trigPin = 33;
const int echoPin = 32;

long duration;
int distance;

void setup() {
//arduino pin 2 back left motor
PMC->PMC_PCER0 |= PMC_PCER0_PID27;//TC0 power ON - Timer Counter 0 channel 0
PIOB->PIO_PDR |= PIO_PDR_P25; // The pin is no more driven by GPIO
PIOB->PIO_ABSR |= PIO_PB25B_TIOA0;// B Assign B25 to alternative periph_B (TIOA0):

TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 //MCK/2 = 42 MHz,
| TC_CMR_WAVE //Waveform mode
| TC_CMR_WAVSEL_UP_RC //Count UP mode till RC
| TC_CMR_ACPA_CLEAR //Clear TIOA0 on RA compare match
| TC_CMR_ACPC_SET; // Set TIOA0 on RC compare match
TC0->TC_CHANNEL[0].TC_RC = 420000-1; //Set the frequency to 66.667Hz (Period 60 ms)
TC0->TC_CHANNEL[0].TC_RA = 410000-1; //Set the duty cycle (Pulse of 10 usec)
TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_SWTRG //Software trigger TC0 channel 0 counter
| TC_CCR_CLKEN;//and enabl

////////////////////////////////////////////////////////////////////
//arduino pin 11 back right motor
PMC->PMC_PCER1 |= PMC_PCER1_PID35;//TC2 power ON - Timer Counter 2 channel 2
PIOD->PIO_PDR |= PIO_PDR_P7; // The pin is no more driven by GPIO
PIOD->PIO_ABSR |= PIO_PD7B_TIOA8;// B Assign B25 to alternative periph_B (TIOA0):

TC2->TC_CHANNEL[2].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 //MCK/2 = 42 MHz,
| TC_CMR_WAVE //Waveform mode
| TC_CMR_WAVSEL_UP_RC //Count UP mode till RC
| TC_CMR_ACPA_CLEAR //Clear TIOA0 on RA compare match
| TC_CMR_ACPC_SET; // Set TIOA0 on RC compare match
TC2->TC_CHANNEL[2].TC_RC = 420000-1; //Set the frequency to 66.667Hz (Period 60 ms)
TC2->TC_CHANNEL[2].TC_RA = 410000-1; //Set the duty cycle (Pulse of 10 usec)
TC2->TC_CHANNEL[2].TC_CCR = TC_CCR_SWTRG //Software trigger TC2 channel 2 counter
| TC_CCR_CLKEN;//and enable

////////////////////////////////////////////////////////////////////
//arduino pin 3 front left motor
PMC->PMC_PCER1 |= PMC_PCER1_PID34;//TC2 power ON - Timer Counter 2 channel 2
PIOC->PIO_PDR |= PIO_PDR_P28; // The pin is no more driven by GPIO
PIOC->PIO_ABSR |= PIO_PC28B_TIOA7;// B Assign B25 to alternative periph_B (TIOA0):

TC2->TC_CHANNEL[1].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 //MCK/2 = 42 MHz,
| TC_CMR_WAVE //Waveform mode
| TC_CMR_WAVSEL_UP_RC //Count UP mode till RC
| TC_CMR_ACPA_CLEAR //Clear TIOA0 on RA compare match
| TC_CMR_ACPC_SET; // Set TIOA0 on RC compare match
TC2->TC_CHANNEL[1].TC_RC = 420000-1; //Set the frequency to 66.667Hz (Period 60 ms)
TC2->TC_CHANNEL[1].TC_RA = 410000-1; //Set the duty cycle (Pulse of 10 usec)
TC2->TC_CHANNEL[1].TC_CCR = TC_CCR_SWTRG //Software trigger TC0 channel 0 counter
| TC_CCR_CLKEN;//and enable

////////////////////////////////////////////////////////////////////
//arduino pin 5 front right motor
PMC->PMC_PCER1 |= PMC_PCER1_PID33;//TC2 power ON - Timer Counter 2 channel 2
PIOC->PIO_PDR |= PIO_PDR_P25; // The pin is no more driven by GPIO
PIOC->PIO_ABSR |= PIO_PC25B_TIOA6;// B Assign B25 to alternative periph_B (TIOA0):

TC2->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 //MCK/2 = 42 MHz,
| TC_CMR_WAVE //Waveform mode
| TC_CMR_WAVSEL_UP_RC //Count UP mode till RC
| TC_CMR_ACPA_CLEAR //Clear TIOA0 on RA compare match
| TC_CMR_ACPC_SET; // Set TIOA0 on RC compare match
TC2->TC_CHANNEL[0].TC_RC = 420000-1; //Set the frequency to 66.667Hz (Period 60 ms)
TC2->TC_CHANNEL[0].TC_RA = 410000-1; //Set the duty cycle (Pulse of 10 usec)
TC2->TC_CHANNEL[0].TC_CCR = TC_CCR_SWTRG //Software trigger TC0 channel 0 counter
| TC_CCR_CLKEN;//and enabl

//for sensor Ultrasonic
pinMode(trigPin, OUTPUT);
pinMode(echoPin, INPUT);
Serial.begin(9600);
}

void loop(){

digitalWrite(trigPin, LOW);
delayMicroseconds(2);

digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);

duration = pulseIn(echoPin, HIGH);
distance = duration * 0.034/2;

//This distance will be used to notify the robot when an obstacle is directly in front.
//Once the robot is within 15cm, actions will have to be taken to avoid the obstacle.
Serial.print("Distance: ");
Serial.println(distance);
}

