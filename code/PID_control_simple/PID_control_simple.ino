//for more information on the PID library: http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

#include <Wire.h>
#include <PID_v1.h>
#include <AS5600.h>

AS5600 as5600;  //create sensor object

unsigned long currentMs;                    //current time variable
unsigned long lastMs;                       // time of last measurement
const unsigned int FREE_RUN_PERIOD_MS = 5;  //sampling period in milliseconds
double sig_angle_deg;                       // angle measurement

// Motor control pins
const int motorPin1 = 10;  // IN1
const int motorPin2 = 11;  // IN2
const int enablePin = 9;   // ENA (PWM pin for speed control)
const int encoderPin = 2;  // (single channel)

volatile int pulseCount = 0;         // pulse counter
const int pulsesPerRevolution = 11;  // pulses per rotation from each encoder wire
const double gearRatio = 9.6;        // reduction ratio of the motor

const unsigned long RPM_SAMPLE_MS = 50;  // calculate RPM every 500 ms
unsigned long lastRpmMs = 0;


double setpoint = 194.4;  // Desired angle (vertical position)
double output = 0;

// PID parameters
double Kp = 70.0;
double Ki = 10;
double Kd = 0.1;
PID myPID(&sig_angle_deg, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void readAndPrintAngle();
void countPulse() {
  pulseCount++;
}

void setup() {
  // Set motor control pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(encoderPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderPin), countPulse, RISING);

  Wire.begin();       // Initialize I2C
  as5600.begin();     // Initialize sensor
  lastMs = millis();  // Initialize timing
  lastRpmMs = millis();
  Serial.begin(9600);  // Initialize Serial Monitor
  delay(2000);
  Serial.print("Test: ");
  Serial.println();

  // Initialize PID controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(FREE_RUN_PERIOD_MS);  // Set sample time in milliseconds
  myPID.SetOutputLimits(-255, 255);         // YOU CAN ADJUST THESE OUTPUT LIMITS IF YOU WISH
}

void loop() {
  // Read and print the angle from AS5600 at the sampling frequency
  currentMs = millis();
  if (currentMs - lastMs >= FREE_RUN_PERIOD_MS) {  // periodic sampling

    readAndPrintAngle();

    myPID.Compute();  // Calculate PID output

    // Set motor direction based on PID output
    if (output > 0) {
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, HIGH);

    } else {
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, LOW);
    }
    // Set a deadzone
    if (abs(sig_angle_deg - setpoint) < 0.5) {
      output = 0;
    }
    analogWrite(enablePin, abs(output));
    if (millis() - lastRpmMs >= RPM_SAMPLE_MS) {
      unsigned long pulses;
      noInterrupts();
      pulses = pulseCount;
      pulseCount = 0;  // reset for next interval
      interrupts();

      double interval_s = (millis() - lastRpmMs) / 1000.0;  // seconds
      lastRpmMs = millis();

      double revs = (double)pulses / (double)pulsesPerRevolution;
      double revs_per_second = 0.0;
      if (interval_s > 0.0) revs_per_second = revs / interval_s;
      double motorRPM = revs_per_second * 60.0;

      double outputRPM = motorRPM / gearRatio;


      // Print the angle to the Serial Monitor
      Serial.print(sig_angle_deg);
      Serial.print(" ");
      // Print PID output for debugging
      Serial.print(output);
      Serial.print(" ");
      Serial.println(outputRPM);
    }
  }
}

void readAndPrintAngle() {
  lastMs = currentMs;
  sig_angle_deg = (float)as5600.readAngle() * 0.0879;  //0.0879=360/4096;  // degrees [0..360)
}
