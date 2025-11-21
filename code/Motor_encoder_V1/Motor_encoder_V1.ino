// Motor control pins 
const int motorPin1 = 10; //IN1
const int motorPin2 = 11; //IN2
const int enablePin = 9; //ENA (PWM pin for speed control)

// Encoder pin
const int encoderPin = 2; // use just one encoder pin for simplicity

volatile long pulseCount = 0; // pulse counter (long = safer)
const int pulsePerRevolution = 11; // pulses per motor shaft revolution from each encode3r wire

// IMPORTANT!! check your motors gear ratio from the datasheet
const float gearRatio = ___; // TO DO: change value

// Measurement settings
const unsigned long settleTimeMs = 3000; // time in ms to let motor reach const speed 
const unsigned long measureTimeMs = 2000; // actual measurement time

// PWM values to test
const int pwmValues[] = {};
const int numPwmValues = sizeof(pwmValues) / sizeof(pwmValues[0]); // = number of elements to run

void countPulse();

//setup
void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  pinMode(encoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin), countPulse, RISING);

  Serial.begin(9600);
  delay(2000);

  Serial.println("PWM,direction,wheel_RPM");
}

void setMotor(int pwm, int dir) {
  pwm = constrain(pwm, 0, 255);

  if (dir > 0) {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  } else {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  }

  analogWrite(enablePin, pwm);
}

// Main measurement protocol
void loop() {
  // We do one full measurement run, then stop.

  // first direction: forward (+1), then backward (-1)
  for (int dir = +1; dir >= -1; dir -= 2) {
    for (int i = 0; i < numPwmValues; i++) {
      int pwm = pwmValues[i];

      // set motor output
      setMotor(pwm, dir);

      // Wait for motor to reach const speed
      delay(settleTimeMs); 

      // reset pulse counter just before measurement
      noInterrupts();
      pulseCount = 0;
      interrupts();

      // measure for a fixed time window 
      unsigned long startTime = millis();
      while(millis() - startTime < measureTimeMs) {
        // do nothing
      }

      //read number of pulses 
      noInterrupts(); 
      long count = pulseCount; 
      interrupts();

      // convert pulses to RPM
      double t_sec = measureTimeMs / 1000.0; 
      double motorRevPerSec = (double)count / (pulsePerRevolution * t_sec); 
      double wheelRPM = motorRevPerSec * 60.0 / gearRatio;

      // print results in CSV
      Serial.print(pwm); 
      Serial.print(",");
      Serial.print(dir);
      Serial.print(",");
      Serial.println(wheelRPM, 1); 
      delay(500);

    }
  }

  // after finishing all measurements: stop motor
  setMotor(0, +1); // motor off
  while (true) {
    //end of measurement
  }
}
  void countPulse() {
    pulseCount++; // increment each pulse
  }
