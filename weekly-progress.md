# About this document

This is a team document where you record your weekly progress. It is written in Markdown, which allows you to format content in a simple and readable way. The document is rendered directly in GitHub without the need for a compiler (unlike LaTeX). The syntax is relatively easy. An overview of commonly used Markdown syntax can be found here:  
https://www.markdownguide.org/basic-syntax/

Below you find an example of the sections that must be included in each weekly progress report.

Use the **same document for all weeks**. For each week, use the **same headers and subheaders**.

Use the GitHub repository to store important project files (code, visuals including videos and figures, data, etc.). If necessary refer to those files in this document using a hyperlink. 

---

# Week 4

## 1. Progress description
In the Netherlands we worked on the stabilisation of the pendulum. Our goal was to stabalize the pendulum from a upright position. We set the point where the pendulum arm stood straight up as the starting point. In the first version of the code the wheel stopped turning when in was stabelized (in the starting point). When you pushed it to de left, the wheel starts sturning to the right, when you pushed it to the right, the wheel starts turning to the left. This stabelized him in the middle. We got this done pretty quickly. Then we wanted to make sure that he could cope with disruptions, this means that if you give it a tap, it will not destabilize again. We changed the PID controlers, in this version the wheel doesn't stop turning when it reach the starting point, and now it could cope with distruptions. 
We wanted to add a new challenge: Stabelize the pendulum from un ustabeled position. This means that the arm is rested to the left or right arm, stards spinnign and stabelize itself while it can cope with distruptions. 
We measured the angle of the arm when he is rested against one of the arms (right or left). We add a part to the code that when the arm reach this angle the wheel stops spinning until its completely still and then starts spinning so that the arm sweeps to the middle and stabelize itself.



## 2. Code
The code used in the Netherlands:
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


double setpoint = 195.05;  // Desired angle (vertical position)
double output = 0;

// PID parameters
double Kp = 60.0;
double Ki = 0.1;
double Kd = 0.1;
PID myPID(&sig_angle_deg, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void readAndPrintAngle();

void setup() {
  // Set motor control pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  Wire.begin();        // Initialize I2C
  as5600.begin();      // Initialize sensor
  lastMs = millis();   // Initialize timing
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
    if (abs(sig_angle_deg - setpoint) < 3) {
      output = 0;
    }
    analogWrite(enablePin, abs(output));

    // Print the angle to the Serial Monitor
    Serial.print(sig_angle_deg);
    Serial.print(" ");
    // Print PID output for debugging
    Serial.println(output);
  }
}

void readAndPrintAngle() {
  lastMs = currentMs;
  sig_angle_deg = (float)as5600.readAngle() * 0.0879;  //0.0879=360/4096;  // degrees [0..360)
}
## 3. Results
Here is a video that shows that the wheel can cope with disturptions:


https://github.com/user-attachments/assets/f32561c5-b290-4a83-9060-c6511d0f13d1


## 4. Reflection 

---

# Week 3

## 1. Progress description
<p>In the Netherlands we worked on the pendulum this week. We started with a spining wheel with no motion in the arm. After we had some difficulties with the code we managed to get the pendulum to move. We started with a swinging arm (see the video in results) but Somtimes we were able to balance it but it was not consistent. 
<p>In Portugal, we started off with the base code that was made available to us, directly applying a way to measure the RPM from the frequency calculation already integrated. This was done by simply multiplying the frequency by the 60 seconds in a minute and diving it by 9.6, which is the reduction rate of our motor. After this, the approach used was slightly different. Instead of measuring the different RPM values for each PWM stated, we decided to use our script to incremenet the PWM value at a rate of 1 PWM every 0.2 seconds.

## 2. Code
- Code used in the Netherlands:
```c
// Motor control pins 
const int motorPin1 = 10; //IN1
const int motorPin2 = 11; //IN2
const int enablePin = 9; //ENA (PWM pin for speed control)

// Encoder pin
const int encoderPin = 2; // use just one encoder pin for simplicity

volatile long pulseCount = 0; // pulse counter (long = safer)
const int pulsePerRevolution = 11; // pulses per motor shaft revolution from each encode3r wire

// IMPORTANT!! check your motors gear ratio from the datasheet
const float gearRatio = 9.6

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
```

- Code used in Portugal:
```c
// Motor control pins
const int motorPin1 = 10;  // IN1
const int motorPin2 = 11;  // IN2
const int enablePin = 9;   // ENA (PWM pin for speed control)

// Encoder pin
const int encoderPin = 2;  // use just one encoder pin for simplicity

volatile int pulseCount = 0;         // pulse counter
const int pulsesPerRevolution = 11;  // pulses per rotation from each encoder wire

unsigned long lastTime = 0;  // store last time measurement
double frequency = 0;        // measured frequency
double RPM = 0;

double output = -255;  // start at the minimum motor output value
const double gearRatio = 9.6;

const double incrementRate = 0.05;        // how much the output changes per loop
const unsigned long updateInterval = 10;  // Interval to update output (in milliseconds)

void countPulse();

void setup() {
  delay(5000);  // delay 5 seconds to give us time to connect to python
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  pinMode(encoderPin, INPUT_PULLUP);  // 1 encoder input

  attachInterrupt(digitalPinToInterrupt(encoderPin), countPulse, RISING);

  Serial.begin(9600);
  lastTime = millis();
}

void loop() {
  if (output < 255) {
    output += incrementRate;
  } else if (output > 255) {
    output = 255;
  }

  // Motor direction + speed
  if (output > 0) {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  } else {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  }
  analogWrite(enablePin, abs(output));

  // calculate frequency
  if (millis() - lastTime >= 500) { // don't change the 500 ms value, completly breaks frequency (and therefore RPM) calculation, changing the value that pulsesPerRevolution multiplies with doesn't really fix it
    noInterrupts();
    int count = pulseCount;
    pulseCount = 0;
    interrupts();

    frequency = count / (pulsesPerRevolution * 0.5);
    RPM = frequency * 60 / gearRatio;

    Serial.print("Output: ");
    Serial.print(output);
    Serial.print("  Frequency: ");
    Serial.print(frequency);
    Serial.print(" Hz");
    Serial.print("  RPM: ");
    Serial.println(RPM);

    lastTime = millis();
  }

  delay(updateInterval);
}

void countPulse() {
  pulseCount++;  // increment each pulse
}
```

## 3. Measurement protocol
There will be measured at 8 different PWM values in both directions.
The values will be: 50, 80, 110, 140, 170, 200, 230, 255

One will peform two measurements per PWM value, one while the wheel is spinning to the right and one while the wheel is spinning to the left. In total there will be 16 measurments. The total measurement lasts 90 seconds, so each individual measurment lasts 5.5 seconds. 

Because of the fact that the motor can rotate in two directions one will first measure all the different PWM values while the wheel turns to the right. Then, automaticly, the wheel stops spinning and switch in turning to the left. All the PWM values will be measured again and this is how to account for this problem.

The following graph is to be expected:
 <img width="752" height="452" alt="image" src="https://github.com/user-attachments/assets/f1e5559b-7ba2-4518-9c76-23f09a0c95c6" />
This is expected because at the maximum PWM value, one would anticipate to reach the maximum RPM count. 

To upload and modify the code arduino IDE is used. A serial moniter will print the data and a python file will read the serial moniter and generate a csv file from the data and, if possible, creates plots.


## 4. Results
<p>The following graph shows the results:</p>
 <img width="640" height="480" alt="image" src="https://github.com/user-attachments/assets/b4ce2a50-342c-4867-a038-0a8fc6dbc08e" />


The experimental graph deviates from the expected graph. This is because in reality there is friction.

Here is a video of the Flywheel:


https://github.com/user-attachments/assets/1bf1f06c-9694-4833-a927-8c63ff81f303

This is the graph that shows the results of the group from Portugal and the group from the Netherlands combined:
<img width="1137" height="705" alt="image (1)" src="https://github.com/user-attachments/assets/78d7b7f8-44a5-4847-ad13-10750d1b12b2" />


## 5. Reflection 
Our improvements for the next week will be to make a new design for the fly wheel. We want to make the wheel bigger and thinner. If we do this we would have a bigger moment of inertia (following the formula I=m*r^2). We also want to try to balance the wheel consistently. 
<p>The deviation of about 150 RPM from the Netherlands's to Portugal's motors is due to the voltage utilized in the measurements. In the Netherlands the power supply was set to 12V as with Portugal it was set to 13.6V. Since the L298N motor driver is expected to drop some of the voltage from the power supply before reaching the motor, the voltage reaching the motor was slightly smaller in the Netherlands, reaching this disparity of around 150 RPM when at maximum power (255 PWM). We expect around the same values, or very close, when the voltage is set equally </p>
<p>When the motor is spinning with the wheel attached to it, there is a drop of around 2 Hz (or 12.5 RPM) as opposed to when the motor is spinning by itself. This is related to the friction between the wheel and the air. Even if this air resistance is very small, it still exists. However, we think this isn't significant enough to hinder anything needed for this project.</p>
<p>There are lots of plausible reasons for the RPM not being exactly linear with the PWM. Air resistance could hinder the RPM when the PWM is close to 0 and is trying to break from the static friction when the motor isn't moving. This seems slightly plausible, as we've noticed that the motor moves very slightly at around 20 PWM (and instantly stops) but only really starts rotating at around 50 PWM. This is described in the graphs uploaded from Portugal's measurements in the "data" folder. Nevertheless, as air resistance shouldn't be significant both of these are probably caused by the internal resistance of the motor.  </p>
