# About this document

This is a team document where you record your weekly progress. It is written in Markdown, which allows you to format content in a simple and readable way. The document is rendered directly in GitHub without the need for a compiler (unlike LaTeX). The syntax is relatively easy. An overview of commonly used Markdown syntax can be found here:  
https://www.markdownguide.org/basic-syntax/

Below you find an example of the sections that must be included in each weekly progress report.

Use the **same document for all weeks**. For each week, use the **same headers and subheaders**.

Use the GitHub repository to store important project files (code, visuals including videos and figures, data, etc.). If necessary refer to those files in this document using a hyperlink. 

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
```

- Code used in Portugal:
<p>(the code for Portugal will be added on tuesday (2/12/2025), as we have no way to retrieve it currently)</p>

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


This graph deviates from the expected graph. This is because in reality there is friction

Here is a video of the Flywheel:


https://github.com/user-attachments/assets/1bf1f06c-9694-4833-a927-8c63ff81f303

This is the graph that shows the resulst of the group from Portugal and the group from the Netherlands combined:
<img width="1137" height="705" alt="image (1)" src="https://github.com/user-attachments/assets/78d7b7f8-44a5-4847-ad13-10750d1b12b2" />


## 5. Reflection 
Our improvements for the next week will be to make a new design for the fly wheel. We want to make the wheel bigger and thinner. If we do this we would have a bigger moment of inertia (following the formula I=m*r^2). We also want to try to balance the wheel consistently. 
<p>The deviation of about 150 RPM from the Netherlands's to Portugal's motors is due to the voltage utilized in the measurements. In the Netherlands the power supply was set to 12V as with Portugal it was set to 13.6V. Since the L298N motor driver is expected to drop some of the voltage from the power supply before reaching the motor, the voltage reaching the motor was slightly smaller in the Netherlands, reaching this disparity of around 150 RPM when at maximum power (255 PWM). We expect around the same values, or very close, when the voltage is set equally </p>
<p>When the motor is spinning with the wheel attached to it, there is a drop of around 2 Hz (or 12.5 RPM) as opposed to when the motor is spinning by itself. This is related to the friction between the wheel and the air. Even if this air resistance is very small, it still exists. However, we think this isn't significant enough to hinder anything needed for this project.</p>
<p>There are lots of plausible reasons for the RPM not being exactly linear with the PWM. Air resistance could hinder the RPM when the PWM is close to 0 and is trying to break from the static friction when the motor isn't moving, as we've also noticed that the motor moves very slightly  </p>

---

# Week 3

## 1. Progress description

## 2. Code

## 3. Measurement protocol

## 4. Results

## 5. Reflection
