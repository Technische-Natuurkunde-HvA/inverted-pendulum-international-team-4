---

# Week 5

## 1. Progress description
<p>In the Netherlands, we were working on the challenges. Our goal was to stabilize the pendulum completely upright, from a upside down starting possition. On Tuesday, we were really struggling. We couldn't even get it to move up from the barrier. We switched our wheel to a new one with a smaller diameter of 10 cm.
When we added this, the wheel had far too little strength, so the new wheel only worked against us.
On Friday, we continued, this time with our first wheel with a diameter of 15 cm. We had removed the barriers but still couldn't stabilize it from below to above. Then we switched to the largest wheel with a diameter of 20 cm.<p>
<p>We finally succeeded! You can find a video of this in the "Results" section.<p>
<p>As for Portugal, since we didn't have dummies and had always worked with bearings since the start of the project, it was natural to keep going in that direction and try to perfect the pendulum with the bearings. We took on the advanced data analysis, adding the measurement of the RPM to the script as well as removing some counterweight. We also experimented with a very brief correction formula for the motor's deadzone that would add or subtract values directly from the PWM. However, this didn't really work, so we redid it by actually adding the motor's deadzone along with a small precaution to try to avoid jitter instead of trying to weasel ourselves out of this in a simple way. <p>
    
## 2. Code

The code used in the Netherlands:

    #include <Wire.h>
    #include <PID_v1.h>
    #include <AS5600.h>
    
    AS5600 as5600;
    
    unsigned long currentMs;
    unsigned long lastMs;
    const unsigned int FREE_RUN_PERIOD_MS = 5;
    double sig_angle_deg;
    
    const int motorPin1 = 10;
    const int motorPin2 = 11;
    const int enablePin = 9;
    
    enum StartupState {
      IDLE, MAX_RIGHT_1, MAX_LEFT_1, MAX_RIGHT_2, MAX_LEFT_2, 
      MAX_RIGHT_3, MAX_LEFT_3, MAX_RIGHT_4, MAX_LEFT_4,
      STABILIZE, RUNNING
    };
    
    StartupState startupState = IDLE;
    unsigned long stateStartTime = 0;
    int phaseCount = 0;
    float error = 0;
    
    double setpoint = 203;
    double output = 0;
    double Kp = 60.0, Ki = 0.1, Kd = 0.1;
    PID myPID(&sig_angle_deg, &output, &setpoint, Kp, Ki, Kd, DIRECT);
    
    void setup() {
      pinMode(motorPin1, OUTPUT);
      pinMode(motorPin2, OUTPUT);
      pinMode(enablePin, OUTPUT);
      
      Wire.begin();
      as5600.begin();
      lastMs = millis();
      Serial.begin(9600);
      delay(2000);
      
      myPID.SetMode(AUTOMATIC);
      myPID.SetSampleTime(FREE_RUN_PERIOD_MS);
      myPID.SetOutputLimits(-255, 255);
      
      readAndPrintAngle();
      
      Serial.println("=== 4x MAX SWINGS -> STABILIZE ===");
      Serial.print("Start angle: "); Serial.println(sig_angle_deg);
      
      startupState = MAX_RIGHT_1;
      stateStartTime = millis();
      phaseCount = 1;
    }
    
    void loop() {
      currentMs = millis();
      
      if (currentMs - lastMs >= FREE_RUN_PERIOD_MS) {
        readAndPrintAngle();
        error = getErrorFromVertical();
        
        if (startupState != RUNNING) {
          runStartupSequence();
        } else {
          myPID.Compute();
          
          if (output > 0) {
            digitalWrite(motorPin1, HIGH);
            digitalWrite(motorPin2, LOW);
          } else {
            digitalWrite(motorPin1, LOW);
            digitalWrite(motorPin2, HIGH);
          }
          
          if (abs(sig_angle_deg - setpoint) < 2) output = 0;
          analogWrite(enablePin, abs(output));
        }
        
        static unsigned long lastPrintTime = 0;
        if (currentMs - lastPrintTime > 100) {
          printStateInfo();
          lastPrintTime = currentMs;
        }
      }
    }
    
    float getErrorFromVertical() {
      float rawError = sig_angle_deg - setpoint;
      while (rawError > 180) rawError -= 360;
      while (rawError < -180) rawError += 360;
      return rawError;
    }
    
    void runStartupSequence() {
      unsigned long currentTime = millis();
      unsigned long stateDuration = currentTime - stateStartTime;
      
      // ALLE VARIABELEN HIER - GEEN IN CASES
      StartupState nextState;
      
      switch (startupState) {
        // 4x MAXIMUM SWINGS
        case MAX_RIGHT_1:
          setMotor(255);
          if (stateDuration > 1200) {
            startupState = MAX_LEFT_1;
            stateStartTime = currentTime;
            phaseCount = 2;
          }
          break;
          
        case MAX_LEFT_1:
          setMotor(-255);
          if (stateDuration > 1000) {
            startupState = MAX_RIGHT_2;
            stateStartTime = currentTime;
            phaseCount = 3;
          }
          break;
          
        case MAX_RIGHT_2:
          setMotor(255);
          if (stateDuration > 1000) {
            startupState = MAX_LEFT_2;
            stateStartTime = currentTime;
            phaseCount = 4;
          }
          break;
          
        case MAX_LEFT_2:
          setMotor(-255);
          if (stateDuration > 1000) {
            startupState = MAX_RIGHT_3;
            stateStartTime = currentTime;
            phaseCount = 5;
          }
          break;
          
        case MAX_RIGHT_3:
          setMotor(255);
          if (stateDuration > 1000) {
            startupState = MAX_LEFT_3;
            stateStartTime = currentTime;
            phaseCount = 6;
          }
          break;
          
        case MAX_LEFT_3:
          setMotor(-255);
          if (stateDuration > 1200) {
            startupState = MAX_RIGHT_4;
            stateStartTime = currentTime;
            phaseCount = 7;
          }
          break;
          
        case MAX_RIGHT_4:
          setMotor(255);
          if (stateDuration > 1200) {
            startupState = MAX_LEFT_4;
            stateStartTime = currentTime;
            phaseCount = 8;
          }
          break;
          
        case MAX_LEFT_4:
          setMotor(-120);
          if (stateDuration > 600) {
            startupState = STABILIZE;
            stateStartTime = currentTime;
            phaseCount = 9;
            Serial.println("All max swings done -> STABILIZE");
          }
          break;
          
        // STABILIZE DIRECT NA MAX SWINGS
        case STABILIZE:
          // Breng naar verticaal
          if (error > 30) setMotor(-180);
          else if (error > 20) setMotor(-120);
          else if (error > 10) setMotor(-80);
          else if (error > 5) setMotor(-40);
          else if (error < -30) setMotor(180);
          else if (error < -20) setMotor(120);
          else if (error < -10) setMotor(80);
          else if (error < -5) setMotor(40);
          else if (abs(error) > 3) setMotor(error > 0 ? -25 : 25);
          else setMotor(0);
          
          if ((abs(error) < 5 && stateDuration > 1500) || stateDuration > 4000) {
            startupState = RUNNING;
            setMotor(0);
            Serial.println("=== PID ACTIVE ===");
          }
          break;
          
        case RUNNING:
        case IDLE:
          break;
      }
    }
    
    void setMotor(int speed) {
      speed = constrain(speed, -255, 255);
      
      if (speed > 0) {
        digitalWrite(motorPin1, HIGH);
        digitalWrite(motorPin2, LOW);
      } else if (speed < 0) {
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, HIGH);
      } else {
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, LOW);
      }
      
      analogWrite(enablePin, abs(speed));
    }
    
    void readAndPrintAngle() {
      lastMs = currentMs;
      sig_angle_deg = (float)as5600.readAngle() * 0.0879;
    }
    
    void printStateInfo() {
      Serial.print("Phase: ");
      switch (startupState) {
        case MAX_RIGHT_1: Serial.print("MAX_RIGHT_1"); break;
        case MAX_LEFT_1: Serial.print("MAX_LEFT_1"); break;
        case MAX_RIGHT_2: Serial.print("MAX_RIGHT_2"); break;
        case MAX_LEFT_2: Serial.print("MAX_LEFT_2"); break;
        case MAX_RIGHT_3: Serial.print("MAX_RIGHT_3"); break;
        case MAX_LEFT_3: Serial.print("MAX_LEFT_3"); break;
        case MAX_RIGHT_4: Serial.print("MAX_RIGHT_4"); break;
        case MAX_LEFT_4: Serial.print("MAX_LEFT_4"); break;
        case STABILIZE: Serial.print("STABILIZE"); break;
        case RUNNING: Serial.print("RUNNING"); break;
        default: Serial.print("IDLE");
      }
      
      Serial.print(" ("); Serial.print(phaseCount); Serial.print(")");
      Serial.print(" | Angle: "); Serial.print(sig_angle_deg);
      Serial.print(" | Error: "); Serial.print(error);
      Serial.println();
    }
Code used in Portugal:
```c
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

const unsigned long RPM_SAMPLE_MS = 50;  // calculate RPM every 50 ms
unsigned long lastRpmMs = 0;


double setpoint = 195.2;  // Desired angle (vertical position)
double output = 0;

// PID parameters
double Kp = 70.0;
double Ki = 55;
double Kd = 1;
PID myPID(&sig_angle_deg, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// deadzone compensation
const int DEAD_NEG = 20;
const int DEAD_POS = 50;
const int STOP_THRESHOLD = 3; // ignores pid outputs below this (hopefully avoids jitter)

void readAndPrintAngle();
void countPulse() {
  pulseCount++;
}

int compensateDeadzone(double pidOut) {
  if (fabs(pidOut) < STOP_THRESHOLD) return 0;

  if (pidOut > 255.0) pidOut = 255.0;
  if (pidOut < -255.0) pidOut = -255.0;

  if (pidOut > 0.0) {
    double scaled = DEAD_POS + (pidOut / 255.0) * (255.0 - DEAD_POS); // scale becomes DEAD_POS to 255 instead of 0 to 255
    int pwm = (int)round(scaled);
    pwm = constrain(pwm, 0, 255);
    return pwm; // positive
  } else { // pidOut < 0
    double mag = fabs(pidOut);
    double scaled = DEAD_NEG + (mag / 255.0) * (255.0 - DEAD_NEG);
    int pwm = (int)round(scaled);
    pwm = constrain(pwm, 0, 255);
    return -pwm; // negative
  }
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

    // set a deadzone
    if (abs(sig_angle_deg - setpoint) < 0.5) {
      output = 0;
    }

    int compOut = compensateDeadzone(output);

    // motor direction and PWM based on compensated output
    if (compOut > 0) {
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, HIGH);
      analogWrite(enablePin, compOut);
    } else if (compOut < 0) {
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, LOW);
      analogWrite(enablePin, abs(compOut));
    } else {
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, LOW);
      analogWrite(enablePin, 0);
    }

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


      Serial.print(sig_angle_deg);
      Serial.print(" ");
      Serial.print(output);        // raw PID output (-255 to 255)
      Serial.print(" ");
      Serial.print(compOut);       // compensated PWM actually written to motor
      Serial.print(" ");
      Serial.println(outputRPM);
    }
  }
}

void readAndPrintAngle() {
  lastMs = currentMs;
  sig_angle_deg = (float)as5600.readAngle() * 0.0879;  //0.0879=360/4096;  // degrees [0..360)
}
```

## 3. Results
This is the video of the compteted challenge:

https://github.com/user-attachments/assets/08ec3768-02b8-4cd1-acb5-1238e4f26114

https://github.com/user-attachments/assets/a8724a11-2e41-4fb6-b088-a9859eba4981


(The pendulum was stabilized and not rotating before the start of this video. We only have some eyewitnesses such as the teachers.)


 
## 4. Reflection
<p>Even if the pendulum can balance itself, with the use of bearings the friction is too small for it to stabilize consistently. We were only able to stabilize it completly 3 times and as such have no actual video evidence of this. We think this is mainly due to the setpoint angle. While the sensor itself is precise, the setpoint changes everytime the pendulum swings. Regardless of what we tried to do such as taping the magnet or fixing the sensor the best way we could. This was a major drawback since measuring the setpoint itself took longer than testing a specific configuration for the PID, impacting how well we could actually balance the pendulum. In an ideal situation we could've set our script's deadzone to 0.01 degrees around the setpoint and it would've balanced fine with swings so small they would barely be noticeable, that are in all actuality, impossible, since the setpoint always changes ever so slightly.<p>
<p>We tried values for $K_p$ between 30 and 80, settling on 70 as the sweetspot. Now the issue were the $K_i$ and $K_d$. We tried various combinations between 5 and 55 for the $K_i$ and between 0.01 and 40 for the $K_d$.
<p>We eventually settled on the final values of: $K_p$ = 70; $K_i$ = 55; $K_d$ = 1.
<p>These values took around 4 hours of fine tuning as we also had yet to figure out that the best values for the deadzone that we'd get would be 0.5 and 1 degree to both sides of the setpoint. This was aggravated by our frustration with the magnetic sensor that would randomly stop working with no apparent cause and the mysterious changes in the setpoint.

---

# Week 4

## 1. Progress description
In the Netherlands we worked on the stabilisation of the pendulum. Our goal was to stabalize the pendulum. We set the point where the pendulum arm stood straight up as the starting point. In the first version of the code the wheel stopped turning when in was stabelized (in the starting point). When you pushed it to de left, the wheel starts sturning to the right, when you pushed it to the right, the wheel starts turning to the left. This stabelized the pendulum in the middle. We got this done pretty quickly. Then we wanted to make sure that he could cope with disruptions, this means that if you give it a tap, it will not destabilize again. We changed the PID controlers, in this version the wheel doesn't stop turning when it reach the starting point, and now it could handle distruptions (see video in results).
We wanted to add a new challenge: Stabelize the pendulum from the bumper. This means that the arm is rested to the left or right bumper, starts spinning and stabilize itself while it can cope with distruptions. 
We measured the angle of the arm when he is rested against one of the bumpers (right or left). We added a part to the code so when the arm reaches this angle, the wheel stops spinning until its completely still and then starts spinning so that the arm sweeps to the middle and stabelize itself. This helps to prevent the wheel from continuously spinning in 1 direction when fallen onto the bumper.
We also desiged a new wheel, this is the design:

We thought that a wheel with a bigger diameter would be better but it turns out that is not the case. It requiers more torque and we have a moter that specializes in turning fast and not in delivering torque.
we made a new design with a smaller diameter:

![WhatsApp Image 2025-11-28 at 21 48 54](https://github.com/user-attachments/assets/de4e2c66-e9ea-4b5a-873d-361d33ec0148)

In Portugal, the progress was pretty similar. However, even when adjusting the PID, we noticed that the wheel would start deviating to either side with no clear pattern. It was theorized that this was due to the bearings, as we haven't started with dummies and have simply used regular bearings. They proportionate less friction when compared to dummies, allowing the main system to move easily, sending the PID to try to control it in a way it has yet to be programmed for, making it start accelerating the motor endlessly which doesn't proportionate inertia and doesn't correct the angle. Therefore, we decided to add a "deadzone", where the PWM output would be equal to zero when within 3 degrees from either side of the desired setpoint. This helped further stabilize the pendulum, but only temporarily, as the previous problem wasn't fixed and was instead moved to the two points at 3 degrees from the setpoint. 

## 2. Code
The code used in the Netherlands:
[Upload//for more information on the PID library: http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

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
    ing PID_control_simple (1).ino…]()

Code used in Portugal:

```c
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
```

## 3. Results
Here is a video that shows that the wheel can cope with disturptions:


https://github.com/user-attachments/assets/f32561c5-b290-4a83-9060-c6511d0f13d1


## 4. Reflection 
---
Our improvements for the next week will be improving the wheel so it has a smaller diameter and is able to spin much faster.
This will allow us to take on extra challenges such as starting upside down. However, to do this we will need to implement some kind of logic to allow the pendulum to start swinging and gaining height, which it cannot do on its own when we can only allow the motor to be fed 12V. For the starting upside down challenge, we also have to extend the motor's cables as they don't have enough length, which will probably be achieved through soldering in Portugal. Anyhow, we still need to find a way to actually have the pendulum stabilized considering the use of bearings and no disturbances first. We also have to find a way to consistently measure the setpoint and to be certain that it's completly precise as this would mess with any subsequent steps, as they all rely on the angle measurement and how far it is from the setpoint.

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
