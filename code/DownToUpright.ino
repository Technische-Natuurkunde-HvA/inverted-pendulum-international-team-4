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