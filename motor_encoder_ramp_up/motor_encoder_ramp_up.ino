// Motor control pins
const int motorPin1 = 10; // IN1 
const int motorPin2 = 11; // IN2 
const int enablePin = 9;  // ENA (PWM pin for speed control)

// Encoder pin
const int encoderPin = 2; // use just one encoder pin for simplicity

volatile int pulseCount = 0;  // pulse counter
const int pulsesPerRevolution = 11; // pulses per rotation from each encoder wire

unsigned long lastTime = 0;   // store last time measurement
double frequency = 0;         // measured frequency
double rpm = 0;               // measured RPM

int output = -255;            // starting at the minimum PWM value (-255)

unsigned long rampTime = 200;  // Time between changes in PWM value (ms)
unsigned long lastRampTime = 0;

void countPulse();

void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  pinMode(encoderPin, INPUT_PULLUP); // 1 encoder input

  attachInterrupt(digitalPinToInterrupt(encoderPin), countPulse, RISING);

  Serial.begin(9600);
  lastTime = millis();
}

void loop() {
  if (millis() - lastTime >= 199) { // print every 199 ms since it increments every 200 ms
    noInterrupts();
    int count = pulseCount;
    pulseCount = 0;
    interrupts();

    frequency = count / (pulsesPerRevolution * 0.5); // frequency (Hz)
    rpm = frequency * 60 / 9.6;  // convert to RPM (frequency times 60 seconds divided by 9.6(gear ratio))

    Serial.print("Output: ");
    Serial.print(output);
    Serial.print("  Frequency: ");
    Serial.print(frequency);
    Serial.print(" Hz  RPM: ");
    Serial.println(rpm);

    lastTime = millis();
  }

  // Motor direction + speed control based on output (positive PWM -> clockwise) (negative PWM -> counter-clockwise)
  if (output > 0) {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  } else {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  }
  analogWrite(enablePin, abs(output));

  if (millis() - lastRampTime >= rampTime) {
    if (output < 255) {
      output++;  // Increment output from -255 to 255
    } else if (output > 255) {
      output--;  // Decrement if somehow went over 255
    }

    lastRampTime = millis();
  }
}

void countPulse() {
  pulseCount++;  // increment pulseCount
}