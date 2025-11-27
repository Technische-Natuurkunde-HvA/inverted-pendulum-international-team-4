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
  if (millis() - lastTime >= 200) {
    noInterrupts();
    int count = pulseCount;
    pulseCount = 0;
    interrupts();

    frequency = count / (pulsesPerRevolution * 0.5);  // frequency in Hz
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
