#include <SharpIR.h>   // Library for Sharp IR sensor (not used directly in this sketch but included)
#include <Servo.h>     // Library to control the servo motor

// Variables for ultrasonic sensor measurements
long duration;           // Stores time for ultrasonic echo pulse
float duration_f;        // Floating-point version of duration
float distance_cm;       // Distance calculated in centimeters

// Define pins
#define SERVO_PIN 10        // Servo motor pin
#define MAX_DISTANCE 200    // Max distance (cm) for ultrasonic sensing
#define STOP_DISTANCE 50    // Stop robot if obstacle is closer than this

Servo myservo;   // Create Servo object to control the scanning servo

// LED indicators
const int leftgreenLed = 9;   // Green LED pin
const int rightredLed = 8;    // Red LED pin

// Ultrasonic sensor pins
const int trigPin = 12;       // Ultrasonic trigger pin
const int echoPin = 11;       // Ultrasonic echo pin

// Motor driver pins for 4 motors (M1, M2, M3, M4)
int M1_enA = 2;  int M1_1A = 30;  int M1_2A = 31;
int M2_enA = 3;  int M2_1A = 32;  int M2_2A = 33;
int M3_enA = 5;  int M3_1A = 34;  int M3_2A = 35;
int M4_enA = 6;  int M4_1A = 36;  int M4_2A = 37;

int pos;              // Current servo position
int previous_pos;     // Previous servo position (not used here but declared)

// Timing and logic flags
unsigned long startTime = 0;
bool obstacleDetected = false;
bool initialScanDone = false;

void setup() {
  Serial.begin(9600);       // Initialize serial monitor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  myservo.attach(SERVO_PIN);   // Attach servo to pin
  delay(500);

  // Set motor driver pins as outputs
  pinMode(M1_enA, OUTPUT); pinMode(M1_1A, OUTPUT); pinMode(M1_2A, OUTPUT);
  pinMode(M2_enA, OUTPUT); pinMode(M2_1A, OUTPUT); pinMode(M2_2A, OUTPUT);
  pinMode(M3_enA, OUTPUT); pinMode(M3_1A, OUTPUT); pinMode(M3_2A, OUTPUT);
  pinMode(M4_enA, OUTPUT); pinMode(M4_1A, OUTPUT); pinMode(M4_2A, OUTPUT);

  // LED indicators as outputs
  pinMode(leftgreenLed, OUTPUT);
  pinMode(rightredLed, OUTPUT);
}

void loop() {
  // Trigger ultrasonic pulse
  delay(500);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure echo pulse duration
  duration = pulseIn(echoPin, HIGH);
  duration_f = duration * 1.0;
  distance_cm = duration * 0.0343 / 2;  // Convert to cm

  // Perform initial scanning sweep with servo
  if (!initialScanDone) {
    Serial.println("Performing obstacle scan...");

    pos = 90; myservo.write(pos); delay(500);
    Serial.print("\n current position = "); Serial.print(pos);

    pos = 180; myservo.write(pos); delay(500);
    Serial.print("\n current position = "); Serial.print(pos);

    pos = 0; myservo.write(pos); delay(500);
    Serial.print("\n current position = "); Serial.print(pos);

    pos = 90; myservo.write(pos); delay(300);  // Return to center
    initialScanDone = true;
  }

  // If path is clear, move forward
  if (distance_cm > STOP_DISTANCE && initialScanDone) {
    Serial.println("ready");
    digitalWrite(leftgreenLed, HIGH);   // Green LED = path clear
    forward();
  }

  // If obstacle detected, stop and blink LEDs
  if (distance_cm < STOP_DISTANCE && initialScanDone) {
    Serial.println("Obstacle DETECTED");
    stop();

    // Blink red + green LEDs as a warning
    digitalWrite(rightredLed, HIGH);
    digitalWrite(leftgreenLed, HIGH);
    delay(300);
    digitalWrite(rightredLed, LOW);
    digitalWrite(leftgreenLed, LOW);
    delay(300);

    initialScanDone = false;   // Restart scan
  }
}

// Motor control functions
void stop() {
  analogWrite(2, 0);
  analogWrite(3, 0);
  analogWrite(5, 0);
  analogWrite(6, 0);
}

// Turn left
void Left() {
  analogWrite(2, 200); analogWrite(3, 200);
  analogWrite(5, 200); analogWrite(6, 200);
  digitalWrite(M1_1A, LOW); digitalWrite(M1_2A, HIGH);
  digitalWrite(M2_1A, HIGH); digitalWrite(M2_2A, LOW);
  digitalWrite(M3_1A, LOW); digitalWrite(M3_2A, HIGH);
  digitalWrite(M4_1A, HIGH); digitalWrite(M4_2A, LOW);
}

// Turn right
void right() {
  analogWrite(2, 200); analogWrite(3, 200);
  analogWrite(5, 200); analogWrite(6, 200);
  digitalWrite(M1_1A, HIGH); digitalWrite(M1_2A, LOW);
  digitalWrite(M2_1A, LOW);  digitalWrite(M2_2A, HIGH);
  digitalWrite(M3_1A, HIGH); digitalWrite(M3_2A, LOW);
  digitalWrite(M4_1A, LOW);  digitalWrite(M4_2A, HIGH);
}

// Move forward
void forward() {
  analogWrite(2, 200); analogWrite(3, 200);
  analogWrite(5, 200); analogWrite(6, 200);
  digitalWrite(M1_1A, LOW); digitalWrite(M1_2A, HIGH);
  digitalWrite(M2_1A, LOW); digitalWrite(M2_2A, HIGH);
  digitalWrite(M3_1A, LOW); digitalWrite(M3_2A, HIGH);
  digitalWrite(M4_1A, LOW); digitalWrite(M4_2A, HIGH);
}
