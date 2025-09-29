// Variables for ultrasonic sensor
long duration;         // Duration of echo pulse
float duration_f;      // Float version of duration
float distance_cm;     // Calculated distance in cm

#include <Servo.h>     // Library to control servo

// Servo motor configuration
#define SERVO_PIN 10       // Servo control pin
#define MAX_DISTANCE 200   // Max sensing distance in cm
#define STOP_DISTANCE 50   // Distance threshold for stopping (cm)

Servo myservo;   // Servo object to control scanning

// LED indicators
const int leftgreenLed = 9;   // Green LED (clear path)
const int rightredLed  = 8;   // Red LED (obstacle detected)

// Ultrasonic sensor pins
const int trigPin = 12;   // Trigger pin
const int echoPin = 11;   // Echo pin

// Motor driver pins for 4 DC motors (M1–M4)
int M1_enA = 2;  int M1_1A = 30;  int M1_2A = 31;
int M2_enA = 3;  int M2_1A = 32;  int M2_2A = 33;
int M3_enA = 5;  int M3_1A = 34;  int M3_2A = 35;
int M4_enA = 6;  int M4_1A = 36;  int M4_2A = 37;

int pos;             // Current servo position
int previous_pos;    // Stores last servo position (not used here)

unsigned long startTime = 0;   // Timing variable
bool obstacleDetected = false; // Obstacle detection flag
bool initialScanDone = false;  // Marks if initial servo scan is done

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(9600);   // Start serial monitor for debugging
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Attach servo motor
  myservo.attach(SERVO_PIN);
  delay(500);

  // Set all motor pins as outputs
  pinMode(M1_enA,OUTPUT); pinMode(M1_1A,OUTPUT); pinMode(M1_2A,OUTPUT);
  pinMode(M2_enA,OUTPUT); pinMode(M2_1A,OUTPUT); pinMode(M2_2A,OUTPUT);
  pinMode(M3_enA,OUTPUT); pinMode(M3_1A,OUTPUT); pinMode(M3_2A,OUTPUT);
  pinMode(M4_enA,OUTPUT); pinMode(M4_1A,OUTPUT); pinMode(M4_2A,OUTPUT);

  // LED indicators as outputs
  pinMode(leftgreenLed,OUTPUT);
  pinMode(rightredLed,OUTPUT);
}

// -------------------- MAIN LOOP --------------------
void loop() {
  delay(500);

  // Trigger ultrasonic pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure echo return time
  duration = pulseIn(echoPin, HIGH);
  duration_f = duration * 1.0;
  distance_cm = duration * 0.0343 / 2;   // Convert to distance (cm)

  // ----------- Initial servo scan -----------
  if (!initialScanDone) {
    Serial.println("Performing obstacle scan...");

    pos = 90; myservo.write(pos); delay(500);
    Serial.print("\n current position = "); Serial.print(pos);

    pos = 180; myservo.write(pos); delay(500);
    Serial.print("\n current position = "); Serial.print(pos);

    pos = 0; myservo.write(pos); delay(500);
    Serial.print("\n current position = "); Serial.print(pos);

    pos = 90; myservo.write(pos); delay(300);   // Return servo to center
    initialScanDone = true;                     // Mark scan as complete
  }

  // ----------- Path clear: move forward -----------
  if (distance_cm > STOP_DISTANCE && initialScanDone) {
    Serial.println("Path is clear. Moving forward...");
    digitalWrite(leftgreenLed, HIGH);   // Green LED on
    forward();
  }

  // ----------- Obstacle detected: stop & blink -----------
  if (distance_cm < STOP_DISTANCE && initialScanDone) {
    Serial.println("Obstacle DETECTED");
    stop();

    // Blink LEDs to signal obstacle
    digitalWrite(rightredLed, HIGH);
    digitalWrite(leftgreenLed, HIGH);
    delay(300);
    digitalWrite(rightredLed, LOW);
    digitalWrite(leftgreenLed, LOW);
    delay(300);

    initialScanDone = false;   // Restart scan cycle
  }
}

// -------------------- MOTOR CONTROL --------------------

// Stop all motors
void stop() {
  analogWrite(2, 0);
  analogWrite(3, 0);
  analogWrite(5, 0);
  analogWrite(6, 0);
}

// Drive robot forward
void forward() {
  analogWrite(2, 200); analogWrite(3, 200);
  analogWrite(5, 200); analogWrite(6, 200);

  digitalWrite(M1_1A, LOW); digitalWrite(M1_2A, HIGH);
  digitalWrite(M2_1A, LOW); digitalWrite(M2_2A, HIGH);
  digitalWrite(M3_1A, LOW); digitalWrite(M3_2A, HIGH);
  digitalWrite(M4_1A, LOW); digitalWrite(M4_2A, HIGH);
}

/* -------------------- UNUSED FUNCTIONS --------------------
void initialScan() {
  // Alternative initial scan function (not used in final version)
}

void scanObstacle() {
  // Alternative obstacle detection logic (commented out)
}
----------------------------------------------------------- */
