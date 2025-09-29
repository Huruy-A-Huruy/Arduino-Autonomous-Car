// Huruy Huruy
// Apr 5

#include <SharpIR.h>   // Library for Sharp IR distance sensors
#include <Servo.h>     // Library for controlling servo motors

// Variables for ultrasonic sensor
long duration;         
float duration_f;      
float distance_cm;     

// Servo pin configuration
#define SERVO_PIN 10
#define MAX_DISTANCE 200   // Max ultrasonic range in cm
#define STOP_DISTANCE 50   // Stop threshold distance in cm

Servo myservo;   // Create servo object

// LED pins for indicators
const int leftgreenLed = 9;
const int rightredLed  = 8;

// Ultrasonic sensor pins
const int trigPin = 12;
const int echoPin = 11; // pins can be changed

// Motor driver pins (for 4 motors)
int M1_enA = 2;
int M1_1A  = 30;
int M1_2A  = 31;
int M2_enA = 3;
int M2_1A  = 32;
int M2_2A  = 33;
int M3_enA = 5;
int M3_1A  = 34;
int M3_2A  = 35;
int M4_enA = 6;
int M4_1A  = 36;
int M4_2A  = 37;

int pos;           // Servo position (not actively used yet)
int previous_pos;  // To track last servo position

// State flags
bool obstacleDetected = false;
bool initialScanDone  = false;

void setup() {
  Serial.begin(9600);       // Start serial monitor for debugging
  pinMode(trigPin, OUTPUT); // Ultrasonic trigger pin
  pinMode(echoPin, INPUT);  // Ultrasonic echo pin

  myservo.attach(SERVO_PIN); // Attach servo to pin 10
  delay(500);

  // Configure motor driver pins
  pinMode(M1_enA, OUTPUT); pinMode(M1_1A, OUTPUT); pinMode(M1_2A, OUTPUT);
  pinMode(M2_enA, OUTPUT); pinMode(M2_1A, OUTPUT); pinMode(M2_2A, OUTPUT);
  pinMode(M3_enA, OUTPUT); pinMode(M3_1A, OUTPUT); pinMode(M3_2A, OUTPUT);
  pinMode(M4_enA, OUTPUT); pinMode(M4_1A, OUTPUT); pinMode(M4_2A, OUTPUT);

  // Configure LEDs
  pinMode(leftgreenLed, OUTPUT);
  pinMode(rightredLed, OUTPUT);
}

void loop() {
  // Read left and right Sharp IR sensors
  int A15Int = analogRead(A11);  
  int A14Int = analogRead(A12);  

  // Calculate distances from IR sensors (empirical formula for Sharp IR)
  float Supply = 4.7; // measured supply voltage for accuracy
  float Left_IR_Volts  = A15Int / 1023.0 * Supply;
  float distanceL      = 27.109 * pow(Left_IR_Volts, -1.187);

  float Right_IR_Volts = A14Int / 1023.0 * Supply;
  float distanceR      = 27.109 * pow(Right_IR_Volts, -1.187);

  // Ultrasonic distance measurement
  digitalWrite(trigPin, LOW);  
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); // 10 µs pulse
  delayMicroseconds(10); 
  digitalWrite(trigPin, LOW);

  duration    = pulseIn(echoPin, HIGH);         // Measure echo return time
  duration_f  = duration * 1.0;                 // Convert to float
  distance_cm = duration * 0.0343 / 2;          // Convert to cm

  // Debugging output
  Serial.print(distanceL);
  Serial.print("|| VL ");
  Serial.print(Left_IR_Volts);
  Serial.print("|| ");
  Serial.print(distanceR);
  Serial.print("|| VR ");
  Serial.println(Right_IR_Volts);
  delay(30);

  // Obstacle avoidance logic
  if (distance_cm < STOP_DISTANCE) {  
    // Too close to obstacle → stop
    Serial.print("stop");
    stop();
  }
  else if (distanceL < 12 && distance_cm > STOP_DISTANCE) {  
    // Object detected on left → veer right
    Serial.print("veer right  ");
    Veer_Left();
  }
  else if (distanceR < 12 && distance_cm > STOP_DISTANCE) { 
    // Object detected on right → veer left
    Serial.print("veer left   ");
    Veer_right();
  }
  else if (distanceR > 12 && distanceL > 12 && distance_cm > STOP_DISTANCE) {  
    // Clear path → go forward
    Serial.print("straight   ");
    forward();
  }
}

// ---- Motor Control Functions ----

// Stop all motors
void stop(){
  analogWrite(2, 0);
  analogWrite(3, 0);
  analogWrite(5, 0);
  analogWrite(6, 0);
}

// Veer left function
void Veer_Left(){         
  analogWrite(2, 100); analogWrite(3, 200);
  analogWrite(5, 100); analogWrite(6, 200);
  
  digitalWrite(M1_1A, LOW); digitalWrite(M1_2A, HIGH);
  digitalWrite(M2_1A, LOW); digitalWrite(M2_2A, HIGH);
  digitalWrite(M3_1A, LOW); digitalWrite(M3_2A, HIGH);
  digitalWrite(M4_1A, LOW); digitalWrite(M4_2A, HIGH);

  digitalWrite(rightredLed, HIGH);   // Right LED ON
  digitalWrite(leftgreenLed, LOW);   // Left LED OFF
}

// Veer right function
void Veer_right(){              
  analogWrite(2, 200); analogWrite(3, 100);
  analogWrite(5, 200); analogWrite(6, 100);
  
  digitalWrite(M1_1A, LOW); digitalWrite(M1_2A, HIGH);
  digitalWrite(M2_1A, LOW); digitalWrite(M2_2A, HIGH);
  digitalWrite(M3_1A, LOW); digitalWrite(M3_2A, HIGH);
  digitalWrite(M4_1A, LOW); digitalWrite(M4_2A, HIGH);

  digitalWrite(leftgreenLed, HIGH);  // Left LED ON
  digitalWrite(rightredLed, LOW);    // Right LED OFF
}

// Move forward function
void forward(){               
  analogWrite(2, 200); analogWrite(3, 200);
  analogWrite(5, 200); analogWrite(6, 200);

  digitalWrite(M1_1A, LOW); digitalWrite(M1_2A, HIGH);
  digitalWrite(M2_1A, LOW); digitalWrite(M2_2A, HIGH);
  digitalWrite(M3_1A, LOW); digitalWrite(M3_2A, HIGH);
  digitalWrite(M4_1A, LOW); digitalWrite(M4_2A, HIGH);

  digitalWrite(leftgreenLed, LOW);   // Both LEDs OFF
  digitalWrite(rightredLed, LOW);
}
