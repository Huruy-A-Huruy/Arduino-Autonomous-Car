// ELE8941 Robotics
// Author: Huruy Huruy
// Description: Pre-programmed sequence to move the robot forward, backward, 
//              and turn left/right using 4 DC motors and enable/direction pins.

// ------------------- Motor Pin Declarations -------------------
int M1_enA = 2, M1_1A = 30, M1_2A = 31;
int M2_enA = 3, M2_1A = 32, M2_2A = 33;
int M3_enA = 5, M3_1A = 34, M3_2A = 35;
int M4_enA = 6, M4_1A = 36, M4_2A = 37;

// ------------------- Setup -------------------
void setup() {
  Serial.begin(9600); // For debugging and step info

  // Set all motor pins as outputs
  pinMode(M1_enA, OUTPUT); pinMode(M1_1A, OUTPUT); pinMode(M1_2A, OUTPUT);
  pinMode(M2_enA, OUTPUT); pinMode(M2_1A, OUTPUT); pinMode(M2_2A, OUTPUT);
  pinMode(M3_enA, OUTPUT); pinMode(M3_1A, OUTPUT); pinMode(M3_2A, OUTPUT);
  pinMode(M4_enA, OUTPUT); pinMode(M4_1A, OUTPUT); pinMode(M4_2A, OUTPUT);
}

// ------------------- Main Loop -------------------
void loop() {
  // ---------- Forward 3 tiles ----------
  Serial.println("3 TILES FWD");
  moveForward(240, 250, 240, 240); // motor speeds
  delay(1400);
  stopAll();

  // ---------- Turn 90° Left ----------
  Serial.println("90 degree left");
  turnLeft(0, 190, 0, 190);
  delay(700);
  stopAll();

  // ---------- Forward after turning ----------
  Serial.println("Forward after turning");
  moveForward(240, 240, 240, 240);
  delay(3400);
  stopAll();

  // ---------- Turn 90° Right ----------
  Serial.println("90 degree right");
  turnRight(180, 0, 180, 0);
  delay(850);
  stopAll();

  // ---------- Forward 3 tiles ----------
  Serial.println("3 TILES FWD");
  moveForward(240, 190, 240, 190);
  delay(1450);
  stopAll();

  // ---------- Reverse 3 tiles ----------
  Serial.println("3 TILES REVERSE");
  moveReverse(240, 190, 240, 190);
  delay(1450);
  stopAll();

  // Pause before repeating the sequence
  delay(10000);
}

// ------------------- Functions -------------------
void moveForward(int M1spd, int M2spd, int M3spd, int M4spd){
  analogWrite(M1_enA, M1spd); digitalWrite(M1_1A, LOW); digitalWrite(M1_2A, HIGH);
  analogWrite(M2_enA, M2spd); digitalWrite(M2_1A, LOW); digitalWrite(M2_2A, HIGH);
  analogWrite(M3_enA, M3spd); digitalWrite(M3_1A, LOW); digitalWrite(M3_2A, HIGH);
  analogWrite(M4_enA, M4spd); digitalWrite(M4_1A, LOW); digitalWrite(M4_2A, HIGH);
}

void moveReverse(int M1spd, int M2spd, int M3spd, int M4spd){
  analogWrite(M1_enA, M1spd); digitalWrite(M1_1A, HIGH); digitalWrite(M1_2A, LOW);
  analogWrite(M2_enA, M2spd); digitalWrite(M2_1A, HIGH); digitalWrite(M2_2A, LOW);
  analogWrite(M3_enA, M3spd); digitalWrite(M3_1A, HIGH); digitalWrite(M3_2A, LOW);
  analogWrite(M4_enA, M4spd); digitalWrite(M4_1A, HIGH); digitalWrite(M4_2A, LOW);
}

void turnLeft(int M1spd, int M2spd, int M3spd, int M4spd){
  analogWrite(M1_enA, M1spd); analogWrite(M2_enA, M2spd);
  analogWrite(M3_enA, M3spd); analogWrite(M4_enA, M4spd);
  digitalWrite(M1_1A, LOW); digitalWrite(M1_2A, HIGH);
  digitalWrite(M2_1A, LOW); digitalWrite(M2_2A, HIGH);
  digitalWrite(M3_1A, LOW); digitalWrite(M3_2A, HIGH);
  digitalWrite(M4_1A, LOW); digitalWrite(M4_2A, HIGH);
}

void turnRight(int M1spd, int M2spd, int M3spd, int M4spd){
  analogWrite(M1_enA, M1spd); analogWrite(M2_enA, M2spd);
  analogWrite(M3_enA, M3spd); analogWrite(M4_enA, M4spd);
  digitalWrite(M1_1A, LOW); digitalWrite(M1_2A, HIGH);
  digitalWrite(M2_1A, LOW); digitalWrite(M2_2A, HIGH);
  digitalWrite(M3_1A, LOW); digitalWrite(M3_2A, HIGH);
  digitalWrite(M4_1A, LOW); digitalWrite(M4_2A, HIGH);
}

void stopAll(){
  analogWrite(M1_enA, 0); analogWrite(M2_enA, 0);
  analogWrite(M3_enA, 0); analogWrite(M4_enA, 0);
}
