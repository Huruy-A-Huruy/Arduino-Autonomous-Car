// April 11, 2024
// Robotics 

#include "SparkFunLSM6DSO.h"   // Library for the LSM6DSO IMU sensor (accelerometer + gyroscope)
#include "Wire.h"              // Library for I2C communication

// Pin assignments
const int Red_Led = 7;          
const int Green_Led = 4;        
const int Push_Button = 46;      

LSM6DSO myIMU;                  // Create an IMU object

// State variables to keep track of whether LEDs are latched ON or OFF
bool Red_LED = 0;      
bool Green_LED = 0;    

void setup()
{
  Wire.begin();                 // Initialize I2C communication
  delay(50);
  Serial.begin(9600);           // Start serial communication at 9600 baud
  delay(300);
  
  // Initialize the IMU sensor
  if(myIMU.begin())
    Serial.println("Ready to use -- Robotics ELE8941 Winter 2024");
  else {
    Serial.println("Could not connect to IMU.");
    Serial.println("Freezing");
  }

  // Load the default/basic IMU settings
  if(myIMU.initialize(BASIC_SETTINGS))
    Serial.println("Loaded Basic Settings.");
  
  // Configure pin modes
  pinMode(Red_Led, OUTPUT);          // Red LED as output
  pinMode(Green_Led, OUTPUT);        // Green LED as output
  pinMode(Push_Button, INPUT_PULLUP);// Push button with internal pull-up resistor
}

void loop() 
{
  // Read accelerometer values (X, Y, Z in g’s)
  float X_acc = myIMU.readFloatAccelX();
  float Y_acc = myIMU.readFloatAccelY();
  float Z_acc = myIMU.readFloatAccelZ();

  // Read gyroscope values (X, Y, Z in degrees/second)
  float X_gyro = myIMU.readFloatGyroX();
  float Y_gyro = myIMU.readFloatGyroY();
  float Z_gyro = myIMU.readFloatGyroZ();   
  
  // Condition: Turn ON and latch Red LED when X acceleration exceeds +0.4g
  if (X_acc > 0.4 && !Red_LED) {
    digitalWrite(Red_Led, HIGH);   // Turn ON Red LED
    Red_LED = 1;                   // Update latch state
  }
  
  // Condition: Turn ON and latch Green LED when Z-axis rotation > 100 deg/s
  if (Z_gyro > 100 && !Green_LED) {
    digitalWrite(Green_Led, HIGH); // Turn ON Green LED
    Green_LED = 1;                 // Update latch state
  }

  // Reset condition: If push button is pressed → turn OFF both LEDs
  if (digitalRead(Push_Button) == LOW) {
    Red_LED = 0;
    Green_LED = 0;
    digitalWrite(Red_Led, LOW);    // Turn OFF Red LED
    digitalWrite(Green_Led, LOW);  // Turn OFF Green LED
  }
  
  // Print gyro values (for debugging in Serial Monitor)
  Serial.print(X_gyro, 0);
  Serial.print("\t");
  Serial.print(Y_gyro, 0);
  Serial.print("\t");
  Serial.println(Z_gyro, 0);  
  delay(100);

  /*
  // Alternative: Print accelerometer values (commented out)
  Serial.print(X_acc, 2);
  Serial.print("\t");
  Serial.print(Y_acc, 2);
  Serial.print("\t");
  Serial.println(Z_acc, 2);  
  delay(100);
  */
}


