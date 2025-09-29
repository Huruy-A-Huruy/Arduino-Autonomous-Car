# ELE8941 Robotics Projects – Huruy Huruy

This repository contains Arduino projects completed as part of the ELE8941 Robotics course. The projects demonstrate pre-programmed robot movements, sensor-based obstacle detection, and IMU-based LED feedback. These projects highlight hands-on skills in robotics, embedded programming, and electronics integration.

---

## Projects Overview

### 1. **IMU LED Latch**
**File:** `IMU_LED_Latch_LabX.ino`  
- Uses an LSM6DSO IMU (accelerometer + gyroscope) to control two LEDs.  
- Red LED lights when X-axis acceleration exceeds a threshold.  
- Green LED lights when Z-axis gyroscope rotation exceeds a threshold.  
- LEDs can be reset with a push button.  
- Demonstrates sensor reading, conditional logic, and output control.

### 2. **Obstacle Avoidance Robot**
**Files:** `Obstacle_Avoidance_Lab7.ino`, `Obstacle_Avoidance_ServoScan_Lab7.ino`  
- Ultrasonic and IR sensors detect obstacles.  
- Servo motor scans for obstacles and determines safe movement direction.  
- Robot performs veering, stopping, and forward movement based on sensor input.  
- LEDs indicate status during operation.  
- Demonstrates sensor integration, motor control with PWM, and decision-making logic.

### 3. **Pre-programmed Movement Sequences**
**Files:** `Preprogrammed_MoveSequence_Lab3.ino`, `Preprogrammed_MoveSequence_Lab3_v2.ino`  
- Executes pre-defined forward, backward, and turning movements.  
- Incorporates PWM motor control for precise speed adjustment.  
- Includes sequences for 90° left/right turns, multiple tile movement, and reversals.  
- Demonstrates programming sequences, timing, and motor coordination.

---

## Hardware Used
- Arduino Uno  
- LSM6DSO IMU  
- Ultrasonic sensor (HC-SR04)  
- Sharp IR sensors  
- Servo motor  
- DC motors with H-bridge driver  
- LEDs and push buttons

---

## Skills Demonstrated
- Embedded C++ programming for Arduino  
- Sensor interfacing (IMU, IR, ultrasonic)  
- Servo and DC motor control with PWM  
- Conditional logic and control flow  
- Robotics sequence planning  
- Debugging and serial monitoring  

---

## How to Run
1. Open the `.ino` file in Arduino IDE.  
2. Connect the required sensors and actuators according to the pin definitions in each code file.  
3. Upload the sketch to your Arduino board.  
4. Monitor output via Serial Monitor for debugging and sensor readings.  
5. Observe LEDs and motor behavior corresponding to programmed logic.  

---

**Author:** Huruy Huruy  
**Course:** ELE8941 Robotics (Winter 2024)  
