// Puck Spinner
// Goal:
// Make a two-wheeled robot escape from a closed arena by moving toward the direction with the greatest open space while avoiding nearby obstacles using a single ultrasonic distance sensor.

// Principle:
// The robot periodically performs a 180° (or 360°) scan of its surroundings by rotating in place while taking distance measurements at several defined angles. The direction with the largest measured obstacle-free distance is selected as the next heading.

// 🔄 Core Behavior
// Scanning phase
// Stop the robot.
// Rotate in small increments.
// At each step, measure distance with the ultrasonic sensor.
// Store all distance readings in a small array (polar scan).
// When scanning is complete, find the angle with the largest distance.
// Decision phase
// Compare the highest measured distance to a safety threshold.
// If the “best direction” is sufficiently open → choose it.
// If all directions are too close → rotate randomly and rescan.

// Movement phase
// Turn the robot toward the selected direction.
// Move forward while the distance ahead remains > threshold.
// If an obstacle is detected during movement → stop + return to scanning phase.
// Repeat, enabling continuous exploration until an exit is found.

// 📌 Data Representation
// scanAngles[] → list of scanned angles (8–16 values)
// scanDistances[] → distance for each angle
// bestIndex → index of the maximum distance
// bestAngle → angle to face before movement


// | State              | Description                                     |
// | ------------------ | ----------------------------------------------- |
// | `SCAN`             | Take multiple ultrasonic samples while rotating |
// | `SELECT_DIRECTION` | Identify the angle with max distance            |
// | `ALIGN`            | Turn robot to face that direction               |
// | `MOVE_FORWARD`     | Drive ahead until obstacle threshold            |
// | `AVOID/STOP`       | Obstacle too close → back to `SCAN`             |

#include <NewPing.h>
#include <Servo.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

// Motor setup
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

// Ultrasonic sensor setup
#define TRIGGER_PIN  9
#define ECHO_PIN     10
#define MAX_DISTANCE 200

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
Servo servoMotor;

#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define SERVO_STEP_ANGLE 20
#define SCAN_ANGLES_COUNT ((SERVO_MAX_ANGLE - SERVO_MIN_ANGLE) / SERVO_STEP_ANGLE + 1)

int scanAngles[SCAN_ANGLES_COUNT];
int scanDistances[SCAN_ANGLES_COUNT];
int bestIndex = -1;
int bestAngle = 0;

#define OBSTACLE_THRESHOLD 30 // in cm
#define FORWARD_SPEED 150
#define TURN_SPEED 100
#define SCAN_DELAY 100 // milliseconds

enum State {
    SCAN,
    SELECT_DIRECTION,
    ALIGN,
    MOVE_FORWARD,
    AVOID_STOP
};

State currentState = SCAN;
unsigned long stateStartTime = 0;

void setup() {
    Serial.begin(9600);
    AFMS.begin();
    servoMotor.attach(6);

    // Initialize scan angles
    for (int i = 0; i < SCAN_ANGLES_COUNT; i++) {
      scanAngles[i] = SERVO_MIN_ANGLE + i * SERVO_STEP_ANGLE;
    }
}

void loop() {
    switch (currentState) {
        case SCAN:
            performScan();
            currentState = SELECT_DIRECTION;
            break;

        case SELECT_DIRECTION:
            selectBestDirection();
            currentState = ALIGN;
            break;

        case ALIGN:
            alignToBestDirection();
            currentState = MOVE_FORWARD;
            break;

        case MOVE_FORWARD:
            moveForward();
            break;

        case AVOID_STOP:
            stopMovement();
            currentState = SCAN;
            break;
    }
}

void performScan() {
    for (int i = 0; i < SCAN_ANGLES_COUNT; i++) {
        servoMotor.write(scanAngles[i]);
        delay(300); // Allow time for servo to reach position
        scanDistances[i] = sonar.ping_cm();
        delay(SCAN_DELAY);
    }
}

void selectBestDirection() {
    bestIndex = -1;
    int maxDistance = -1;

    for (int i = 0; i < SCAN_ANGLES_COUNT; i++) {
        if (scanDistances[i] > maxDistance) {
            maxDistance = scanDistances[i];
            bestIndex = i;
        }
    }

    if (bestIndex != -1) {
        bestAngle = scanAngles[bestIndex];
    }
}

void alignToBestDirection() {
    int turnAngle = bestAngle - 90; // Assuming 90 is straight ahead
    if (turnAngle > 0) {
        // Turn right
        leftMotor->setSpeed(TURN_SPEED);
        rightMotor->setSpeed(0);
    } else {
        // Turn left
        leftMotor->setSpeed(0);
        rightMotor->setSpeed(TURN_SPEED);
    }
    delay(abs(turnAngle) * 10); // Simple proportional delay for turning
    stopMovement();
}

void moveForward() {
    leftMotor->setSpeed(FORWARD_SPEED);
    rightMotor->setSpeed(FORWARD_SPEED);

    while (true) {
        int distance = sonar.ping_cm();
        if (distance > 0 && distance < OBSTACLE_THRESHOLD) {
            currentState = AVOID_STOP;
            break;
        }
        delay(100);
    }
}

void stopMovement() {
    leftMotor->setSpeed(0);
    rightMotor->setSpeed(0);
}