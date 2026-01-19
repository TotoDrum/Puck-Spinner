// Puck Spinner

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
#include <EEPROM.h>

// Ultrasonic sensor setup
#define LEFT_SERVO_PIN   5
#define RIGHT_SERVO_PIN  6
#define TRIG_PIN         9
#define ECHO_PIN         10
#define MAX_DISTANCE 200

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
Servo leftWheel;
Servo rightWheel;

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

// Forward declarations
void performScan();
void selectBestDirection();
void alignToBestDirection();
void moveForward();
void stopMovement();

void setup() {
    Serial.begin(9600);

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);

    leftWheel.attach(LEFT_SERVO_PIN);
    rightWheel.attach(RIGHT_SERVO_PIN);

    // Initialize scan angles
    for (int i = 0; i < SCAN_ANGLES_COUNT; i++) {
      scanAngles[i] = SERVO_MIN_ANGLE + i * SERVO_STEP_ANGLE;
    }
    Serial.println("Initiating Puck Spinner Robot");
}

void loop() {
    switch (currentState) {
        case SCAN:
            //performScan();
            currentState = SELECT_DIRECTION;
            break;

        case SELECT_DIRECTION:
            //selectBestDirection();
            currentState = ALIGN;
            break;

        case ALIGN:
            //alignToBestDirection();
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
        leftWheel.write(scanAngles[i]);
        rightWheel.write(scanAngles[i]);
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
        leftWheel.write(TURN_SPEED);
        rightWheel.write(0);
    } else {
        // Turn left
        leftWheel.write(0);
        rightWheel.write(TURN_SPEED);
    }
    delay(abs(turnAngle) * 10); // Simple proportional delay for turning
    stopMovement();
}

void moveForward() {
    leftWheel.write(FORWARD_SPEED);
    rightWheel.write(FORWARD_SPEED);

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
    leftWheel.write(0);
    rightWheel.write(0);
}
