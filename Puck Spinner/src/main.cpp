#include "PuckSpinner.hpp"

#define LEFT_SERVO_PIN   5
#define RIGHT_SERVO_PIN  6
#define TRIG_PIN         9
#define ECHO_PIN         10

#define SWITCH_PIN       2

#define LEFT_STOP   1418
#define RIGHT_STOP  1512

#define DRIVE_DELTA 600 // speed for moving forward / turning
#define OBSTACLE_CM 25 // stop if object closer than this
#define SCAN_ANGLE 45 // degrees to turn per scan step
#define SCAN_STEPS 8 // number of scan steps (360/45)
#define ANGLE_DELTA 63 // delay per 10 degrees turn
#define DISTANCE_DELTA 100 // milliseconds per cm, <<to be calibrated>>

enum Mode {
    ALGORITHM1,
    ALGORITHM2,
    CALIBRATION
};

enum State {
    SCAN,
    SELECT_DIRECTION,
    ALIGN,
    MOVE_FORWARD,
    AVOID_STOP
};

// Global array for storing distances at different angles
long distances[SCAN_STEPS]; // from 180 to -180 in steps of SCAN_ANGLE

Servo leftWheel;
Servo rightWheel;
State currentState = SCAN;
Mode mode = ALGORITHM1; // select mode here


// HC-SR04 Ultrasonic distance sensor reading
long readDistanceCM() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(5);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    unsigned long dur = pulseIn(ECHO_PIN, HIGH, 30000UL);
    if (dur == 0) return -1;

    long cm = (long)(dur * 0.0343f / 2.0f);

    return cm;
}

long readDistanceCM_filtered() {
  const int N = 5;
  long v[N];
  int k = 0;

  for (int i = 0; i < N; i++) {
    long d = readDistanceCM();
    if (d > 0) v[k++] = d;
    delay(10);
  }

  if (k == 0) return -1;

  for (int i = 0; i < k - 1; i++)
    for (int j = i + 1; j < k; j++)
      if (v[j] < v[i]) { long t = v[i]; v[i] = v[j]; v[j] = t; }

  return v[k / 2]; // median
}

long getDistanceStable() {
  static long lastGood = -1;
  static unsigned long lastGoodMs = 0;

  long d = readDistanceCM_filtered();
  if (d > 0) {
    lastGood = d;
    lastGoodMs = millis();
    return d;
  }

  // If we had a good reading recently, reuse it
  if (lastGood > 0 && (millis() - lastGoodMs) < 300) {
    return lastGood;
  }

  return -1;
}

int getAverageDistance() {
    long total = 0;
    int count = 0;
    long d = 0;

    for (int i = 0; i < 5; i++) {
        d = getDistanceStable();
        if (d > 0) {
            total += d;
            count++;
        }
        delay(10);
    }
    if (count == 0) return -1;
    return total / count;
}

// Scanning function
void performScan() {
    long distance = 0;

    stopWheels();
    for (int i = 0; i < SCAN_STEPS; i++) {
        turnDegrees(SCAN_ANGLE);
        distance = getAverageDistance();
        distances[i] = distance;
        Serial.print("Angle: ");
        Serial.println((i - SCAN_STEPS / 2) * SCAN_ANGLE);
        Serial.print("Distance: ");
        Serial.println(distance);
        stopWheels();
    }
    Serial.println("Scan complete.");
    // print all values
    for (int i = 0; i < SCAN_STEPS; i++) {
        Serial.print("Angle ");
        Serial.print((i - SCAN_STEPS / 2) * SCAN_ANGLE);
        Serial.print(": ");
        Serial.println(distances[i]);
    }
}

// Direction selection function
void selectBestDirection() {
    int bestIndex = -1;
    long maxDistance = -1;

    for (int i = 0; i < SCAN_STEPS; i++) {
        if (distances[i] > maxDistance) {
            maxDistance = distances[i];
            bestIndex = i;
        }
    }

    if (bestIndex != -1) {
        int bestAngle = (bestIndex - SCAN_STEPS / 2) * SCAN_ANGLE;
        Serial.print("Best angle selected: ");
        Serial.println(bestAngle);
    } else {
        Serial.println("No valid direction found.");
    }
}

// move a certain distance forward
void moveForwardDistance(int cm) {
    int timePerCm = DISTANCE_DELTA;

    leftWheel.write(LEFT_STOP + DRIVE_DELTA);
    rightWheel.write(RIGHT_STOP - DRIVE_DELTA);
    delay(cm * timePerCm);
    stopWheels();
}

// Alignment function
void alignToBestDirection() {
    // Placeholder for alignment logic
    // In a real implementation, this would rotate the robot to face the chosen direction
}

// move a certain distance forward
void moveForwardDistance(int cm) {
    // Placeholder for moving forward a specific distance
    // In a real implementation, this would involve wheel encoders or timing
}

// turn x amount of degrees
void turnDegrees(int degrees) {
    for (int i = 0; i < abs(degrees) / 10; i++) {
        if (degrees > 0) { // turn right
            leftWheel.write(LEFT_STOP + DRIVE_DELTA);
            rightWheel.write(RIGHT_STOP + DRIVE_DELTA);
        } else { // turn left
            leftWheel.write(LEFT_STOP - DRIVE_DELTA);
            rightWheel.write(RIGHT_STOP - DRIVE_DELTA);
        }
        delay(ANGLE_DELTA);
    }
    stopWheels();
    delay(200);
}

void stopWheels() {
    leftWheel.write(LEFT_STOP);
    rightWheel.write(RIGHT_STOP);
}

void moveForward() {
    leftWheel.write(LEFT_STOP + DRIVE_DELTA);
    rightWheel.write(RIGHT_STOP - DRIVE_DELTA);
}

void turn90right() {
    leftWheel.write(LEFT_STOP + DRIVE_DELTA);
    rightWheel.write(RIGHT_STOP + DRIVE_DELTA);
    delay(600); // adjust this delay for a proper 90 degree turn
    stopWheels();
    delay(200);
}

void setup() {
    Serial.begin(9600);

    // HC-RO4 setup
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);

    // Switch setup
    pinMode(SWITCH_PIN, INPUT_PULLUP);

    leftWheel.attach(LEFT_SERVO_PIN);
    rightWheel.attach(RIGHT_SERVO_PIN);

    Serial.println("Initialisation done.");
    Serial.println("Selected mode :");

    // select mode based on switch
    if (digitalRead(SWITCH_PIN) == LOW) {
        mode = ALGORITHM1;
        Serial.println("Algorithm 1");
    } else {
        mode = ALGORITHM2;
        Serial.println("Algorithm 2");
    }
}

// Algo 1 main loop
void Algorithm1Loop() {
    switch (currentState) {
        case SCAN:
            Serial.println("Scanning...");
            performScan();
            currentState = SELECT_DIRECTION;
            break;

        case SELECT_DIRECTION:
            Serial.println("Selecting direction...");
            selectBestDirection();
            //currentState = ALIGN;
            currentState = AVOID_STOP;
            break;

        case ALIGN:
            Serial.println("Aligning...");
            //alignToBestDirection();
            currentState = MOVE_FORWARD;
            break;

        case MOVE_FORWARD:
            Serial.println("Moving forward...");
            moveForward();
            break;

        case AVOID_STOP:
            Serial.println("Obstacle detected, stopping...");
            stopWheels();
            //currentState = SCAN;
            break;
    }
}

void Algorithm2Loop() {
    static unsigned long lastPing = 0;
    static long distance = 0;

    if (millis() - lastPing >= 60) {
        lastPing = millis();
        distance = getDistanceStable();
        Serial.print("Distance: ");
        Serial.println(distance);
    }

    static int consecutiveObstacles = 0;

    if (distance > 0 && distance < OBSTACLE_CM) {
        stopWheels();
        consecutiveObstacles++;

            if (consecutiveObstacles >= 3) {
            Serial.println("3 obstacles detected, reversing and scanning...");
            // Reverse
            leftWheel.write(LEFT_STOP - DRIVE_DELTA);
            rightWheel.write(RIGHT_STOP + DRIVE_DELTA);
            delay(1000); // reverse for 1 second
            stopWheels();
            delay(200);

            // Perform full scan
            performScan();
            selectBestDirection();

            consecutiveObstacles = 0; // reset counter
        } else {
            turnDegrees(90); // turn right on obstacle
        }
    } else {
        moveForward();
        consecutiveObstacles = 0; // reset counter when path is clear
    }
}

void loop() {
    if (mode == ALGORITHM1) {
        Algorithm1Loop();
    } else if (mode == ALGORITHM2) {
        Algorithm2Loop();
    }
}
