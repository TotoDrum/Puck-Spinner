#include "PuckSpinner.hpp"

#define LEFT_SERVO_PIN   5
#define RIGHT_SERVO_PIN  6
#define TRIG_PIN         9
#define ECHO_PIN         10

#define SWITCH_PIN       2

#define LEFT_STOP   1418
#define RIGHT_STOP  1512

#define DRIVE_DELTA 500

#define OBSTACLE_CM 25 // stop if object closer than this

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
long distances[7]; // from -90 to +90 in steps of 30 degrees

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


// Scanning function
void performScan() {
    for (int angle = -90; angle <= 90; angle += 30) {
        turnDegrees(angle);
        delay(500);
    }
}

// Direction selection function
// store directions in an global array and pick the best one
void selectBestDirection() {

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

void turnDegrees(int degrees) {
    // Placeholder for turning a specific number of degrees
    // In a real implementation, this would involve controlling the wheels to turn the robot
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

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);

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
            performScan();
            currentState = SELECT_DIRECTION;
            Serial.println("Scanning...");
            break;

        case SELECT_DIRECTION:
            //selectBestDirection();
            currentState = ALIGN;
            Serial.println("Selecting direction...");
            break;

        case ALIGN:
            //alignToBestDirection();
            currentState = MOVE_FORWARD;
            Serial.println("Aligning...");
            break;

        case MOVE_FORWARD:
            moveForward();
            Serial.println("Moving forward...");
            break;

        case AVOID_STOP:
            stopWheels();
            currentState = SCAN;
            Serial.println("Obstacle detected, stopping...");
            break;
    }
}

void loop() {
    if (mode == ALGORITHM1) {
        Algorithm1Loop();
    } else if (mode == ALGORITHM2) {
        static unsigned long lastPing = 0;

        static long distance = 0;
        if (millis() - lastPing >= 60) {
            lastPing = millis();
            distance = getDistanceStable();
            Serial.print("Distance: ");
            Serial.println(distance);
        }
        //Algorithm2Loop();
        if (distance > 0 && distance < OBSTACLE_CM) {
            stopWheels();
            turn90right();
        } else {
            moveForward();
        }
    }
    delay(60); // small delay to avoid sonar spam
}
