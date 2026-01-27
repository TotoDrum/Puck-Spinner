#include "PuckSpinner.hpp"

#define LEFT_SERVO_PIN   5
#define RIGHT_SERVO_PIN  6
#define TRIG_PIN         9
#define ECHO_PIN         10

#define SWITCH_PIN       2

#define LEFT_STOP   1418
#define RIGHT_STOP  1518

#define DRIVE_DELTA 600 // speed for moving forward / turning
#define OBSTACLE_CM 10 // stop if object closer than this
#define SCAN_ANGLE 45 // degrees to turn per scan step
#define SCAN_STEPS 8 // number of scan steps (360/45)
#define DISTANCE_DELTA 20 // milliseconds per cm, <<to be calibrated>>

// Tunables (start here, then calibrate)
#define TURN_FAST_DELTA   600   // keep your current speed
#define TURN_SLOW_DELTA   250   // slower near the end
#define TURN_BRAKE_DELTA  200   // tiny reverse pulse
#define TURN_BRAKE_MS      18   // 12–25ms usually
#define TURN_SETTLE_MS     30   // let chassis settle

#define FWD_FAST_DELTA   600
#define FWD_BRAKE_DELTA  180
#define FWD_BRAKE_MS      15
#define FWD_SETTLE_MS     20

enum Mode {
    ALGORITHM1,
    ALGORITHM2,
    CALIBRATION
};

enum State {
    SCAN,
    QUICK_SCAN,
    SELECT_DIRECTION,
    ALIGN,
    MOVE_FORWARD,
    AVOID_STOP
};

enum MotionState {
    MOVING_FWD,
    TURNING,
    REVERSING,
    STOPPED
};

MotionState motion = STOPPED;
unsigned long motionChangedMs = 0;

void setMotion(MotionState m) {
    motion = m;
    motionChangedMs = millis();
}

enum Algo2State {
  A2_FORWARD,
  A2_TURN_90,
  A2_BACKUP,
  A2_SCAN,
  A2_TURN_TO_BEST,
  A2_RECOVER_FORWARD
};


// Global array for storing distances at different angles
long distances[SCAN_STEPS]; // from 180 to -180 in steps of SCAN_ANGLE
long bestAngle = -1;
long bestDistance = -1;

int LEFT_TRIM  = 0;
int RIGHT_TRIM = -20;

Servo leftWheel;
Servo rightWheel;
State currentState = SCAN;

Mode mode = ALGORITHM1;


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

    if (k == 0)
        return -1;

    for (int i = 0; i < k - 1; i++) {
        for (int j = i + 1; j < k; j++) {
            if (v[j] < v[i]) {
                long t = v[i];
                v[i] = v[j];
                v[j] = t;
            }
        }
    }
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

struct DistanceState {
    long cm = -1;
    unsigned long lastMs = 0;
    bool valid = false;
};

DistanceState dist;

void updateDistance() {
    static unsigned long lastPing = 0;

    if (millis() - lastPing < 60) return;
    lastPing = millis();

    long d = getDistanceStable();
    if (d > 0) {
        dist.cm = d;
        dist.lastMs = millis();
        dist.valid = true;
    } else {
        if (millis() - dist.lastMs > 300) dist.valid = false;
    }
}

bool distanceUsable() {
    if (!dist.valid) return false;
    if (millis() - motionChangedMs < 80) return false;
    if (motion == TURNING) return false;
    return true;
}

void debugDistance() {
  Serial.print("Dist: ");
  Serial.print(dist.cm);
  Serial.print(" cm  valid=");
  Serial.print(dist.valid);
  Serial.print(" age=");
  Serial.println(millis() - dist.lastMs);
}

// Scanning function
void performScan() {
    static const int scanAngles[SCAN_STEPS] = {-180, -135, -90, -45, 0, 45, 90, 135};

    stopWheels();
    delay(50);

    int currentAngle = 0;

    for (int i = 0; i < SCAN_STEPS; i++) {
        int target = scanAngles[i];
        int delta = target - currentAngle;

        setMotion(TURNING);
        turnDegrees(delta);
        setMotion(STOPPED);

        delay(80); // let vibrations settle

        long d = getAverageDistance();
        distances[i] = d;

        Serial.print("Scan angle ");
        Serial.print(target);
        Serial.print(" => ");
        Serial.println(d);

        currentAngle = target;
    }

    // return to forward (0°)
    if (currentAngle != 0) {
        setMotion(TURNING);
        turnDegrees(-currentAngle);
        setMotion(STOPPED);
    }

    Serial.println("Scan complete.");
}


void quick_scan() {
    long distance = 0;

    stopWheels();
    for (int i = 0; i < SCAN_STEPS; i+=2) {
        turnDegrees(SCAN_ANGLE * 2);
        distance = getAverageDistance();
        distances[i] = distance;
        debugDistance();
        stopWheels();
    }

    for (int i = 1; i < SCAN_STEPS; i+=2) {
        distances[i] = -1;
    }

    Serial.println("Quick scan complete.");
    // print all values
    for (int i = 0; i < SCAN_STEPS; i++) {
        Serial.print("Angle ");
        Serial.print((i - SCAN_STEPS / 2) * SCAN_ANGLE);
        Serial.print(": ");
        Serial.println(distances[i]);
    }
}

void selectBestDirection() {
    static const int scanAngles[SCAN_STEPS] = {-180, -135, -90, -45, 0, 45, 90, 135};

    bestAngle = -1;
    bestDistance = -1;

    int bestIndexFront = -1;
    long bestFrontDist = -1;
    int bestIndexAny = -1;
    long bestAnyDist = -1;

    for (int i = 0; i < SCAN_STEPS; i++) {
        long d = distances[i];
        if (d <= 0) continue;

        if (d > bestAnyDist) {
            bestAnyDist = d;
            bestIndexAny = i;
        }

        int a = scanAngles[i];
        if (a >= -90 && a <= 90 && d > bestFrontDist) {
            bestFrontDist = d;
            bestIndexFront = i;
        }
    }

    int chosen = (bestIndexFront != -1) ? bestIndexFront : bestIndexAny;

    if (chosen != -1) {
        bestAngle = scanAngles[chosen];
        bestDistance = distances[chosen];
        Serial.print("Best angle selected: ");
        Serial.print(bestAngle);
        Serial.print("  dist=");
        Serial.println(bestDistance);
    } else {
        bestAngle = 180;
        bestDistance = -1;
        Serial.println("No valid direction found.");
    }
}

// Alignment function
void alignToBestDirection() {
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
        Serial.print("Aligning to best angle: ");
        Serial.println(bestAngle);
        turnDegrees(bestAngle);
    } else {
        Serial.println("No valid direction to align.");
    }
}

// turn x amount of degrees
void turnDegrees(int degrees) {
    if (degrees == 0) return;

    int dir = (degrees > 0) ? +1 : -1;      // +1 = right, -1 = left
    float a  = abs(degrees) / 90.0f;

    int tFast = (int)(297 * a);
    int tSlow = (int)(40  * a);

    if (tFast < 10) tFast = 10;
    if (tSlow < 8)  tSlow = 8;

    spinDelta(dir * TURN_FAST_DELTA);
    delay(tFast);
    spinDelta(dir * TURN_SLOW_DELTA);
    delay(tSlow);
    spinDelta(-dir * TURN_BRAKE_DELTA);
    delay(TURN_BRAKE_MS);

    stopWheels();
    delay(TURN_SETTLE_MS);
}

void stopWheels() {
    leftWheel.writeMicroseconds(LEFT_STOP);
    rightWheel.writeMicroseconds(RIGHT_STOP);
}

void driveDelta(int delta) {
    setWheelsUS(LEFT_STOP + delta + LEFT_TRIM, RIGHT_STOP - delta + RIGHT_TRIM);
}

void spinDelta(int delta) {
    setWheelsUS(LEFT_STOP + delta + LEFT_TRIM, RIGHT_STOP + delta + RIGHT_TRIM);
}

void setWheelsUS(int leftUS, int rightUS) {
    leftWheel.writeMicroseconds(leftUS);
    rightWheel.writeMicroseconds(rightUS);
}

// Move forward function
void moveForwardDistance(int cm) {
  long t = (long)cm * DISTANCE_DELTA;

  driveDelta(+FWD_FAST_DELTA);
  delay(t);

  driveDelta(-FWD_BRAKE_DELTA);
  delay(FWD_BRAKE_MS);

  stopWheels();
  delay(FWD_SETTLE_MS);
}

void moveForward() {
    driveDelta(DRIVE_DELTA);
}

void turn90right() {
    spinDelta(+TURN_FAST_DELTA);
    delay(297);
    spinDelta(+TURN_SLOW_DELTA);
    delay(40);
    spinDelta(-TURN_BRAKE_DELTA);
    delay(TURN_BRAKE_MS);
    stopWheels();
    delay(TURN_SETTLE_MS);
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
    stopWheels();
}
void Algorithm2Loop() {
    updateDistance();

    static Algo2State st = A2_FORWARD;

    static unsigned long stateStartMs = 0;
    auto enter = [&](Algo2State next) {
        st = next;
        stateStartMs = millis();
    };
    auto inStateMs = [&]() -> unsigned long { return millis() - stateStartMs; };

    const int CAUTION_CM   = OBSTACLE_CM + 8;
    const int EMERGENCY_CM = 6;
    const unsigned long CONFIRM_MS = 180;
    const unsigned long BRAKE_MS   = 80;
    const unsigned long TURN90_MS  = 420;
    const unsigned long TURNMAX_MS = 650;
    const unsigned long BACKUP_MS  = 800;
    const unsigned long STUCK_MS   = 900;

    static long lastCm = -1;
    static unsigned long lastProgressMs = 0;

    if (st == A2_FORWARD && distanceUsable()) {
        long cm = dist.cm;

        if (cm > 0 && cm < 150) {
            if (lastCm < 0) {
                lastCm = cm;
                lastProgressMs = millis();
            } else {
                if (abs(cm - lastCm) >= 2) {
                    lastProgressMs = millis();
                    lastCm = cm;
                }
            }
        } else {
            lastCm = -1;
            lastProgressMs = millis();
        }
    }

    bool usable = distanceUsable() && dist.cm > 0;
    long cm = usable ? dist.cm : 9999;

    static unsigned long lastObsHitMs = 0;
    static uint8_t obsHits = 0;

    bool emergency = usable && (cm <= EMERGENCY_CM);
    bool rawObstacle = usable && (cm < OBSTACLE_CM);
    bool caution = usable && (cm < CAUTION_CM);

    if (rawObstacle) {
        if (millis() - lastObsHitMs <= CONFIRM_MS) obsHits++;
        else obsHits = 1;
        lastObsHitMs = millis();
    } else {
        if (millis() - lastObsHitMs > CONFIRM_MS) obsHits = 0;
    }

    bool obstacleConfirmed = emergency || (obsHits >= 2);

    bool stuck = (st == A2_FORWARD) && (millis() - lastProgressMs > STUCK_MS);

    static int consecutiveObstacles = 0;

    static bool actionStarted = false;

    switch (st) {

        case A2_FORWARD: {
            setMotion(MOVING_FWD);

            if (caution) {
                moveForward();
                if (inStateMs() > 0) { /* noop */ }
                if (caution && !obstacleConfirmed) {
                    stopWheels();
                    setMotion(STOPPED);
                    enter(A2_RECOVER_FORWARD);
                }
                break;
            }

            moveForward();

            if (obstacleConfirmed) {
                stopWheels();
                setMotion(STOPPED);

                consecutiveObstacles++;

                if (consecutiveObstacles >= 2 || emergency) {
                    enter(A2_BACKUP);
                } else {
                    enter(A2_TURN_90);
                }
            }
            else if (stuck) {
                stopWheels();
                setMotion(STOPPED);
                consecutiveObstacles = 2;
                enter(A2_BACKUP);
            }
            else {
                if (consecutiveObstacles > 0 && !rawObstacle) consecutiveObstacles--;
            }
            break;
        }

        case A2_TURN_90: {
            setMotion(TURNING);

            if (!actionStarted) {
                actionStarted = true;
                turnDegrees(90);
            }

            if (emergency) {
                stopWheels();
                setMotion(STOPPED);
                actionStarted = false;
                enter(A2_BACKUP);
                break;
            }

            if (inStateMs() >= TURN90_MS) {
                stopWheels();
                setMotion(STOPPED);
                actionStarted = false;

                enter(A2_RECOVER_FORWARD);
            }
            break;
        }

        case A2_BACKUP: {
            setMotion(REVERSING);

            if (!actionStarted) {
                actionStarted = true;
                driveDelta(-DRIVE_DELTA);
            }

            if (inStateMs() >= BACKUP_MS) {
                stopWheels();
                setMotion(STOPPED);
                actionStarted = false;
                enter(A2_SCAN);
            }
            break;
        }

        case A2_SCAN: {
            setMotion(TURNING);

            stopWheels();
            delay(30);

            performScan();
            selectBestDirection();

            setMotion(STOPPED);
            enter(A2_TURN_TO_BEST);
            break;
        }

        case A2_TURN_TO_BEST: {
            setMotion(TURNING);

            if (!actionStarted) {
                actionStarted = true;
                int a = (int)bestAngle;
                if (a > 160) a = 160;
                if (a < -160) a = -160;
                turnDegrees(a);
            }

            if (emergency) {
                stopWheels();
                setMotion(STOPPED);
                actionStarted = false;
                enter(A2_BACKUP);
                break;
            }

            if (inStateMs() >= TURNMAX_MS) {
                stopWheels();
                setMotion(STOPPED);
                actionStarted = false;
                enter(A2_RECOVER_FORWARD);
            }
            break;
        }

        case A2_RECOVER_FORWARD: {
            setMotion(STOPPED);

            if (inStateMs() < BRAKE_MS) {
                stopWheels();
                break;
            }

            updateDistance();
            bool stillBlocked = distanceUsable() && dist.cm > 0 && dist.cm < OBSTACLE_CM;

            if (stillBlocked) {
                consecutiveObstacles++;
                if (consecutiveObstacles >= 2) enter(A2_BACKUP);
                else enter(A2_TURN_90);
                break;
            }

            setMotion(MOVING_FWD);
            moveForward();

            if (inStateMs() >= (BRAKE_MS + 180)) {
                stopWheels();
                setMotion(STOPPED);

                lastCm = -1;
                lastProgressMs = millis();

                enter(A2_FORWARD);
            }
            break;
        }
    }
}

void Algorithm1Loop() {    static unsigned long lastPing = 0;
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
            driveDelta(-DRIVE_DELTA);
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
        delay(1000); // placeholder
    } else if (mode == ALGORITHM2) {
        Algorithm2Loop();
    } else if (mode == CALIBRATION) {
        moveForward();
        delay(1000);
    }
}
