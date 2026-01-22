#include "PuckSpinner.hpp"

#define LEFT_SERVO_PIN   5
#define RIGHT_SERVO_PIN  6
#define TRIG_PIN         9
#define ECHO_PIN         10

#define SWITCH_PIN       2

#define LEFT_STOP   1418
#define RIGHT_STOP  1518

#define DRIVE_DELTA 600 // speed for moving forward / turning
#define OBSTACLE_CM 25 // stop if object closer than this
#define SCAN_ANGLE 45 // degrees to turn per scan step
#define SCAN_STEPS 8 // number of scan steps (360/45)
#define DISTANCE_DELTA 20 // milliseconds per cm, <<to be calibrated>>
#define SCAN_FORWARD_INDEX 0  // we'll tune this: 0..7

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
    stopWheels();
    delay(80);

    // Scan absolute headings in a continuous sweep: 0,45,90,...,315
    for (int i = 0; i < SCAN_STEPS; i++) {
        if (i > 0) {
            setMotion(TURNING);
            turnDegrees(SCAN_ANGLE);   // always turn the same way
            setMotion(STOPPED);
            delay(80);                 // settle after turning
        }

        long d = getAverageDistance(); // keep using your averaging function
        distances[i] = d;

        Serial.print("Scan abs ");
        Serial.print(i * SCAN_ANGLE);
        Serial.print(" => ");
        Serial.println(d);
    }

    // Complete the full circle back to forward (adds last 45°: 315->360/0)
    setMotion(TURNING);
    turnDegrees(SCAN_ANGLE);
    setMotion(STOPPED);

    Serial.println("Scan complete (360).");
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
    bestAngle = 180;      // safe fallback
    bestDistance = -1;

    // We want at least some clearance beyond OBSTACLE_CM
    const long MIN_CLEAR = OBSTACLE_CM + 10;

    long bestFrontDist = -1;
    int bestFrontRelAngle = 0;

    long bestAnyDist = -1;
    int bestAnyRelAngle = 180;

    for (int i = 0; i < SCAN_STEPS; i++) {
        long d = distances[i];
        if (d <= 0) continue;

        int absA = ((i - SCAN_FORWARD_INDEX + SCAN_STEPS) % SCAN_STEPS) * SCAN_ANGLE;
        int relA = (absA <= 180) ? absA : absA - 360;


        // Track best anywhere (as fallback)
        if (d > bestAnyDist) {
            bestAnyDist = d;
            bestAnyRelAngle = relA;
        }

        // Prefer front hemisphere and enforce minimum clearance
        if (abs(relA) <= 60 && d >= MIN_CLEAR) {
            // primary: max distance; secondary: closer to straight ahead
            if (d > bestFrontDist || (d == bestFrontDist && abs(relA) < abs(bestFrontRelAngle))) {
                bestFrontDist = d;
                bestFrontRelAngle = relA;
            }
        }
    }

    if (bestFrontDist > 0) {
        bestAngle = bestFrontRelAngle;
        bestDistance = bestFrontDist;
        Serial.print("Best FRONT angle: ");
        Serial.print(bestAngle);
        Serial.print("  dist=");
        Serial.println(bestDistance);
        return;
    }

    // If nothing safe in front, pick best anywhere IF it's at least valid,
    // otherwise keep fallback 180.
    if (bestAnyDist > 0) {
        bestAngle = bestAnyRelAngle;
        bestDistance = bestAnyDist;
        Serial.print("Best ANY angle: ");
        Serial.print(bestAngle);
        Serial.print("  dist=");
        Serial.println(bestDistance);
    } else {
        Serial.println("No valid direction found -> fallback 180");
        bestAngle = 180;
        bestDistance = -1;
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
        mode = CALIBRATION;
        Serial.println("Calibration");
    } else {
        mode = ALGORITHM2;
        Serial.println("Algorithm 2");
    }
    stopWheels();
}

/*
// Algo 1 main loop
void Algorithm1Loop() {
    switch (currentState) {
        case SCAN:
            Serial.println("Scanning...");
            // performScan();
            currentState = SELECT_DIRECTION;
            break;

        case QUICK_SCAN:
            Serial.println("Quick scanning...");
            //quick_scan();
            currentState = SELECT_DIRECTION;
            break;

        case SELECT_DIRECTION:
            Serial.println("Selecting direction...");
            //selectBestDirection();
            currentState = ALIGN;
            break;

        case ALIGN:
            Serial.println("Aligning...");
            //alignToBestDirection();
            currentState = MOVE_FORWARD;
            break;

        case MOVE_FORWARD:
            Serial.println("Moving forward...");
            //moveForwardDistance(bestDistance);
            break;

        case AVOID_STOP:
            Serial.println("Obstacle detected, stopping...");
            stopWheels();
            currentState = QUICK_SCAN;
            break;
    }
}*/

void Algorithm2Loop() {
    updateDistance();

    // ----------------- Tunables -----------------
    const int  EMERGENCY_CM        = 5;
    const int  OBSTACLE_ENTER_CM   = OBSTACLE_CM;
    const int  OBSTACLE_EXIT_CM    = OBSTACLE_CM + 10;
    const int  OBSTACLE_HITS_REQ   = 3;

    const unsigned long DIST_MAX_AGE_MS = 200;
    const unsigned long FWD_STABLE_MS   = 250;

    const unsigned long SCAN_COOLDOWN_MS = 5000;
    const unsigned long OBST_WINDOW_MS   = 2500;
    const int  OBST_IN_WINDOW_REQ = 5;

    const unsigned long BACKUP_MS = 700;
    const unsigned long RECOVER_FWD_MS = 250;
    const unsigned long SETTLE_AFTER_TURN_MS = 100;

    const int  AVOID_TURN_DEG = 45;
    const unsigned long STUCK_TIMEOUT_MS = 1700;

    // ----------------- State machine -----------------
    enum Algo2State {
        A2_FORWARD,
        A2_AVOID_TURN,
        A2_BACKUP,
        A2_SCAN,
        A2_TURN_TO_BEST,
        A2_RECOVER_FORWARD
    };

    static Algo2State st = A2_FORWARD;
    static unsigned long stateStartMs = 0;

    auto enter = [&](Algo2State next) {
        st = next;
        stateStartMs = millis();
    };
    auto inStateMs = [&]() -> unsigned long { return millis() - stateStartMs; };

    // ----------------- Fresh + forwardStable gating -----------------
    bool fresh = dist.valid && (millis() - dist.lastMs) < DIST_MAX_AGE_MS;
    bool forwardStable = (motion == MOVING_FWD) && (millis() - motionChangedMs) > FWD_STABLE_MS;

    long d = (fresh ? dist.cm : -1);

    // ----------------- Emergency detection (HARD rule) -----------------
    bool emergency = (fresh && forwardStable && d > 0 && d <= EMERGENCY_CM);

    // ----------------- Soft obstacle latch (persistence + hysteresis) -----------------
    static int  obstacleHits = 0;
    static bool obstacleLatched = false;

    if (fresh && forwardStable) {
        if (!obstacleLatched) {
            if (d > 0 && d <= OBSTACLE_ENTER_CM) obstacleHits++;
            else obstacleHits = 0;

            if (obstacleHits >= OBSTACLE_HITS_REQ) {
                obstacleLatched = true;
                obstacleHits = 0;
            }
        } else {
            if (d >= OBSTACLE_EXIT_CM) obstacleLatched = false;
        }
    } else {
        obstacleHits = 0;
    }

    bool obstacle = obstacleLatched;

    // ----------------- Circling detector -----------------
    static unsigned long windowStartMs = 0;
    static int obstaclesInWindow = 0;

    auto noteObstacleEvent = [&]() {
        unsigned long now = millis();
        if (now - windowStartMs > OBST_WINDOW_MS) {
            windowStartMs = now;
            obstaclesInWindow = 0;
        }
        obstaclesInWindow++;
    };

    // ----------------- Stuck detector (only when near something) -----------------
    static long lastCm = -1;
    static unsigned long lastProgressMs = 0;

    if (st == A2_FORWARD && fresh && forwardStable) {
        if (d > 0 && d < 80) {
            if (lastCm < 0) {
                lastCm = d;
                lastProgressMs = millis();
            } else if (abs(d - lastCm) >= 2) {
                lastProgressMs = millis();
                lastCm = d;
            }
        } else {
            lastCm = -1;
            lastProgressMs = millis();
        }
    }

    bool stuck = (st == A2_FORWARD) && (millis() - lastProgressMs > STUCK_TIMEOUT_MS);

    // ----------------- Scan cooldown -----------------
    static unsigned long lastScanMs = 0;
    bool canScan = (millis() - lastScanMs) > SCAN_COOLDOWN_MS;
    bool shouldScan = canScan && (stuck || obstaclesInWindow >= OBST_IN_WINDOW_REQ);

    // ----------------- Main behavior -----------------
    switch (st) {
        case A2_FORWARD: {
            setMotion(MOVING_FWD);
            moveForward();

            if (emergency) {
                stopWheels();
                setMotion(STOPPED);
                enter(A2_BACKUP);
                break;
            }

            if (obstacle) {
                stopWheels();
                setMotion(STOPPED);
                noteObstacleEvent();

                if (shouldScan) enter(A2_SCAN);
                else enter(A2_AVOID_TURN);
                break;
            }

            if (stuck && canScan) {
                stopWheels();
                setMotion(STOPPED);
                enter(A2_SCAN);
                break;
            }

            break;
        }

        case A2_AVOID_TURN: {
            static bool turnRight = true;

            setMotion(TURNING);
            turnDegrees(turnRight ? +AVOID_TURN_DEG : -AVOID_TURN_DEG);
            setMotion(STOPPED);
            delay(SETTLE_AFTER_TURN_MS);

            updateDistance();
            bool stillClose = (dist.valid && (millis() - dist.lastMs) < 250 && dist.cm > 0 && dist.cm <= OBSTACLE_ENTER_CM);
            if (stillClose) turnRight = !turnRight;
            else turnRight = !turnRight;

            enter(A2_RECOVER_FORWARD);
            break;
        }

        case A2_BACKUP: {
            setMotion(REVERSING);
            driveDelta(-DRIVE_DELTA);

            if (inStateMs() >= BACKUP_MS) {
                stopWheels();
                setMotion(STOPPED);
                enter(A2_SCAN);
            }
            break;
        }

        case A2_SCAN: {
            stopWheels();
            setMotion(STOPPED);
            delay(120);

            setMotion(TURNING);
            performScan();
            setMotion(STOPPED);

            selectBestDirection();
            lastScanMs = millis();

            obstaclesInWindow = 0;
            windowStartMs = millis();
            lastCm = -1;
            lastProgressMs = millis();

            enter(A2_TURN_TO_BEST);
            break;
        }

        case A2_TURN_TO_BEST: {
            setMotion(TURNING);
            turnDegrees((int)bestAngle);
            setMotion(STOPPED);
            delay(SETTLE_AFTER_TURN_MS);

            enter(A2_RECOVER_FORWARD);
            break;
        }

        case A2_RECOVER_FORWARD: {
            setMotion(MOVING_FWD);
            moveForward();

            if (inStateMs() >= RECOVER_FWD_MS) {
                stopWheels();
                setMotion(STOPPED);

                obstacleLatched = false;
                obstacleHits = 0;

                lastCm = -1;
                lastProgressMs = millis();

                enter(A2_FORWARD);
            }
            break;
        }
    }
}

// -------------- DEBUGGING --------------
void printCalMenu() {
  Serial.println();
  Serial.println("=== CALIBRATION MENU ===");
  Serial.println("d  -> stream distance (stable)");
  Serial.println("t  -> turn sign test (+45 then -45)");
  Serial.println("q  -> square test (4x 90 degrees)");
  Serial.println("s  -> scan only (print bins 0..315)");
  Serial.println("b  -> scan + best direction (prints bestAngle)");
  Serial.println("f  -> forward for 1s (check drift)");
  Serial.println("x  -> stop wheels");
  Serial.println("========================");
}

void calib_distance_stream() {
  Serial.println("Distance stream (press any key to stop)...");
  while (!Serial.available()) {
    updateDistance();
    if (dist.valid) {
      Serial.print("cm=");
      Serial.print(dist.cm);
      Serial.print(" age=");
      Serial.println(millis() - dist.lastMs);
    } else {
      Serial.println("cm=INVALID");
    }
    delay(120);
  }
  while (Serial.available()) Serial.read();
}

void calib_turn_sign_test() {
  Serial.println("TURN SIGN TEST:");
  Serial.println("Step 1: turnDegrees(+45). Robot should rotate RIGHT.");
  setMotion(TURNING);
  turnDegrees(+45);
  setMotion(STOPPED);
  delay(400);

  Serial.println("Step 2: turnDegrees(-45). Robot should return to original heading.");
  setMotion(TURNING);
  turnDegrees(-45);
  setMotion(STOPPED);

  Serial.println("If it doesn't return, your sign convention is wrong OR timing isn't symmetric.");
}

void calib_square_test() {
  Serial.println("SQUARE TEST: doing 4x 90 degrees. Should end near original heading.");
  for (int i = 0; i < 4; i++) {
    setMotion(TURNING);
    turnDegrees(90);
    setMotion(STOPPED);
    delay(500);
  }
  Serial.println("Done. If it's drifting a lot, your turn scaling isn't linear or you're slipping.");
}

void calib_scan_only() {
  Serial.println("SCAN ONLY TEST: place robot and do not touch it during scan.");
  setMotion(TURNING);
  performScan();
  setMotion(STOPPED);

  Serial.println("Scan bins:");
  for (int i = 0; i < SCAN_STEPS; i++) {
    Serial.print(i * SCAN_ANGLE);
    Serial.print(": ");
    Serial.println(distances[i]);
  }
}

void calib_scan_and_best() {
  Serial.println("SCAN+BEST TEST:");
  setMotion(TURNING);
  performScan();
  setMotion(STOPPED);

  selectBestDirection();

  Serial.print("bestAngle=");
  Serial.print(bestAngle);
  Serial.print("  bestDistance=");
  Serial.println(bestDistance);

  for (int i = 0; i < SCAN_STEPS; i++) {
    Serial.print(i * SCAN_ANGLE);
    Serial.print(": ");
    Serial.println(distances[i]);
  }
}

void calib_forward_1s() {
  Serial.println("Forward 1s (watch drift).");
  setMotion(MOVING_FWD);
  moveForward();
  delay(1000);
  stopWheels();
  setMotion(STOPPED);
  Serial.println("Done.");
}

void CalibrationLoop() {
  static bool first = true;
  if (first) { first = false; printCalMenu(); }

  if (Serial.available()) {
    char c = Serial.read();
    // flush extra chars
    while (Serial.available()) Serial.read();

    if      (c == 'd') calib_distance_stream();
    else if (c == 't') calib_turn_sign_test();
    else if (c == 'q') calib_square_test();
    else if (c == 's') calib_scan_only();
    else if (c == 'b') calib_scan_and_best();
    else if (c == 'f') calib_forward_1s();
    else if (c == 'x') { stopWheels(); setMotion(STOPPED); Serial.println("Stopped."); }
    else if (c == 'm') printCalMenu();
    else Serial.println("Unknown key. Press 'm' for menu.");
  }
}


void loop() {
    if (mode == ALGORITHM1) {
        //Algorithm1Loop();
        delay(1000); // placeholder
    } else if (mode == ALGORITHM2) {
        Algorithm2Loop();
    } else if (mode == CALIBRATION) {
        CalibrationLoop();
    }
}
