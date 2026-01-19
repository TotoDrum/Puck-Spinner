#include "PuckSpinner.hpp"

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
