#include <Arduino.h>
#include <Servo.h>

// Functions to be used in main.cpp

// Scanner
void performScan();
long readDistanceCM();
long readDistanceCM_filtered();
long getDistanceStable();
void turnDegrees(int);
void turn90right();

// Movement
void moveForward();
void stopWheels();
void turnLeft();
void turnRight();
void turnDegrees(int);
