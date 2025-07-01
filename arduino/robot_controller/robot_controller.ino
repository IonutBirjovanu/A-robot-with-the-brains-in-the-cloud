#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// PCA9686 setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
const uint16_t SERVOMIN = 150;    // pulse for 0°
const uint16_t SERVOMAX = 600;    // pulse for 180°
const uint8_t NUM_SERVOS = 18;

// Each leg: [hip, shoulder, elbow]
const uint8_t legServos[4][3] = {
  {0, 1, 2},   // FL
  {3, 5, 6},   // FR
  {8, 9, 10},   // RL
  {12, 13, 14}  // RR
};

// The offsets for each leg servo
uint8_t legOffsetsCrouched[4][3] = {
  {40, 50, 65},  // FL
  {100, 90, 80},  // FR
  {60, 100, 90},  // RL
  {80, 60, 70}   // RR
};

// The offsets for each leg servo
uint8_t legOffsets[4][3] = {
  {90, 70, 65},  // FL
  {50, 70, 80},  // FR
  {90, 90, 90},  // RL
  {50, 70, 70}   // RR
};

uint8_t currentAngles[NUM_SERVOS] = {0, 15, 65, 140, 0, 130, 80, 0, 0, 140, 80, 0, 140, 10, 70};  // Stores current angle for each servo

// Map angle to PWM pulse
uint16_t angleToPulse(uint8_t angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// Move a servo to its target angle smoothly
void sweepServoToAngle(uint8_t servoIndex, uint8_t targetAngle, int duration_ms) {
  uint8_t current = currentAngles[servoIndex];
  int stepCount = abs(targetAngle - current);
  if (stepCount == 0) return;

  int stepDelay = duration_ms / stepCount;
  int direction = (targetAngle > current) ? 1 : -1;

  for (int angle = current; angle != targetAngle; angle += direction) {
    pwm.setPWM(servoIndex, 0, angleToPulse(angle));
    delay(stepDelay);
  }
  pwm.setPWM(servoIndex, 0, angleToPulse(targetAngle));  // Ensure exact final position
  currentAngles[servoIndex] = targetAngle;  // Update tracker
}

// Directly set absolute servo angles for one leg
void setLegDirectAngles(uint8_t legIndex, uint8_t hipAng, uint8_t shAng, uint8_t elAng) {
  pwm.setPWM(legServos[legIndex][0], 0, angleToPulse(hipAng));
  pwm.setPWM(legServos[legIndex][1], 0, angleToPulse(shAng));
  pwm.setPWM(legServos[legIndex][2], 0, angleToPulse(elAng));

  int id = legIndex * 4;

  currentAngles[id] = hipAng;
  currentAngles[id + 1] = shAng;
  currentAngles[id + 2] = elAng;

}

// Set servo angles for one leg using the sweeping function
void setLegAngles(uint8_t legIndex, uint8_t hipAng, uint8_t shAng, uint8_t elAng, int duration_ms = 100) {
  sweepServoToAngle(legServos[legIndex][0], hipAng, duration_ms);
  sweepServoToAngle(legServos[legIndex][1], shAng, duration_ms);
  sweepServoToAngle(legServos[legIndex][2], elAng, duration_ms);
}

// Set servo angles relative to per-leg offsets
void setLegRelativeAngles(uint8_t legIndex, int8_t hipDelta, int8_t shDelta, int8_t elDelta) { // They start off from the stasnding position
  setLegAngles(
    legIndex,
    constrain(legOffsets[legIndex][0] + hipDelta, 0, 180),
    constrain(legOffsets[legIndex][1] + shDelta, 0, 180),
    constrain(legOffsets[legIndex][2] + elDelta, 0, 180)
  );
}

void standPosition() {
  Serial.println("Standing still.");

  for (int i = 0; i < 4; i++) {
    setLegAngles(i,
      legOffsets[i][0],
      legOffsets[i][1],
      legOffsets[i][2]
    );
  }

  delay(100);
}

void crouchPosition() {
  Serial.println("Crouching.");

  for (int i = 0; i < 4; i++) {
    setLegAngles(i,
      legOffsetsCrouched[i][0],
      legOffsetsCrouched[i][1],
      legOffsetsCrouched[i][2]
    );
  }

  delay(100);
}

void powerUpPosition() {

  setLegDirectAngles(0, currentAngles[0], currentAngles[1], currentAngles[2]);
  setLegDirectAngles(1, currentAngles[4], currentAngles[5], currentAngles[6]);

  delay(1000);

  setLegDirectAngles(2, currentAngles[8], currentAngles[9], currentAngles[10]);
  setLegDirectAngles(3, currentAngles[12], currentAngles[13], currentAngles[14]);

  delay(100);
}

void layFlatPosition() {      // Set all the legs in their crouched position
  crouchPosition();           // Added a buffer position
  delay(100);
  
  Serial.println("Laying flat.");

  setLegAngles(0, 0, 15, 65);     //FL
  setLegAngles(1, 140, 130, 80);  //FR
  setLegAngles(2, 0, 140, 80);    //RL
  setLegAngles(3, 140, 10, 70);   //RR

  delay(1000);
}

void sitPosition() {          // The front legs are in the standing position, and the back ones are crouching
  crouchPosition();           // Added a buffer position
  delay(100);
  
  Serial.println("Sitting.");

  setLegAngles(0, 50, 50, 65);    //FL
  setLegAngles(1, 90, 90, 80);    //FR
  setLegAngles(2, 0, 140, 80);    //RL
  setLegAngles(3, 140, 10, 70);   //RR

  delay(1000);
}

void pawPosition() {          // Same as sitting, but with the left foot stretched
  crouchPosition();           // Added a buffer position
  delay(100);
  
  Serial.println("Holding paw.");

  setLegAngles(2, 0, 140, 80);    //RL
  setLegAngles(3, 140, 10, 70);   //RR
  setLegAngles(1, 100, 90, 65);   //FR
  setLegAngles(0, 120, 140, 75);  //FL

  delay(1000);
}

void salutePosition() {          // Same as the paw position, but also waving
  crouchPosition();           // Added a buffer position
  delay(100);
  
  Serial.println("Greeting.");

  setLegAngles(2, 0, 140, 80);    //RL
  setLegAngles(3, 140, 10, 70);   //RR
  setLegAngles(1, 100, 90, 65);   //FR
  setLegAngles(0, 120, 140, 75);  //FL

  delay(200);

  setLegAngles(0, 120, 140, 75);  //FL
  delay(100);
  setLegAngles(0, 100, 140, 75);  //FL
  delay(100);
  setLegAngles(0, 130, 140, 75);  //FL
  delay(100);
  setLegAngles(0, 100, 140, 75);  //FL
  delay(100);
  setLegAngles(0, 130, 140, 75);  //FL

  delay(1000);

  setLegAngles(0, 50, 50, 65);    //FL
  setLegAngles(1, 90, 90, 80);    //FR
}

void stepForwardCycle() {
  // Phase 1: Lift FL
  setLegRelativeAngles(0, -10, -10, -10);
  delay(100);

  // Phase 2: Swing FL forward
  setLegRelativeAngles(0, -10, 20, 0);
  delay(100);

  // Phase 3: Place FL down
  setLegRelativeAngles(0, 10, 20, 0);
  delay(100);

  // Phase 4: Lift FR
  setLegRelativeAngles(1, 10, 10, 10);
  delay(100);

  // Phase 5: Swing FR forward
  setLegRelativeAngles(1, 10, -20, 0);
  delay(100);

  // Phase 6: Place FR down
  setLegRelativeAngles(1, -10, -20, 0);
  delay(100);

  // Phase 7: Drag FR + FL + RR + RL backward
  setLegRelativeAngles(0, 0, 0, 0);
  setLegRelativeAngles(1, 0, 0, 0);
  setLegRelativeAngles(2, 20, 15, 0);
  setLegRelativeAngles(3, -20, -15, 0);
  delay(100);

  // Phase 8: Lift RR
  setLegRelativeAngles(3, 20, -15, -10);
  delay(100);

  // Phase 9: Place RR down
  setLegRelativeAngles(3, 0, 0, 0);
  delay(100);

  // Phase 10: Lift RL
  setLegRelativeAngles(2, -20, 15, 10);
  delay(100);

  // Phase 11: Place RL down
  setLegRelativeAngles(2, 0, 0, 0);
  delay(100);

}

void stepBackwardCycle() {
  // Phase 1: Lift FL
  setLegRelativeAngles(0, -10, -10, -10);
  delay(100);

  // Phase 2: Swing FL backward
  setLegRelativeAngles(0, -10, -20, 0);
  delay(100);

  // Phase 3: Place FL down
  setLegRelativeAngles(0, 10, -20, 0);
  delay(100);

  // Phase 4: Lift FR
  setLegRelativeAngles(1, 10, 10, 10);
  delay(100);

  // Phase 5: Swing FR backward
  setLegRelativeAngles(1, 10, 20, 0);
  delay(100);

  // Phase 6: Place FR down
  setLegRelativeAngles(1, -10, 20, 0);
  delay(100);

  // Phase 7: Drag FR + FL + RR + RL forward
  setLegRelativeAngles(0, 0, 0, 0);
  setLegRelativeAngles(1, 0, 0, 0);
  setLegRelativeAngles(2, -20, -15, 0);
  setLegRelativeAngles(3, 20, 15, 0);
  delay(100);

  // Phase 8: Lift RR
  setLegRelativeAngles(3, -20, 15, -10);
  delay(100);

  // Phase 9: Place RR down
  setLegRelativeAngles(3, 0, 0, 0);
  delay(100);

  // Phase 10: Lift RL
  setLegRelativeAngles(2, 20, -15, 10);
  delay(100);

  // Phase 11: Place RL down
  setLegRelativeAngles(2, 0, 0, 0);
  delay(100);
}

void turnRightCycle() {
  // Phase 1: Lift FR
  setLegRelativeAngles(1, 10, 10, 20);
  delay(100);

  // Phase 2: Place FR down
  setLegRelativeAngles(1, 0, 0, 10);
  delay(100);

  // Phase 3: Lift FL
  setLegRelativeAngles(0, -20, -15, 10);
  delay(100);

  // Phase 4: Place FL down
  setLegRelativeAngles(0, 0, 0, 10);
  delay(100);

  // Phase 5: Drag FL + FR left
  setLegRelativeAngles(0, 0, 0, 0);
  setLegRelativeAngles(1, 0, 0, 0);
  delay(100);

  // Phase 6: Lift RR
  setLegRelativeAngles(3, 20, -15, -10);
  delay(100);

  // Phase 7: Place RR down
  setLegRelativeAngles(3, 0, 0, 0);
  delay(100);

  // Phase 8: Lift RL
  setLegRelativeAngles(2, -20, 15, 10);
  delay(100);

  // Phase 9: Place RL down
  setLegRelativeAngles(2, 0, 0, 0);
  delay(100);
}

void turnLeftCycle() {
  // Phase 1: Lift FL
  setLegRelativeAngles(0, -10, -10, -20);
  delay(100);

  // Phase 2: Place FL down
  setLegRelativeAngles(0, 0, 0, -10);
  delay(100);
  
  // Phase 3: Lift FR
  setLegRelativeAngles(1, 20, 15, -10);
  delay(100);

  // Phase 4: Place FR down
  setLegRelativeAngles(1, 0, 0, -10);
  delay(100);

  // Phase 5: Drag FL + FR right
  setLegRelativeAngles(0, 0, 0, 0);
  setLegRelativeAngles(1, 0, 0, 0);
  delay(100);

  // Phase 6: Lift RR
  setLegRelativeAngles(3, 20, -15, -10);
  delay(100);

  // Phase 7: Place RR down
  setLegRelativeAngles(3, 0, 0, 0);
  delay(100);

  // Phase 8: Lift RL
  setLegRelativeAngles(2, -20, 15, 10);
  delay(100);

  // Phase 9: Place RL down
  setLegRelativeAngles(2, 0, 0, 0);
  delay(100);
}

void moveForward(int steps) {
  standPosition();
  delay(100);
  for (int i = 0; i < steps; i++) {
    Serial.println("Moving forward.");
    stepForwardCycle();
  }
  standPosition();
  delay(100);
}

void moveBackward(int steps) {
  standPosition();
  delay(100);
  for (int i = 0; i < steps; i++) {
    Serial.println("Moving backward.");
    stepBackwardCycle();
  }
  standPosition();
  delay(100);
}

void moveLeft(int steps) {
  standPosition();
  delay(100);
  for (int i = 0; i < steps; i++) {
    Serial.println("Turning left.");
    turnLeftCycle();
  }
  standPosition();
  delay(100);
}

void moveRight(int steps) {
  standPosition();
  delay(100);
  for (int i = 0; i < steps; i++) {
    Serial.println("Turning right.");
    turnRightCycle();
  }
  standPosition();
  delay(100);
}

void setup() {
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);

  delay(1000);
  powerUpPosition();  // the powering up sequence
  delay(2000);

  Serial.begin(115200);
  while (!Serial);
  Serial.println("Quadruped robot controller ready.");

  delay(2000);

  standPosition();
}

void loop() {
  // Accept commands via Serial Monitor for now
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    if (line.startsWith("move,")) {
      int first = line.indexOf(',');
      int second = line.indexOf(',', first + 1);
      String dir  = line.substring(first + 1, second);
      int steps   = line.substring(second + 1).toInt();

      if (dir == "forward") moveForward(steps);
      else if (dir == "backward") moveBackward(steps);
      else if (dir == "left") moveLeft(steps);
      else if (dir == "right") moveRight(steps);
      else if (dir == "stand") standPosition();
      else if (dir == "flat") layFlatPosition();
      else if (dir == "sit") sitPosition();
      else if (dir == "paw") pawPosition();
      else if (dir == "greet") salutePosition();
      else standPosition();

      Serial.println("Finished instruction.");
    }

    // Testing direct servo control from Serial:
    else {
      int start = 0;
      while (start < line.length()) {
        int colon = line.indexOf(':', start);
        int space = line.indexOf(' ', start);
        if (space == -1) space = line.length();

        if (colon > start && colon < space) {
          int idx = line.substring(start, colon).toInt();
          int ang = line.substring(colon + 1, space).toInt();
          if (idx >= 0 && idx < NUM_SERVOS && ang >= 0 && ang <= 180) {
            pwm.setPWM(idx, 0, angleToPulse(ang));
            currentAngles[idx] = ang;
          }
        }
        start = space + 1;
      }
      Serial.println("Finished instruction.");
    }
    
  }
}


