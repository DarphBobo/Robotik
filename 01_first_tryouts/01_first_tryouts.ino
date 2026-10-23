#include "MeMegaPi.h"

// Fahr-Motor an PORT3
MeMegaPiDCMotor driveMotorB(PORT3B);

// Lenk-Motor an PORT1: steuert die Lenkung der Vorderachse
MeMegaPiDCMotor steeringMotorB(PORT1B);

// Linienfolger-Sensor (Sensor 1 = links, Sensor 2 = rechts)
MeLineFollower lineFinder(PORT_5);

// Encoder-Pins
const byte ENCODER_INTERRUPT_PIN = 18;
const byte ENCODER_SIGNAL_PIN = 31;
volatile long encoderPulseCount = 0;

// Motor-Geschwindigkeiten (-255 bis 255)
const int DRIVE_SPEED = 50;
const int DRIVE_SPEED_BACKWARDS = -50;
const int STEERING_SPEED = 150;

// Lenkdauer
const unsigned long STEERING_DURATION_MS = 1500;
// Rückstellschritt Dauer
const unsigned long SEARCH_STEP_DURATION_MS = 1500;

// Summe der bisherigen Lenkzeit in ms. (negativ = rechts; positiv = links)
// Wird in steer() entsprechend verrgrößert / verringer
long steeringBalance = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("=== Makeblock MegaPi gestartet ===");

  pinMode(ENCODER_INTERRUPT_PIN, INPUT_PULLUP);
  pinMode(ENCODER_SIGNAL_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_INTERRUPT_PIN), encoderISR, RISING);
}

void loop() {
  delay(200);
  followLine();
}

void followLine() {
  int sensorState = lineFinder.readSensors();
  switch (sensorState) {
    case S1_IN_S2_IN:
      Serial.println("Beide Sensoren auf der Linie: stoppen");
      stopDriveMotor();
      driveBackward();
      delay(2000);
      stopDriveMotor();
      steerRight();

      break;

    case S1_IN_S2_OUT:
      Serial.println("Rechter Sensor (S2) verlässt die Linie: Lenkung nach links einschlagen");
      steerLeft();
      driveForward();
      break;

    case S1_OUT_S2_IN:
      Serial.println("Linker Sensor (S1) verlässt die Linie: Lenkung nach rechts einschlagen");
      steerRight();
      driveForward();
      break;

    case S1_OUT_S2_OUT:
      Serial.println("Beide Sensoren außerhalb der Linie: Linie wird gesucht");
      steerStraight();
      driveForward();
      break;

    default:
      break;
  }
}

void driveForward() {
  driveMotorB.run(DRIVE_SPEED);
}

void driveBackward() {
  driveMotorB.run(DRIVE_SPEED_BACKWARDS);
}

void stopDriveMotor() {
  driveMotorB.stop();
}

// Schlägt die Lenkung mit speed für durationMs aus und aktualisiert steeringBalance um +/- durationMs
void steer(int speed, unsigned long durationMs) {
  stopDriveMotor();
  steeringMotorB.run(speed);
  steeringBalance += (speed > 0) ? (long)durationMs : -(long)durationMs;
  delay(durationMs);
  steeringMotorB.stop();
  driveForward();
  delay(500);
  stopDriveMotor();
}

void steerLeft() {
  steer(STEERING_SPEED, STEERING_DURATION_MS);
}

void steerRight() {
  steer(-STEERING_SPEED, STEERING_DURATION_MS);
}

// Stellt die Lenkung schrittweise gerade, bis steeringBalance wieder 0 ist.
void steerStraight() {
  stopDriveMotor();
  while (steeringBalance != 0) {
    if (steeringBalance > 0) {
      steer(-STEERING_SPEED, SEARCH_STEP_DURATION_MS);
    } else {
      steer(STEERING_SPEED, SEARCH_STEP_DURATION_MS);
    }
  }
}

// Interrupt Service Routine für die Encoder-Zählung (brauchen wir das? Encoder Zählung war im Beispiel weil wir einen Encoder Motor verwenden.
// Hier könnte irgendwie die tatsächliche Drehung ermittelt werden) --> evtl Umstellen auf MeEncoderOnBoard?
void encoderISR() {
  if (digitalRead(ENCODER_SIGNAL_PIN) > 0) {
    encoderPulseCount++;
  } else {
    encoderPulseCount--;
  }
}
