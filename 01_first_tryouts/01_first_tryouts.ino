#include "MeMegaPi.h"

// Fahr-Motoren an PORT3 (= SLOT3) und PORT4 (= SLOT4)
MeEncoderOnBoard driveMotorB(SLOT3);
MeEncoderOnBoard driveMotorC(SLOT4);

// Lenk-Motor an PORT1 (= SLOT1)
MeEncoderOnBoard steeringMotorB(SLOT1);

// Linienfolger-Sensor (Sensor 1 = links, Sensor 2 = rechts)
MeLineFollower lineFinder(PORT_5);

// Fahr-Geschwindigkeit in U/min für den Encoder-Motor
const float DRIVE_SPEED_RPM = 15;

// Lenk-Geschwindigkeit in U/min für den Encoder-Motor
const float STEERING_SPEED_RPM = 250;

// Lenkwinkel in Grad pro Korrekturschritt sowie der max. Lenkeinschlag.
const long STEERING_STEP_DEG = 300;
const long STEERING_MAX_ANGLE_DEG = 360;

// Timeout für max. Lenkung
const unsigned long STEERING_TIMEOUT_MS = 2000;

// Aktuell angefahrener Soll-Lenkwinkel in Grad (negativ = rechts; positiv = links).
long steeringAngle = 0;

// Interrupt Service Routinen für die Encoder (Pulszählung, von der Library benötigt)
void isr_process_driveEncoder() {
  if (digitalRead(driveMotorB.getPortB()) == 0) {
    driveMotorB.pulsePosMinus();
  } else {
    driveMotorB.pulsePosPlus();
  }
}

void isr_process_driveEncoderC() {
  if (digitalRead(driveMotorC.getPortB()) == 0) {
    driveMotorC.pulsePosMinus();
  } else {
    driveMotorC.pulsePosPlus();
  }
}

void isr_process_steeringEncoder() {
  if (digitalRead(steeringMotorB.getPortB()) == 0) {
    steeringMotorB.pulsePosMinus();
  } else {
    steeringMotorB.pulsePosPlus();
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("=== Makeblock MegaPi gestartet ===");

  attachInterrupt(driveMotorB.getIntNum(), isr_process_driveEncoder, RISING);
  attachInterrupt(driveMotorC.getIntNum(), isr_process_driveEncoderC, RISING);
  attachInterrupt(steeringMotorB.getIntNum(), isr_process_steeringEncoder, RISING);

  // Standard-Encoder-Setup von Makeblock
  // PWM-Frequenz auf Timer1 (Lenkung, Pin 12), Timer2 (Fahr-Motor SLOT3, Pin 9)
  // und Timer3 (Fahr-Motor SLOT4, Pin 5) hochsetzen
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  TCCR3A = _BV(WGM30);
  TCCR3B = _BV(CS31) | _BV(WGM32);

  // Fahr-Motoren-Aufdruck (beide baugleich): 185 RPM, Ratio 1:46
  driveMotorB.setPulse(7);
  driveMotorB.setRatio(46);
  driveMotorB.setSpeedPid(0.18, 0, 0);

  driveMotorC.setPulse(7);
  driveMotorC.setRatio(46);
  driveMotorC.setSpeedPid(0.18, 0, 0);

  // Lenkmotor-Aufdruck: 86 RPM, Ratio 1:75 (abweichend von den Fahr-Motoren)
  steeringMotorB.setPulse(7);
  steeringMotorB.setRatio(75);
  steeringMotorB.setPosPid(1.8, 0, 1.2);
  steeringMotorB.setSpeedPid(0.18, 0, 0);
  // Deadband verkleinert (Default 10°): moveTo()'s Abbremsrampe bremst ab
  // (pos_error - deadband) ab, ein kleineres Deadband hält die Lenkung
  // länger auf Geschwindigkeit, bevor sie kurz vor dem Ziel abbremst.
  steeringMotorB.setPosDeadBand(3);

  steeringMotorB.moveTo(0, STEERING_SPEED_RPM);
  waitForSteering();
}

void loop() {
  delayWithMotorUpdate(200);
  followLine();
}

// Ruft die Encoder-PID beider Motoren auf; muss regelmäßig laufen, sonst wird PWM nicht aktualisiert.
void updateMotors() {
  driveMotorB.loop();
  driveMotorC.loop();
  steeringMotorB.loop();
}

// Wartet wie delay(), hält aber die Encoder-PID am Laufen statt zu blockieren.
void delayWithMotorUpdate(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    updateMotors();
  }
}

void followLine() {
  int sensorState = lineFinder.readSensors();
  switch (sensorState) {
    case S1_IN_S2_IN:
      Serial.println("Beide Sensoren auf der Linie: stoppen");
      stopDriveMotor();
      driveBackward();
      delayWithMotorUpdate(2000);
      stopDriveMotor();
      steerRight();

      break;

    case S1_IN_S2_OUT:
      Serial.println("Linker Sensor (S1) auf der Fahrbahnlinie: zu weit links, Lenkung nach rechts einschlagen");
      steerLeft();
      driveBackward();
      delayWithMotorUpdate(2000);
      stopDriveMotor();
      steerStraight();
      //steerRight();
      driveForwardHalf();
      break;

    case S1_OUT_S2_IN:
      Serial.println("Rechter Sensor (S2) auf der Fahrbahnlinie: zu weit rechts, Lenkung nach links einschlagen");
      steerRight();
      driveBackward();
      delayWithMotorUpdate(2000);
      stopDriveMotor();
      steerStraight();
      //steerLeft();
      driveForwardHalf();
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

void driveForwardHalf() {
  driveMotorB.runSpeed(DRIVE_SPEED_RPM / 2);
  driveMotorC.runSpeed(DRIVE_SPEED_RPM / 2);
}

void driveForward() {
  driveMotorB.runSpeed(DRIVE_SPEED_RPM);
  driveMotorC.runSpeed(DRIVE_SPEED_RPM);
}

void driveBackward() {
  driveMotorB.runSpeed(-DRIVE_SPEED_RPM);
  driveMotorC.runSpeed(-DRIVE_SPEED_RPM);
}

void stopDriveMotor() {
  driveMotorB.runSpeed(0);
  driveMotorC.runSpeed(0);
}

// Fährt den Soll-Lenkwinkel um deltaDeg weiter und wartet, bis der Encoder-Motor
// die Zielposition tatsächlich erreicht hat (closed-loop statt Zeitschätzung).
void steer(long deltaDeg) {
  stopDriveMotor();
  steeringAngle = constrain(steeringAngle + deltaDeg, -STEERING_MAX_ANGLE_DEG, STEERING_MAX_ANGLE_DEG);
  steeringMotorB.moveTo(steeringAngle, STEERING_SPEED_RPM);
  waitForSteering();
  Serial.print("Lenkung Soll: ");
  Serial.print(steeringAngle);
  Serial.print(" Ist: ");
  Serial.println(steeringMotorB.getCurPos());
  driveForward();
  delayWithMotorUpdate(500);
  stopDriveMotor();
}

// Blockiert, bis der Lenkmotor den zuletzt per moveTo() gesetzten Zielwinkel erreicht hat.
void waitForSteering() {
  unsigned long start = millis();
  while (!steeringMotorB.isTarPosReached() && millis() - start < STEERING_TIMEOUT_MS) {
    updateMotors();
  }
}

void steerLeft() {
  steer(STEERING_STEP_DEG);
}

void steerRight() {
  steer(-STEERING_STEP_DEG);
}

// Stellt die Lenkung direkt auf den bekannten Mittelpunkt (0 Grad) zurück.
void steerStraight() {
  //stopDriveMotor();
  steeringAngle = 0;
  steeringMotorB.moveTo(steeringAngle, STEERING_SPEED_RPM);
  waitForSteering();
}
