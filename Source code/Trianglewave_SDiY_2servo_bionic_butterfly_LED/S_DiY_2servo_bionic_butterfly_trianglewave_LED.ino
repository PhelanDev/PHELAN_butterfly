/*S-DiY RC Bionic Butterfly project  
 * -----------------------------------------------------------------------------
 * PROJECT: S-DiY 2 SERVO FLAP RC BIONIC BUTTERFLY 
 * AUTHOR:  [S-DiY channel/ TruongVanSu91]
 * DATE:    02/2026
 * COPYRIGHT (C) 2026. ALL RIGHTS RESERVED.
 * -----------------------------------------------------------------------------
 */

#include <Servo.h>
#include <PPMReader.h>

const byte interruptPin = 3;
const byte ledPin = 6;         
const byte channelAmount = 8;
PPMReader ppm(interruptPin, channelAmount);

Servo servoLeft, servoRight;

const int centerAngle = 90;    
const int flapAmplitude = 65;   
const int maxPitchTrim = 20;    

float phase = 0, perchPhase = 0, currentOffset = 0;
bool isPerchingState = false, wasFailsafe = true;

int lastPosL = 90, lastPosR = 90;
int lastChValues[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
const int ppmDeadband = 3; 

int ledDebounceCounter = 0;
const int ledThreshold = 5; 

unsigned long lastLogTime = 0;
const int logInterval = 1000; 

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);      
  digitalWrite(ledPin, LOW);    

  servoLeft.attach(9, 500, 2500);  
  servoRight.attach(10, 500, 2500);
  servoLeft.write(centerAngle);
  servoRight.write(centerAngle);
  
  Serial.println("--- BIONIC BUTTERFLY SYSTEM INITIALIZED ---");
}

int readPPMFiltered(byte ch) {
  int raw = ppm.latestValidChannelValue(ch + 1, 1500);
  if (abs(raw - lastChValues[ch]) > ppmDeadband) {
    lastChValues[ch] = raw;
  }
  return lastChValues[ch];
}

void loop() {
  unsigned long ch3Raw = ppm.latestValidChannelValue(3, 0); 
  bool isConnected = (ch3Raw != 0);

  if (!isConnected) {
    digitalWrite(ledPin, LOW); 
    processFailsafe();
  } else {
    int steering = readPPMFiltered(0); 
    int elevator = readPPMFiltered(1); 
    int throttle = readPPMFiltered(2);    
    int ch5Trim   = readPPMFiltered(4); 
    int rawCh6    = readPPMFiltered(5); 
    int ch7Switch = readPPMFiltered(6); 
    int ch8Raw    = readPPMFiltered(7); 

    if (ch8Raw > 1550) { if (ledDebounceCounter < ledThreshold) ledDebounceCounter++; }
    else if (ch8Raw < 1450) { if (ledDebounceCounter > 0) ledDebounceCounter--; }
    digitalWrite(ledPin, (ledDebounceCounter >= ledThreshold) ? HIGH : LOW);

    int steerValue = (abs(steering - 1500) > 10) ? map(steering, 900, 2100, -15, 15) : 0;
    int finalSteer = steerValue + map(ch5Trim, 1000, 2000, -15, 15);
    int elevOffset = (abs(elevator - 1500) > 10) ? map(elevator, 900, 2100, -15, 15) : 0;
    int pitchTrim = map(rawCh6, 1000, 2000, -maxPitchTrim, maxPitchTrim);

    if (throttle > 1100) isPerchingState = false;
    else if (ch7Switch > 1600 && throttle < 1100) isPerchingState = true;
    if (ch7Switch < 1400) isPerchingState = false;

    if (isPerchingState) {
      updateCurrentOffsetPerch(rawCh6);
    } else if (throttle < 1050) {
      updateCurrentOffsetGlide();
    } else {
      updateCurrentOffsetTriangle(throttle); 
    }

    int outL = centerAngle + (int)currentOffset + finalSteer + elevOffset + pitchTrim;
    int outR = centerAngle - (int)currentOffset + finalSteer - elevOffset - pitchTrim;
    
    updateServos(outL, outR);
  }

  // --- SERIAL MONITOR LOGGING ---
  if (millis() - lastLogTime > logInterval) {
    lastLogTime = millis();
    if (isConnected) {
      Serial.print("[CONNECTED] CH3: "); Serial.print(lastChValues[2]);
      Serial.print(" | LED: "); Serial.println(digitalRead(ledPin) ? "ON" : "OFF");
    } else {
      Serial.println("[!] WARNING: NO SIGNAL DETECTED");
    }
  }
  delay(3); 
}

void updateCurrentOffsetTriangle(int throttle) {
  if (wasFailsafe) { phase = PI; wasFailsafe = false; }
  float speedFactor = map(throttle, 1050, 2000, 60, 120) / 1000.0;
  phase += speedFactor;
  if (phase >= 2 * PI) phase -= 2 * PI;
  float triangleValue = (2.0 / PI) * asin(sin(phase));
  currentOffset = triangleValue * flapAmplitude;
}

void updateCurrentOffsetPerch(int rawCh6) {
  wasFailsafe = true;
  float maxUpAngle = map(rawCh6, 1000, 2000, 30, 60);  
  perchPhase += 0.01;  
  float goalAngle = maxUpAngle + (sin(perchPhase) * 20.0) - 20.0;  
  if (abs(currentOffset - goalAngle) > 0.5) currentOffset += (currentOffset < goalAngle) ? 1.5 : -1.5;
}

void updateCurrentOffsetGlide() {
  wasFailsafe = true;
  if (currentOffset < 0) currentOffset += 2.0;
  else if (currentOffset > 0) currentOffset -= 2.0;
  if (abs(currentOffset) < 2.5) currentOffset = 0;
}

void processFailsafe() {
  if (currentOffset < 0) currentOffset += 1.5;
  else if (currentOffset > 0) currentOffset -= 1.5;
  if (abs(currentOffset) < 1.5) currentOffset = 0;
  updateServos(centerAngle + (int)currentOffset, centerAngle - (int)currentOffset);
  wasFailsafe = true;
}

void updateServos(int targetL, int targetR) {
  targetL = constrain(targetL, 5, 175);
  targetR = constrain(targetR, 5, 175);
  if (abs(targetL - lastPosL) >= 1) { servoLeft.write(targetL); lastPosL = targetL; }
  if (abs(targetR - lastPosR) >= 1) { servoRight.write(targetR); lastPosR = targetR; }
}