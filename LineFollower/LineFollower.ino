#include <QTRSensors.h>
const int m11Pin = 7;
const int m12Pin = 6;
const int m21Pin = 5;
const int m22Pin = 4;
const int m1Enable = 11;
const int m2Enable = 10;

int m1Speed = 0;
int m2Speed = 0;


// increase kp’s value and see what happens
float kp = 10;
float ki = 0;
float kd = 3;


const int s1 = A0;
const int s2 = A1;
const int s3 = A2;
const int s4 = A3;
const int s5 = A4;
const int s6 = A5;

int low = 400;
int high = 600;


int p = 0;
int i = 0;
int d = 0;

int error = 0;
int lastError = 0;

const int maxSpeed = 255;
const int minSpeed = -255;

const int baseSpeed = 220;

unsigned long calibrationTime = millis();
QTRSensors qtr;

const int sensorCount = 6;
int sensorValues[sensorCount];
int sensors[sensorCount] = { 0, 0, 0, 0, 0, 0 };

long calibrationInterval = 0;
void setup() {

  // pinMode setup
  pinMode(m11Pin, OUTPUT);
  pinMode(m12Pin, OUTPUT);
  pinMode(m21Pin, OUTPUT);
  pinMode(m22Pin, OUTPUT);
  pinMode(m1Enable, OUTPUT);
  pinMode(m2Enable, OUTPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5 }, sensorCount);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // turn on Arduino's LED to indicate we are in calibration mode

  // calibrate the sensor. For maximum grade the line follower should do the movement itself, without human interaction.
  int mRSpeed = -205;
  int mLSpeed = 255;
  bool ok = 1;
  int nr = 0;
  while (nr < 15) {
    qtr.calibrate();

    setMotorSpeed(mRSpeed, mLSpeed);
    if (analogRead(s6) < low && analogRead(s5) < low && analogRead(s4) < low && analogRead(s3) < low && analogRead(s1) < low && analogRead(s2) < low && ok) {
      mRSpeed = -1 * mRSpeed;
      mLSpeed = -1 * mLSpeed;
      nr++;
      ok = 0;
    }

    if ((analogRead(s4) > high || analogRead(s3) > high || analogRead(s2) > high || analogRead(s1) > high || analogRead(s5) > high || analogRead(s6) > high) && ok == 0) {
      ok = 1;
    }

    if (nr > 10 && analogRead(s4) > high && analogRead(s3) > high)
      break;
    // do motor movement here, with millis() as to not ruin calibration)
  }
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(9600);
}

void loop() {
  // inefficient code, written in loop. You must create separate functions
  int error = map(qtr.readLineBlack(sensorValues), 0, 5000, -32, 32);

  p = error;
  i = i + error;
  d = error - lastError;
  lastError = error;

  int motorSpeed = kp * p + ki * i + kd * d;  // = error in this case

  m1Speed = baseSpeed;
  m2Speed = baseSpeed;

  // a bit counter intuitive because of the signs
  // basically in the first if, you substract the error from m1Speed (you add the negative)
  // in the 2nd if you add the error to m2Speed (you substract the negative)
  // it's just the way the values of the sensors and/or motors lined up
  if (error < 0) {
    m1Speed += motorSpeed;
  } else if (error > 0) {
    m2Speed -= motorSpeed;
  }

  if (error > 15) {
    slowDown(m1Speed, m2Speed);
  }

  // make sure it doesn't go past limits. You can use -255 instead of 0 if calibrated programmed properly.
  // making sure we don't go out of bounds
  // maybe the lower bound should be negative, instead of 0? This of what happens when making a steep turn
  m1Speed = constrain(m1Speed, -250, maxSpeed);
  m2Speed = constrain(m2Speed, -250, maxSpeed);


  setMotorSpeed(m1Speed, m2Speed);


  //  DEBUGGING
  //  Serial.print("Error: ");
  //  Serial.println(error);
  //  Serial.print("M1 speed: ");
  //  Serial.println(m1Speed);
  //
  //  Serial.print("M2 speed: ");
  //  Serial.println(m2Speed);
  //
  //  delay(250);
}

void slowDown(int speed1, int speed2) {
  int aux1 = speed1, aux2 = speed2;
  while (speed1 != 0 && speed2 != 0) {
    if (speed1 != 0)
      speed1 -= 2;
    if (speed2 != 0)
      speed2 -= 2;
  }
  while (speed1 != aux1 && speed2 != aux2) {
    if (speed1 != aux1)
      speed1++;
    if (speed2 != aux2)
      speed2++;
  }
  setMotorSpeed(speed1, speed2);
}
// calculate PID value based on error, kp, kd, ki, p, i and d.
void pidControl(float kp, float ki, float kd) {
  // TODO
}


// each arguments takes values between -255 and 255. The negative values represent the motor speed in reverse.
void setMotorSpeed(int motor1Speed, int motor2Speed) {
  // remove comment if any of the motors are going in reverse
  motor1Speed = -motor1Speed;
  motor2Speed = -motor2Speed;
  if (motor1Speed == 0) {
    digitalWrite(m11Pin, LOW);
    digitalWrite(m12Pin, LOW);
    analogWrite(m1Enable, motor1Speed);
  } else {
    if (motor1Speed > 0) {
      digitalWrite(m11Pin, HIGH);
      digitalWrite(m12Pin, LOW);
      analogWrite(m1Enable, motor1Speed);
    }
    if (motor1Speed < 0) {
      digitalWrite(m11Pin, LOW);
      digitalWrite(m12Pin, HIGH);
      analogWrite(m1Enable, -motor1Speed);
    }
  }
  if (motor2Speed == 0) {
    digitalWrite(m21Pin, LOW);
    digitalWrite(m22Pin, LOW);
    analogWrite(m2Enable, motor2Speed);
  } else {
    if (motor2Speed > 0) {
      digitalWrite(m21Pin, HIGH);
      digitalWrite(m22Pin, LOW);
      analogWrite(m2Enable, motor2Speed);
    }
    if (motor2Speed < 0) {
      digitalWrite(m21Pin, LOW);
      digitalWrite(m22Pin, HIGH);
      analogWrite(m2Enable, -motor2Speed);
    }
  }
}