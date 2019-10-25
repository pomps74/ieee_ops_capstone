#define EN1 5
#define EN2 6
#define IN1 2 //LEFT
#define IN2 3
#define IN3 7 //RIGHT
#define IN4 8
#define LEFT 16
#define RIGHT 17
#define FRONT 18

#define SENSOR_OFFSET 4
#define CENTER_VALUE 925
#define WALL_THRESH 500
#define TURN_THRESH 600
#define SPEED_OFFSET 30
#define TURN_TIME 480

double lowLeft = 0;
double lowRight = 0;
double lowFront = 0;

const double kP = 6;
const double kD = 4;
double oldEP = 0;

double speedOffset = 0;
double startTurn = 0;

void setup() {
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(LEFT, INPUT);
  pinMode(RIGHT, INPUT);
  pinMode(FRONT, INPUT);

  Serial.begin(9600);
  setupIR(LEFT, &lowLeft);
  setupIR(RIGHT, &lowRight);
  setupIR(FRONT, &lowFront);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void loop() {
  Serial.print("Left: ");
  Serial.print(getIR(LEFT, &lowLeft));
  Serial.print(" Right: ");
  Serial.print(getIR(RIGHT, &lowRight));

  //checkTurn();
  //realPID();
  //PID();
  PID2();
  //PID3();
}

void setupIR(int pin, double* low) {
  for (int k = 0; k < 100 ; k++)
    *low += analogRead(pin);
  *low /= 100;
}

int getIR(int pin, int low) {
  int input = analogRead(pin);
  input = constrain(input, low, 1023);
  return input;
}

void PID2() {  // CW IS POSITIVE
  double leftSense = getIR(LEFT, &lowLeft);
  double rightSense = getIR(RIGHT, &lowRight);
  double frontSense = getIR(FRONT, &lowFront);
  double eP = 0;
  double eD = 0;
  if (leftSense > WALL_THRESH && rightSense > WALL_THRESH) {  // Both walls
    speedOffset = 0;
    eP = leftSense - rightSense - SENSOR_OFFSET;
    eD = eP - oldEP;
  }
  else if (leftSense > WALL_THRESH && frontSense < TURN_THRESH) {  // No wall on right
    speedOffset = SPEED_OFFSET;
    eP = 2 * (leftSense - CENTER_VALUE);
    eD = eP - oldEP;
  }
  else if (rightSense > WALL_THRESH && frontSense < TURN_THRESH) {  // No wall on left
    speedOffset = SPEED_OFFSET;
    eP = 2 * (CENTER_VALUE - rightSense);
    eD = eP - oldEP;
  }
  else if (frontSense > TURN_THRESH) {
    startTurn = millis();
    speedOffset = SPEED_OFFSET;
    while (millis() - startTurn < TURN_TIME) {
      leftSense = getIR(LEFT, &lowLeft);
      rightSense = getIR(RIGHT, &lowRight);
      eP = leftSense - rightSense - SENSOR_OFFSET;
      eD = eP - oldEP;
      oldEP = eP;
      setLeft(kP * eP + kD * eD);
      setRight(kP * eP + kD * eD);
    }
  }
  oldEP = eP;
  setLeft(kP * eP + kD * eD);
  setRight(kP * eP + kD * eD);
}

void setLeft(double err) {
  double base = 1023 - speedOffset;
  err = base + err;
  err = constrain(err, 0, 1023);
  double mapped = map(err, 0, 1023, 0, 255);
  mapped = constrain(mapped, 0, 255);
  analogWrite(EN1, mapped);
}

void setRight(double err) {
  double base = 1023 - speedOffset;
  err = base - err;
  err = constrain(err, 0, 1023);
  double mapped = map(err, 0, 1023, 0, 250);
  mapped = constrain(mapped, 0, 250);
  analogWrite(EN2, mapped);
}









void PID() {  // CW IS POSITIVE
  double leftSense = getIR(LEFT, &lowLeft);
  double rightSense = getIR(RIGHT, &lowRight);
  double eP = 0;
  double eD = 0;

  eP = leftSense - rightSense - SENSOR_OFFSET;
  eD = eP - oldEP;

  oldEP = eP;
  setLeft(kP * eP + kD * eD);
  setRight(kP * eP + kD * eD);
}

void realPID() {  // CW IS POSITIVE
  double leftSense = getIR(LEFT, &lowLeft);
  double rightSense = getIR(RIGHT, &lowRight);
  double eP = 0;
  double eD = 0;
  if (leftSense > WALL_THRESH && rightSense > WALL_THRESH) {  // Both walls
    eP = leftSense - rightSense - SENSOR_OFFSET;
    eD = eP - oldEP;
  }
  else if (leftSense > WALL_THRESH) {  // No wall on right
    eP = 2 * (leftSense - CENTER_VALUE);
    eD = eP - oldEP;
  }
  else if (rightSense > WALL_THRESH) {  // No wall on left
    eP = 2 * (CENTER_VALUE - rightSense);
    eD = eP - oldEP;
  }
  oldEP = eP;
  setLeft(kP * eP + kD * eD);
  setRight(kP * eP + kD * eD);
}

void PID3() {  // CW IS POSITIVE
  double leftSense = getIR(LEFT, &lowLeft);
  double rightSense = getIR(RIGHT, &lowRight);
  double frontSense = getIR(FRONT, &lowFront);
  double eP = 0;
  double eD = 0;
  if (leftSense > WALL_THRESH && rightSense > WALL_THRESH) {  // Both walls
    eP = leftSense - rightSense - SENSOR_OFFSET;
    eD = eP - oldEP;
  }
  else if (leftSense > WALL_THRESH) {  // No wall on right
    if (frontSense < TURN_THRESH) { // Not close enough to turn
      eP = 2 * (leftSense - CENTER_VALUE);
      eD = eP - oldEP;
    }
    else
      turnRight();
  }
  else if (rightSense > WALL_THRESH) {  // No wall on left
    if (frontSense < TURN_THRESH) { // Not close enough to turn
      eP = 2 * (CENTER_VALUE - rightSense);
      eD = eP - oldEP;
    }
    else
      turnLeft();
  }
  oldEP = eP;
  setLeft(kP * eP + kD * eD);
  setRight(kP * eP + kD * eD);
}

void checkTurn() {
  double leftSense = getIR(LEFT, &lowLeft);
  double rightSense = getIR(RIGHT, &lowRight);
  double frontSense = getIR(FRONT, &lowFront);
  if (frontSense >= TURN_THRESH) { // Right distance for a turn
    if (leftSense < WALL_THRESH && rightSense >= WALL_THRESH) { // Left turn
      turnLeft();
    }
    else if (rightSense < WALL_THRESH && leftSense >= WALL_THRESH) {  // Right turn
      turnRight();
    }
    else if (rightSense >= WALL_THRESH && leftSense >= WALL_THRESH) {
      endHandler();
    }
  }
}

void turnRight() {
  analogWrite(EN1, 255);
  analogWrite(EN2, 240);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  delay(480);
  digitalWrite(IN3, HIGH);
}

void turnLeft() {
  analogWrite(EN1, 255);
  analogWrite(EN2, 240);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  delay(480);
  digitalWrite(IN1, HIGH);
}

void endHandler() { // Dead end, so stop
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
