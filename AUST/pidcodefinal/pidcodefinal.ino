// ===== Motor pin definitions =====
#define lmf 3    // Left Motor Forward - AIN1
#define lmb 5    // Left Motor Backward - AIN2
#define rmf 6    // Right Motor Forward - BIN1
#define rmb 9    // Right Motor Backward - BIN2
#define lme 10   // Left Motor Enable (PWM) - PWMA
#define rme 11   // Right Motor Enable (PWM) - PWMB

// ===== Sensor & PID variables =====
int sensor[8];             // Sensor binary values
int pos = 0;               // Weighted sum
int positions = 0;         // Position on the line (0 to 7000)
int lastPosition = 3500;   // Last known position
int sum = 0;               // Count of active sensors
int lastSeenDirection = 0; // 1 = left, 2 = right

// PID tuning
int kp = 10;     // Proportional gain
int kd = 0;      // Derivative gain
int error = 0;   // For derivative term
int p = 0, pid = 0;

// Speeds
int baseSpeedLeft = 200;
int baseSpeedRight = 200;
int lms = 0, rms = 10;
int whiteTol = 30;
uint32_t m1=0;
// ===== Arduino Setup =====
void setup() {
  pinMode(lmf, OUTPUT); pinMode(lmb, OUTPUT);
  pinMode(rmf, OUTPUT); pinMode(rmb, OUTPUT);
  pinMode(lme, OUTPUT); pinMode(rme, OUTPUT);

  Serial.begin(9600);
  delay(1000);
  Serial.println("Line Follower Initialized...");
}

// ===== Main Loop =====
void loop() {
  readSensors();

  // All white (line lost)
  if (sum == 8) {
    Serial.println("All sensors white — Recovering");
    while(millis() - m1 < whiteTol ){}
    recover();
    return;
  }

  // All black (stop/junction)
  if (sum == 0) {
    Serial.println("All sensors black — Stop or Junction");
    stopMotors();
    delay(1000);
    return;
  }

  // Left turn detection
  if (sensor[0] == 1 && sensor[1] == 1 && sum <= 2) {
    Serial.println("Left turn detected");
    turnLeft();
    return;
  }

  // Right turn detection
  if (sensor[6] == 1 && sensor[7] == 1 && sum <= 2) {
    Serial.println("Right turn detected");
    turnRight();
    return;
  }

  // Normal PID line following
  PID();
  setMotorSpeeds(lms, rms);

  // Debug output
  Serial.print("Pos: "); Serial.print(positions);
  Serial.print(" | PID: "); Serial.print(pid);
  Serial.print(" | L: "); Serial.print(lms);
  Serial.print(" | R: "); Serial.println(rms);

  delay(50);
}

// ===== PID Controller =====
void PID() {
  p = 3500 - positions;
  pid = (p / kp) + kd * (p - error);
  error = p;

  rms = baseSpeedRight + pid;
  lms = baseSpeedLeft - pid;

  rms = constrain(rms, -255, 255);
  lms = constrain(lms, -255, 255);
}

// ===== Sensor Reading =====
void readSensors() {
  pos = 0;
  sum = 0;

  for (int i = 0; i < 8; i++) {
    sensor[i] = (analogRead(i) > 400) ? 1 : 0;
    pos += sensor[i] * i * 1000;
    sum += sensor[i];
  }

  if (sum != 0) {
    positions = pos / sum;
    lastPosition = positions;

    // Direction flag for recovery
    if (positions < 3000) {
      lastSeenDirection = 1; // Left
      m1 = millis();
    } else if (positions > 4000) {
      lastSeenDirection = 2; // Right
      m1 = millis();
    }
  } else {
    positions = -1; // Line lost
  }

  // Debug sensors
  Serial.print("Sensors: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(sensor[i]);
    Serial.print(" ");
  }
  Serial.print(" | Sum: "); Serial.println(sum);
}

// ===== Set Motor Speeds =====
void setMotorSpeeds(int left, int right) {
  // Left motor
  if (left >= 0) {
    digitalWrite(lmf, HIGH);
    digitalWrite(lmb, LOW);
  } else {
    left = -left;
    digitalWrite(lmf, LOW);
    digitalWrite(lmb, HIGH);
  }

  // Right motor
  if (right >= 0) {
    digitalWrite(rmf, HIGH);
    digitalWrite(rmb, LOW);
  } else {
    right = -right;
    digitalWrite(rmf, LOW);
    digitalWrite(rmb, HIGH);
  }

  analogWrite(lme, constrain(left, 0, 255));
  analogWrite(rme, constrain(right, 0, 255));
}

// ===== Stop Both Motors =====
void stopMotors() {
  digitalWrite(lmf, LOW); digitalWrite(lmb, LOW);
  digitalWrite(rmf, LOW); digitalWrite(rmb, LOW);
  analogWrite(lme, 0); analogWrite(rme, 0);
}

// ===== Turn Left (Junction) =====
void turnLeft() {
  digitalWrite(lmf, LOW);
  digitalWrite(lmb, HIGH);
  digitalWrite(rmf, HIGH);
  digitalWrite(rmb, LOW);
  analogWrite(lme, 180);
  analogWrite(rme, 180);
  delay(300); // Tune as needed
}

// ===== Turn Right (Junction) =====
void turnRight() {
  digitalWrite(lmf, HIGH);
  digitalWrite(lmb, LOW);
  digitalWrite(rmf, LOW);
  digitalWrite(rmb, HIGH);
  analogWrite(lme, 180);
  analogWrite(rme, 180);
  delay(300); // Tune as needed
}

// ===== Recovery Logic =====
void recover() {
  Serial.print("Recovering... Last seen direction: ");
  Serial.println(lastSeenDirection);

  if (lastSeenDirection == 1) {
    // Turn left slowly
    digitalWrite(lmf, LOW);
    digitalWrite(lmb, HIGH);
    digitalWrite(rmf, HIGH);
    digitalWrite(rmb, LOW);
  } else if (lastSeenDirection == 2) {
    // Turn right slowly
    digitalWrite(lmf, HIGH);
    digitalWrite(lmb, LOW);
    digitalWrite(rmf, LOW);
    digitalWrite(rmb, HIGH);
  } else {
    // Unknown direction, stop
    stopMotors();
    return;
  }

  analogWrite(lme, 150);
  analogWrite(rme, 150);
  delay(100); // Small move before checking again
}
