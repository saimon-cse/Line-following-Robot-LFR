// Motor pin definitions
#define lmf 3    // Left Motor Forward - AIN1
#define lmb 5    // Left Motor Backward - AIN2
#define rmf 6    // Right Motor Forward - BIN1
#define rmb 9    // Right Motor Backward - BIN2
#define lme 10   // Left Motor Enable (PWM) - PWMA
#define rme 11   // Right Motor Enable (PWM) - PWMB

// Sensor and PID variables
int sensor[8];
int pos = 0, positions = 0;
int sum = 0;
int pid = 0;
int kp = 10;     // Proportional gain
int kd = 0;     // Derivative gain
int p = 0;
int error = 0;

// Base speed
int rbs = 200;
int lbs = 200;

// Motor speeds
int lms = 0;
int rms = 10;

void setup() {
  // Motor pins
  pinMode(lmf, OUTPUT);
  pinMode(lmb, OUTPUT);
  pinMode(rmf, OUTPUT);
  pinMode(rmb, OUTPUT);
  pinMode(lme, OUTPUT);
  pinMode(rme, OUTPUT);

  // Initialize Serial Monitor
  Serial.begin(9600);
  delay(1000);
  Serial.println("Line Follower Initialized...");
}

void loop() {
  check();        // Read sensors

  // Handle sharp turns first
  if ((sensor[0] == 1 && sum == 1) || (sensor[7] == 1 && sum == 1)) {
    sharpTurn();
    return; // Skip the rest of loop to finish turn
  }

  PID();          // Apply PID logic
  mos(lms, rms);  // Move motors

  // Debug info
  Serial.print("Position: ");
  Serial.print(positions);
  Serial.print(" | PID: ");
  Serial.print(pid);
  Serial.print(" | LSpeed: ");
  Serial.print(lms);
  Serial.print(" | RSpeed: ");
  Serial.println(rms);

  delay(100);
}


// ----- PID CONTROL -----
void PID() {
  p = 3500 - positions;
  pid = (p / kp) + kd * (p - error);
  error = p;

  Serial.println("Error: " + String(error));
  rms = rbs + pid;
  lms = lbs - pid;

  // Clamp values to [-255, 255]
  rms = constrain(rms, -255, 255);
  lms = constrain(lms, -255, 255);
}

// ----- SENSOR READING -----
void check() {
  pos = 0;
  sum = 0;

  for (int i = 0; i < 8; i++) {
    sensor[i] = analogRead(i);
    sensor[i] = (sensor[i] > 400) ? 1 : 0;
    pos += sensor[i] * i * 1000;
    sum += sensor[i];
  }

  if (sum != 0)
    positions = pos / sum;
  else
    positions = 3500; // Default to center

  // Debug: Show raw sensor data
  Serial.print("Sensors: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(sensor[i]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.println("Sum: " + String(sum));
}

// ----- MOTOR CONTROL -----
void mos(int a, int b) {
  // Left motor
  if (a >= 0) {
    digitalWrite(lmf, HIGH);
    digitalWrite(lmb, LOW);
  } else {
    a = -a;
    digitalWrite(lmf, LOW);
    digitalWrite(lmb, HIGH);
  }

  // Right motor
  if (b >= 0) {
    digitalWrite(rmf, HIGH);
    digitalWrite(rmb, LOW);
  } else {
    b = -b;
    digitalWrite(rmf, LOW);
    digitalWrite(rmb, HIGH);
  }

  analogWrite(lme, constrain(a, 0, 255));
  analogWrite(rme, constrain(b, 0, 255));
}


void sharpTurn() {
  // Sharp left
  if (sensor[0] == 1 && sum == 1) {
    Serial.println("Sharp Left Detected");
    digitalWrite(lmf, LOW);
    digitalWrite(lmb, HIGH);
    digitalWrite(rmf, HIGH);
    digitalWrite(rmb, LOW);
    analogWrite(lme, 180);  // You can adjust speed
    analogWrite(rme, 180);
    delay(150);  // Delay to complete turn
    return;
  }

  // Sharp right
  if (sensor[7] == 1 && sum == 1) {
    Serial.println("Sharp Right Detected");
    digitalWrite(lmf, HIGH);
    digitalWrite(lmb, LOW);
    digitalWrite(rmf, LOW);
    digitalWrite(rmb, HIGH);
    analogWrite(lme, 180);
    analogWrite(rme, 180);
    delay(150);
    return;
  }
}
