// --- Motor Pins ---
#define IN1 9    // Left motor forward
#define IN2 6    // Left motor backward
#define IN3 10   // Right motor forward
#define IN4 11   // Right motor backward
#define sensorValue arr

// --- Sensor Pins ---
int sensors[6] = {A0, A1, A2, A3, A4, A5};
int sensorValue[6];

// --- Sensor threshold ---
int threshold = 500; // Adjust based on testing

unsigned long whiteTol = 90; // tolerance time in ms
unsigned long whiteStart = 0;

int lastPos = 0; // 1 = left, 2 = right

void setup() {
  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Sensor pins
  for (int i = 0; i < 6; i++) {
    pinMode(sensors[i], INPUT);
  }

  Serial.begin(9600);
}

void loop() {
  bool allWhite = true;

  // Read sensor values
  for (int i = 0; i < 6; i++) {
    sensorValue[i] = (analogRead(sensors[i]) > threshold) ? 0 : 1;
    if (sensorValue[i] == 1) allWhite = false; // Found black
    Serial.print(sensorValue[i]);
    Serial.print(" ");
  }
  Serial.println();

  // Update last position memory
  if (sensorValue[0] && !sensorValue[5]) lastPos = 2; // right
  if (!sensorValue[0] && sensorValue[5]) lastPos = 1; // left

  // Movement logic
  if (!allWhite) {
    // Reset white tolerance timer
    whiteStart = millis();


    if (((sensorValue[2] * sensorValue[3]) || (sensorValue[4] * sensorValue[3]) || (arr[2]*arr[3]*arr[4])) && (arr[0]+arr[5]!=1)) {
      forward();
    }
    else if (arr[5] || (arr[5] * arr[4]) ||(arr[5] * arr[4] * arr[3])) {
      turnLeft();
    }
    else if (sensorValue[0] || (sensorValue[1] && sensorValue[0]) ||(arr[1] && arr[2]) || (arr[0] && arr[1] && arr[2])) {
      turnRight();
    }
    // else {
    //   stopMotors();
    // }
  }
  else {
    // All white detected
    if (millis() - whiteStart < whiteTol) {
      forward(); // keep moving straight within tolerance
    }
    else {
      // After tolerance, follow last known direction
      if (lastPos == 1) {
        turnLeft();
      }
      else if (lastPos == 2) {
        turnRight();
      }
      else {
        stopMotors(); // if no memory, just stop
      }
    }
  }

 delay(10);
}

// --- Movement Functions ---
void forward() {
  digitalWrite(IN1, HIGH); // Left forward
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); // Right forward
  digitalWrite(IN4, LOW);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}