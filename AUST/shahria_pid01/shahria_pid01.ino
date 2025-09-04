#define lmf 3
#define lmb 5
#define rmf 6
#define rmb 9
#define lme 10
#define rme 11

int sensor[8];
int pos = 0;
int sum = 0;
int avg = 0;
int last_error = 0;

int kp = 10;
int kd = 0;
int pid = 0;

int rbase = 200;
int lbase = 200;

int lmotor = 0;
int rmotor = 10;

int sensorWeight[8] = {-4, -3, -2, -1, 1, 2, 3, 4};

void setup() {
  pinMode(lmf, OUTPUT);
  pinMode(lmb, OUTPUT);
  pinMode(rmf, OUTPUT);
  pinMode(rmb, OUTPUT);
  pinMode(lme, OUTPUT);
  pinMode(rme, OUTPUT);
  Serial.begin(9600);
  delay(1000);
}

void loop() {
  sensor_reading();
  PID();
  motor(lmotor, rmotor);

  Serial.print("Error: "); Serial.print(avg);
  Serial.print(" | PID: "); Serial.print(pid);
  Serial.print(" | LSpeed: "); Serial.print(lmotor);
  Serial.print(" | RSpeed: "); Serial.println(rmotor);

  delay(5);
}

void PID() {
  int error = avg;
  pid = (error * kp) + ((error - last_error) * kd);
  last_error = error;

  rmotor = rbase - pid;
  lmotor = lbase + pid;

  rmotor = constrain(rmotor, -255, 255);
  lmotor = constrain(lmotor, -255, 255);
}

void sensor_reading() {
  pos = 0;
  sum = 0;
  for (int i = 0; i < 8; i++) {
    sensor[i] = analogRead(i);
    sensor[i] = (sensor[i] > 400) ? 1 : 0;
    pos += sensor[i] * sensorWeight[i];
    sum += sensor[i];
  }
  if (sum != 0) avg = pos / sum;
}

void motor(int a, int b) {
  if (a >= 0) {
    digitalWrite(lmf, HIGH);
    digitalWrite(lmb, LOW);
  } else {
    a = -a;
    digitalWrite(lmf, LOW);
    digitalWrite(lmb, HIGH);
  }

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
