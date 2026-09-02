 // Motor Pins
#define ENA 11
#define IN1 7
#define IN2 8

#define ENB 6
#define IN3 4
#define IN4 5

 // Encoder Pins
#define LEFT_ENC_A 2
#define LEFT_ENC_B 3

#define RIGHT_ENC_A 9
#define RIGHT_ENC_B 10

 // Encoder Constants
const float COUNTS_PER_REV = 896.0;

 // Encoder Variables
volatile long leftCount = 0;
volatile long rightCount = 0;

long prevLeftCount = 0;
long prevRightCount = 0;

 // RPM Variables
float leftRPM = 0.0;
float rightRPM = 0.0;

float targetRPM = 0.0;

 // PID Gains
float leftKp = 0.80;
float leftKi = 0.00;
float leftKd = 0.00;

float rightKp = 0.80;
float rightKi = 0.00;
float rightKd = 0.00;

 // Left PID Variables
float leftError = 0;
float leftPrevError = 0;
float leftIntegral = 0;
float leftDerivative = 0;

 // Right PID Variables
float rightError = 0;
float rightPrevError = 0;
float rightIntegral = 0;
float rightDerivative = 0;

 // PWM Variables
int leftPWM = 0;
int rightPWM = 0;

 // Timing
unsigned long previousTime = 0;
const int sampleTime = 50;

 // Left Encoder Interrupt
void leftEncoderISR() {
  if (digitalRead(LEFT_ENC_A) == digitalRead(LEFT_ENC_B))
    leftCount++;
  else
    leftCount--;
}

 // Right Encoder Interrupt
void rightEncoderISR() {
  if (digitalRead(RIGHT_ENC_A) == digitalRead(RIGHT_ENC_B))
    rightCount--;
  else
    rightCount++;
}

 // Setup
void setup() {

  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);


  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);

  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);


  attachInterrupt(
    digitalPinToInterrupt(LEFT_ENC_A),
    leftEncoderISR,
    CHANGE);

  attachInterrupt(
    digitalPinToInterrupt(RIGHT_ENC_A),
    rightEncoderISR,
    CHANGE);

  // Forward Direction
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  previousTime = millis();

  Serial.println("# PID Controller Ready");
}
 // Read Serial Commands
void readSerial() {
  if (!Serial.available())
    return;

  String data = Serial.readStringUntil('\n');

  data.trim();

  int commaIndex = data.indexOf(',');

  if (commaIndex == -1)
    return;

  String command = data.substring(0, commaIndex);

  float value = data.substring(commaIndex + 1).toFloat();


  if (command == "LP") {
    leftKp = value;

    Serial.print("# Left Kp = ");
    Serial.println(leftKp);
  }

  else if (command == "LI") {
    leftKi = value;

    Serial.print("# Left Ki = ");
    Serial.println(leftKi);
  }

  else if (command == "LD") {
    leftKd = value;

    Serial.print("# Left Kd = ");
    Serial.println(leftKd);
  }

  else if (command == "RP") {
    rightKp = value;

    Serial.print("# Right Kp = ");
    Serial.println(rightKp);
  }

  else if (command == "RI") {
    rightKi = value;

    Serial.print("# Right Ki = ");
    Serial.println(rightKi);
  }

  else if (command == "RD") {
    rightKd = value;

    Serial.print("# Right Kd = ");
    Serial.println(rightKd);
  }

  else if (command == "R") {
    targetRPM = value;

    Serial.print("# Target RPM = ");
    Serial.println(targetRPM);
  }
}

 // Calculate Wheel RPM
void calculateRPM(float dt) {

  long currentLeft;
  long currentRight;

  noInterrupts();

  currentLeft = leftCount;
  currentRight = rightCount;

  interrupts();

  long deltaLeft = currentLeft - prevLeftCount;
  long deltaRight = currentRight - prevRightCount;

  prevLeftCount = currentLeft;
  prevRightCount = currentRight;

  leftRPM =
    (deltaLeft * 60.0) / (COUNTS_PER_REV * dt);

  rightRPM =
    (deltaRight * 60.0) / (COUNTS_PER_REV * dt);
}

 // Apply PWM To Motors
void driveMotors() {

  analogWrite(ENA, leftPWM);

  analogWrite(ENB, rightPWM);
}

 // Left PID Controller
void leftPID(float dt) {

  leftError = targetRPM - leftRPM;

  leftIntegral += leftError * dt;

  // Anti-windup
  leftIntegral = constrain(leftIntegral, -300, 300);

  leftDerivative = (leftError - leftPrevError) / dt;

  float output =
    leftKp * leftError + leftKi * leftIntegral + leftKd * leftDerivative;

  leftPrevError = leftError;

  leftPWM = constrain((int)output, 0, 255);
}


 // Right PID Controller
void rightPID(float dt) {

  rightError = targetRPM - rightRPM;

  rightIntegral += rightError * dt;

  // Anti-windup
  rightIntegral = constrain(rightIntegral, -300, 300);

  rightDerivative = (rightError - rightPrevError) / dt;

  float output =
    rightKp * rightError + rightKi * rightIntegral + rightKd * rightDerivative;

  rightPrevError = rightError;

  rightPWM = constrain((int)output, 0, 255);
}


 // Reset PID
void resetPID() {

  leftIntegral = 0;
  rightIntegral = 0;

  leftPrevError = 0;
  rightPrevError = 0;

  leftPWM = 0;
  rightPWM = 0;
}


 // Send Telemetry
void sendTelemetry()
{
    Serial.print("DATA,");

    // Time (milliseconds)
    Serial.print(millis());
    Serial.print(",");

    // Target RPM
    Serial.print(targetRPM);
    Serial.print(",");

    // Left RPM
    Serial.print(leftRPM);
    Serial.print(",");

    // Right RPM
    Serial.print(rightRPM);
    Serial.print(",");

    // Left PWM
    Serial.print(leftPWM);
    Serial.print(",");

    // Right PWM
    Serial.print(rightPWM);
    Serial.print(",");

    // Encoder Counts
    Serial.print(leftCount);
    Serial.print(",");

    Serial.println(rightCount);
}

 // Main Loop
 
void loop() {

  // Read commands from PC
  readSerial();

  unsigned long currentTime = millis();

  if (currentTime - previousTime >= sampleTime) {

    float dt = (currentTime - previousTime) / 1000.0;

    previousTime = currentTime;

    // Calculate wheel RPM
    calculateRPM(dt);

    // If target speed is zero, stop motors
    if (targetRPM == 0) {

      resetPID();

      driveMotors();

    } else {

      // Calculate PID outputs
      leftPID(dt);

      rightPID(dt);

      // Apply PWM
      driveMotors();
    }

    // Send telemetry to PC
    sendTelemetry();
  }
}


