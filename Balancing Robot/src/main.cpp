#include <Arduino.h>
#include <Arduino_LSM6DS3.h>
#include <WiFiNINA.h>

#define DIR_PIN_1 A1
#define STEP_PIN_1 A0
#define DIR_PIN_2 A2
#define STEP_PIN_2 A3

class Motor{
public:
  void stop(void) { enable = false; }
  
  void start(void) { enable = true; }
  
  void init(int _stepPin, int _dirPin, uint64_t _stepTime, bool _direction) {
    stepPin     = _stepPin;
    dirPin      = _dirPin;
    stepTime    = _stepTime;
    direction   = _direction;

    togglePulse = LOW;
    enable      = false;

    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
  }

  void run(void) {
    currentTime = micros();
    digitalWrite(dirPin, direction);
    if (!enable) return;
    if ((currentTime - lastPulseTime) > stepTime) {
      pulseCount++;
      if (pulseCount % 2 == 0)
        stepCount++;

      togglePulse = !togglePulse;
      digitalWrite(stepPin, togglePulse);
      lastPulseTime = currentTime;
    }
  }

  void changeSpeed(double _speed) {
    // RPM to stepTime
    const double stepAngle = 1.8; // Step angle in degrees
    const double stepsPerRevolution = 360.f / stepAngle;
    const uint8_t microsteps = 1;  // 1, 2, 4, 8, 16
    const double minPulseDuration = 1.9; // Minimum step pulse duration in microseconds

    double stepsPerSecond = (_speed * stepsPerRevolution) / 60.0;
    float microstepDuration = minPulseDuration / microsteps;
    stepTime = 1e6 / (stepsPerSecond * microstepDuration);

    static uint64_t updateTime = millis() + 100;
    if (millis() > updateTime) {
      updateTime += 100;
      Serial.print(_speed);
      Serial.print('\t');
      Serial.println(stepTime);
    }
  }

  void changeDirection(bool _direction) { direction = _direction; }

  uint64_t steps(void) { return stepCount; }

private:
  uint64_t stepTime, lastPulseTime = 0, currentTime;
  uint64_t pulseCount = 0, stepCount = 0;
  int32_t stepPin, dirPin;
  bool direction, togglePulse, enable;
};

struct IMU_Data{
  float ax, ay, az, gx, gy, gz;
};

Motor motor_L, motor_R;

bool blinkMode;
uint64_t blinkTime;

void InitMotors();
void InitIMU();
void InitLED();

void ReadIMU(IMU_Data&);
void UpdateMotorValues(IMU_Data&);
double CalculateAngle(IMU_Data&);
double ComputePID(double&);
void RunMotors();
void UpdateLED();

void setup() {
  Serial.begin(9600);
  Serial.println("Serial Initialized!");

  InitMotors();
  InitIMU();
  InitLED();
}

void loop() {
  static IMU_Data data;
  ReadIMU(data);  

  UpdateMotorValues(data);
  RunMotors();

  UpdateLED();
}

void InitMotors() {
  motor_L.init(STEP_PIN_1, DIR_PIN_1, 1.9, HIGH);
  motor_R.init(STEP_PIN_2, DIR_PIN_2, 1000000, HIGH);
  motor_L.start();
  motor_R.start();

  // Serial.println("Motors Initialized!");
  Serial.print("accAngle");
  Serial.print('\t');
  Serial.print("gyroAngle");
  Serial.print('\t');
  Serial.println("calculatedAngle");
}

void InitIMU() {
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
  }

  // Serial.print("Gyroscope sample rate = ");
  // Serial.print(IMU.gyroscopeSampleRate());
  // Serial.println(" Hz");
  // Serial.print("Accelerometer sample rate = ");
  // Serial.print(IMU.accelerationSampleRate());
  // Serial.println(" Hz");
  // Serial.println();
  // Serial.println("Acceleration in g's | Gyroscope in degrees/second");
  // Serial.println("aX\taY\taZ | gX\tgY\tgZ");
}

void InitLED() {
  pinMode(LEDR, OUTPUT);
  digitalWrite(LEDR, HIGH);

  blinkTime = millis() + 500UL;
}

void ReadIMU(IMU_Data& data) {
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(data.ax, data.ay, data.az);

    // Serial.print(data.ax);
    // Serial.print('\t');
    // Serial.print(data.ay);
    // Serial.print('\t');
    // Serial.print(data.az);
    // Serial.print('\t');
  }
  
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(data.gx, data.gy, data.gz);

    // Serial.print(data.gx);
    // Serial.print('\t');
    // Serial.print(data.gy);
    // Serial.print('\t');
    // Serial.print(data.gz);
  }
}

void UpdateMotorValues(IMU_Data& data) {
  double calculatedAngle = CalculateAngle(data);

  double motorSpeed = ComputePID(calculatedAngle);

  motor_L.changeSpeed(abs(motorSpeed));
  motor_R.changeSpeed(abs(motorSpeed));

  motor_L.changeDirection((motorSpeed < 0) ? 0 : 1);
  motor_R.changeDirection((motorSpeed < 0) ? 0 : 1);
}

double CalculateAngle(IMU_Data& data) {  
  // Loop Time Calculation
  uint64_t currTime = millis();
  static uint64_t prevTime = currTime;  // declare and initially assign prevTime to currTime
  uint64_t dt = currTime - prevTime; 
  prevTime = currTime;  // update prevTime to currTime

  // Angle Variables
  static float calculatedAngle = 0;

  // Two Angle Calculation Methods
  float accAngle = atan2(data.ay, data.az) * RAD_TO_DEG;
  float gyroAngle = calculatedAngle + (data.gx + 0.12) * dt / 1000;  // -0.12 is steady state error

  // Complementary Filter Factors
  static const double tau = 0.5;  // effective filter time (0.5s)
  double a = tau / (tau + (double)dt / 1000.0);
  
  // Complementary Filtered Angle (high pass filter on gyro; low pass filter on acc)
  calculatedAngle = a * gyroAngle + (1-a) * accAngle;

  // static uint64_t printTime = millis() + 100;
  // if (millis() > printTime) {
  //   printTime += 100;
  //   Serial.print(accAngle);
  //   Serial.print('\t');
  //   Serial.print(gyroAngle);
  //   Serial.print('\t');
  //   Serial.print(calculatedAngle);
  //   Serial.print('\t');
  // }

  return calculatedAngle;
}

double ComputePID(double& calculatedAngle) {
  // Loop Time Calculation
  uint64_t currTime = millis();
  static uint64_t prevTime = currTime;  // declare and initially assign prevTime to currTime
  uint64_t dt = currTime - prevTime; 
  double dt_s = (double)dt / 1000.0;
  prevTime = currTime;  // update prevTime to currTime

  // PID constants
  static const double Kp = 12.0;
  static const double Ki = 0.0;
  static const double Kd = 0.0;

  // Target angles
  static const double targetAngle = -4;

  // Variables for PID control
  static double previousError = 0.0;
  static double integral = 0.0;

  double error = targetAngle - calculatedAngle;

  // Proportional term
  double proportional = Kp * error;

  // Integral term
  integral += Ki * error * dt_s;
  integral = constrain(integral, -50, 50);

  // Derivative term
  double derivative = Kd * (error - previousError) / dt_s;
  previousError = error;

  // Compute the PID output
  double output = proportional + integral + derivative;

  // static uint64_t printTime = millis() + 100;
  // if (millis() > printTime) {
  //   printTime += 100;
  //   Serial.print(proportional);
  //   Serial.print('\t');
  //   Serial.print(integral);
  //   Serial.print('\t');
  //   Serial.print(derivative);
  //   Serial.print('\t');
  //   Serial.println(output);
  // }

  return output;
}

void RunMotors() {
  motor_L.run();
  motor_R.run();
}

void UpdateLED() {
  if (millis() >= blinkTime) {
    digitalWrite(LEDR, blinkMode ? HIGH : LOW);
    blinkMode = !blinkMode;
    blinkTime += 500UL;
  }
}