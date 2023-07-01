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
    if ((currentTime - deltaTime) > stepTime) {
      pulseCount++;
      if (pulseCount % 2 == 0)
        stepCount++;

      togglePulse = !togglePulse;
      digitalWrite(stepPin, togglePulse);
      deltaTime = currentTime;
    }
  }

  void changeSpeed(uint64_t _speed) { stepTime = _speed; }

  void changeDirection(bool _direction) { direction = _direction; }

  uint64_t steps(void) { return stepCount; }

private:
  uint64_t stepTime, deltaTime, currentTime;
  uint64_t pulseCount = 0;
  uint64_t stepCount = 0;
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
double CalculateAngle(IMU_Data&, unsigned long);
double ComputePID(double, unsigned long);
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
  motor_L.init(STEP_PIN_1, DIR_PIN_1, 1000, HIGH);
  motor_R.init(STEP_PIN_2, DIR_PIN_2, 1000, HIGH);
  motor_L.start();
  motor_R.start();

  Serial.println("Motors Initialized!");
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
  // Loop Time Calculation
  unsigned long currTime = millis();
  static unsigned long prevTime = currTime;  // declare and initially assign prevTime to currTime
  unsigned long dt = currTime - prevTime; 
  prevTime = currTime;  // update prevTime to currTime

  // Calculate Angle
  double calculatedAngle = CalculateAngle(data, dt);

  // Output for Motors
  double controlOutput = ComputePID(calculatedAngle, dt);

  motor_L.changeSpeed(abs(controlOutput));
  motor_R.changeSpeed(abs(controlOutput));

  motor_L.changeDirection((controlOutput < 0) ? 0 : 1);
  motor_R.changeDirection((controlOutput < 0) ? 0 : 1);
}

double CalculateAngle(IMU_Data& data, unsigned long dt) {  
  // Two Angle Calculation Methods
  float accAngle = atan2(data.ay, data.az) * RAD_TO_DEG;
  float gyroRate = (data.gx + 0.12);  // -0.12 is steady state error

  // Complementary Filter Factors
  static const double tau = 0.5;  // effective filter time (0.5s)
  double a = tau / (tau + (double)dt / 1000.0);
  
  // Complementary Filtered Angle (high pass filter on gyro; low pass filter on acc)
  static float calculatedAngle = 0;
  calculatedAngle = a * (calculatedAngle + gyroRate * dt / 1000) + (1-a) * accAngle;
  return calculatedAngle;
}

double ComputePID(double calculatedAngle, unsigned long dt) {
  // PID constants
  static const double Kp = 40.0;
  static const double Ki = 40.0;
  static const double Kd = 0.05;

  // Target angles
  static const double targetAngle = 1.31;

  // Variables for PID control
  static double previousError = 0.0;
  static double integral = 0.0;

  double error = targetAngle - calculatedAngle;

  // Proportional term
  double proportional = Kp * error;

  // Integral term
  integral += Ki * error * dt;
  integral = constrain(integral, -300 / Ki / dt, 300 / Ki / dt);

  // Derivative term
  double derivative = Kd * (error - previousError) / dt;
  previousError = error;

  // Compute the PID output
  double output = proportional + integral - derivative;

  return constrain(output, -255, 255);
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