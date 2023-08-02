#include <Arduino.h>
#include <Arduino_LSM6DS3.h>
#include "DRV8825.h"
#include "StatusLED.h"
#include "IMU_Data.h"

#define DEBUG false

#define MOTOR_STEPS 200

#define DIR_PIN_1 2
#define STEP_PIN_1 3
#define MODE0_1 6
#define MODE1_1 5
#define MODE2_1 4

#define DIR_PIN_2 7
#define STEP_PIN_2 8
#define MODE0_2 11
#define MODE1_2 10
#define MODE2_2 9

DRV8825 stepper1(MOTOR_STEPS, DIR_PIN_1, STEP_PIN_1,
                 MODE0_1, MODE1_1, MODE2_1);

DRV8825 stepper2(MOTOR_STEPS, DIR_PIN_2, STEP_PIN_2,
                 MODE0_2, MODE1_2, MODE2_2);

static const double TARGET_ANGLE = 3.5;

StatusLED LED;

void InitIMU();
void InitMotors();

IMU_Data ReadIMU();
double CalculateAngle(const IMU_Data&);
double ComputePID(const double&);
void UpdateMotors(const float&, const double&);

void setup() {
#if DEBUG
    Serial.begin(115200);
    delay(5000);
    Serial.println("Serial Initialized!");
#endif

    InitIMU();
    InitMotors();
    LED.InitLED();

#if DEBUG
    Serial.println("Acceleration in g's | Gyroscope in degrees/second");
    Serial.println("aX\taY\taZ | gX\tgY\tgZ");
#endif
}

void loop() {
    const IMU_Data data = ReadIMU();

    const double calculatedAngle = CalculateAngle(data);

    const double angleError = TARGET_ANGLE - calculatedAngle;
    const double speed = ComputePID(angleError);

#if DEBUG
    Serial.print(calculatedAngle);
    Serial.print('\t');
    Serial.print(speed);
    Serial.print('\t');
#endif

    UpdateMotors(abs(speed), angleError);
    stepper1.nextAction();
    stepper2.nextAction();

    LED.UpdateLED();
}

void InitMotors() {
    stepper1.begin();
    stepper2.begin();
    stepper1.enable();
    stepper2.enable();

#if DEBUG
    Serial.println("Motors Initialized!");
#endif
}

void InitIMU() {
    if (!IMU.begin()) {
#if DEBUG
        Serial.println("Failed to initialize IMU!");
#endif
    } else {
#if DEBUG
        Serial.println("IMU Initialized!");
#endif
    }

#if DEBUG
    Serial.print("Gyroscope sample rate = ");
    Serial.print(IMU.gyroscopeSampleRate());
    Serial.println(" Hz");
    Serial.print("Accelerometer sample rate = ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println(" Hz");
    Serial.println();
#endif
}

IMU_Data ReadIMU() {
    static IMU_Data data{};
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(data.ax, data.ay, data.az);
#if DEBUG
        Serial.print(data.ax);
        Serial.print('\t');
        Serial.print(data.ay);
        Serial.print('\t');
        Serial.print(data.az);
        Serial.print('\t');
#endif
    }

    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(data.gx, data.gy, data.gz);
#if DEBUG
        Serial.print(data.gx);
        Serial.print('\t');
        Serial.print(data.gy);
        Serial.print('\t');
        Serial.print(data.gz);
#endif
    }

    return data;
}

double CalculateAngle(const IMU_Data& data) {
    // Loop Time Calculation
    const uint32_t currTime = millis();
    static uint32_t prevTime = currTime;  // declare and initially assign prevTime to currTime
    const uint32_t dt = currTime - prevTime;
    prevTime = currTime;  // update prevTime to currTime

    // Angle Variables
    static double calculatedAngle = 0;

    // Two Angle Calculation Methods
    const double accAngle = atan2(data.ay, data.az) * RAD_TO_DEG;
    const double gyroAngle = calculatedAngle + (data.gx + 0.12) * dt / 1000;  // -0.12 is steady state error

    // Complementary Filter Factors
    static const double tau = 0.5;  // effective filter time (0.5s)
    double a = tau / (tau + (double)dt / 1000.0);

    // Complementary Filtered Angle (high pass filter on gyro; low pass filter on acc)
    calculatedAngle = a * gyroAngle + (1-a) * accAngle;

#if DEBUG
    static uint64_t printTime = millis() + 100;
    if (millis() > printTime) {
        printTime += 100;
        Serial.print(accAngle);
        Serial.print('\t');
        Serial.print(gyroAngle);
        Serial.print('\t');
        Serial.print(calculatedAngle);
        Serial.print('\t');
    }
#endif

    return calculatedAngle;
}

double ComputePID(const double& angleError) {
    // Loop Time Calculation
    uint64_t currTime = millis();
    static uint64_t prevTime = 0;  // declare and initially assign prevTime to currTime
    uint64_t dt = currTime - prevTime;
    double dt_s = (double)dt / 1000.0;
    prevTime = currTime;  // update prevTime to currTime

    // PID constants
    static const double Kp = 10.0;
    static const double Ki = 1.0;
    static const double Kd = 0.1;

    // Variables for PID control
    static double previousError = 0.0;
    static double integral = 0.0;

    // Proportional term
    double proportional = Kp * angleError;

    // Integral term
    integral += Ki * angleError * dt_s;
    integral = constrain(integral, -200, 200);

    // Derivative term
    double derivative = Kd * (angleError - previousError) / dt_s;
    previousError = angleError;

    // Compute the PID output
    double output = proportional + integral + derivative;

#if DEBUG
    static uint64_t printTime = millis() + 100;
    if (millis() > printTime) {
        printTime += 100;
        Serial.print(proportional);
        Serial.print('\t');
        Serial.print(integral);
        Serial.print('\t');
        Serial.print(derivative);
        Serial.print('\t');
        Serial.println(output);
    }
#endif

    return output;
}

void UpdateMotors(const float& speed, const double& angleError) {
    stepper1.setRPM(speed);
    stepper2.setRPM(speed);

    stepper1.startRotate(-angleError);
    stepper2.startRotate(angleError);
}
