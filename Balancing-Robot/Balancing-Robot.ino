// Define pin connections & motor's steps per revolution
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
  
  void init(int _stepPin, int _dirPin, uint64_t _delayTime, bool _direction) {
    stepPin     = _stepPin;
    dirPin      = _dirPin;
    delayTime   = _delayTime;
    direction   = _direction;

    togglePulse = LOW;
    enable      = false;

    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
  }
  
  void update(void) {
    currentTime = micros();
    digitalWrite(dirPin, direction);
    if (enable == 1) {
      if ((currentTime - deltaTime) > delayTime) {
        pulseCount++;

        if (pulseCount % 2 == 0) {
          stepCount++;
        }

        togglePulse = togglePulse == LOW ? HIGH : LOW;
        digitalWrite(stepPin, togglePulse);
        deltaTime = currentTime;
      }
    }
  }

  void changeSpeed(uint64_t _speed) { delayTime = _speed; }

  void changeDirection(bool _direction) { direction = _direction; }

  uint64_t steps(void) { return stepCount; }

private:
  uint64_t delayTime, deltaTime, currentTime;
  uint64_t pulseCount = 0;
  uint64_t stepCount = 0;
  int32_t stepPin, dirPin;
  bool direction, togglePulse, enable;
};

Motor stepperOne, stepperTwo;
uint64_t blinkTime;

void InitLED();
void InitMotors();
void InitGyro();

void setup() {
  Serial.begin(9600);
  Serial.println("Serial Initialized!");

  InitLED();
  InitMotors();
  InitGyro();
}

void loop() {
  stepperOne.update();
  stepperTwo.update();
  
  if (stepperOne.steps() == 2000){
    stepperOne.changeDirection(LOW);
    stepperOne.changeSpeed(600);
  }

  float x, y, z;

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);

    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);
  }

  if (millis() >= blinkTime) {
    digitalWrite(LEDR, digitalRead(LEDR) ? LOW : HIGH);
    blinkTime += 500;
  }
}

void InitLED() {
  pinMode(LEDR, OUTPUT);
  digitalWrite(LEDR, HIGH);

  blinkTime = millis() + 500;
}

void InitMotors() {
  stepperOne.init(STEP_PIN_1, DIR_PIN_1, 1000, HIGH);
  stepperTwo.init(STEP_PIN_2, DIR_PIN_2, 1000, HIGH);
  stepperOne.start();
  stepperTwo.start();

  Serial.println("Motors Initialized!");
}

void InitGyro() {
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
  }

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");
}
