#include <TMC2209.h>

HardwareSerial& serial_stream = Serial0;

// Instantiate TMC2209
TMC2209 stepper_driver;
const long SERIAL_BAUD_RATE = 115200;
const uint8_t RUN_CURRENT_PERCENT = 100;
const float TMC2209_CLOCK_HZ = 12000000.0;

// Numeric constants
const float TWO_POW_24 = 16777216.0;

// Stepper driver pin definitions
#define STP_PIN 1
#define DIR_PIN 2
#define EN_PIN 21

// GPIO pin definitions
#define SWM_PIN 18
#define SWC_PIN 20
#define SWP_PIN 19

// Steps per rotation for the NEMA17
#define STEPS_PER_REV 200.0f
#define SPEED_STEPPING 0.5f
#define MAX_SPEED 10.0f
#define MAX_USTEPS 8

static float motor_speed = 1.0f;
static uint8_t motor_usteps = 2;

static void moveAtVelocityHz(float speedHz, uint8_t microstepPower) {
  int microsteps = 1 << microstepPower;

  stepper_driver.setMicrostepsPerStepPowerOfTwo(microstepPower);

  float microstepsPerSecond = speedHz * STEPS_PER_REV * microsteps;
  int32_t vactual = (int32_t)((microstepsPerSecond * TWO_POW_24) / TMC2209_CLOCK_HZ);

  stepper_driver.moveAtVelocity(vactual);
}

static void gpioCallback() {
  if (digitalRead(SWM_PIN) == LOW) {
    motor_speed -= SPEED_STEPPING;

    if (motor_speed < 0.0f)
      motor_speed = 0;
  }

  if (digitalRead(SWC_PIN) == LOW) {
    motor_usteps++;

    if (motor_usteps > MAX_USTEPS)
      motor_usteps = 0;
  }

  if (digitalRead(SWP_PIN) == LOW) {
    motor_speed += SPEED_STEPPING;

    if (motor_speed > MAX_SPEED)
      motor_speed = MAX_SPEED;
  }

  moveAtVelocityHz(motor_speed, motor_usteps);

  Serial.print("Motor speed / microsteps: ");
  Serial.print(motor_speed);
  Serial.print(" / ");
  Serial.println(motor_usteps);
}

void setup() {
  pinMode(STP_PIN, OUTPUT);
  digitalWrite(STP_PIN, LOW);

  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  pinMode(SWM_PIN, INPUT);
  pinMode(SWC_PIN, INPUT);
  pinMode(SWP_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(SWM_PIN), gpioCallback, FALLING);
  attachInterrupt(digitalPinToInterrupt(SWC_PIN), gpioCallback, FALLING);
  attachInterrupt(digitalPinToInterrupt(SWP_PIN), gpioCallback, FALLING);

  delay(100);

  Serial.begin(SERIAL_BAUD_RATE);

  stepper_driver.setup(serial_stream, SERIAL_BAUD_RATE, TMC2209::SERIAL_ADDRESS_0, 17, 16);

  delay(100);

  if (stepper_driver.isSetupAndCommunicating()) {
    stepper_driver.setMicrostepsPerStep(motor_usteps);
    stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
    stepper_driver.enableCoolStep();
    stepper_driver.enable();

    moveAtVelocityHz(motor_speed, motor_usteps);

    Serial.println("Stepper driver running...");
  } else {
    Serial.println("Stepper driver setup error!");
  }
}

void loop() {
  if (not stepper_driver.isSetupAndCommunicating()) {
    Serial.println("Stepper driver not setup and communicating!");
    delay(2000);
    return;
  }

  bool hardware_disabled = stepper_driver.hardwareDisabled();
  TMC2209::Settings settings = stepper_driver.getSettings();
  TMC2209::Status status = stepper_driver.getStatus();

  if (hardware_disabled) {
    Serial.println("Stepper driver is hardware disabled!");
  } else if (not settings.software_enabled) {
    Serial.println("Stepper driver is software disabled!");
  } else if (not status.standstill) {
    Serial.print("Moving at velocity: ");
    Serial.print(motor_speed);

    uint32_t interstep_duration = stepper_driver.getInterstepDuration();
    Serial.print(", which is equal to an interstep_duration of ");
    Serial.println(interstep_duration);
  } else {
    Serial.println("Not moving, something is wrong!");
  }

  Serial.println();
  delay(2000);
}
