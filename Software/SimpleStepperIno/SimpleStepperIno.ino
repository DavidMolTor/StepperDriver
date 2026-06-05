#include <TMC2209.h>

HardwareSerial & serial_stream = Serial0;

// Instantiate TMC2209
TMC2209 stepper_driver;
const long    SERIAL_BAUD_RATE = 115200;
const uint8_t RUN_CURRENT_PERCENT = 100;

// Stepper driver pin definitions
#define STP_PIN 1
#define DIR_PIN 2
#define EN_PIN  21

// Steps per rotation for the NEMA17
#define STEPS_PER_REV 200.0f
#define MAX_RPM       600.0f

typedef enum {
    MICROSTEP_FULL      = 1,
    MICROSTEP_HALF      = 2,
    MICROSTEP_QUARTER   = 4,
    MICROSTEP_EIGHTH    = 8,
    MICROSTEP_SIXTEENTH = 16
} microstep_mode_t;

static uint32_t step_delay_us = 0;
static microstep_mode_t current_microstep = MICROSTEP_HALF;

static float current_speed_rpm = 0.0f;
static float target_speed_rpm  = 0.0f;

static float accel_rpm_per_s = 10.0f;

static long last_ramp_update;

static uint32_t rpm_to_step_delay_us(float rpm, microstep_mode_t microstep) {
    if (rpm <= 0.0f) {
        return 0;
    }

    if (rpm > MAX_RPM) {
        rpm = MAX_RPM;
    }

    float steps_per_rev = STEPS_PER_REV * (float)microstep;
    float steps_per_sec = (rpm * steps_per_rev) / 60.0f;

    if (steps_per_sec < 1.0f) {
        steps_per_sec = 1.0f;
    }

    return (uint32_t)(1000000.0f / steps_per_sec);
}

static void stepper_apply_speed() {
    step_delay_us = rpm_to_step_delay_us(current_speed_rpm, current_microstep);
}

static void stepper_set_direction(bool clockwise) {
    digitalWrite(DIR_PIN, clockwise ? HIGH : LOW);
}

static void stepper_set_speed_rpm(float rpm) {
    if (rpm < 0.0f) {
        rpm = 0.0f;
    }

    if (rpm > MAX_RPM) {
        rpm = MAX_RPM;
    }

    target_speed_rpm = rpm;
}

static void stepper_set_acceleration_rpm(float rpm_per_s) {
    if (rpm_per_s < 1.0f) {
        rpm_per_s = 1.0f;
    }

    accel_rpm_per_s = rpm_per_s;
}

static void stepper_update_speed_ramp(void) {
    long now = micros();
    int64_t dt_us = now - last_ramp_update;
    last_ramp_update = now;

    if (dt_us <= 0) {
        return;
    }

    float dt_s = (float)dt_us / 1000000.0f;
    float max_delta_rpm = accel_rpm_per_s * dt_s;
    float error = target_speed_rpm - current_speed_rpm;

    if (fabsf(error) <= max_delta_rpm) {
        current_speed_rpm = target_speed_rpm;
    } else if (error > 0.0f) {
        current_speed_rpm += max_delta_rpm;
    } else {
        current_speed_rpm -= max_delta_rpm;
    }

    stepper_apply_speed();
}

static void stepper_run(void) {
    stepper_update_speed_ramp();

    if (step_delay_us == 0) {
        delayMicroseconds(1000);
        return;
    }

    digitalWrite(STP_PIN, HIGH);
    delayMicroseconds(2);
    digitalWrite(STP_PIN, LOW);

    if (step_delay_us > 2) {
        delayMicroseconds(step_delay_us - 2);
    }
}

void setup()
{
  current_speed_rpm   = 0.0f;
  target_speed_rpm    = 60.0f;
  step_delay_us       = 0;
    
  last_ramp_update = micros();

  pinMode(STP_PIN, OUTPUT);
  digitalWrite(STP_PIN, LOW);

  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  delay(100);

  Serial.begin(SERIAL_BAUD_RATE);

  stepper_driver.setup(serial_stream, SERIAL_BAUD_RATE, TMC2209::SERIAL_ADDRESS_0, 17, 16);

  delay(100);

  if (stepper_driver.isSetupAndCommunicating()) {
    stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
    stepper_driver.setMicrostepsPerStep(2);
    stepper_driver.enableCoolStep();
    stepper_driver.enable();


    stepper_set_direction(true);
    stepper_set_speed_rpm(60.0f);

    Serial.println("Stepper driver running...");
  } else {
    Serial.println("Stepper driver setup error!");
  }
}

void loop()
{
  stepper_run();
}
