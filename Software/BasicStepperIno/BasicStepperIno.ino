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

const int32_t VELOCITY = 200;

void setup()
{
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
    stepper_driver.setMicrostepsPerStep(2);
    stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
    stepper_driver.enableCoolStep();
    stepper_driver.enable();

    stepper_driver.moveAtVelocity(VELOCITY);

    Serial.println("Stepper driver running...");
  } else {
    Serial.println("Stepper driver setup error!");
  }
}

void loop()
{
    if (not stepper_driver.isSetupAndCommunicating())
    {
        Serial.println("Stepper driver not setup and communicating!");
        return;
    }

        bool hardware_disabled = stepper_driver.hardwareDisabled();
        TMC2209::Settings settings = stepper_driver.getSettings();
        TMC2209::Status status = stepper_driver.getStatus();

    if (hardware_disabled)
    {
        Serial.println("Stepper driver is hardware disabled!");
    }
    else if (not settings.software_enabled)
    {
        Serial.println("Stepper driver is software disabled!");
    }
    else if ((not status.standstill))
    {
        Serial.print("Moving at velocity ");
        Serial.println(VELOCITY);

        uint32_t interstep_duration = stepper_driver.getInterstepDuration();
        Serial.print("which is equal to an interstep_duration of ");
        Serial.println(interstep_duration);
    }
    else
    {
        Serial.println("Not moving, something is wrong!");
    }

    Serial.println();
    delay(2000);
}
