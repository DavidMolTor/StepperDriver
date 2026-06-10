#ifndef TMC2209_H
#define TMC2209_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rom/ets_sys.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"

#include "driver/uart.h"
#include "driver/gpio.h"

// UART parameter definition
#define UART_BUFFER_SIZE    1024
#define UART_QUEUE_SIZE     10
#define UART_BAUD_RATE      115200

// TMC2209 default internal clock definition
#define TMC2209_DEFAULT_CLOCK 12000000

enum StandstillMode
{
    NORMAL          = 0,
    FREEWHEELING    = 1,
    STRONG_BRAKING  = 2,
    BRAKING         = 3
};

enum CurrentIncrement
{
    CURRENT_INCREMENT_1 = 0,
    CURRENT_INCREMENT_2 = 1,
    CURRENT_INCREMENT_4 = 2,
    CURRENT_INCREMENT_8 = 3
};

enum MeasurementCount
{
    MEASUREMENT_COUNT_32 = 0,
    MEASUREMENT_COUNT_8 = 1,
    MEASUREMENT_COUNT_2 = 2,
    MEASUREMENT_COUNT_1 = 3
};

enum DriverConnected
{
    DISABLED    = 0,
    CONNECTED   = 1,
    SETUP       = 2
};


class TMC2209
{
public:
    TMC2209();

    void setup(uart_port_t port, int tx, int rx, uint8_t address);
    void setGPIO(gpio_num_t enable, gpio_num_t step, gpio_num_t dir);

    void setEnabled(bool status);

    void setStepsPerRev(uint32_t steps);
    void setMicrosteps(uint8_t exponent);

    void setRunCurrent(uint8_t percent);
    void setHoldCurrent(uint8_t percent);
    void setHoldDelay(uint8_t percent);

    void setDoubleEdge(bool status);
    void setVSense(bool status);
    void setInverseDirection(bool status);

    void setStandstillMode(StandstillMode mode);

    void setAutomaticCurrentScaling(bool status);
    void setAutomaticGradientAdaptation(bool status);
    void setPwmOffset(uint8_t offset);
    void setPwmGradient(uint8_t gradient);

    void setPowerDownDelay(uint8_t delay);

    const static uint8_t REPLY_DELAY_MAX = 15;
    void setReplyDelay(uint8_t delay);

    void moveVelocity(int32_t speed_hz);
    void moveUsingStepDirInterface();

    void setStealthChop(bool status);
    void setStealthChopDurationThreshold(uint32_t threshold);
    void setStallGuardThreshold(uint8_t threshold);

    void setCoolStep(bool status, uint8_t lower_threshold = 1, uint8_t upper_threshold = 0);
    void setCoolStepCurrentIncrement(CurrentIncrement current_increment);
    void setCoolStepMeasurementCount(MeasurementCount measurement_count);
    void setCoolStepDurationThreshold(uint32_t threshold);

    void setAnalogCurrentScaling(bool status);
    void setExternalSenseResistors(bool status);

    uint8_t getVersion();
    bool hardwareDisabled();
    DriverConnected getConnected();

    uint16_t getMicrostepsPerStep();

    // Settings definition
    struct Settings
    {
        bool is_communicating;
        bool is_setup;
        bool software_enabled;
        uint16_t microsteps_per_step;
        bool inverse_motor_direction_enabled;
        bool stealth_chop_enabled;
        uint8_t standstill_mode;
        uint8_t irun_percent;
        uint8_t irun_register_value;
        uint8_t ihold_percent;
        uint8_t ihold_register_value;
        uint8_t iholddelay_percent;
        uint8_t iholddelay_register_value;
        bool automatic_current_scaling_enabled;
        bool automatic_gradient_adaptation_enabled;
        uint8_t pwm_offset;
        uint8_t pwm_gradient;
        bool cool_step_enabled;
        bool analog_current_scaling_enabled;
        bool internal_sense_resistors_enabled;
    };
    Settings getSettings();

    // Status definition
    struct Status
    {
        uint32_t over_temperature_warning   : 1;
        uint32_t over_temperature_shutdown  : 1;
        uint32_t short_to_ground_a          : 1;
        uint32_t short_to_ground_b          : 1;
        uint32_t low_side_short_a           : 1;
        uint32_t low_side_short_b           : 1;
        uint32_t open_load_a                : 1;
        uint32_t open_load_b                : 1;
        uint32_t over_temperature_120c      : 1;
        uint32_t over_temperature_143c      : 1;
        uint32_t over_temperature_150c      : 1;
        uint32_t over_temperature_157c      : 1;
        uint32_t reserved0                  : 4;
        uint32_t current_scaling            : 5;
        uint32_t reserved1                  : 9;
        uint32_t stealth_chop_mode          : 1;
        uint32_t standstill                 : 1;
    };
    Status getStatus();

    // Global status definition
    struct GlobalStatus
    {
        uint32_t reset      : 1;
        uint32_t drv_err    : 1;
        uint32_t uv_cp      : 1;
        uint32_t reserved   : 29;
    };
    GlobalStatus getGlobalStatus();
    
    void clearReset();
    void clearDriverError();

    uint8_t getInterfaceTransmissionCounter();
    uint32_t getInterstepDuration();
    uint16_t getStallGuardResult();

    uint8_t getPwmScaleSum();
    int16_t getPwmScaleAuto();
    uint8_t getPwmOffsetAuto();
    uint8_t getPwmGradientAuto();

    uint16_t getMicrostepCounter();

private:
    int buffer_size;
    uart_port_t uart_port;
    QueueHandle_t uart_queue;
    uart_config_t uart_config;

    gpio_num_t pin_enable;
    gpio_num_t pin_step;
    gpio_num_t pin_dir;

    uint8_t serial_address;

    uint32_t steps_per_rev;

    void initialize(uint8_t address);
    int serialAvailable();
    size_t serialWrite(uint8_t value);
    int serialRead();
    void serialFlush();
    bool serialWaitTxDone(uint32_t timeout);

    // Serial Settings constants
    const static uint8_t BYTE_MAX_VALUE                 = 0xFF;
    const static uint8_t BITS_PER_BYTE                  = 8;

    const static uint32_t ECHO_DELAY_INC_MICROSECONDS   = 1;
    const static uint32_t ECHO_DELAY_MAX_MICROSECONDS   = 4000;

    const static uint32_t REPLY_DELAY_INC_MICROSECONDS  = 1;
    const static uint32_t REPLY_DELAY_MAX_MICROSECONDS  = 10000;

    const static uint8_t STEPPER_DRIVER_FEATURE_OFF     = 0;
    const static uint8_t STEPPER_DRIVER_FEATURE_ON      = 1;

    const static uint8_t MAX_READ_RETRIES               = 5;
    const static uint32_t READ_RETRY_DELAY_MS           = 20;

    // Driver parameter constants
    const static uint8_t CURRENT_SCALING_MAX    = 31;
    const static uint8_t MAX_MICROSTEP_CONFIG   = 8;

    // Reply datagram definition
    const static uint8_t REPLY_DATAGRAM_SIZE    = 8;
    const static uint8_t DATA_SIZE              = 4;
    union ReplyDatagram
    {
        struct
        {
            uint64_t sync               : 4;
            uint64_t reserved           : 4;
            uint64_t serial_address     : 8;
            uint64_t register_address   : 7;
            uint64_t rw                 : 1;
            uint64_t data               : 32;
            uint64_t crc                : 8;
        };
        uint64_t bytes;
    };

    // Reply datagram parameters
    const static uint8_t SYNC           = 0b101;
    const static uint8_t RW_READ        = 0;
    const static uint8_t RW_WRITE       = 1;
    const static uint8_t SERIAL_ADDRESS = 0b11111111;

    // Read datagram definition
    const static uint8_t READ_REQUEST_DATAGRAM_SIZE = 4;
    union ReadRequestDatagram
    {
        struct
        {
            uint32_t sync               : 4;
            uint32_t reserved           : 4;
            uint32_t serial_address     : 8;
            uint32_t register_address   : 7;
            uint32_t rw                 : 1;
            uint32_t crc                : 8;
        };
        uint32_t bytes;
    };

    // Global configuration registers
    const static uint8_t ADDRESS_GCONF = 0x00;
    union GlobalConfig
    {
        struct
        {
            uint32_t i_scale_analog         : 1;
            uint32_t internal_rsense        : 1;
            uint32_t enable_spread_cycle    : 1;
            uint32_t shaft                  : 1;
            uint32_t index_otpw             : 1;
            uint32_t index_step             : 1;
            uint32_t pdn_disable            : 1;
            uint32_t mstep_reg_select       : 1;
            uint32_t multistep_filt         : 1;
            uint32_t test_mode              : 1;
            uint32_t reserved               : 22;
        };
        uint32_t bytes;
    };
    GlobalConfig global_config;

    // Global status register
    const static uint8_t ADDRESS_GSTAT = 0x01;
    union GlobalStatusUnion
    {
        struct
        {
            GlobalStatus global_status;
        };
        uint32_t bytes;
    };

    const static uint8_t ADDRESS_IFCNT = 0x02;

    // Reply delay register
    const static uint8_t ADDRESS_REPLYDELAY = 0x03;
    union ReplyDelay
    {
        struct
        {
            uint32_t reserved_0 : 8;
            uint32_t replydelay : 4;
            uint32_t reserved_1 : 20;
        };
        uint32_t bytes;
    };

    // Input register
    const static uint8_t ADDRESS_IOIN = 0x06;
    union Input
    {
        struct
        {
            uint32_t enn        : 1;
            uint32_t reserved_0 : 1;
            uint32_t ms1        : 1;
            uint32_t ms2        : 1;
            uint32_t diag       : 1;
            uint32_t reserved_1 : 1;
            uint32_t pdn_serial : 1;
            uint32_t step       : 1;
            uint32_t spread_en  : 1;
            uint32_t dir        : 1;
            uint32_t reserved_2 : 14;
            uint32_t version    : 8;
        };
        uint32_t bytes;
    };
    const static uint8_t VERSION = 0x21;

    // Driver current register
    const static uint8_t ADDRESS_IHOLD_IRUN = 0x10;
    union DriverCurrent
    {
        struct
        {
            uint32_t ihold      : 5;
            uint32_t reserved_0 : 3;
            uint32_t irun       : 5;
            uint32_t reserved_1 : 3;
            uint32_t iholddelay : 4;
            uint32_t reserved_2 : 12;
        };
        uint32_t bytes;
    };
    DriverCurrent driver_current;

    // Driver current parameter constants
    const static uint8_t CURRENT_SETTING_MIN    = 0;
    const static uint8_t CURRENT_SETTING_MAX    = 31;
    const static uint8_t HOLD_DELAY_MIN         = 0;
    const static uint8_t HOLD_DELAY_MAX         = 15;
    const static uint8_t IHOLD_DEFAULT          = 16;
    const static uint8_t IRUN_DEFAULT           = 31;
    const static uint8_t IHOLDDELAY_DEFAULT     = 1;

    // Power down register
    const static uint8_t ADDRESS_TPOWERDOWN = 0x11;
    const static uint8_t TPOWERDOWN_DEFAULT = 20;

    // Interstep duration register
    const static uint8_t ADDRESS_TSTEP = 0x12;

    // PWM threshold register
    const static uint8_t ADDRESS_TPWMTHRS   = 0x13;
    const static uint32_t TPWMTHRS_DEFAULT  = 0;

    // Velocity movement register
    const static uint8_t ADDRESS_VACTUAL    = 0x22;
    const static int32_t VACTUAL_DEFAULT    = 0;
    const static int32_t VACTUAL_STEP_DIR   = 0;

    // CoolStep and StallGuard registers
    const static uint8_t ADDRESS_TCOOLTHRS  = 0x14;
    const static uint8_t TCOOLTHRS_DEFAULT  = 0;
    const static uint8_t ADDRESS_SGTHRS     = 0x40;
    const static uint8_t SGTHRS_DEFAULT     = 0;
    const static uint8_t ADDRESS_SG_RESULT  = 0x41;

    // CoolStep configuration register
    const static uint8_t ADDRESS_COOLCONF = 0x42;
    const static uint8_t COOLCONF_DEFAULT = 0;
    union CoolConfig
    {
        struct
        {
            uint32_t semin      : 4;
            uint32_t reserved_0 : 1;
            uint32_t seup       : 2;
            uint32_t reserved_1 : 1;
            uint32_t semax      : 4;
            uint32_t reserved_2 : 1;
            uint32_t sedn       : 2;
            uint32_t seimin     : 1;
            uint32_t reserved_3 : 16;
        };
        uint32_t bytes;
    };
    CoolConfig cool_config;
    bool cool_step_enabled;
    const static uint8_t SEIMIN_UPPER_LIMIT     = 20;
    const static uint8_t SEIMIN_LOWER_SETTING   = 0;
    const static uint8_t SEIMIN_UPPER_SETTING   = 1;
    const static uint8_t SEMIN_OFF              = 0;
    const static uint8_t SEMIN_MIN              = 1;
    const static uint8_t SEMIN_MAX              = 15;
    const static uint8_t SEMAX_MIN              = 0;
    const static uint8_t SEMAX_MAX              = 15;

    // Microstepping register
    const static uint8_t ADDRESS_MSCNT      = 0x6A;
    const static uint8_t ADDRESS_MSCURACT   = 0x6B;

    // Driver configuration register
    const static uint8_t ADDRESS_CHOPCONF = 0x6C;
    union ChopperConfig
    {
        struct
        {
            uint32_t toff           : 4;
            uint32_t hstart         : 3;
            uint32_t hend           : 4;
            uint32_t reserved_0     : 4;
            uint32_t tbl            : 2;
            uint32_t vsense         : 1;
            uint32_t reserved_1     : 6;
            uint32_t mres           : 4;
            uint32_t interpolation  : 1;
            uint32_t double_edge    : 1;
            uint32_t diss2g         : 1;
            uint32_t diss2vs        : 1;
        };
        uint32_t bytes;
    };
    ChopperConfig chopper_config;
    uint8_t toff = TOFF_DEFAULT;

    // Driver configuration constants
    const static uint32_t CONFIG_DEFAULT        = 0x10000053;
    const static uint8_t TBL_DEFAULT            = 0b10;
    const static uint8_t HEND_DEFAULT           = 0;
    const static uint8_t HSTART_DEFAULT         = 5;
    const static uint8_t TOFF_DEFAULT           = 3;
    const static uint8_t TOFF_DISABLE           = 0;
    const static uint8_t DOUBLE_EDGE_DISABLE    = 0;
    const static uint8_t DOUBLE_EDGE_ENABLE     = 1;
    const static uint8_t VSENSE_DISABLE         = 0;
    const static uint8_t VSENSE_ENABLE          = 1;

    // Microstepping constants
    const static size_t MICROSTEPS_PER_STEP_MIN = 1;
    const static size_t MICROSTEPS_PER_STEP_MAX = 256;

    // Driver status register
    const static uint8_t ADDRESS_DRV_STATUS = 0x6F;
    union DriverStatus
    {
        struct
        {
            Status status;
        };
        uint32_t bytes;
    };

    // PWM configuration register
    const static uint8_t ADDRESS_PWMCONF = 0x70;
    union PwmConfig
    {
        struct
        {
            uint32_t pwm_offset     : 8;
            uint32_t pwm_grad       : 8;
            uint32_t pwm_freq       : 2;
            uint32_t pwm_autoscale  : 1;
            uint32_t pwm_autograd   : 1;
            uint32_t freewheel      : 2;
            uint32_t reserved       : 2;
            uint32_t pwm_reg        : 4;
            uint32_t pwm_lim        : 4;
        };
        uint32_t bytes;
    };
    PwmConfig pwm_config;

    // PWM configuration constants
    const static uint32_t PWM_CONFIG_DEFAULT    = 0xC10D0024;
    const static uint8_t PWM_OFFSET_MIN         = 0;
    const static uint8_t PWM_OFFSET_MAX         = 255;
    const static uint8_t PWM_OFFSET_DEFAULT     = 0x24;
    const static uint8_t PWM_GRAD_MIN           = 0;
    const static uint8_t PWM_GRAD_MAX           = 255;
    const static uint8_t PWM_GRAD_DEFAULT       = 0x14;

    // PWM scaling register
    const static uint8_t ADDRESS_PWM_SCALE = 0x71;
    union PwmScale
    {
        struct
        {
            uint32_t pwm_scale_sum  : 8;
            uint32_t reserved_0     : 8;
            uint32_t pwm_scale_auto : 9;
            uint32_t reserved_1     : 7;
        };
        uint32_t bytes;
    };

    // Automatic PWM configuration register
    const static uint8_t ADDRESS_PWM_AUTO = 0x72;
    union PwmAuto
    {
        struct
        {
            uint32_t pwm_offset_auto    : 8;
            uint32_t reserved_0         : 8;
            uint32_t pwm_gradient_auto  : 8;
            uint32_t reserved_1         : 8;
        };
        uint32_t bytes;
    };

    void setSerialMode(uint8_t address);
    bool serialOperationMode();

    void setRegisterDefaults();
    void readStoreRegisters();

    void minimizeMotorCurrent();

    uint32_t reverseData(uint32_t data);

    template <typename Datagram>
    uint8_t calculateCrc(Datagram &datagram, uint8_t datagram_size);
    template <typename Datagram>
    void sendDatagramUnidirectional(Datagram &datagram, uint8_t datagram_size);
    template <typename Datagram>
    void sendDatagramBidirectional(Datagram &datagram, uint8_t datagram_size);

    void write(uint8_t address, uint32_t data);
    uint32_t read(uint8_t address);

    void writeDriverCurrent();

    void writeGlobalConfig();
    uint32_t readGlobalConfigBytes();

    void writeChopperConfig();
    uint32_t readChopperConfigBytes();

    void writePwmConfig();
    uint32_t readPwmConfigBytes();
};

#endif