#include "TMC2209.h"


TMC2209::TMC2209()
{

}

void TMC2209::setup(uart_port_t port, int tx, int rx, uint8_t address)
{
    // Setup UART buffered IO with event queue
    uart_port   = port;
    buffer_size = (UART_BUFFER_SIZE * 2);
    ESP_ERROR_CHECK(uart_driver_install(uart_port, buffer_size, buffer_size, UART_QUEUE_SIZE, &uart_queue, 0));

    // Set the UART parameter configuration
    uart_config.baud_rate           = UART_BAUD_RATE;
    uart_config.data_bits           = UART_DATA_8_BITS;
    uart_config.parity              = UART_PARITY_DISABLE;
    uart_config.stop_bits           = UART_STOP_BITS_1;
    uart_config.flow_ctrl           = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 122;
    uart_config.source_clk          = UART_SCLK_DEFAULT;
    ESP_ERROR_CHECK(uart_param_config(uart_port, &uart_config));

    // Set the UART pins
    ESP_ERROR_CHECK(uart_set_pin(uart_port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    initialize(address);
}

void TMC2209::setGPIO(gpio_num_t enable, gpio_num_t step, gpio_num_t dir)
{
    pin_enable  = enable;
    pin_step    = step;
    pin_dir     = dir;
    
    // Default button configuration
    gpio_config_t io_conf = {
        .pin_bit_mask   = 1ULL,
        .mode           = GPIO_MODE_OUTPUT,
        .pull_up_en     = GPIO_PULLUP_DISABLE,
        .pull_down_en   = GPIO_PULLDOWN_DISABLE,
        .intr_type      = GPIO_INTR_DISABLE
    };

    // Configure each button
    io_conf.pin_bit_mask = 1ULL << enable;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    io_conf.pin_bit_mask = 1ULL << step;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    io_conf.pin_bit_mask = 1ULL << dir;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Set default pin levels
    gpio_set_level(pin_enable, 1);
    gpio_set_level(pin_step, 0);
    gpio_set_level(pin_dir, 0);
}

void TMC2209::setEnabled(bool status)
{
    gpio_set_level(pin_enable, status ? 0 : 1);
}

void TMC2209::setStepsPerRev(uint32_t steps)
{
    steps_per_rev = steps;
}

void TMC2209::setMicrosteps(uint8_t exponent)
{
    if (exponent <= MAX_MICROSTEP_CONFIG)
        chopper_config.mres = MAX_MICROSTEP_CONFIG - exponent;

    writeChopperConfig();
}

void TMC2209::setRunCurrent(uint8_t percent)
{
    driver_current.irun = (CURRENT_SETTING_MAX - CURRENT_SETTING_MIN) * (percent / 100.0f);
    writeDriverCurrent();
}

void TMC2209::setHoldCurrent(uint8_t percent)
{
    driver_current.ihold = (CURRENT_SETTING_MAX - CURRENT_SETTING_MIN) * (percent / 100.0f);
    writeDriverCurrent();
}

void TMC2209::setHoldDelay(uint8_t percent)
{

    driver_current.iholddelay = (HOLD_DELAY_MAX - HOLD_DELAY_MIN) * (percent / 100.0f);
    writeDriverCurrent();
}

void TMC2209::setDoubleEdge(bool status)
{
    chopper_config.double_edge = status ? DOUBLE_EDGE_ENABLE : DOUBLE_EDGE_DISABLE;
    writeChopperConfig();
}

void TMC2209::setVSense(bool status)
{
    chopper_config.vsense = status ? VSENSE_ENABLE : VSENSE_DISABLE;
    writeChopperConfig();
}

void TMC2209::setInverseDirection(bool status)
{
    global_config.shaft = status ? 1 : 0;
    writeGlobalConfig();
}

void TMC2209::setStandstillMode(StandstillMode mode)
{
    pwm_config.freewheel = mode;
    writePwmConfig();
}

void TMC2209::setAutomaticCurrentScaling(bool status)
{
    pwm_config.pwm_autoscale = status ? STEPPER_DRIVER_FEATURE_ON : STEPPER_DRIVER_FEATURE_OFF;
    writePwmConfig();
}

void TMC2209::setAutomaticGradientAdaptation(bool status)
{
    pwm_config.pwm_autograd = status ? STEPPER_DRIVER_FEATURE_ON : STEPPER_DRIVER_FEATURE_OFF;
    writePwmConfig();
}

void TMC2209::setPwmOffset(uint8_t offset)
{
    pwm_config.pwm_offset = offset;
    writePwmConfig();
}

void TMC2209::setPwmGradient(uint8_t gradient)
{
    pwm_config.pwm_grad = gradient;
    writePwmConfig();
}

void TMC2209::setPowerDownDelay(uint8_t delay)
{
    write(ADDRESS_TPOWERDOWN, delay);
}

void TMC2209::setReplyDelay(uint8_t delay)
{
    if (delay > REPLY_DELAY_MAX)
    {
        delay = REPLY_DELAY_MAX;
    }
    ReplyDelay delay_data;
    delay_data.bytes        = 0;
    delay_data.replydelay   = delay;
    write(ADDRESS_REPLYDELAY, delay_data.bytes);
}

void TMC2209::moveVelocity(int32_t speed_hz)
{
    uint32_t microsteps = 2^(MAX_MICROSTEP_CONFIG - chopper_config.mres);
    int32_t microsteps_second = speed_hz * steps_per_rev * microsteps * TMC2209_DEFAULT_CLOCK / (1 << 24);
    write(ADDRESS_VACTUAL, microsteps_second);
}

void TMC2209::moveUsingStepDirInterface()
{
    write(ADDRESS_VACTUAL, VACTUAL_STEP_DIR);
}

void TMC2209::setStealthChop(bool status)
{
    global_config.enable_spread_cycle = status ? 0 : 1;
    writeGlobalConfig();
}

void TMC2209::setCoolStepDurationThreshold(uint32_t threshold)
{
    write(ADDRESS_TCOOLTHRS, threshold);
}

void TMC2209::setStealthChopDurationThreshold(uint32_t threshold)
{
    write(ADDRESS_TPWMTHRS, threshold);
}

void TMC2209::setStallGuardThreshold(uint8_t threshold)
{
    write(ADDRESS_SGTHRS, threshold);
}

void TMC2209::setCoolStep(bool status, uint8_t lower_threshold, uint8_t upper_threshold)
{
    if (status)
    {
        lower_threshold = lower_threshold < SEMIN_MIN ? SEMIN_MIN : lower_threshold;
        lower_threshold = lower_threshold > SEMIN_MAX ? SEMIN_MAX : lower_threshold;
        cool_config.semin = lower_threshold;

        upper_threshold = upper_threshold < SEMAX_MIN ? SEMAX_MIN : upper_threshold;
        upper_threshold = upper_threshold > SEMAX_MAX ? SEMAX_MAX : upper_threshold;
        cool_config.semax = upper_threshold;
    } else {
        cool_config.semin = SEMIN_OFF;
    }

    write(ADDRESS_COOLCONF, cool_config.bytes);
    cool_step_enabled = status;
}

void TMC2209::setCoolStepCurrentIncrement(CurrentIncrement current_increment)
{
    cool_config.seup = current_increment;
    write(ADDRESS_COOLCONF, cool_config.bytes);
}

void TMC2209::setCoolStepMeasurementCount(MeasurementCount measurement_count)
{
    cool_config.sedn = measurement_count;
    write(ADDRESS_COOLCONF, cool_config.bytes);
}

void TMC2209::setAnalogCurrentScaling(bool status)
{
    global_config.i_scale_analog = status ? 1 : 0;
    writeGlobalConfig();
}

void TMC2209::setExternalSenseResistors(bool status)
{
    global_config.internal_rsense = status ? 0 : 1;
    writeGlobalConfig();
}

uint8_t TMC2209::getVersion()
{
    Input input;
    input.bytes = read(ADDRESS_IOIN);

    return input.version;
}

bool TMC2209::hardwareDisabled()
{
    Input input;
    input.bytes = read(ADDRESS_IOIN);

    return input.enn;
}

DriverConnected TMC2209::getConnected()
{
    if (serialOperationMode())
        return DriverConnected::SETUP;
    else if(getVersion() == VERSION)
        return DriverConnected::CONNECTED;
    else
        return DriverConnected::DISABLED;
}

uint16_t TMC2209::getMicrostepsPerStep()
{
    return 2^(MAX_MICROSTEP_CONFIG - chopper_config.mres);
}

TMC2209::Settings TMC2209::getSettings()
{
    Settings settings;
    settings.is_communicating = getConnected() != DriverConnected::DISABLED;

    if (settings.is_communicating)
    {
        readStoreRegisters();

        settings.is_setup                               = global_config.pdn_disable;
        settings.software_enabled                       = (chopper_config.toff > TOFF_DISABLE);
        settings.microsteps_per_step                    = getMicrostepsPerStep();
        settings.inverse_motor_direction_enabled        = global_config.shaft;
        settings.stealth_chop_enabled                   = not global_config.enable_spread_cycle;
        settings.standstill_mode                        = pwm_config.freewheel;
        settings.irun_percent                           = (driver_current.irun / (CURRENT_SETTING_MAX - CURRENT_SETTING_MIN)) * 100.0f;
        settings.irun_register_value                    = driver_current.irun;
        settings.ihold_percent                          = (driver_current.ihold / (CURRENT_SETTING_MAX - CURRENT_SETTING_MIN)) * 100.0f;
        settings.ihold_register_value                   = driver_current.ihold;
        settings.iholddelay_percent                     = (driver_current.iholddelay / (HOLD_DELAY_MAX - HOLD_DELAY_MIN)) * 100.0f;
        settings.iholddelay_register_value              = driver_current.iholddelay;
        settings.automatic_current_scaling_enabled      = pwm_config.pwm_autoscale;
        settings.automatic_gradient_adaptation_enabled  = pwm_config.pwm_autograd;
        settings.pwm_offset                             = pwm_config.pwm_offset;
        settings.pwm_gradient                           = pwm_config.pwm_grad;
        settings.cool_step_enabled                      = cool_step_enabled;
        settings.analog_current_scaling_enabled         = global_config.i_scale_analog;
        settings.internal_sense_resistors_enabled       = global_config.internal_rsense;
    }
    else
    {
        settings.is_setup                               = false;
        settings.software_enabled                       = false;
        settings.microsteps_per_step                    = 0;
        settings.inverse_motor_direction_enabled        = false;
        settings.stealth_chop_enabled                   = false;
        settings.standstill_mode                        = pwm_config.freewheel;
        settings.irun_percent                           = 0;
        settings.irun_register_value                    = 0;
        settings.ihold_percent                          = 0;
        settings.ihold_register_value                   = 0;
        settings.iholddelay_percent                     = 0;
        settings.iholddelay_register_value              = 0;
        settings.automatic_current_scaling_enabled      = false;
        settings.automatic_gradient_adaptation_enabled  = false;
        settings.pwm_offset                             = 0;
        settings.pwm_gradient                           = 0;
        settings.cool_step_enabled                      = false;
        settings.analog_current_scaling_enabled         = false;
        settings.internal_sense_resistors_enabled       = false;
    }

    return settings;
}

TMC2209::Status TMC2209::getStatus()
{
    DriverStatus driver_status;
    driver_status.bytes = 0;
    driver_status.bytes = read(ADDRESS_DRV_STATUS);
    return driver_status.status;
}

TMC2209::GlobalStatus TMC2209::getGlobalStatus()
{
    GlobalStatusUnion global_status_union;
    global_status_union.bytes = 0;
    global_status_union.bytes = read(ADDRESS_GSTAT);
    return global_status_union.global_status;
}

void TMC2209::clearReset()
{
    GlobalStatusUnion global_status_union;
    global_status_union.bytes = 0;
    global_status_union.global_status.reset = 1;
    write(ADDRESS_GSTAT, global_status_union.bytes);
}

void TMC2209::clearDriverError()
{
    GlobalStatusUnion global_status_union;
    global_status_union.bytes = 0;
    global_status_union.global_status.drv_err = 1;
    write(ADDRESS_GSTAT, global_status_union.bytes);
}

uint8_t TMC2209::getInterfaceTransmissionCounter()
{
    return read(ADDRESS_IFCNT);
}

uint32_t TMC2209::getInterstepDuration()
{
    return read(ADDRESS_TSTEP);
}

uint16_t TMC2209::getStallGuardResult()
{
    return read(ADDRESS_SG_RESULT);
}

uint8_t TMC2209::getPwmScaleSum()
{
    PwmScale pwm_scale;
    pwm_scale.bytes = read(ADDRESS_PWM_SCALE);

    return pwm_scale.pwm_scale_sum;
}

int16_t TMC2209::getPwmScaleAuto()
{
    PwmScale pwm_scale;
    pwm_scale.bytes = read(ADDRESS_PWM_SCALE);

    return pwm_scale.pwm_scale_auto;
}

uint8_t TMC2209::getPwmOffsetAuto()
{
    PwmAuto pwm_auto;
    pwm_auto.bytes = read(ADDRESS_PWM_AUTO);

    return pwm_auto.pwm_offset_auto;
}

uint8_t TMC2209::getPwmGradientAuto()
{
    PwmAuto pwm_auto;
    pwm_auto.bytes = read(ADDRESS_PWM_AUTO);

    return pwm_auto.pwm_gradient_auto;
}

uint16_t TMC2209::getMicrostepCounter()
{
    return read(ADDRESS_MSCNT);
}

void TMC2209::initialize(uint8_t address)
{
    setSerialMode(address);
    setRegisterDefaults();
    clearDriverError();

    minimizeMotorCurrent();
    setEnabled(false);
    setAutomaticCurrentScaling(false);
    setAutomaticGradientAdaptation(false);
}

int TMC2209::serialAvailable()
{
    size_t length = 0;
    if (uart_get_buffered_data_len(uart_port, &length) == ESP_OK)
    {
        return length;
    }

    return 0;
}

size_t TMC2209::serialWrite(uint8_t value)
{
    int written = uart_write_bytes(uart_port, &value, 1);
    if (written < 0)
    {
        return 0;
    }

    return static_cast<size_t>(written);
}

int TMC2209::serialRead()
{
    uint8_t value = 0;
    int read = uart_read_bytes(uart_port, &value, 1, 0);

    if (read == 1)
    {
        return static_cast<int>(value);
    }

    return -1;
}

void TMC2209::setSerialMode(uint8_t address)
{
    serial_address = address;

    global_config.bytes             = 0;
    global_config.i_scale_analog    = 0;
    global_config.pdn_disable       = 1;
    global_config.mstep_reg_select  = 1;
    global_config.multistep_filt    = 1;

    writeGlobalConfig();
}

void TMC2209::setRegisterDefaults()
{
    driver_current.bytes        = 0;
    driver_current.ihold        = IHOLD_DEFAULT;
    driver_current.irun         = IRUN_DEFAULT;
    driver_current.iholddelay   = IHOLDDELAY_DEFAULT;
    write(ADDRESS_IHOLD_IRUN, driver_current.bytes);

    chopper_config.bytes    = CONFIG_DEFAULT;
    chopper_config.tbl      = TBL_DEFAULT;
    chopper_config.hend     = HEND_DEFAULT;
    chopper_config.hstart   = HSTART_DEFAULT;
    chopper_config.toff     = TOFF_DEFAULT;
    write(ADDRESS_CHOPCONF, chopper_config.bytes);

    pwm_config.bytes = PWM_CONFIG_DEFAULT;
    write(ADDRESS_PWMCONF, pwm_config.bytes);

    cool_config.bytes = COOLCONF_DEFAULT;
    write(ADDRESS_COOLCONF, cool_config.bytes);

    write(ADDRESS_TPOWERDOWN, TPOWERDOWN_DEFAULT);
    write(ADDRESS_TPWMTHRS, TPWMTHRS_DEFAULT);
    write(ADDRESS_VACTUAL, VACTUAL_DEFAULT);
    write(ADDRESS_TCOOLTHRS, TCOOLTHRS_DEFAULT);
    write(ADDRESS_SGTHRS, SGTHRS_DEFAULT);
    write(ADDRESS_COOLCONF, COOLCONF_DEFAULT);
}

void TMC2209::readStoreRegisters()
{
    global_config.bytes     = readGlobalConfigBytes();
    chopper_config.bytes    = readChopperConfigBytes();
    pwm_config.bytes        = readPwmConfigBytes();
}

bool TMC2209::serialOperationMode()
{
    GlobalConfig global_config;
    global_config.bytes = readGlobalConfigBytes();

    return global_config.pdn_disable;
}

void TMC2209::minimizeMotorCurrent()
{
    driver_current.irun     = CURRENT_SETTING_MIN;
    driver_current.ihold    = CURRENT_SETTING_MIN;
    writeDriverCurrent();
}

uint32_t TMC2209::reverseData(uint32_t data)
{
    uint32_t reversed_data = 0;
    uint8_t right_shift;
    uint8_t left_shift;
    for (uint8_t i = 0; i < DATA_SIZE; ++i)
    {
        right_shift = (DATA_SIZE - i - 1) * BITS_PER_BYTE;
        left_shift = i * BITS_PER_BYTE;
        reversed_data |= ((data >> right_shift) & BYTE_MAX_VALUE) << left_shift;
    }
    return reversed_data;
}

template <typename Datagram>
uint8_t TMC2209::calculateCrc(Datagram &datagram, uint8_t datagram_size)
{
    uint8_t crc = 0;
    uint8_t byte;
    for (uint8_t i = 0; i < (datagram_size - 1); ++i)
    {
        byte = (datagram.bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
        for (uint8_t j = 0; j < BITS_PER_BYTE; ++j)
        {
            if ((crc >> 7) ^ (byte & 0x01))
            {
                crc = (crc << 1) ^ 0x07;
            }
            else
            {
                crc = crc << 1;
            }
            byte = byte >> 1;
        }
    }
    return crc;
}

template <typename Datagram>
void TMC2209::sendDatagramUnidirectional(Datagram &datagram, uint8_t datagram_size)
{
    uint8_t byte;

    for (uint8_t i = 0; i < datagram_size; ++i)
    {
        byte = (datagram.bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
        serialWrite(byte);
    }
}

template <typename Datagram>
void TMC2209::sendDatagramBidirectional(Datagram &datagram, uint8_t datagram_size)
{
    uint8_t byte;

    uart_wait_tx_done(uart_port, 100);

    while (serialAvailable() > 0) {
        byte = serialRead();
    }

    for (uint8_t i = 0; i < datagram_size; ++i) {
        byte = (datagram.bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
        serialWrite(byte);
    }

    uart_wait_tx_done(uart_port, 100);

    uint32_t echo_delay = 0;
    while ((serialAvailable() < datagram_size) && (echo_delay < ECHO_DELAY_MAX_MICROSECONDS)) {
        ets_delay_us(ECHO_DELAY_INC_MICROSECONDS);

        echo_delay += ECHO_DELAY_INC_MICROSECONDS;
    }

    if (echo_delay >= ECHO_DELAY_MAX_MICROSECONDS)
    {
        return;
    }

    for (uint8_t i = 0; i < datagram_size; ++i) {
        byte = serialRead();
    }
}

void TMC2209::write(uint8_t register_address, uint32_t data)
{
    ReplyDatagram write_datagram;
    write_datagram.bytes            = 0;
    write_datagram.sync             = SYNC;
    write_datagram.serial_address   = serial_address;
    write_datagram.register_address = register_address;
    write_datagram.rw               = RW_WRITE;
    write_datagram.data             = reverseData(data);
    write_datagram.crc              = calculateCrc(write_datagram, REPLY_DATAGRAM_SIZE);

    sendDatagramUnidirectional(write_datagram, REPLY_DATAGRAM_SIZE);
}

uint32_t TMC2209::read(uint8_t register_address)
{
    ReadRequestDatagram read_request_datagram;
    read_request_datagram.bytes             = 0;
    read_request_datagram.sync              = SYNC;
    read_request_datagram.serial_address    = serial_address;
    read_request_datagram.register_address  = register_address;
    read_request_datagram.rw                = RW_READ;
    read_request_datagram.crc               = calculateCrc(read_request_datagram, READ_REQUEST_DATAGRAM_SIZE);

    for (uint8_t retry = 0; retry < MAX_READ_RETRIES; retry++)
    {
        sendDatagramBidirectional(read_request_datagram, READ_REQUEST_DATAGRAM_SIZE);

        uint32_t reply_delay = 0;
        while ((serialAvailable() < REPLY_DATAGRAM_SIZE) && (reply_delay < REPLY_DELAY_MAX_MICROSECONDS))
        {
            ets_delay_us(REPLY_DELAY_INC_MICROSECONDS);

            reply_delay += REPLY_DELAY_INC_MICROSECONDS;
        }

        if (reply_delay >= REPLY_DELAY_MAX_MICROSECONDS)
        {
            return 0;
        }

        uint64_t byte;
        uint8_t byte_count = 0;

        ReplyDatagram read_reply_datagram;
        read_reply_datagram.bytes = 0;

        for (uint8_t i = 0; i < REPLY_DATAGRAM_SIZE; ++i)
        {
            byte = serialRead();
            read_reply_datagram.bytes |= (byte << (byte_count++ * BITS_PER_BYTE));
        }

        auto crc = calculateCrc(read_reply_datagram, REPLY_DATAGRAM_SIZE);
        if (crc == read_reply_datagram.crc)
        {
            return reverseData(read_reply_datagram.data);
        }

        vTaskDelay(pdMS_TO_TICKS(READ_RETRY_DELAY_MS));
    }

    return 0;
}

void TMC2209::writeGlobalConfig()
{
    write(ADDRESS_GCONF, global_config.bytes);
}

uint32_t TMC2209::readGlobalConfigBytes()
{
    return read(ADDRESS_GCONF);
}

void TMC2209::writeDriverCurrent()
{
    write(ADDRESS_IHOLD_IRUN, driver_current.bytes);

    if (driver_current.irun >= SEIMIN_UPPER_LIMIT)
    {
        cool_config.seimin = SEIMIN_UPPER_SETTING;
    }
    else
    {
        cool_config.seimin = SEIMIN_LOWER_SETTING;
    }
    if (cool_step_enabled)
    {
        write(ADDRESS_COOLCONF, cool_config.bytes);
    }
}

void TMC2209::writeChopperConfig()
{
    write(ADDRESS_CHOPCONF, chopper_config.bytes);
}

uint32_t TMC2209::readChopperConfigBytes()
{
    return read(ADDRESS_CHOPCONF);
}

void TMC2209::writePwmConfig()
{
    write(ADDRESS_PWMCONF, pwm_config.bytes);
}

uint32_t TMC2209::readPwmConfigBytes()
{
    return read(ADDRESS_PWMCONF);
}
