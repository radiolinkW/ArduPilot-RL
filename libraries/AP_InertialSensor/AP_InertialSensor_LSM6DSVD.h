#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

/* enable debug to see a register dump on startup */
#define LSM6DSVD_DEBUG 0

class AP_InertialSensor_LSM6DSVD : public AP_InertialSensor_Backend
{
public:
    virtual ~AP_InertialSensor_LSM6DSVD() { }
    void start(void) override;
    bool update() override;

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                            enum Rotation rotation);
private:
    AP_InertialSensor_LSM6DSVD(AP_InertialSensor &imu,
                              AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                              enum Rotation rotation);

    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

    struct PACKED sensor_raw_data {
        int16_t x;
        int16_t y;
        int16_t z;
    };

    void _poll_data();
    void _fifo_reset();

    bool _init_sensor();
    bool _hardware_init();

    void _gyro_init();
    void _accel_init();

    void _set_gyro_scale();
    void _set_accel_scale();

    uint8_t _register_read(uint8_t reg);
    void _register_write(uint8_t reg, uint8_t val, bool checked=false);
    bool block_read(uint8_t reg, uint8_t *buf, uint32_t size);
    void accumulate_samples(uint16_t n_samples);

    #if LSM6DSVD_DEBUG
    void        _dump_registers();
    uint8_t lsm_reg[150];
    #endif

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;
    AP_HAL::Semaphore *_spi_sem;
    float _gyro_scale;
    float _accel_scale;
    uint8_t _gyro_instance;
    uint8_t _accel_instance;
    enum Rotation _rotation;

    // temp scaling for FIFO temperature
    float temp_sensitivity = 1/256;
    const float temp_zero = 25; // degC

    float temp_filtered;
    LowPassFilter2pFloat temp_filter;
};
