//#pragma once
//
//#include <AP_HAL/AP_HAL.h>
//#include <AP_HAL/utility/OwnPtr.h>
//#include <AP_Math/AP_Math.h>
//#include <Filter/LowPassFilter2p.h>
//
//#include "AP_InertialSensor.h"
//#include "AP_InertialSensor_Backend.h"
//
//
//class AP_InertialSensor_SH5001 : public AP_InertialSensor_Backend
//{
//public:
//    virtual ~AP_InertialSensor_SH5001();
//
//    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
//                                            AP_HAL::OwnPtr<AP_HAL::Device> dev,
//                                            enum Rotation rotation);
//
//    /* update accel and gyro state */
//    bool update() override;
//    void accumulate() override;
//
//    void start() override;
//    bool get_output_banner(char* banner, uint8_t banner_len) override;
//
//private:
//    AP_InertialSensor_SH5001(AP_InertialSensor &imu,
//                             AP_HAL::OwnPtr<AP_HAL::Device> dev,
//                             enum Rotation rotation);
//
//    /* Initialize sensor*/
//    bool hardware_init();
//    bool check_whoami();
//    void Adjust_Cf();
//    void fifo_reset();
//
//    /* FIFO operations */
//    void fifo_setup();
//    uint16_t fifo_get_count();
//    bool fifo_read_samples(uint8_t n_samples);
//
//    /* Read samples */
//    void read_sensor();
//
//    bool block_read(uint8_t reg, uint8_t *buf, uint32_t size);
//    uint8_t register_read(uint8_t reg);
//    void register_write(uint8_t reg, uint8_t val, bool checked=false);
//
//    // instance numbers of accel and gyro data
//    uint8_t gyro_instance;
//    uint8_t accel_instance;
//
//    const enum Rotation rotation;
//
//    // scale factors
//    static constexpr float SCALE_RANGE_16BIT = 32768; // 2^15;
//    static constexpr float GYRO_SCALE_2000DPS = radians(1) / (SCALE_RANGE_16BIT / 2000.0);
//    static constexpr float ACCEL_SCALE_16G = (GRAVITY_MSS / (SCALE_RANGE_16BIT / 16));
//    // accel and gyro scaling
//    float accel_scale;
//    float gyro_scale;
//
//    AP_HAL::OwnPtr<AP_HAL::Device> dev;
//    AP_HAL::Device::PeriodicHandle periodic_handle;
//
//    float temp_filtered;
//    LowPassFilter2pFloat temp_filter;
//};
