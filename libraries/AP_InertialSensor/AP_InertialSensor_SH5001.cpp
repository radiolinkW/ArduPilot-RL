/*#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor_SH5001.h"
#include <utility>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>

// SH5001 registers
#define SH5001_CHIP_ID          0x1F

// FIFO registers and settings
#define SH5001_FIFO_CONF0       0x35
#define SH5001_FIFO_CONF1       0x36
#define SH5001_FIFO_CONF2       0x37
#define SH5001_FIFO_CONF3       0x38
#define SH5001_FIFO_CONF4       0x39
#define SH5001_FIFO_STA0        0x1B
#define SH5001_FIFO_STA1        0x1C
#define SH5001_FIFO_DATA        0x1D

// FIFO configuration bits
#define SH5001_FIFO_MODE_STREAM 0x02
#define SH5001_FIFO_ACC_X_EN    0x01
#define SH5001_FIFO_ACC_Y_EN    0x02
#define SH5001_FIFO_ACC_Z_EN    0x04
#define SH5001_FIFO_GYRO_X_EN   0x08
#define SH5001_FIFO_GYRO_Y_EN   0x10
#define SH5001_FIFO_GYRO_Z_EN   0x20

#define SH5001_ACC_ODR_500HZ	0x01
#define SH5001_ACC_ODR_1000HZ   0x00
#define SH5001_ACC_ODR_2000HZ	0x08
#define SH5001_ACC_RANGE_16G	0x30
#define SH5001_GYRO_ODR_500HZ	0x01
#define SH5001_GYRO_ODR_1000HZ  0x00
#define SH5001_GYRO_ODR_2000HZ	0x08
#define SH5001_GYRO_RANGE_2000	0x60

#define SH5001_ACC_XL           0x00
#define SH5001_ACC_XH           0x01
#define SH5001_ACC_YL           0x02
#define SH5001_ACC_YH           0x03
#define SH5001_ACC_ZL           0x04
#define SH5001_ACC_ZH           0x05
#define SH5001_GYRO_XL          0x06
#define SH5001_GYRO_XH          0x07
#define SH5001_GYRO_YL          0x08
#define SH5001_GYRO_YH          0x09
#define SH5001_GYRO_ZL          0x0A
#define SH5001_GYRO_ZH          0x0B
#define SH5001_TEMP_ZL          0x0C
#define SH5001_TEMP_ZH          0x0D

#define SH5001_ACC_CONF0        0x20
#define SH5001_ACC_CONF1        0x21
#define SH5001_ACC_CONF2        0x22
#define SH5001_GYRO_CONF0       0x23
#define SH5001_GYRO_CONF1       0x24
#define SH5001_GYRO_CONF2       0x25

#define SH5001_POWER_MODE       0x30

// SH5001 chip ID value
#define SH5001_WHOAMI           0xA1

// SH5001 reset value
#define SH5001_WATERMARK_DIV    (12U)
#define SH5001_FIFO_BUFFER      (500U)

#define SH5001_FILTER_EN	(0x01U)
#define SH5001_BYPASS_EN	(0x02U)

#define SH5001_BACKEND_SAMPLE_RATE 1000
const uint32_t BACKEND_PERIOD_US = 1000000UL / SH5001_BACKEND_SAMPLE_RATE;

extern const AP_HAL::HAL& hal;

AP_InertialSensor_SH5001::AP_InertialSensor_SH5001(AP_InertialSensor &imu,
                                                   AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                                   enum Rotation _rotation)
    : AP_InertialSensor_Backend(imu)
    , rotation(_rotation)
    , dev(std::move(_dev))
{
}

AP_InertialSensor_SH5001::~AP_InertialSensor_SH5001()
{
}

AP_InertialSensor_Backend *AP_InertialSensor_SH5001::probe(AP_InertialSensor &imu,
                                                           AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                                           enum Rotation _rotation)
{
    if (!_dev) {
        return nullptr;
    }

    if (_dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
        _dev->set_read_flag(0x80);
    }

    auto sensor = new AP_InertialSensor_SH5001(imu, std::move(_dev), _rotation);
    if (!sensor || !sensor->hardware_init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

bool AP_InertialSensor_SH5001::check_whoami()
{
    uint8_t whoami = register_read(SH5001_CHIP_ID);
    return whoami == SH5001_WHOAMI;
}

void AP_InertialSensor_SH5001::Adjust_Cf()
{
	uint8_t reg_addr[15] = { 0x8C, 0x8D, 0x8E, 0x8F, 0x98, 0x99, 0x9A, 0x9B, 0xA4, 0xA5, 0xA6, 0xA7, 0xC4, 0xC5, 0xC6};
	uint8_t regDataold[15];
	uint8_t regDatanew[15];
	int16_t OldData[6];
	int16_t NewData[6];
	int i = 0;

//	SH5001_read(SH5001_ADDRESS, reg_addr[12], 1, &regDataold[12]);
	regDataold[12] = register_read(reg_addr[12]);
	//printf("Adjust read %x= %x\r\n", reg_addr[12], regDataold[12]);
	if(regDataold[12] != 0x06)
		return;

	for(i = 0; i < 15; i++)
	{
		//SH5001_read(SH5001_ADDRESS, reg_addr[i], 1, &regDataold[i]);
		regDataold[i] = register_read(reg_addr[i]);
		//printf("Adjust read = %x\r\n", regDataold[i]);
	}

	OldData[0] = (short)(regDataold[1] << 8 | regDataold[0]);
	OldData[1] = (short)(regDataold[3] << 8 | regDataold[2]);
	OldData[2] = (short)(regDataold[5] << 8 | regDataold[4]);
	OldData[3] = (short)(regDataold[7] << 8 | regDataold[6]);
	OldData[4] = (short)(regDataold[9] << 8 | regDataold[8]);
	OldData[5] = (short)(regDataold[11] << 8 | regDataold[10]);

	NewData[0] = OldData[0] * 1.693;
	NewData[1] = OldData[1] * 0.588 - 91;
	NewData[2] = OldData[2] * 1.693;
	NewData[3] = OldData[3] * 0.585 - 313;
	NewData[4] = OldData[4] * 1.679;
	NewData[5] = OldData[5] * 0.590 - 143;

	//printf("OldData %d %d %d %d %d %d \r\n",OldData[0], OldData[1], OldData[2], OldData[3], OldData[4], OldData[5]);
	//printf("NewData %d %d %d %d %d %d \r\n",NewData[0], NewData[1], NewData[2], NewData[3], NewData[4], NewData[5]);

	regDatanew[0] = NewData[0] & 0xFF;
	regDatanew[1] = (NewData[0] >> 8) & 0xFF;
	regDatanew[2] = NewData[1] & 0xFF;
	regDatanew[3] = (NewData[1] >> 8) & 0xFF;
	regDatanew[4] = NewData[2] & 0xFF;
	regDatanew[5] = (NewData[2] >> 8) & 0xFF;
	regDatanew[6] = NewData[3] & 0xFF;
	regDatanew[7] = (NewData[3] >> 8) & 0xFF;
	regDatanew[8] = NewData[4] & 0xFF;
	regDatanew[9] = (NewData[4] >> 8) & 0xFF;
	regDatanew[10] = NewData[5] & 0xFF;
	regDatanew[11] = (NewData[5] >> 8) & 0xFF;
	regDatanew[12] = 0x04;
	regDatanew[13] = 0x04;
	regDatanew[14] = 0x04;

	for(i = 0; i < 15; i++)
	{
		//printf("Adjust write = %x\r\n", regDatanew[i]);
		//SH5001_write(SH5001_ADDRESS, reg_addr[i], 1, &regDatanew[i]);
		register_write(reg_addr[i],regDatanew[i]);
	}
}

bool AP_InertialSensor_SH5001::hardware_init()
{
    WITH_SEMAPHORE(dev->get_semaphore());

    dev->setup_checked_registers(8, dev->bus_type() == AP_HAL::Device::BUS_TYPE_I2C?200:20);

    // initially run the bus at low speed
    dev->set_speed(AP_HAL::Device::SPEED_LOW);

    if (!check_whoami()) {
        AP_HAL::panic("SH5001: bad WHOAMI");
        return false;
    }

    //Initialization sequence

    //SoftReset
    register_write(0x2B,0x01);
    register_write(0x00,0x73);
    hal.scheduler->delay(50);

    //DriveStart
    register_write(0x2B,0x01);
    hal.scheduler->delay(2);
    register_write(0x2B,0x00);
    hal.scheduler->delay(1);

    //ADCReset
    register_write(0x30,0x08);
    register_write(0xD2,0x00);
    register_write(0xD1,0x6B);
    register_write(0xD5,0x02);
    hal.scheduler->delay(5);
    register_write(0xD1,0x68);
    register_write(0xD5,0x00);
    register_write(0x30,0x00);
    hal.scheduler->delay(50);

    //CVAReset
    uint8_t regDEData = register_read(0xDE);

    uint8_t regData = regDEData & 0xC7;
    register_write(0xDE,regData);
    hal.scheduler->delay(5);

    regData = regDEData | 0x38;
    register_write(0xDE,regData);
    hal.scheduler->delay(5);

    register_write(0xDE,regDEData);
    hal.scheduler->delay(5);

    register_write(0xCD,0x12);
    register_write(0xCE,0x12);
    register_write(0xCF,0x12);
    hal.scheduler->delay(1);

    register_write(0xCD,0x02);
    register_write(0xCE,0x02);
    register_write(0xCF,0x02);
    hal.scheduler->delay(200);

    //ACC reset
    register_write(0x30,0x08);
    register_write(0xD8,0xE0);
    hal.scheduler->delay(5);

    register_write(0xD8,0x00);
    register_write(0x30,0x00);

    // SH5001_OSC_FREQ
    if(register_read(0xDA) != 0x07){
    	register_write(0xDA,0x07);
    }

    // enable digital filter and bypass
    regData = register_read(SH5001_ACC_CONF0);
    register_write(SH5001_ACC_CONF0,(regData & 0xFC) | SH5001_FILTER_EN | SH5001_BYPASS_EN);
    regData = register_read(SH5001_GYRO_CONF0);
    register_write(SH5001_GYRO_CONF0,(regData & 0x7CU) | SH5001_FILTER_EN | SH5001_BYPASS_EN);

    // set accel config: 16G range, 1kHz ODR
    regData =  register_read(SH5001_ACC_CONF1);
    register_write(SH5001_ACC_CONF1, (regData & 0x80) | SH5001_ACC_RANGE_16G | SH5001_ACC_ODR_1000HZ, true);
    // set gyro config: 2000dps range, 1kHz ODR
    regData =  register_read(SH5001_GYRO_CONF1);
    register_write(SH5001_GYRO_CONF1, (regData & 0x80) | SH5001_GYRO_RANGE_2000 | SH5001_GYRO_ODR_1000HZ, true);

    // configure filters
    regData =  register_read(SH5001_ACC_CONF2);
    register_write(SH5001_ACC_CONF2, (regData & 0xF0) | 0x09); // accel filter BW ~100Hz
    regData =  register_read(SH5001_GYRO_CONF2);
    register_write(SH5001_GYRO_CONF2, (regData & 0xF0U) | 0x09); // gyro filter BW ~100Hz

    //SH5001_IMU_Data_Stablize
    regData = register_read(SH5001_ACC_CONF0);
    regData &= 0xFE;
    register_write(SH5001_ACC_CONF0,regData);
    regData |= 0x1;
    register_write(SH5001_ACC_CONF0,regData);

    regData = register_read(SH5001_GYRO_CONF0);
    regData &= 0xFE;
    register_write(SH5001_GYRO_CONF0,regData);
    regData |= 0x1;
    register_write(SH5001_GYRO_CONF0,regData);

    Adjust_Cf();
    // configure FIFO
    //fifo_setup();

    // set scale factors
    accel_scale = ACCEL_SCALE_16G;
    gyro_scale = GYRO_SCALE_2000DPS;

    dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    return true;
}

void AP_InertialSensor_SH5001::fifo_setup()
{
    WITH_SEMAPHORE(dev->get_semaphore());
    
    // Reset FIFO
    fifo_reset();

    //FIFO Freq: SH5001_FIFO_FREQ_X1_8
    register_write(SH5001_FIFO_CONF4,0x88);

    // Enable accel, gyro in FIFO
    register_write(SH5001_FIFO_CONF3, 
                 (uint8_t)(SH5001_FIFO_ACC_X_EN | SH5001_FIFO_ACC_Y_EN | SH5001_FIFO_ACC_Z_EN |
                 SH5001_FIFO_GYRO_X_EN | SH5001_FIFO_GYRO_Y_EN | SH5001_FIFO_GYRO_Z_EN));

    // Set watermark level
    // fifoWaterMarkLevel <= channelNum*2*(INT(1024/(channelNum*2))-1)
    // eg. fifomode = SH5001_FIFO_ACC_X_EN | SH5001_FIFO_ACC_Y_EN | SH5001_FIFO_ACC_Z_EN
    // then fifoWaterMarkLevel maximum is 1014
    uint8_t regdata = register_read(SH5001_FIFO_CONF2);
    register_write(SH5001_FIFO_CONF2,(regdata & 0x88)| (uint8_t)((120 >> 8)& 0x07));
    register_write(SH5001_FIFO_CONF1,(uint8_t)120);
    
    // FIFO mode: stream mode
    register_write(SH5001_FIFO_CONF0, SH5001_FIFO_MODE_STREAM);

}

uint16_t AP_InertialSensor_SH5001::fifo_get_count()
{
    uint8_t buf[2];
    if (!block_read(SH5001_FIFO_STA0, buf, 2)) {
        return 0;
    }
    return ((buf[1] & 0x0F) << 8) | buf[0];
}

void AP_InertialSensor_SH5001::fifo_reset()
{
    uint8_t regDate =0;
    regDate = register_read(SH5001_FIFO_CONF0);
    regDate |= 0x80;
	register_write(SH5001_FIFO_CONF0, regDate);
}

bool AP_InertialSensor_SH5001::fifo_read_samples(uint8_t n_samples)
{
    uint8_t buf[SH5001_FIFO_BUFFER];
    if (!block_read(SH5001_FIFO_DATA, buf, n_samples)) {
        return false;
    }
    uint8_t j = 0;
	int16_t accData[3] ={0};
	int16_t gyroData[3] ={0};
    for (uint8_t i = 0; i < (uint16_t)(n_samples/SH5001_WATERMARK_DIV); i++) {
        // Parse accel data (6 bytes)
    	accData[0] = (int16_t)((short)(buf[j+1] << 8) | buf[j]);
    	accData[1] = (int16_t)((short)(buf[j+3] << 8) | buf[j+2]);
    	accData[2] = (int16_t)((short)(buf[j+5] << 8) | buf[j+4]);

    	gyroData[0] =  (int16_t)((short)(buf[j+7] << 8) | buf[j+6]);
    	gyroData[1] =  (int16_t)((short)(buf[j+9] << 8) | buf[j+8]);
    	gyroData[2] =  (int16_t)((short)(buf[j+11] << 8) | buf[j+10]);

    	Vector3f accel{
    		(float)accData[0],
    		(float)accData[1],
    		(float)accData[2]
    	};

        Vector3f gyro{
    		(float)gyroData[0],
    		(float)gyroData[1],
    		(float)gyroData[2]
    	  };
        // Process the data (same as read_sensor)
        accel *= accel_scale;
        gyro *= gyro_scale;
        
        _rotate_and_correct_accel(accel_instance, accel);
        _rotate_and_correct_gyro(gyro_instance, gyro);
        
        _notify_new_accel_raw_sample(accel_instance, accel, 0);
        _notify_new_gyro_raw_sample(gyro_instance, gyro);
        j += SH5001_WATERMARK_DIV;
    }
    
    return true;
}

void AP_InertialSensor_SH5001::start()
{
    if (!_imu.get_gyro_instance(gyro_instance) || !_imu.get_accel_instance(accel_instance)) {
        return;
    }

    WITH_SEMAPHORE(dev->get_semaphore());

    // setup sensor rotations from probe()
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);

    // register backend
    if (!_imu.register_gyro(gyro_instance, SH5001_BACKEND_SAMPLE_RATE, dev->get_bus_id_devtype(DEVTYPE_INS_SH5001)) ||
        !_imu.register_accel(accel_instance, SH5001_BACKEND_SAMPLE_RATE, dev->get_bus_id_devtype(DEVTYPE_INS_SH5001))) {
        return;
    }

    // start the timer process to read samples
    periodic_handle = dev->register_periodic_callback(BACKEND_PERIOD_US, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_SH5001::read_sensor, void));
}

bool AP_InertialSensor_SH5001::get_output_banner(char* banner, uint8_t banner_len)
{
    snprintf(banner, banner_len, "IMU%u: SH5001", gyro_instance);
    return true;
}

void AP_InertialSensor_SH5001::read_sensor()
{
    // Check FIFO status
//    uint16_t fifo_count = fifo_get_count();
//    if (fifo_count == 0) {
//        return;
//    }
//
//    // Calculate number of available samples
//    uint16_t samples_available = (uint16_t)(fifo_count / SH5001_WATERMARK_DIV) * SH5001_WATERMARK_DIV;
//    if (samples_available == 0) {
//        return;
//    }
//
//    // Read samples from FIFO
//    if (!fifo_read_samples(samples_available)) {
//        fifo_reset();
//        return;
//    }

	uint8_t buf[12];
	int16_t accData[3] ={0};
	int16_t gyroData[3] ={0};
	if (!block_read(SH5001_ACC_XL, buf, sizeof(buf))) {
		return;
	}
	dev->adjust_periodic_callback(periodic_handle, BACKEND_PERIOD_US);
	accData[0] = (int16_t)((short)(buf[1] << 8) | buf[0]);
	accData[1] = (int16_t)((short)(buf[3] << 8) | buf[2]);
	accData[2] = (int16_t)((short)(buf[5] << 8) | buf[4]);

	gyroData[0] =  (int16_t)((short)(buf[7] << 8) | buf[6]);
	gyroData[1] =  (int16_t)((short)(buf[9] << 8) | buf[8]);
	gyroData[2] =  (int16_t)((short)(buf[11] << 8) | buf[10]);

	Vector3f accel{
		(float)accData[0],
		(float)accData[1],
		(float)accData[2]
	};

    Vector3f gyro{
		(float)gyroData[0],
		(float)gyroData[1],
		(float)gyroData[2]
	  };

    accel *= accel_scale;
	gyro *= gyro_scale;
	_rotate_and_correct_accel(accel_instance, accel);
	_rotate_and_correct_gyro(gyro_instance, gyro);

	_notify_new_accel_raw_sample(accel_instance, accel, 0);
	_notify_new_gyro_raw_sample(gyro_instance, gyro);

}

bool AP_InertialSensor_SH5001::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

void AP_InertialSensor_SH5001::accumulate()
{
    // nothing to do
}

bool AP_InertialSensor_SH5001::block_read(uint8_t reg, uint8_t *buf, uint32_t size)
{
    return dev->read_registers(reg, buf, size);
}

uint8_t AP_InertialSensor_SH5001::register_read(uint8_t reg)
{
    uint8_t val = 0;
    dev->read_registers(reg, &val, 1);
    return val;
}

void AP_InertialSensor_SH5001::register_write(uint8_t reg, uint8_t val, bool checked)
{
    dev->write_register(reg, val, checked);
}
*/
