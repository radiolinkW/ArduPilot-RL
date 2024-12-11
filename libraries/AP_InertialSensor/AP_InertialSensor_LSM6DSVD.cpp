/*
 * AP_InertialSensor_LSM6DSVD.cpp
 *
 *  Created on: Sep 7, 2023
 *      Author: chao
 */
/*
 *  This program is free software
*/
#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor_LSM6DSVD.h"

#include <utility>

#include <AP_HAL/GPIO.h>

#include <hal.h>

extern const AP_HAL::HAL& hal;

#define WHO_AM_I          0x71U
#define FIFO_WATERMARK    32
#define TEMP_SEN			1/256
#define TEMP_ZERO			25
/*
 *  Accelerometer and Gyroscope registers
*/
#define LSM6DSVD_WHO_AM_I                    0x0FU
#define LSM6DSVD_CTRL3                       0x12U
#   define LSM6DSVD_CTRL3_SW_RESET				(0x1 << 0)
#   define LSM6DSVD_CTRL3_IF_INC				(0x1 << 2)
#   define LSM6DSVD_CTRL3_BDU				(0x1 << 6)
#   define LSM6DSVD_CTRL3_BOOT				(0x1 << 7)
#define LSM6DSVD_FUNC_CFG_ACCESS              0x1U
#   define LSM6DSVD_FUNC_CFG_ACCESS_SW_POR				(0x1 << 2)
#   define LSM6DSVD_FUNC_CFG_ACCESS_EMB_FUNC_REG_ACCESS				(0x1 << 7)
#define LSM6DSVD_FIFO_CTRL1                   0x7U
#define LSM6DSVD_FIFO_CTRL3                  0x9U
#	define LSM6DSVD_XL_BATCHED_AT_960Hz			0x9
#	define LSM6DSVD_GY_BATCHED_AT_960Hz			(0x9 << 4)
#	define LSM6DSVD_XL_BATCHED_AT_7680Hz			0xc
#	define LSM6DSVD_GY_BATCHED_AT_7680Hz			(0xc << 4)
#define LSM6DSVD_FIFO_CTRL4                  0x0AU
#	define LSM6DSVD_TEMP_BATCHED_AT_60Hz		(0x3 << 4)
#	define LSM6DSVD_BYPASS_MODE					0x0
#	define LSM6DSVD_STREAM_MODE					0x6
#define LSM6DSVD_CTRL6                       0x15U
#	define LSM6DSVD_2000dps						0x4
#define LSM6DSVD_CTRL2                       0x11U
#	define GYRO_HIGH_PER						(0x0 << 4)
#	define LSM6DSVD_ODR_AT_960Hz				0x9
#	define LSM6DSVD_ODR_AT_7680Hz				0xc
#define LSM6DSVD_CTRL4                       0x13U
#	define LSM6DSVD_DRDY_MASK					(0x1 << 3)
#define LSM6DSVD_CTRL6                       0x15U
#	define LSM6DSVD_GY_ULTRA_LIGHT				(0x0 << 4)
#define LSM6DSVD_CTRL7                       0x16U
#	define LSM6DSVD_LPF1_G_EN					(0x1 << 0)
#define LSM6DSVD_CTRL8                       0x17U
#	define LSM6DSVD_16g 						(0x3 << 0)
#	define LSM6DSVD_XL_STRONG					(0x4 << 5)
#define LSM6DSVD_CTRL9                       0x18U
#	define LSM6DSVD_LPF2_XL_EN					(0x1 << 3)
#define LSM6DSVD_CTRL1                       0x10U
#define LSM6DSVD_FIFO_STATUS1                0x1BU
#define LSM6DSVD_EMB_FUNC_CFG                0x63U
#	define LSM6DSVD_EMB_FUNC_IRQ_MASK_G_SETTL	(0x1 << 5)
#	define LSM6DSVD_EMB_FUNC_IRQ_MASK_XL_SETTL	(0x1 << 4)
#define LSM6DSVD_UI_INT_OIS                 0x6FU
#	define LSM6DSVD_DRDY_MASK_OIS				(0x1 << 6)
#define LSM6DSVD_FIFO_DATA_OUT_TAG           0x78U
#	define LSM6DSVD_FIFO_EMPTY						0x00
#	define LSM6DSVD_GY_NC_TAG						0x01
#	define LSM6DSVD_XL_NC_TAG						0x02
#	define LSM6DSVD_TEMPERATURE_TAG					0x03

typedef struct
{
	uint8_t tag;
	union{
	uint8_t data_8[6];
	int16_t	data_16[3];
	};
} fifo_data;


AP_InertialSensor_LSM6DSVD::AP_InertialSensor_LSM6DSVD(AP_InertialSensor &imu,
                                                     AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                                     enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
    , _rotation(rotation)

{
}

AP_InertialSensor_Backend *AP_InertialSensor_LSM6DSVD::probe(AP_InertialSensor &_imu,
                                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                                            enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    AP_InertialSensor_LSM6DSVD *sensor =
        new AP_InertialSensor_LSM6DSVD(_imu,std::move(dev),
                                      rotation);
    if (!sensor || !sensor->_init_sensor()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_InertialSensor_LSM6DSVD::_init_sensor()
{
    _spi_sem = _dev->get_semaphore();

    bool success = _hardware_init();

#if LSM6DSVD_DEBUG
    _dump_registers();
#endif
    return success;
}

bool AP_InertialSensor_LSM6DSVD::_hardware_init()
{
    _spi_sem->take_blocking();

    uint8_t whoami;

    // set flag for reading registers
    _dev->set_read_flag(0x80);

    whoami = _register_read(LSM6DSVD_WHO_AM_I);
    if (whoami != WHO_AM_I) {
        DEV_PRINTF("LSM6DSVD: unexpected acc/gyro WHOAMI 0x%x\n", whoami);
        goto fail_whoami;
    }

    _fifo_reset();

    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    _gyro_init();
    _accel_init();

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    hal.scheduler->delay(10);

    _spi_sem->give();
    return true;

fail_whoami:
    _spi_sem->give();
    return false;
}

void AP_InertialSensor_LSM6DSVD::_fifo_reset()
{
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    /* Restore default configuration */
    _dev->write_register(LSM6DSVD_CTRL3, LSM6DSVD_CTRL3_BOOT | 0x44);
    //hal.scheduler->delay_microseconds(1);
    do{
    	//hal.scheduler->delay_microseconds(1);
    }while(!(((_register_read(LSM6DSVD_CTRL3) & (LSM6DSVD_CTRL3_BOOT | LSM6DSVD_CTRL3_SW_RESET)) > 0) &&
    		((_register_read(LSM6DSVD_FUNC_CFG_ACCESS) & LSM6DSVD_FUNC_CFG_ACCESS_SW_POR) == 0)));

    /* Enable Block Data Update */
    int reg_ctrl3 = _register_read(LSM6DSVD_CTRL3);
    _dev->write_register(LSM6DSVD_CTRL3, reg_ctrl3 | LSM6DSVD_CTRL3_BDU);

    /*
       * Set FIFO watermark (number of unread sensor data TAG + 6 bytes
       * stored in FIFO) to FIFO_WATERMARK samples
       */
    //int watermark = _register_read(LSM6DSVD_FIFO_CTRL1);
    _dev->write_register(LSM6DSVD_FIFO_CTRL1, FIFO_WATERMARK);

    /* Set FIFO batch XL/Gyro ODR to 960Hz */
    _dev->write_register(LSM6DSVD_FIFO_CTRL3, LSM6DSVD_XL_BATCHED_AT_960Hz | LSM6DSVD_GY_BATCHED_AT_960Hz);
    //_dev->write_register(LSM6DSVD_FIFO_CTRL3, LSM6DSVD_XL_BATCHED_AT_7680Hz | LSM6DSVD_GY_BATCHED_AT_7680Hz);

    //set temp fifo
    uint8_t reg_fifo_ctrl4 = _register_read(LSM6DSVD_FIFO_CTRL4);
    _dev->write_register(LSM6DSVD_FIFO_CTRL4, (reg_fifo_ctrl4 & ~0x30) | LSM6DSVD_TEMP_BATCHED_AT_60Hz);

#if 0
    //set filt
    uint8_t reg_ctrl4 = _register_read(LSM6DSVD_CTRL4);
    _dev->write_register(LSM6DSVD_CTRL4, (reg_ctrl4 & ~0x08) | LSM6DSVD_DRDY_MASK);
    uint8_t reg_emb_func_cfg = _register_read(LSM6DSVD_EMB_FUNC_CFG);
    _dev->write_register(LSM6DSVD_EMB_FUNC_CFG, (reg_emb_func_cfg & ~0x30) |
    					LSM6DSVD_EMB_FUNC_IRQ_MASK_G_SETTL | LSM6DSVD_EMB_FUNC_IRQ_MASK_XL_SETTL);
    uint8_t reg_ui_int_ois = _register_read(LSM6DSVD_UI_INT_OIS);
    _dev->write_register(LSM6DSVD_UI_INT_OIS, reg_ui_int_ois & ~0x40);
    uint8_t reg_ctrl7 = _register_read(LSM6DSVD_CTRL7);
    _dev->write_register(LSM6DSVD_CTRL7, (reg_ctrl7 & ~0x1) | LSM6DSVD_LPF1_G_EN);
    uint8_t reg_ctrl6 = _register_read(LSM6DSVD_CTRL6);
    _dev->write_register(LSM6DSVD_CTRL6, (reg_ctrl6 & ~0x70) | LSM6DSVD_GY_ULTRA_LIGHT);
    uint8_t reg_ctrl9 = _register_read(LSM6DSVD_CTRL9);
    _dev->write_register(LSM6DSVD_CTRL9, (reg_ctrl9 & ~0x08) | LSM6DSVD_LPF2_XL_EN);
    uint8_t reg_ctrl8 = _register_read(LSM6DSVD_CTRL8);
    _dev->write_register(LSM6DSVD_CTRL8, (reg_ctrl8 & ~0xE0) | LSM6DSVD_XL_STRONG);
#endif

    /* Set FIFO mode to Stream mode (aka Continuous Mode) */
    reg_fifo_ctrl4 = _register_read(LSM6DSVD_FIFO_CTRL4);
    _dev->write_register(LSM6DSVD_FIFO_CTRL4, (reg_fifo_ctrl4 & ~0x07) | LSM6DSVD_STREAM_MODE);

    hal.scheduler->delay_microseconds(1);

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    notify_accel_fifo_reset(_accel_instance);
    notify_gyro_fifo_reset(_gyro_instance);
}

/*
  start the sensor going
 */
void AP_InertialSensor_LSM6DSVD::start(void)
{
    if (!_imu.register_gyro(_gyro_instance, 960, _dev->get_bus_id_devtype(DEVTYPE_INS_LSM6DSVD)) ||
        !_imu.register_accel(_accel_instance, 960, _dev->get_bus_id_devtype(DEVTYPE_INS_LSM6DSVD))) {
        return;
    }

    set_accel_orientation(_accel_instance, _rotation);
    set_gyro_orientation(_gyro_instance, _rotation);

    //_set_accel_max_abs_offset(_accel_instance, 5.0f);

    /* start the timer process to read samples */
    _dev->register_periodic_callback(1000000UL / 960, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_LSM6DSVD::_poll_data, void));
}

uint8_t AP_InertialSensor_LSM6DSVD::_register_read(uint8_t reg)
{
    uint8_t val = 0;
    _dev->read_registers(reg, &val, 1);
    return val;
}


void AP_InertialSensor_LSM6DSVD::_register_write(uint8_t reg, uint8_t val, bool checked)
{
    _dev->write_register(reg, val, checked);
}

void AP_InertialSensor_LSM6DSVD::_gyro_init()
{
	/* Set full scale */
    int reg_ctrl6 = _register_read(LSM6DSVD_CTRL6);
    _dev->write_register(LSM6DSVD_CTRL6, (reg_ctrl6 & ~0x0f) | LSM6DSVD_2000dps);
    _set_gyro_scale();
    hal.scheduler->delay(1);

    /* Set Output Data Rate */
    _dev->write_register(LSM6DSVD_CTRL2, LSM6DSVD_ODR_AT_960Hz | GYRO_HIGH_PER);
    //_dev->write_register(LSM6DSVD_CTRL2, LSM6DSVD_ODR_AT_7680Hz | GYRO_HIGH_PER);
    hal.scheduler->delay(1);
}

void AP_InertialSensor_LSM6DSVD::_accel_init()
{
	/* Set full scale */
	int reg_ctrl8 = _register_read(LSM6DSVD_CTRL8);
	_dev->write_register(LSM6DSVD_CTRL8, (reg_ctrl8 & ~0x03) | LSM6DSVD_16g);
	_set_accel_scale();
	hal.scheduler->delay(1);

	/* Set Output Data Rate */
	_dev->write_register(LSM6DSVD_CTRL1, LSM6DSVD_ODR_AT_960Hz | GYRO_HIGH_PER);
	//_dev->write_register(LSM6DSVD_CTRL1, LSM6DSVD_ODR_AT_7680Hz | GYRO_HIGH_PER);
	hal.scheduler->delay(1);
}

void AP_InertialSensor_LSM6DSVD::_set_gyro_scale()
{
	//2000dps
    _gyro_scale = 0.0174532f / 16.4f;
}

void AP_InertialSensor_LSM6DSVD::_set_accel_scale()
{
    //16g
    /* convert to G/LSB to (m/s/s)/LSB */
    _accel_scale = GRAVITY_MSS / 2048;
}

/**
 * Timer process to poll for new data from the LSM6DSVD.
 */
void AP_InertialSensor_LSM6DSVD::_poll_data()
{
	//palToggleLine(PAL_LINE(GPIOC,4U));

	uint8_t buff[2];
	_dev->read_registers(LSM6DSVD_FIFO_STATUS1, buff, 2);
	uint16_t samples = (buff[1] & 0x1)*256 + buff[0];

	//_dump_registers();
	if(samples >= FIFO_WATERMARK){
		//clear fifo
		uint8_t reg_fifo_ctrl4 = _register_read(LSM6DSVD_FIFO_CTRL4);
		_dev->write_register(LSM6DSVD_FIFO_CTRL4, (reg_fifo_ctrl4 & ~0x07) | LSM6DSVD_BYPASS_MODE);

		hal.scheduler->delay_microseconds(1);

		//set stream mode
		_dev->write_register(LSM6DSVD_FIFO_CTRL4, (reg_fifo_ctrl4 & ~0x07) | LSM6DSVD_STREAM_MODE);

		goto check_registers;
	}
	if(samples == 0){
		return;
	}
	accumulate_samples(samples);

    // check next register value for correctness
check_registers:
    AP_HAL::Device::checkreg reg;
    if (!_dev->check_next_register(reg)) {
        log_register_change(_dev->get_bus_id(), reg);
        _inc_accel_error_count(_accel_instance);
    }

}

#if 0
void AP_InertialSensor_LSM6DSVD::accumulate_samples(uint16_t n_samples){
	fifo_data raw_data;
	int32_t raw_accx = 0,raw_accy = 0,raw_accz = 0;
	int32_t raw_gyrox = 0,raw_gyroy = 0,raw_gyroz = 0;
	uint8_t num_acc = 0,num_gyro = 0;

	while(n_samples--){
		if (!_dev->read_registers(LSM6DSVD_FIFO_DATA_OUT_TAG, (uint8_t *)&raw_data, 7)) {
		     DEV_PRINTF("LSM6DSVD: error reading data\n");
		     return;
		 }
#if 1
		//Don't know why the data is inverted!
		uint8_t temp;

		temp = raw_data.data_8[0];
		raw_data.data_8[0] = raw_data.data_8[1];
		raw_data.data_8[1] = temp;

		temp = raw_data.data_8[2];
		raw_data.data_8[2] = raw_data.data_8[3];
		raw_data.data_8[3] = temp;

		temp = raw_data.data_8[4];
		raw_data.data_8[4] = raw_data.data_8[5];
		raw_data.data_8[5] = temp;

		//palToggleLine(PAL_LINE(GPIOC,4U));
		switch (raw_data.tag >> 3)
		  {
		case LSM6DSVD_FIFO_EMPTY:
			break;
		case LSM6DSVD_GY_NC_TAG:
			//palToggleLine(PAL_LINE(GPIOA,15U));
			raw_gyrox += raw_data.data_16[0];
			raw_gyroy += raw_data.data_16[1];
			raw_gyroz += raw_data.data_16[2];
			num_gyro++;
			break;
		case LSM6DSVD_XL_NC_TAG:
			//palToggleLine(PAL_LINE(GPIOC,15U));
			raw_accx += raw_data.data_16[0];
			raw_accy += raw_data.data_16[1];
			raw_accz += raw_data.data_16[2];
			num_acc++;
			break;
		case LSM6DSVD_TEMPERATURE_TAG:
		{
			palToggleLine(PAL_LINE(GPIOA,15U));
			float temperature = ((float)raw_data.data_16[0]/256.0f) + temp_zero;
			temp_filtered = temp_filter.apply(temperature);
		}
			break;
		default:
		    break;
		  }
#endif
	}


	Vector3f accel_data{float(raw_accx), float(raw_accy), float(raw_accz)};
	accel_data /= num_acc;
	accel_data *= _accel_scale;

	Vector3f gyro_data{float(raw_gyrox), float(raw_gyroy), float(raw_gyroz)};
	gyro_data /= num_gyro;
	gyro_data *= _gyro_scale;

//	Vector3f accel_data{0, 0, 9.80665};
//	Vector3f gyro_data{0, 0, 0};

	_rotate_and_correct_accel(_accel_instance, accel_data);
	_notify_new_accel_raw_sample(_accel_instance, accel_data);

	_rotate_and_correct_gyro(_gyro_instance, gyro_data);
	_notify_new_gyro_raw_sample(_gyro_instance, gyro_data);
}
#endif

void AP_InertialSensor_LSM6DSVD::accumulate_samples(uint16_t n_samples){
	uint8_t fifo[7];
	int32_t raw_accx = 0,raw_accy = 0,raw_accz = 0;
	int32_t raw_gyrox = 0,raw_gyroy = 0,raw_gyroz = 0;
	uint8_t num_acc = 0,num_gyro = 0;

	while(n_samples--){
		if (!_dev->read_registers(LSM6DSVD_FIFO_DATA_OUT_TAG, fifo, 7)) {
		     DEV_PRINTF("LSM6DSVD: error reading data\n");
		     return;
		 }

		int16_t data[3];
		memcpy(data, &fifo[1], 6);

		//palToggleLine(PAL_LINE(GPIOC,4U));
		switch (fifo[0] >> 3)
		  {
		case LSM6DSVD_FIFO_EMPTY:
			break;
		case LSM6DSVD_GY_NC_TAG:
			//palToggleLine(PAL_LINE(GPIOA,15U));
			raw_gyrox += data[0];
			raw_gyroy += data[1];
			raw_gyroz += data[2];
			num_gyro++;
			break;
		case LSM6DSVD_XL_NC_TAG:
			//palToggleLine(PAL_LINE(GPIOC,15U));
			raw_accz += data[0];
			raw_accy += data[1];
			raw_accx += data[2];
			num_acc++;
			break;
		case LSM6DSVD_TEMPERATURE_TAG:
		{
			//palToggleLine(PAL_LINE(GPIOA,15U));
			float temperature = ((float)data[0]/256.0f) + temp_zero;
			temp_filtered = temp_filter.apply(temperature);
		}
			break;
		default:
		    break;
		  }
	}

	if(num_acc > 0){
		Vector3f accel_data{float(raw_accx), float(raw_accy), float(raw_accz)};
		accel_data /= num_acc;
		accel_data *= _accel_scale;

		_rotate_and_correct_accel(_accel_instance, accel_data);
		_notify_new_accel_raw_sample(_accel_instance, accel_data);
	}

	if(num_gyro > 0){
		Vector3f gyro_data{float(raw_gyrox), float(raw_gyroy), float(raw_gyroz)};
		gyro_data /= num_gyro;
		gyro_data *= _gyro_scale;

		_rotate_and_correct_gyro(_gyro_instance, gyro_data);
		_notify_new_gyro_raw_sample(_gyro_instance, gyro_data);
	}

}

bool AP_InertialSensor_LSM6DSVD::block_read(uint8_t reg, uint8_t *buf, uint32_t size)
{
    return _dev->read_registers(reg, buf, size);
}

bool AP_InertialSensor_LSM6DSVD::update()
{
    update_gyro(_gyro_instance);
    update_accel(_accel_instance);
    _publish_temperature(_accel_instance, temp_filtered);
    return true;
}

#if LSM6DSVD_DEBUG
/*
 *  dump all config registers - used for debug
*/
void AP_InertialSensor_LSM6DSVD::_dump_registers(void)
{
    hal.console->println("LSM6DSVD registers:");

    const uint8_t first = LSM6DSVD_FUNC_CFG_ACCESS;
    const uint8_t last = 0x7e;
    for (uint8_t reg=first; reg<=last; reg++) {
        uint8_t v = _register_read(reg);
        //hal.console->printf("%02x:%02x ", reg, v);
        lsm_reg[reg] = 0;
        lsm_reg[reg] = v;
        if ((reg - (first-1)) % 16 == 0) {
            //hal.console->printf("\n");
        }
    }
    //hal.console->printf("\n");
}
#endif




