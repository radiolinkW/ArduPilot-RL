/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <utility>

#include <AP_HAL/AP_HAL.h>

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_InertialSensor_BMI323.h"

#include <hal.h>

// BMI323 registers (not the complete list)
enum BMI323Register : uint8_t {
    BMI323_REG_CHIP_ID = 0x00,
    BMI323_REG_STATUS = 0x02,
	BMI323_REG_TEMP_DATA = 0x09,
    BMI323_REG_FIFO_FILL_LEVEL = 0x15,
	BMI323_REG_FIFO_DATA = 0x16,
    BMI323_REG_ACC_CONF = 0x20,
    BMI323_REG_GYR_CONF = 0x21,
	BMI323_REG_FIFO_CONF = 0x36,
	BMI323_REG_FIFO_CTRL = 0x37,
    BMI323_REG_CMD = 0x7E,
};

/*
 * End of Bosch microcode copyright
 */

#define BMI323_CHIP_ID 									0x0043
#define BMI325_CHIP_ID 									0x0045
#define BMI323_CMD_SOFT_RESET 							0xDEAF

#define BMI323_DRDY_TEMP_MASK                         	0x0020

#define BMI323_ACC_ODR_1600HZ 						    0x0C
#define BMI323_ACC_ODR_6400HZ                           0x0E
#define BMI323_ACC_RANGE_16G                           (0x03 << 4)
#define BMI323_ACC_BW_ODR_HALF                         (0x00 << 7)
#define BMI323_ACC_BW_ODR_QUARTER                      (0x01 << 7)
#define BMI323_ACC_AVG4                                (0x02 << 8)
#define BMI323_ACC_MODE_HIGH_PERF                      (0x07 << 12)

#define BMI323_GYR_ODR_1600HZ                           0x0C
#define BMI323_GYR_ODR_6400HZ                           0x0E
#define BMI323_GYR_RANGE_2000DPS                       (0x04 << 4)
#define BMI323_GYR_BW_ODR_HALF                         (0x00 << 7)
#define BMI323_GYR_BW_ODR_QUARTER                      (0x01 << 7)
#define BMI323_GYR_AVG4                                (0x02 << 8)
#define BMI323_GYR_MODE_HIGH_PERF                      (0x07 << 12)

#define BMI323_FIFO_STOP_ON_FULL                       (0x0001)
#define BMI323_FIFO_TIME_EN                            (0x0100)
#define BMI323_FIFO_ACC_EN                             (0x0200)
#define BMI323_FIFO_GYR_EN                             (0x0400)
#define BMI323_FIFO_TEMP_EN                            (0x0800)
#define BMI323_FIFO_ALL_EN                             (0x0F00)

#define BMI323_FIFO_FLUSH								0x0001

#define BMI323_BACKEND_SAMPLE_RATE 1600

extern const AP_HAL::HAL& hal;

AP_InertialSensor_BMI323::AP_InertialSensor_BMI323(AP_InertialSensor &imu,
                                                   AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                                   enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
    , _rotation(rotation)
{
}

AP_InertialSensor_Backend *
AP_InertialSensor_BMI323::probe(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    auto sensor = new AP_InertialSensor_BMI323(imu, std::move(dev), rotation);

    if (!sensor) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_InertialSensor_Backend *
AP_InertialSensor_BMI323::probe(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    auto sensor = new AP_InertialSensor_BMI323(imu, std::move(dev), rotation);

    if (!sensor) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_BMI323::start()
{
    _dev->get_semaphore()->take_blocking();

    configure_fifo();

    configure_accel();

    configure_gyro();

    //_dump_registers();

    _dev->get_semaphore()->give();

    int BMI323_SAMPLE_RATE = BMI323_BACKEND_SAMPLE_RATE;

#ifdef BMI323_FAST_SAMPLE
    //On the stm32f405 chip, arm the FC log will be blocked
    if(enable_fast_sampling(_accel_instance)){
    	BMI323_SAMPLE_RATE = 6400;
    	GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IMU:BMI323 fast sampling enabled 6.4kHz");
    }
#endif

    enum DevTypes devtype;

    switch(dev_type){
    case BOSCH_IMU::BMI323:
    	devtype = DEVTYPE_INS_BMI323;
    	break;
    case BOSCH_IMU::BMI325:
    	devtype = DEVTYPE_INS_BMI325;
    	break;
    default:
    	devtype = DEVTYPE_INS_BMI323;
    	break;
    }

    if (!_imu.register_accel(_accel_instance, BMI323_SAMPLE_RATE, _dev->get_bus_id_devtype(devtype)) ||
        !_imu.register_gyro(_gyro_instance, BMI323_SAMPLE_RATE, _dev->get_bus_id_devtype(devtype))) {
        return;
    }

    // setup sensor rotations from probe()
    set_gyro_orientation(_gyro_instance, _rotation);
    set_accel_orientation(_accel_instance, _rotation);

    //16g
    /* convert to G/LSB to (m/s/s)/LSB */
    _accel_scale = GRAVITY_MSS / 2048;
    //2000dps
    _gyro_scale = 0.0174532f / 16.4f;

    /* Call read_fifo() at 1600Hz */
    periodic_handle = _dev->register_periodic_callback(1000000/BMI323_SAMPLE_RATE, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_BMI323::read_fifo, void));
}

bool AP_InertialSensor_BMI323::update()
{
    update_accel(_accel_instance);
    update_gyro(_gyro_instance);
    return true;
}

/*
  16bit register in BMI323
*/
bool AP_InertialSensor_BMI323::read_registers(uint8_t reg, uint16_t *data, uint8_t len)
{
    // for SPI we need to discard the first returned byte. See
    // datasheet for explanation
    uint8_t b[len*2+2];
    b[0] = reg | 0x80;
    memset(&b[1], 0, len*2+1);
    if (!_dev->transfer(b, len*2+2, b, len*2+2)) {
        return false;
    }
    for(int i=1;i<=len;i++){
    	*data = b[i*2] + (b[i*2+1] << 8);
    	data++;
    }
    //memcpy(data, &b[2], len*2);
    return true;
}

/*
  16bit register in BMI323
*/
bool AP_InertialSensor_BMI323::write_register(uint8_t reg, uint16_t v)
{
    uint8_t data[3];
    data[0] = reg & 0x7F;
    data[1] = v & 0x00FF;
    data[2] = v >> 8;

    if(!_dev->transfer(data, 3, nullptr, 0)){
    	return false;
    }
    return true;
}

void AP_InertialSensor_BMI323::configure_accel()
{
    //set 16g range, 1600hz odr, high performance,1/4 BW is 3 db cutoff frequency
#ifdef BMI323_FAST_SAMPLE
	if(!enable_fast_sampling(_accel_instance)){
		write_register(BMI323_REG_ACC_CONF, BMI323_ACC_ODR_1600HZ | BMI323_ACC_RANGE_16G |
											BMI323_ACC_BW_ODR_QUARTER | BMI323_ACC_MODE_HIGH_PERF);
	}else{
		write_register(BMI323_REG_ACC_CONF, BMI323_ACC_ODR_6400HZ | BMI323_ACC_RANGE_16G |
											BMI323_ACC_BW_ODR_QUARTER | BMI323_ACC_MODE_HIGH_PERF);
	}
#else
	write_register(BMI323_REG_ACC_CONF, BMI323_ACC_ODR_1600HZ | BMI323_ACC_RANGE_16G |
											BMI323_ACC_BW_ODR_QUARTER | BMI323_ACC_MODE_HIGH_PERF);
#endif
}

void AP_InertialSensor_BMI323::configure_gyro()
{
	//set 2000dps/s, 1600hz odr, high performance,1/4 BW is 3 db cutoff frequency
#ifdef BMI323_FAST_SAMPLE
	if(!enable_fast_sampling(_accel_instance)){
		write_register(BMI323_REG_GYR_CONF, BMI323_GYR_ODR_1600HZ | BMI323_GYR_RANGE_2000DPS |
											BMI323_GYR_BW_ODR_QUARTER | BMI323_GYR_MODE_HIGH_PERF);
	}else{
		write_register(BMI323_REG_GYR_CONF, BMI323_GYR_ODR_6400HZ | BMI323_GYR_RANGE_2000DPS |
											BMI323_GYR_BW_ODR_QUARTER | BMI323_GYR_MODE_HIGH_PERF);
	}
#else
		write_register(BMI323_REG_GYR_CONF, BMI323_GYR_ODR_1600HZ | BMI323_GYR_RANGE_2000DPS |
											BMI323_GYR_BW_ODR_QUARTER | BMI323_GYR_MODE_HIGH_PERF);
#endif
}

void AP_InertialSensor_BMI323::configure_fifo()
{
	//enable accel and gyro fifo, the rate rely on sensor odr
	write_register(BMI323_REG_FIFO_CONF, BMI323_FIFO_ACC_EN | BMI323_FIFO_GYR_EN);
	//write_register(BMI323_REG_FIFO_CONF, BMI323_FIFO_ACC_EN);

    fifo_reset();
}

void AP_InertialSensor_BMI323::fifo_reset()
{
    // flush and reset FIFO
    write_register(BMI323_REG_FIFO_CTRL, BMI323_FIFO_FLUSH);

    notify_accel_fifo_reset(_accel_instance);
    notify_gyro_fifo_reset(_gyro_instance);
}

/*
  read fifo
 */
void AP_InertialSensor_BMI323::read_fifo(void)
{
	uint16_t fifo_fill = 0, frame_num = 0;
	uint16_t data[6];

	read_registers(BMI323_REG_FIFO_FILL_LEVEL,&fifo_fill,1);
	frame_num = fifo_fill/6 ;
	//palToggleLine(PAL_LINE(GPIOA,15U));

	while(frame_num--){
	 	//palToggleLine(PAL_LINE(GPIOC,15U));
		//palToggleLine(PAL_LINE(GPIOC,13U));
	 	read_registers(BMI323_REG_FIFO_DATA,data,6);

	 	Vector3f accel((int16_t)data[0], (int16_t)data[1], (int16_t)data[2]);
	 	accel *= _accel_scale;
	 	_rotate_and_correct_accel(_accel_instance, accel);
	 	_notify_new_accel_raw_sample(_accel_instance, accel);

	 	Vector3f gyro((int16_t)data[3], (int16_t)data[4], (int16_t)data[5]);
	 	gyro *= _gyro_scale;
	 	_rotate_and_correct_gyro(_gyro_instance, gyro);
	 	_notify_new_gyro_raw_sample(_gyro_instance, gyro);
	 }

#if 0
	//average sample
	if(frame_num != 0){
		uint8_t num = frame_num;
		Vector3f accel_average(0, 0, 0);
		Vector3f gyro_average(0, 0, 0);
		while(frame_num--){
			//palToggleLine(PAL_LINE(GPIOC,15U));
			palToggleLine(PAL_LINE(GPIOC,13U));
			read_registers(BMI323_REG_FIFO_DATA,data,6);

			Vector3f accel((int16_t)data[0], (int16_t)data[1], (int16_t)data[2]);
			accel *= _accel_scale;
			accel_average += accel/num;

			Vector3f gyro((int16_t)data[3], (int16_t)data[4], (int16_t)data[5]);
			gyro *= _gyro_scale;
			gyro_average += gyro/num;
		}

		_rotate_and_correct_accel(_accel_instance, accel_average);
		_notify_new_accel_raw_sample(_accel_instance, accel_average);

		_rotate_and_correct_gyro(_gyro_instance, gyro_average);
		_notify_new_gyro_raw_sample(_gyro_instance, gyro_average);
	}
#endif

	uint16_t reg_status = 0;
	read_registers(BMI323_REG_STATUS,&reg_status,1);
	if(reg_status & BMI323_DRDY_TEMP_MASK){
		//palToggleLine(PAL_LINE(GPIOC,15U));
		uint16_t temp_data = 0;
		read_registers(BMI323_REG_TEMP_DATA,&temp_data,1);
		float temp_degc = (int16_t)temp_data/512.0f +23.0f;
		_publish_temperature(_accel_instance, temp_degc);
	}

}

bool AP_InertialSensor_BMI323::hardware_init()
{
	WITH_SEMAPHORE(_dev->get_semaphore());

	_dev->set_speed(AP_HAL::Device::SPEED_LOW);

	//reset BMI323
	write_register(BMI323_REG_CMD,BMI323_CMD_SOFT_RESET);

	hal.scheduler->delay(5);

	//After the software reset, the first read register data is incorrect
	uint16_t whoami = 0, i = 10;
	while(i--){
		read_registers(BMI323_REG_CHIP_ID, &whoami, 1);
	}

	_dev->set_speed(AP_HAL::Device::SPEED_HIGH);

	switch(whoami & 0x00FF){
	case BMI323_CHIP_ID:
		dev_type = BOSCH_IMU::BMI323;
		return true;
		break;
	case BMI325_CHIP_ID:
		dev_type = BOSCH_IMU::BMI325;
		return true;
		break;
	}

    return false;
}

bool AP_InertialSensor_BMI323::init()
{
    _dev->set_read_flag(0x80);

    return hardware_init();
}

#if BMI323_DEBUG
/*
 *  dump all config registers - used for debug
*/
void AP_InertialSensor_BMI323::_dump_registers(void)
{
    hal.console->println("BMI323 registers:");

#if 1
    const uint8_t first = 0x00;
    const uint8_t last = 0x7f;
    uint16_t data;
    for (uint8_t reg=first; reg<=last; reg++) {
        read_registers(reg,&data,1);
        bmi323_reg[reg] = 0;
        bmi323_reg[reg] = data;
    }
#else
    read_registers(0x00,bmi323_reg,0x7f);
#endif
}
#endif
