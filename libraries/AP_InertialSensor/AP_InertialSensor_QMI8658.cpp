/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor_QMI8658.h"
#include <utility>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define QMI8658_DISABLE_ALL				(0x0)
#define QMI8658_ACC_ENABLE				(0x1)
#define QMI8658_GYR_ENABLE				(0x2)
#define QMI8658_ACCGYR_ENABLE			(QMI8658_ACC_ENABLE | QMI8658_GYR_ENABLE)

// registers we use
#define QMI8658_WHOAMI		0x00
#define QMI8658_REVISION	0x01
#define QMI8658_CTRL1		0x02
#define QMI8658_CTRL2		0x03
#define QMI8658_CTRL3		0x04
#define QMI8658_CTRL4		0x05
#define QMI8658_CTRL5		0x06
#define QMI8658_CTRL6		0x07
#define QMI8658_CTRL7		0x08
#define QMI8658_CTRL9		0x0A
#define QMI8658_FIFOCTRL	0x14
#define QMI8658_FIFOCOUNT	0x15
#define QMI8658_FIFOSTATUS	0x16
#define QMI8658_FIFODATA	0x17
#define QMI8658_STATUSINT	0x2D
#define QMI8658_STATUS0		0x2E
#define QMI8658_AX_L		0x35
#define QMI8658_GZ_L		0x3F
#define QMI8658_GZ_H		0x40
#define QMI8658_COD_STATUS	0x46
#define QMI8658_RESET_DONE	0x4D
#define QMI8658_RESET		0x60

#define QMI8658_BACKEND_SAMPLE_RATE   897

const uint32_t QMI8658_BACKEND_PERIOD_US = 1000000UL / QMI8658_BACKEND_SAMPLE_RATE;

uint8_t fifo_discard = 0;

AP_InertialSensor_QMI8658::AP_InertialSensor_QMI8658(AP_InertialSensor &imu,
													 AP_HAL::OwnPtr<AP_HAL::Device> _dev,
													 enum Rotation _rotation)
	: AP_InertialSensor_Backend(imu)
    , rotation(_rotation)
    , dev(std::move(_dev))
{
}

AP_InertialSensor_Backend *AP_InertialSensor_QMI8658::probe(AP_InertialSensor &imu,
															AP_HAL::OwnPtr<AP_HAL::Device> _dev,
															enum Rotation _rotation)
{
	if (!_dev) {
		return nullptr;
	}

	if (_dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
		_dev->set_read_flag(0x80);
	}

	AP_InertialSensor_QMI8658 *sensor =
			new AP_InertialSensor_QMI8658(imu, std::move(_dev), _rotation);
	if (!sensor || !sensor->hardware_init()) {
		delete sensor;
		return nullptr;
	}

	DEV_PRINTF("QMI8658 probe\n");
	return sensor;
}

void AP_InertialSensor_QMI8658::start()
{
	if (!_imu.register_accel(accel_instance, QMI8658_BACKEND_SAMPLE_RATE, dev->get_bus_id_devtype(DEVTYPE_INS_QMI8658)) ||
		!_imu.register_gyro(gyro_instance, QMI8658_BACKEND_SAMPLE_RATE,   dev->get_bus_id_devtype(DEVTYPE_INS_QMI8658))) {
		DEV_PRINTF("QMI8658 start failed\n");
		return;
	}

	// setup sensor rotations from probe()
	set_gyro_orientation(gyro_instance, rotation);
	set_accel_orientation(accel_instance, rotation);

	DEV_PRINTF("QMI8658 start success\n");
	//setup callbacks
#if defined(QMI8658_USE_FIFO)
	periodic_handle = dev->register_periodic_callback(QMI8658_BACKEND_PERIOD_US, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_QMI8658::read_fifo, void));
#else
	periodic_handle = dev->register_periodic_callback(QMI8658_BACKEND_PERIOD_US, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_QMI8658::read_xyz, void));
#endif /* QMI8658_USE_FIFO */
}

bool AP_InertialSensor_QMI8658::get_output_banner(char* banner, uint8_t banner_len) {
	return false;
}

/*
  publish any pending data
 */
bool AP_InertialSensor_QMI8658::update()
{
	update_accel(accel_instance);
	update_gyro(gyro_instance);
	return true;
}

/*
  accumulate new samples
 */
void AP_InertialSensor_QMI8658::accumulate()
{
	// nothing to do
}

bool AP_InertialSensor_QMI8658::accumulate_samples(const uint8_t *fifo_data, uint8_t count)
{
	uint8_t i=0;
	uint8_t fifo_count = 0;
	int16_t raw_acc_xyz[3],raw_gyro_xyz[3];
	for(i=0; i < count; i++)
	{
		//Discard the first 8 sets of data
		if(fifo_discard < 8) fifo_discard++;
		else
		{
			raw_acc_xyz[0]  = (int16_t)((uint16_t)(fifo_data[fifo_count+1]<<8) |( fifo_data[fifo_count+0]));
			raw_acc_xyz[1]  = (int16_t)((uint16_t)(fifo_data[fifo_count+3]<<8) |( fifo_data[fifo_count+2]));
			raw_acc_xyz[2]  = (int16_t)((uint16_t)(fifo_data[fifo_count+5]<<8) |( fifo_data[fifo_count+4]));
			raw_gyro_xyz[0] = (int16_t)((uint16_t)(fifo_data[fifo_count+7]<<8) |( fifo_data[fifo_count+6]));
			raw_gyro_xyz[1] = (int16_t)((uint16_t)(fifo_data[fifo_count+9]<<8) |( fifo_data[fifo_count+8]));
			raw_gyro_xyz[2] = (int16_t)((uint16_t)(fifo_data[fifo_count+11]<<8) |( fifo_data[fifo_count+10]));

			Vector3f accel{float(raw_acc_xyz[0]), float(raw_acc_xyz[1]), float(raw_acc_xyz[2])};
			Vector3f gyro{float(raw_gyro_xyz[0]), float(raw_gyro_xyz[1]), float(raw_gyro_xyz[2])};

			accel *= accel_scale;
			gyro *= gyro_scale;

			// these four calls are about 40us
			_rotate_and_correct_accel(accel_instance, accel);
			_rotate_and_correct_gyro(gyro_instance, gyro);

			_notify_new_accel_raw_sample(accel_instance, accel);
			_notify_new_gyro_raw_sample(gyro_instance, gyro);
		}

		fifo_count+=12;
	}

	return true;
}

/*
  timer function called at ODR rate
 */
void AP_InertialSensor_QMI8658::read_fifo()
{
	uint8_t fifo_status[2] = {0, 0};
	uint16_t fifo_bytes = 0;
	uint16_t fifo_level = 0;

	send_ctl9cmd(qmi8658_Ctrl9_Cmd_Req_Fifo);
	block_read(QMI8658_FIFOCOUNT, fifo_status, 2);
	fifo_bytes = (uint16_t)(((fifo_status[1] & 0x03) << 8) | fifo_status[0]);
	fifo_level = fifo_bytes / 6;
	fifo_bytes = fifo_level * 12;

	debug_count++;
	if(debug_count > 449)
	{
		debug_count = 0;
		DEV_PRINTF("fifo level: %u bytes: %u\n", fifo_level, fifo_bytes);
	}

	if(fifo_level == 0)
		return;

	// adjust the periodic callback to be synchronous with the incoming data
	// this means that we rarely run read_fifo() without updating the sensor data
	dev->adjust_periodic_callback(periodic_handle, QMI8658_BACKEND_PERIOD_US);

	uint8_t data[fifo_bytes];
	block_read(QMI8658_FIFOCOUNT, data, fifo_bytes);
	block_read(QMI8658_FIFOCOUNT, fifo_status, 2);
	fifo_bytes = (uint16_t)(((fifo_status[1] & 0x03) << 8) | fifo_status[0]);
	if(fifo_bytes > 0)
		send_ctl9cmd(qmi8658_Ctrl9_Cmd_Rst_Fifo);
	else
		accumulate_samples(data, fifo_level);

	register_write(QMI8658_FIFOCTRL, qmi8658_Fifo_64 | qmi8658_Fifo_Fifo);
}

void AP_InertialSensor_QMI8658::read_xyz(void)
{
	uint8_t buf_reg[12];
	uint8_t status = 0;
	int16_t raw_acc_xyz[3] = {0};
	int16_t raw_gyro_xyz[3] = {0};

	status = register_read(QMI8658_STATUS0);
	if(status & 0x03)
	{
		dev->adjust_periodic_callback(periodic_handle, QMI8658_BACKEND_PERIOD_US);
		block_read(QMI8658_AX_L, buf_reg, 12);
		raw_acc_xyz[0]  = (int16_t)((uint16_t)(buf_reg[1]<<8) |( buf_reg[0]));
		raw_acc_xyz[1]  = (int16_t)((uint16_t)(buf_reg[3]<<8) |( buf_reg[2]));
		raw_acc_xyz[2]  = (int16_t)((uint16_t)(buf_reg[5]<<8) |( buf_reg[4]));
		raw_gyro_xyz[0] = (int16_t)((uint16_t)(buf_reg[7]<<8) |( buf_reg[6]));
		raw_gyro_xyz[1] = (int16_t)((uint16_t)(buf_reg[9]<<8) |( buf_reg[8]));
		raw_gyro_xyz[2] = (int16_t)((uint16_t)(buf_reg[11]<<8) |( buf_reg[10]));

		Vector3f accel{float(raw_acc_xyz[0]), float(raw_acc_xyz[1]), float(raw_acc_xyz[2])};
		Vector3f gyro{float(raw_gyro_xyz[0]), float(raw_gyro_xyz[1]), float(raw_gyro_xyz[2])};

		accel *= accel_scale;
		gyro *= gyro_scale;

		// these four calls are about 40us
		_rotate_and_correct_accel(accel_instance, accel);
		_rotate_and_correct_gyro(gyro_instance, gyro);

		_notify_new_accel_raw_sample(accel_instance, accel);
		_notify_new_gyro_raw_sample(gyro_instance, gyro);
	}
}

bool AP_InertialSensor_QMI8658::block_read(uint8_t reg, uint8_t *buf, uint32_t size)
{
	return dev->read_registers(reg, buf, size);
}

uint8_t AP_InertialSensor_QMI8658::register_read(uint8_t reg)
{
	uint8_t val = 0;
	dev->read_registers(reg, &val, 1);
	return val;
}

void AP_InertialSensor_QMI8658::register_write(uint8_t reg, uint8_t val, bool checked)
{
	dev->write_register(reg, val, checked);
}

void AP_InertialSensor_QMI8658::enable_sensor(uint8_t enable_flags)
{
	register_write(QMI8658_CTRL7, enable_flags);
	hal.scheduler->delay(2);
}

void AP_InertialSensor_QMI8658::send_ctl9cmd(enum qmi8658_Ctrl9Command cmd)
{
	uint8_t status = 0x00;
	uint8_t cmd_done = 0x80;
	uint8_t count = 0;
	uint8_t retry = 0;
	uint8_t ret1 = 0;
	uint8_t ret2 = 0;

	while(retry++ < 3)
	{
		register_write(QMI8658_CTRL9, cmd);
		status = register_read(QMI8658_STATUSINT);
		while(((status & cmd_done) != cmd_done) && (count++ < 100))
		{
			hal.scheduler->delay(1);
			status = register_read(QMI8658_STATUSINT);
		}
		if(count < 100)
			ret1 = 1;
		else
			ret1 = 0;

		register_write(QMI8658_CTRL9, qmi8658_Ctrl9_Cmd_Ack);
		count = 0;
		status = register_read(QMI8658_STATUSINT);
		while(((status & cmd_done) == cmd_done) && (count++ < 100))
		{
			hal.scheduler->delay(1);
			status = register_read(QMI8658_STATUSINT);
		}
		if(count < 100)
			ret2 = 1;
		else
			ret2 = 0;

		if((ret1 == 0) || (ret2 == 0))
			continue;
		else
			break;
	}
}

bool AP_InertialSensor_QMI8658::hardware_init(void)
{
	WITH_SEMAPHORE(dev->get_semaphore());

	uint8_t ctl_data;

	uint8_t whoami = register_read(QMI8658_WHOAMI);

	if(whoami != 0x05)
		return false;

	//soft reset
	soft_reset();

	//spi address auto increment, data Big-Endian
	register_write(QMI8658_CTRL1, 0x60);

	// calibration on demand
	on_demand_cali();

	//Disable SyncSample, Disable DRDY, Disable Gyro/Acc
	register_write(QMI8658_CTRL7, 0x00);

	//acc 16g range, 897Hz odr
	register_write(QMI8658_CTRL2, Qmi8658AccRange_16g | Qmi8658AccOdr_1000Hz);
	accel_scale = GRAVITY_MSS / 2048;
	//acc lpf bw(13.37% of ODR)
	ctl_data = register_read(QMI8658_CTRL5);
	ctl_data &= 0xf0;
	ctl_data |= A_LSP_MODE_3;
	ctl_data |= 0x01;
	register_write(QMI8658_CTRL5, ctl_data);

	//gyro 2048dps, 897Hz odr
	register_write(QMI8658_CTRL3, Qmi8658GyrRange_2048dps | Qmi8658GyrOdr_1000Hz);
	gyro_scale = radians(1) / 16;
	//gyro lpf bw(13.37% of ODR)
	ctl_data = register_read(QMI8658_CTRL5);
	ctl_data &= 0x0f;
	ctl_data |= G_LSP_MODE_3;
	ctl_data |= 0x10;
	register_write(QMI8658_CTRL5, ctl_data);

#if defined(QMI8658_USE_FIFO)
	//fifo config
	//reset fifoctl
	register_write(QMI8658_FIFOCTRL, 0x00);
	hal.scheduler->delay(2);
	send_ctl9cmd(qmi8658_Ctrl9_Cmd_Rst_Fifo);
	//fifo 64 samples, fifo mode
	register_write(QMI8658_FIFOCTRL, qmi8658_Fifo_64 | qmi8658_Fifo_Fifo);
#endif

	//anble acc and gyro
	enable_sensor(QMI8658_ACCGYR_ENABLE);

	hal.scheduler->delay(300);

	return true;
}
void AP_InertialSensor_QMI8658::soft_reset(void)
{
	uint8_t reset_done = 0x00;
	uint16_t retry = 0;

	register_write(QMI8658_RESET, 0xb0);
	hal.scheduler->delay(10);
	while(reset_done != 0x80)
	{
		hal.scheduler->delay(1);
		reset_done = register_read(QMI8658_RESET_DONE);
		if(retry++ > 500)
			break;
	}
}
void AP_InertialSensor_QMI8658::on_demand_cali(void)
{
	register_write(QMI8658_CTRL9, qmi8658_Ctrl9_Cmd_On_Demand_Cali);
	hal.scheduler->delay(2200);
	register_write(QMI8658_CTRL9, qmi8658_Ctrl9_Cmd_Ack);
	hal.scheduler->delay(10);
}
