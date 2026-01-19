#pragma once

#include <stdint.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter2p.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_QMI8658: public AP_InertialSensor_Backend
{
public:
	static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
											AP_HAL::OwnPtr<AP_HAL::Device> dev,
											enum Rotation rotation);

	/* update accel and gyro state */
	bool update() override;
	void accumulate() override;

	void start() override;

	// get a startup banner to output to the GCS
	bool get_output_banner(char* banner, uint8_t banner_len) override;

private:
	AP_InertialSensor_QMI8658(AP_InertialSensor &imu,
							  AP_HAL::OwnPtr<AP_HAL::Device> dev,
							  enum Rotation rotation);

	/* Initialize sensor*/
	bool hardware_init();

	/* soft reset*/
	void soft_reset();

	/* on demand cali */
	void on_demand_cali();

	/* Read samples from FIFO */
	void read_fifo();

	/* Read samples from data register*/
	void read_xyz();

	bool accumulate_samples(const uint8_t *fifo_data, uint8_t count);

	enum qmi8658_GyrRange
	{
		Qmi8658GyrRange_16dps = 0 << 4,
		Qmi8658GyrRange_32dps = 1 << 4,
		Qmi8658GyrRange_64dps = 2 << 4,
		Qmi8658GyrRange_128dps = 3 << 4,
		Qmi8658GyrRange_256dps = 4 << 4,
		Qmi8658GyrRange_512dps = 5 << 4,
		Qmi8658GyrRange_1024dps = 6 << 4,
		Qmi8658GyrRange_2048dps = 7 << 4
	};

	/*!
	 * \brief Gyroscope output rate configuration.
	 */
	enum qmi8658_GyrOdr
	{
		Qmi8658GyrOdr_8000Hz = 0x00,
		Qmi8658GyrOdr_4000Hz = 0x01,
		Qmi8658GyrOdr_2000Hz = 0x02,
		Qmi8658GyrOdr_1000Hz = 0x03,
		Qmi8658GyrOdr_500Hz	= 0x04,
		Qmi8658GyrOdr_250Hz	= 0x05,
		Qmi8658GyrOdr_125Hz	= 0x06,
		Qmi8658GyrOdr_62_5Hz 	= 0x07,
		Qmi8658GyrOdr_31_25Hz	= 0x08
	};

	enum qmi8658_AccRange
	{
		Qmi8658AccRange_2g = 0x00 << 4,
		Qmi8658AccRange_4g = 0x01 << 4,
		Qmi8658AccRange_8g = 0x02 << 4,
		Qmi8658AccRange_16g = 0x03 << 4
	};

	enum qmi8658_AccOdr
	{
		Qmi8658AccOdr_8000Hz = 0x00,
		Qmi8658AccOdr_4000Hz = 0x01,
		Qmi8658AccOdr_2000Hz = 0x02,
		Qmi8658AccOdr_1000Hz = 0x03,
		Qmi8658AccOdr_500Hz = 0x04,
		Qmi8658AccOdr_250Hz = 0x05,
		Qmi8658AccOdr_125Hz = 0x06,
		Qmi8658AccOdr_62_5Hz = 0x07,
		Qmi8658AccOdr_31_25Hz = 0x08,
		Qmi8658AccOdr_LowPower_128Hz = 0x0c,
		Qmi8658AccOdr_LowPower_21Hz = 0x0d,
		Qmi8658AccOdr_LowPower_11Hz = 0x0e,
		Qmi8658AccOdr_LowPower_3Hz = 0x0f
	};

	enum qmi8658_LpfMode
	{
		A_LSP_MODE_0 = 0x00<<1,
		A_LSP_MODE_1 = 0x01<<1,
		A_LSP_MODE_2 = 0x02<<1,
		A_LSP_MODE_3 = 0x03<<1,

		G_LSP_MODE_0 = 0x00<<5,
		G_LSP_MODE_1 = 0x01<<5,
		G_LSP_MODE_2 = 0x02<<5,
		G_LSP_MODE_3 = 0x03<<5
	};

	enum qmi8658_Ctrl9Command
	{
		qmi8658_Ctrl9_Cmd_Ack					= 0X00,
		qmi8658_Ctrl9_Cmd_Rst_Fifo				= 0X04,
		qmi8658_Ctrl9_Cmd_Req_Fifo				= 0X05,
		qmi8658_Ctrl9_Cmd_WoM_Setting			= 0x08,
		qmi8658_Ctrl9_Cmd_AccelHostDeltaOffset	= 0x09,
		qmi8658_Ctrl9_Cmd_GyroHostDeltaOffset	= 0x0A,
		qmi8658_Ctrl9_Cmd_EnableTap				= 0x0C,
		qmi8658_Ctrl9_Cmd_EnablePedometer		= 0x0D,
		qmi8658_Ctrl9_Cmd_Motion				= 0x0E,
		qmi8658_Ctrl9_Cmd_ResetPedometer		= 0x0F,
		qmi8658_Ctrl9_Cmd_CopyUsid				= 0x10,
		qmi8658_Ctrl9_Cmd_SetRpu				= 0x11,
		qmi8658_Ctrl9_Cmd_AHB_Clock_Gating		= 0x12,
		qmi8658_Ctrl9_Cmd_On_Demand_Cali		= 0xA2,
		qmi8658_Ctrl9_Cmd_Apply_Gyro_Gain		= 0xAA
	};

	enum qmi8658_FifoSize
	{
		qmi8658_Fifo_16 = (0 << 2),
		qmi8658_Fifo_32 = (1 << 2),
		qmi8658_Fifo_64 = (2 << 2),
		qmi8658_Fifo_128 = (3 << 2)
	};

	enum qmi8658_FifoMode
	{
		qmi8658_Fifo_Bypass = 0,
		qmi8658_Fifo_Fifo = 1,
		qmi8658_Fifo_Stream = 2,
		qmi8658_Fifo_StreamToFifo = 3
	};

	bool block_read(uint8_t reg, uint8_t *buf, uint32_t size);
	uint8_t register_read(uint8_t reg);
	void register_write(uint8_t reg, uint8_t val, bool checked=false);
	void enable_sensor(uint8_t enable_flags);
	void send_ctl9cmd(enum qmi8658_Ctrl9Command cmd);

	// instance numbers of accel and gyro data
	uint8_t gyro_instance;
	uint8_t accel_instance;

	const enum Rotation rotation;

	float accel_scale;
	float gyro_scale;


	AP_HAL::OwnPtr<AP_HAL::Device> dev;
	AP_HAL::Device::PeriodicHandle periodic_handle;

	uint32_t debug_count;
};
