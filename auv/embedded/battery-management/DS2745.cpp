#include "DS2745.h"

/*
#if sizeof(short) != 2
#error This code assumes sizeof(short) == 16-bits
#endif
*/

void DS2745::Write(Register reg, uint8_t value)
{
	uint8_t buf[2];
	buf[0] = reg;
	buf[1] = value;
	int success = i2c->write(I2C_Address, (char*)buf, sizeof(buf));

	if (success != 0) {
		extern Serial pc;
		pc.printf("ds2745: I2C write not acknowledged!\r\n");
	}
}

uint8_t DS2745::Read(Register reg)
{
	uint8_t buf[1];
	int success = i2c->read(I2C_Address, (char*)buf, sizeof(buf));

	if (success != 0) {
		extern Serial pc;
		pc.printf("ds2745: I2C read not acknowledged!\r\n");
	}
	return buf[0];
}

uint16_t DS2745::Read16bit(Register first)
{
	uint8_t buf[2];
	int success = i2c->read(I2C_Address, (char*)buf, sizeof(buf));

	if (success != 0) {
		extern Serial pc;
		pc.printf("ds2745: I2C multiread not acknowledged!\r\n");
	}

	return buf[1] << 8 + buf[0];
}

bool DS2745::SanityCheck()
{
	/*
	uint8_t r = Read(config_op_status);
	return r & (0x1 << 3); // This is the SA (Single AFE) bit in the register.
	// FIXME we could be a lot smarter here.
	*/
	return true;
}

void DS2745::SetStatusLED(bool on)
{
	if (on)
		Write(status_config, Read(status_config) | BIT(3));
	else
		Write(status_config, Read(status_config) & ~BIT(3));
}

float DS2745::ReadVoltage()
{
	uint16_t adcval = (Read16bit(voltage_msb) & ~0x8000) >> 5;
	return (float)adcval * 4.88e-3; /* Units are 4.88 mV per quant */
}

float DS2745::ReadCurrent()
{
	signed short adcval = (signed short)Read16bit(current_msb);
	return (float)adcval * 625e-6; /* Units are 625 microamps per quant */
}

float DS2745::ReadAccumulatedCurrent()
{
	uint16_t adcval = Read16bit(accum_current_msb);
	return (float)adcval * 2.5e-3; /* Units are 2.5 milliamp-hours per quant */
}

float DS2745::ReadTemperature()
{
	uint16_t adcval = (Read16bit(temperature_msb) & ~0x8000) >> 5;
	return (float)adcval * 0.125; /* Units are 0.125 deg C per quant */
}

void DS2745::SetAccumulationBias(float bias_mv)
{

}

void DS2745::SetOffsetBias(float bias_mv)
{

}

void DS2745::Init()
{
	// FIXME check POR bit and set accumulator, etc?
}
