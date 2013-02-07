#include "mbed.h"
#include <stdint.h>
#include "util.h"

class DS2745
{
public:
	DS2745(I2C *_i2c) : i2c(_i2c) {}

	/** Is the ISL9208 connected and apparently working? */
	bool SanityCheck();

	/** Set STAT LED status */
	void SetStatusLED(bool on);

	/** Read voltage in volts */
	float ReadVoltage();

	/** Read the current in amps */
	float ReadCurrent();

	/** Read accumulated current in amp-hours */
	float ReadAccumulatedCurrent();

	/** Read the temperature in degrees celcius */
	float ReadTemperature();

	/** Set the current accumulation bias */
	void SetAccumulationBias(float bias_mv);

	/** Set the current measurment offset bias */
	void SetOffsetBias(float bias_mv);

	//-------------------------------------------------------------------------

	/** Initialise the DS2745 with our specific settings */
	void Init();

//protected:
	typedef enum
	{
		status_config		= 0x01,
		temperature_msb		= 0x0a,
		temperature_lsb		= 0x0b,
		voltage_msb			= 0x0c,
		voltage_lsb			= 0x0d,
		current_msb			= 0x0e,
		current_lsb			= 0x0f,
		accum_current_msb	= 0x10,
		accum_current_lsb	= 0x11,
		offset_bias			= 0x61,
		accum_bias			= 0x62
	} Register;

	/** Low-level register write */
	void Write(Register reg, uint8_t value);

	/** Low-level register read */
	uint8_t Read(Register reg);

	/** Read two consecutive registers as a 16-bit value (MSB first) */
	uint16_t Read16bit(Register first);

	/** I2C slave address of the DS2745 */
	static const uint8_t I2C_Address = 0x48;

	I2C *i2c;
};



