#include "mbed.h"
#include <stdint.h>
#include "util.h"

class ISL9208
{
public:
	ISL9208(I2C *_i2c, AnalogIn *_adc) : adc(_adc), i2c(_i2c) {}

	typedef enum
	{
		ExternalOverTemp		= BIT(5),
		InternalOverTemp		= BIT(4),
		LoadFail_Vmon			= BIT(3),
		ShortCircuit			= BIT(2),
		DischargeOverCurrent	= BIT(1),
		ChargeOverCurrent		= BIT(0)
	} StatusBits;

	/** Is the ISL9208 connected and apparently working? */
	bool SanityCheck();

	/** Check the WKUP pin status */
	bool WKUP();

	/** Read ISL9208 operating status bits (IMPORTANT: a read clears the register) */
	uint8_t ReadStatusBits();

	/** Read battery cell voltage, where cell is in range 1 to 6 inclusive */
	float ReadCellVoltage(uint8_t cell);

	/** Enable/disable cell balancing FETs (shunts the cell through 17.8 ohm
	  * resistor + 5 ohm FET on resistance) IMPORTANT: user is responsible for
	  * ensuring only one balance FET is used at once, or otherwise keeping
	  * device power disappation in check */
	void SetBalanceFET(uint8_t cell, bool enable_shunt);

	/** Read internal (ISL9208 die) or external (main switch FET thermistor) temperature */
	float ReadTemperature(bool internal);

	/** Set main switch FETs on or off */
	void MainFETControl(bool CFET, bool DFET);

	/** Query main switch FET status */
	void MainFETStatus(bool &CFET, bool &DFET);

	// FIXME future: API for sleep, VMON/load testing.

	//-------------------------------------------------------------------------

	/** Initialise the ISL9208 with our specific settings */
	void Init();

protected:
	typedef enum
	{
		config_op_status	= 0x00,
		operating_status	= 0x01,
		cell_balance		= 0x02,
		analog_out			= 0x03,
		fet_control			= 0x04,
		discharge_set		= 0x05,
		charge_set			= 0x06,
		feature_set			= 0x07,
		write_enable		= 0x08
	} Register;

	/** Low-level register write */
	void	Write(Register reg, uint8_t value);

	/** Low-level register read */
	uint8_t Read(Register reg);

	/** I2C slave address of the ISL9208 */
	static const uint8_t I2C_Address = 0x50;

	AnalogIn	*adc;
	I2C			*i2c;
};



