#include "ISL9208.h"

void ISL9208::Write(Register reg, uint8_t value)
{
	uint8_t buf[2];
	buf[0] = reg;
	buf[1] = value;
	int success = i2c->write(I2C_Address, (char*)buf, sizeof(buf));

	if (success != 0) {
		extern Serial pc;
		pc.printf("isl9208: I2c write not acknowledged!\r\n");
	}
}

uint8_t ISL9208::Read(Register reg)
{
	uint8_t buf[1];
	int success = i2c->read(I2C_Address, (char*)buf, sizeof(buf));

	if (success != 0) {
		extern Serial pc;
		pc.printf("isl9208: I2c read not acknowledged!\r\n");
	}
	return buf[0];
}

float ISL9208::ReadCellVoltage(uint8_t cell)
{
	if (cell < 1 || cell > 6)
		return 0.0;

	if (cell == 6)
		cell = 7; // the way the hardware is wired...

	// Request voltage from analog mux
	Write(analog_out, cell);
	wait(0.001); // let voltage stabilize (spec is 0.1 ms)

	// Read ADC
	float retval = adc->read() * 6.6; // read() returns in [0,1] corresponding to [0, 3.3V]. ADC voltages are 0.5 cell voltages

	// Put analog mux back into low power state
	Write(analog_out, 0x0);

	return retval;
}

bool ISL9208::SanityCheck()
{
	uint8_t r = Read(config_op_status);
	return r & (0x1 << 3); // This is the SA (Single AFE) bit in the register.
	// FIXME we could be a lot smarter here.
}

bool ISL9208::WKUP()
{
	uint8_t r = Read(config_op_status);
	return r & BIT(4);
}

uint8_t ISL9208::ReadStatusBits()
{
	return Read(operating_status) & 0x3f;
}

void ISL9208::MainFETControl(bool CFET, bool DFET)
{
	Write(fet_control, (int(CFET) << 1) | (int(DFET) << 0));
}

void ISL9208::MainFETStatus(bool &CFET, bool &DFET)
{
	uint8_t r = Read(fet_control);
	CFET = r & BIT(1);
	DFET = r & BIT(0);
}

void ISL9208::SetBalanceFET(uint8_t cell, bool enable_shunt)
{
	if (cell < 1 || cell > 6)
		return;

	if (cell == 6)
		cell = 7; // the way the hardware is wired...

	Write(cell_balance, BIT(cell));
}

float ISL9208::ReadTemperature(bool internal)
{
	/* Coefficients for the external NTC thermistor */
	static const float NTC_B = 3435.f, NTC_R25 = 10000.f, NTC_T25 = 298.15,
				 NTC_VoltDivRes = 46400.f;

	// Enable thermistor current
	if (!internal)
	{
		Write(write_enable, BIT(7));
		Write(feature_set, BIT(5) | Read(feature_set));
		wait(0.001);
	}

	// Request voltage from analog mux
	Write(analog_out, internal ? 0x5 : 0x4);
	wait(0.001); // let voltage stabilize
	float adcvoltratio = adc->read(); // assumed supply ratiometric [0 .. 1.0]

	// Put analog mux back into low power state
	Write(analog_out, 0x0);

	//extern Serial pc;
	//pc.printf("raw temp voltage = %.2f\r\n", adcvoltratio * 3.3);

	if (!internal)
	{
		// Disable thermistor current
		Write(feature_set, Read(feature_set) & ~BIT(5));
		Write(write_enable, 0x0);

		// Convert into degrees celcius
		float resistance = NTC_VoltDivRes * (1 / adcvoltratio - 1);
		float temp_k = NTC_B / (log(resistance / (NTC_R25 * exp(-NTC_B / NTC_T25))));

		return temp_k - NTC_T25 + 25.f;
	}
	else
	{
		// Convert into degrees celcius (Eq14 in Intersil app note #1333)
		return (adcvoltratio * 3.3f - 1.31f) / -0.0035f + 25.f;
	}
}

void ISL9208::Init()
{
	// Enable writes to config registers
	Write(write_enable, BIT(7));

	// Perform a power-on reset (this I2C write won't ack, so do a low-level
	// operation so as to let us ignore the no-ack error)
	uint8_t buf[2];
	buf[0] = feature_set;
	buf[1] = BIT(2);
	i2c->write(I2C_Address, (char*)buf, sizeof(buf));
	wait(0.05);

	// Re-enable config register writing
	Write(write_enable, BIT(7) | BIT(6) | BIT(5));

	// Set our configuration
	Write(discharge_set,
								// Discharge overcurrent automatically disables FETs
								// Overcurrent threshold = 20A
					BIT(2)        // Short-circuit threshold = 70A
								// Overcurrent timeout = 160 ms
		 );

	Write(charge_set,
								// Charge overcurrent automatically disables FETs
								// Charge overcurrent = 20A
								// Shortcircuit timeout = 190 usec
					BIT(0)        // Charge overcurrent timeout = 160 ms
		 );

	Write(feature_set,
								// Automatic temperature scan enabled
								// Internal 3.3V Regulator enabled
								// Temp3V3 pin off (for now, enable as we need it)
								// Internal and External Thermal Shutdown enabled
								// WKUP pin enabled
					BIT(0)        // Wake-up Polarity = Rising edge,
		 );

	// Disable writes to config registers
	Write(write_enable, 0x0);
}
