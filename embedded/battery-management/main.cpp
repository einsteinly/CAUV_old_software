#include "mbed.h"
#include <stdint.h>

#include "ISL9208.h"
#include "DS2745.h"

//--------------------------------- Pin definitions ---------------------------
DigitalOut myled(LED1);
Serial pc(USBTX, USBRX);
I2C i2c1(p9, p10), i2c2(p28, p27); // SDA, SCL
AnalogIn adc1(p20), adc2(p19);

int main()
{
	pc.baud(115200);
	i2c1.frequency(100000);
	i2c2.frequency(100000);

	pc.printf("Hello world!\r\n");

	ISL9208 balance1(&i2c1, &adc1);
	DS2745  fuelgauge1(&i2c1);
	//ISL9208 balance2(&i2c2, &adc2);
	//DS2745  fuelgauge2(&i2c2);

	balance1.Init();
	fuelgauge1.Init();
	//balance2.Init();
	//fuelgauge2.Init();

	pc.printf("Balance/Protection Board #1: ISL9208 [%s], DS2745 [%s]\r\n", balance1.SanityCheck() ? "pass" : "fail", fuelgauge1.SanityCheck() ? "pass" : "fail");
	//pc.printf("Balance/Protection Board #2: ISL9208 [%s], DS2745 [%s]\r\n", balance2.SanityCheck() ? "pass" : "fail", fuelgauge2.SanityCheck() ? "pass" : "fail");

	// turn on CFET and DFET
	balance1.MainFETControl(true, true);
	//balance2.MainFETControl(true, true);

	bool toggle = false;
	int cell_to_shunt = 2;

	while(1)
	{
		// Das blinkenlights
		myled = 1;
		balance1.MainFETControl(true, true);
		//balance2.MainFETControl(true, true);
		wait(0.5);
		myled = 0;
		balance1.MainFETControl(false, false);
		//balance2.MainFETControl(false, false);
		wait(0.5);

		fuelgauge1.SetStatusLED(toggle = !toggle);
		//fuelgauge2.SetStatusLED(toggle);

		//balance1.SetBalanceFET(cell_to_shunt, false);
		//balance2.SetBalanceFET(cell_to_shunt, false);
		//cell_to_shunt++; if (cell_to_shunt > 6) cell_to_shunt = 1;
		balance1.SetBalanceFET(cell_to_shunt, toggle);
		//balance2.SetBalanceFET(cell_to_shunt, toggle);
		pc.printf("Shunting cell #%d\r\n", cell_to_shunt);

		pc.printf("\nBalance/Protection Board #1 Cell Voltages:\r\n");
		for (int cell = 1; cell <= 6; cell++)
		{
			pc.printf("\tCell %d = %.2f V\r\n", cell, balance1.ReadCellVoltage(cell));
		}

		pc.printf("Temperatures: internal = %.2f\r\n", balance1.ReadTemperature(true));
		pc.printf("              external = %.2f\r\n", balance1.ReadTemperature(false));

		uint8_t stat = balance1.ReadStatusBits();
		pc.printf("Status bits: ExtOverTemp       %d\r\n", stat & ISL9208::ExternalOverTemp ? 1 : 0);
		pc.printf("             IntOverTemp       %d\r\n", stat & ISL9208::InternalOverTemp ? 1 : 0);
		pc.printf("             LoadFail_Vmon     %d\r\n", stat & ISL9208::LoadFail_Vmon ? 1 : 0);
		pc.printf("             ShortCircuit      %d\r\n", stat & ISL9208::ShortCircuit ? 1 : 0);
		pc.printf("             DischargeOC       %d\r\n", stat & ISL9208::DischargeOverCurrent ? 1 : 0);
		pc.printf("             ChargeOC          %d\r\n", stat & ISL9208::ChargeOverCurrent ? 1 : 0);

		pc.printf("gauge v=%g i=%g t=%g\r\n", fuelgauge1.ReadVoltage(), fuelgauge1.ReadCurrent(), fuelgauge1.ReadTemperature());


		/// -----------------------
		/*
		pc.printf("\nBalance/Protection Board #2 Cell Voltages:\r\n");
		for (int cell = 1; cell <= 6; cell++)
		{
			pc.printf("\tCell %d = %.2f V\r\n", cell, balance2.ReadCellVoltage(cell));
		}

		pc.printf("Temperatures: internal = %.2f\r\n", balance2.ReadTemperature(true));
		pc.printf("              external = %.2f\r\n", balance2.ReadTemperature(false));

		stat = balance2.ReadStatusBits();
		pc.printf("Status bits: ExtOverTemp       %d\r\n", stat & ISL9208::ExternalOverTemp ? 1 : 0);
		pc.printf("             IntOverTemp       %d\r\n", stat & ISL9208::InternalOverTemp ? 1 : 0);
		pc.printf("             LoadFail_Vmon     %d\r\n", stat & ISL9208::LoadFail_Vmon ? 1 : 0);
		pc.printf("             ShortCircuit      %d\r\n", stat & ISL9208::ShortCircuit ? 1 : 0);
		pc.printf("             DischargeOC       %d\r\n", stat & ISL9208::DischargeOverCurrent ? 1 : 0);
		pc.printf("             ChargeOC          %d\r\n", stat & ISL9208::ChargeOverCurrent ? 1 : 0);

		pc.printf("gauge v=%g i=%g t=%g\r\n", fuelgauge2.ReadVoltage(), fuelgauge2.ReadCurrent(), fuelgauge2.ReadTemperature());
		*/
		wait(2);
	}
}
