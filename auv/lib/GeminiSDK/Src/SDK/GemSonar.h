#ifndef __GEMSONAR_H__
#define __GEMSONAR_H__

class CGemSonar
{
public:	
	int m_nBeams;					// the number of beams 
	int m_nChans;					// the number of receive channels
	int m_decimation;			// the decimation
	double m_gtxFreq;			// the global clock frequency
	double m_acqFreq;			// the acq_clock frequency in the fpga
	double m_sampleFreq;  // the adc sample frequency
	double m_rxFreq;		  // receive frequency
	double m_nomLambda;   // nominal acoustic wavelength at 1500m/s
	double m_nomLineRng;  // The two way range of one line assuming 1500m/s
	double m_ieSpacing;		// inter element spacing	
	
	int m_nSram;					// the number of sram chips
	int m_sramAddrWidth;	// bits
	int m_sramDataWidth;	// bits	
	int m_sramSize;				// the size of the available zbt memory in bytes
	int m_sramModSize;    // the amount of allocated space in the zbt for a modifier table
	
	int m_nFlash;					// the number of flash chips
	int m_flashAddrWidth; // bits
	int m_flashDataWidth; // bits		
	int m_flashSize;			// the size of the available flash memory in bytes
	int m_flashImageSize; // the space allocated to each of fpga .bit files in flash
	int m_flashModSize;		// the space allocated to each of the modifier tables in flash	
	int m_flashModBaseAddr; // the base address for the modifiers in the

	int m_nFpgaImage;			// the number of fpga images stored in the flash

	int m_modTabSize;			// the actual size of a modifier table
	int m_focusTabSize;		// the size of an individual focus table
	int m_nFocusTab;			// the number of focus tables in 1 modifier
	
	int m_nModTab;				// the number of modifier tables that will fit into the flash
	
	double m_minVgaGain;	// in dB
	double m_maxVgaGain;	// in dB
	int m_nAdcBits;				
	int m_maxAdcVal;

#if 0
// ML506

  #pragma message ("***********************")
  #pragma message ("ML506 CONSTANTS DEFINED")
  #pragma message ("***********************")

	CGemSonar()
	{
		m_nBeams            = 256;
		m_nChans            = 96;
		m_decimation        = 32;
		m_gtxFreq           = 125e6;
		m_acqFreq           = m_gtxFreq * 5 / 12;
		m_sampleFreq        = m_acqFreq / 18.0;   // the divide by 18 is a function of the serial ADCs
		m_rxFreq            = m_sampleFreq / 4.0;
		
		m_nSram             = 1;
		m_nFlash            = 2;
		m_sramAddrWidth     = 18;
		m_sramDataWidth     = 32;
		m_flashAddrWidth    = 25;
		m_flashDataWidth    = 16;
		m_flashImageSize    = (int)pow(2.0,22);   // 4Mb allotted for each fpga image at the start of the flash
		m_flashModSize      = (int)pow(2.0,21);	  // 2Mb allotted for each modifier table in the flash		
		m_nFpgaImage        = 2;
		m_flashModBaseAddr  =(m_nFpgaImage * m_flashImageSize);
		m_minVgaGain        = 0;
		m_maxVgaGain        = 75;
		m_nAdcBits          = 12;

		Calculate();
	}
#else
// Gemini Board Version
	CGemSonar()
	{
		m_nBeams            = 256;
		m_nChans            = 96;
		m_decimation        = 32;
		m_gtxFreq           = 125e6;
		m_acqFreq           = m_gtxFreq * 5 / 12;
		m_sampleFreq        = m_acqFreq / 18.0;   // the divide by 18 is a function of the serial ADCs
		m_rxFreq            = m_sampleFreq / 4.0;
		
		m_nSram             = 2;
		m_nFlash            = 2;
		m_sramAddrWidth     = 20;                 // 21 = 8Mb, 20 = 4Mb, 19 = 2Mb, 18 = 1Mb
		m_sramDataWidth     = 32;
		m_flashAddrWidth    = 25;                 // 25 = 64Mb, 24 = 32 Mb
		m_flashDataWidth    = 16;
		m_flashImageSize    = (int)pow(2.0,22);   // 4Mb allotted for each fpga image at the start of the flash
		m_flashModSize      = (int)pow(2.0,21);	  // 2Mb allotted for each modifier table in the flash		
		m_nFpgaImage        = 2;
		m_flashModBaseAddr  = m_nFpgaImage * m_flashImageSize;
		m_minVgaGain        = 0;
		m_maxVgaGain        = 75;
		m_nAdcBits          = 12;

		Calculate();
	}
#endif

	void Calculate()
	{
		m_nomLambda     = 1500.0 / m_rxFreq;
		m_ieSpacing     = 1.1e-3;                 //m_nomLambda * 0.530478;
		m_nomLineRng    = m_nomLambda * m_decimation / 4 / 2;

		m_sramSize      = m_nSram * m_sramDataWidth / 8 * (int)pow(2.0, m_sramAddrWidth); // 1 Mbyte
		m_flashSize     = m_nFlash * m_flashDataWidth / 8 * (int)pow(2.0, m_flashAddrWidth); // 32 Mb		
		m_sramModSize   = m_sramSize / 4;         // 1/4 of zbt allocated to modifiers
		m_focusTabSize  = 4 * m_nChans * m_nBeams;
		m_nFocusTab     = (int)floor((double)(m_sramModSize / m_focusTabSize)); // the number of focus tables that will fit into the available zbt 
		m_modTabSize    = m_nFocusTab * m_focusTabSize + m_nChans*4;	// JGS 4*nChans additional bytes for frt
		m_nModTab       = (m_flashSize - m_flashModBaseAddr) / m_flashModSize;

		m_maxAdcVal     = (int)pow(2.0, m_nAdcBits) - 1;
	}
};

#if 0
class CGemRxFilter
{
public:	
	CGemRxFilter()
	{
		m_centreFrequency = 240000.0;
		m_sampleFrequency = m_centreFrequency * 4;
		m_bandwidth = 30000.0;
		m_decimation = 54;
		m_nTaps = 108; // The maximum filter size is 108 for a 240kHz transducer - see adder slave vhdl code	
									 // Must be divisible by 4 for correct adder slave fpga operation		
		m_maxCoeff = 0;
		m_unityGain = 0;
		m_worstCaseGain = 0;
		m_pH = NULL;
		m_pCoeffs = NULL;			
	}

	~CGemRxFilter()
	{
		if (m_pH)
			free(m_pH);
		if (m_pCoeffs)
			free(m_pCoeffs);
	}
		
	double m_sampleFrequency;
	double m_centreFrequency;
	double m_bandwidth;
	int m_nTaps;
	int m_decimation;
	int m_fpgaDecimation;
	int m_fpgaNtaps;
	double m_maxCoeff;
	double m_unityGain;
	double m_worstCaseGain;
	double* m_pH; // unscaled filter coefficients
	short* m_pCoeffs; // the actual coefficents to be written to the .coe file
};

#endif

#endif // __GEMSONAR_H__

