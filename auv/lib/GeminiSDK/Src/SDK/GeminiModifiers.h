#ifndef __GEMINIMODIFIERS_H__
#define __GEMINIMODIFIERS_H__

#include "GeminiStructures.h"
#include "GeminiModifierConstants.h"
#include "MComplex.h"
#include "malloc.h"
#include "GemSonar.h"

struct COMPLEX_SHORT
{
	short Q;
	short I;
};

class CGeminiModifierVars
{
public:
	BOOL    m_bTdbDisabled;

	int     m_nBeams;
	int     m_nChans;
	int     m_decimation;
	int     m_nFtabs;

	double  m_aperture;
	double  m_ieSpacing;
	double  m_sos;
	double  m_scanFreq;
	double  m_rxFreq;
	double  m_centreChan;
	double  m_lambda;

	double* m_pTheta;	

	Complex<double>* m_pCal;

	CGeminiModifierVars()
	{		
    m_pTheta        = NULL;
		m_pCal          = NULL;		
		m_nFtabs        = 2;
		m_sos           = 1500;
		m_aperture      = 120.0; // degrees
		m_decimation    = 32;
		m_scanFreq      = (((125e6 * 5.0) / 12.0) / 18.0) / m_decimation;
		m_rxFreq        = m_scanFreq * m_decimation / 4.0;
		m_nBeams        = 256;
		m_nChans        = 96;
		m_bTdbDisabled  = false;		
		m_ieSpacing     = 0.00110; // 1500.0 / m_rxFreq / 2.0;
	}
	
	~CGeminiModifierVars()
	{
		if (m_pTheta)
		{
			free(m_pTheta);
      m_pTheta = NULL;
    }

		if (m_pCal)
		{
			free(m_pCal);
			m_pCal = NULL;
	  }
	}
};

class CGeminiFocusRangeTable
{
public:
	int     m_nFtabs;
	int     m_rangeLine[GEM_MAX_FTABS];
	double  m_rangeM[GEM_MAX_FTABS];
};

class CGeminiFlashBlock
{
public:
	unsigned  m_addr;
	unsigned  m_size;
	BYTE*     m_pData;

	CGeminiFlashBlock()
	{
		m_pData = NULL;
		m_addr  = 0x800000;
		m_size  = 0;
	}
};


class CGeminiModifiers
{
public:

       CGeminiModifiers(void);
       ~CGeminiModifiers(void);

  void InitCalibrationValues(bool useFiles, char *txFile, char *rxFile);
	bool Calculate(void);
  void FreeModifiers(void);
	
  CGeminiFlashBlock       m_flashBlocks[MAX_FLASH_BLOCKS];

private:	
  void PrepareModifierVars(CGeminiModifierVars* pMV, double sos);
  bool CalcXdModifiers(COMPLEX_SHORT* pData, CGeminiModifierVars* pMV, double range);
  void CalculateOptimalFocusRanges(CGeminiModifierVars* pMV);
  void GenerateTDBFlags(COMPLEX_SHORT* pData, CGeminiModifierVars* pMV, double range);
  void TranslateModifiers(CGeminiModifierVars* pMV, COMPLEX_SHORT* pUntranslated, COMPLEX_SHORT* pTranslated);

	CGeminiFocusRangeTable  m_frt;
	CGeminiModifierVars	    m_modVars;
	CGemSonar               m_gemSonar;

  bool    m_useCalibrationFiles;
  
  char    m_txCalibrationFile[301];
  char    m_rxCalibrationFile[301];

};

#endif //__GEMINIMODIFIERS_H__

