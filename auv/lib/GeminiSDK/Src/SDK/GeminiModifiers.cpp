// The following calculates the modifiers 

#ifdef _WIN32
	#include "stdafx.h"
#endif
#include <cstdio>
#include <cstring>
#include "GeminiNetwork.h"
#include "GeminiModifiers.h"

//*****************************************************************************
// Selected extracts from the SRDv mathlib

#ifndef ML_2_PI
#define ML_2_PI     6.28318530717958647692528676655901
#endif

#define ML_D2R(x)   ((x) * 0.0174532925199432957692369076848861)
#define ML_SQR(x)   ((x) * (x))
#define ML_Round(x) floor(x + 0.5)
#define ML_ABS(a)   (a < 0 ? -(a) : (a))


//*****************************************************************************
CGeminiModifiers::CGeminiModifiers(void)
{
  m_useCalibrationFiles  = false;
  
  m_txCalibrationFile[0] = 0;
  m_rxCalibrationFile[0] = 0;
}

//*****************************************************************************
CGeminiModifiers::~CGeminiModifiers(void)
{
  FreeModifiers();
}

//*****************************************************************************
bool CGeminiModifiers::Calculate(void)
{
	double mb               = (int)pow(2.0, 20);
	double modSizeMb        = (double)m_gemSonar.m_modTabSize / mb;
	double flashModSizeMb   = (double)m_gemSonar.m_flashModSize / mb;
	double sramModSizeMb    = (double)m_gemSonar.m_sramModSize / mb;

	m_frt.m_nFtabs          = m_gemSonar.m_nFocusTab;
	m_modVars.m_nFtabs      = m_frt.m_nFtabs;
	
	m_modVars.m_nChans      = m_gemSonar.m_nChans;
	m_modVars.m_decimation  = m_gemSonar.m_decimation;

  bool retValue = true;

	// Currently focus ranges are calculated assuming 1500m/s 
	// We may need to calculate this for differnt speeds of sound

	CalculateOptimalFocusRanges(&m_modVars);

	COMPLEX_SHORT* pData = NULL;
	COMPLEX_SHORT* pTranslated = NULL;
	
	int size = m_modVars.m_nBeams * m_modVars.m_nChans * m_modVars.m_nFtabs + m_modVars.m_nChans; //nChans FRT entries
	pData = (COMPLEX_SHORT*)realloc(pData,size * sizeof(COMPLEX_SHORT));
	pTranslated = (COMPLEX_SHORT*)realloc(pTranslated,size * sizeof(COMPLEX_SHORT));
	
	for (int mod = 0; mod < m_gemSonar.m_nModTab; mod++)
	{		
		double sos = 1400 + (mod * 3.2);
		PrepareModifierVars(&m_modVars,sos);
		COMPLEX_SHORT* pD = pData;

		for (int tab = 0; tab < m_frt.m_nFtabs; tab++)
		{
			if (CalcXdModifiers(pD,&m_modVars,m_frt.m_rangeM[tab]))
			{			 
			  GenerateTDBFlags(pD,&m_modVars,m_frt.m_rangeM[tab]); 
			  pD += m_modVars.m_nBeams * m_modVars.m_nChans;		
      }
      else
      {
        retValue = false;

        // Break out of the for loop
        break;
      }
		}

    // Break out of the for loop if previous loop failed
    if (retValue == false)
      break;		

		TranslateModifiers(&m_modVars, pData, pTranslated);

		// We should now have a modifier table ready to go so transfer to a FlashBlock

		m_flashBlocks[mod].m_pData = (BYTE*) realloc(m_flashBlocks[mod].m_pData, m_gemSonar.m_modTabSize);
		m_flashBlocks[mod].m_addr = m_gemSonar.m_flashModBaseAddr + mod * m_gemSonar.m_flashModSize;
		m_flashBlocks[mod].m_size = m_gemSonar.m_modTabSize;
		BYTE* pFlash = m_flashBlocks[mod].m_pData;
		memcpy(pFlash, pTranslated, m_gemSonar.m_modTabSize);				
	}	

	free(pData); 
	free(pTranslated);

  return retValue;
}

//*****************************************************************************
void CGeminiModifiers::FreeModifiers(void)
{
	for (int mod = 0; mod < m_gemSonar.m_nModTab; mod++)
	{
	  if (m_flashBlocks[mod].m_pData)
	  {
	    free(m_flashBlocks[mod].m_pData);
	    m_flashBlocks[mod].m_pData = NULL;
	  }
  }
}

//*****************************************************************************
void CGeminiModifiers::PrepareModifierVars(CGeminiModifierVars* pMV, double sos)
{
	pMV->m_pCal		= (Complex<double>*) realloc(pMV->m_pCal, pMV->m_nChans * pMV->m_nBeams * sizeof(Complex<double>));
	pMV->m_pTheta	= (double*) realloc(pMV->m_pTheta, pMV->m_nBeams * sizeof(double));	

	if ((pMV->m_pCal != NULL) && (pMV->m_pTheta != NULL))
	{
	  pMV->m_centreChan = ((double)pMV->m_nChans - 1) / 2;
	  pMV->m_sos        = sos;		
	  pMV->m_lambda     = pMV->m_sos/pMV->m_rxFreq;		
  		
	  for (int beam = 0; beam < pMV->m_nBeams; beam++)
	  {		
		  double	theta = asin((2.0 * (double)beam + 1 - pMV->m_nBeams) / pMV->m_nBeams  * sin(ML_D2R(pMV->m_aperture/2)));
		  pMV->m_pTheta[beam] = theta;
	  }
  	
	  int j = 0;

	  for (int chan = 0; chan < pMV->m_nChans; chan++)
	  {		
		  for (int beam = 0; beam < pMV->m_nBeams; beam++)
		  {				
			  // Interpolate into the cal tables
			  // For now just set to unity
			  pMV->m_pCal[j] = Complex<double>(1.0,0);
			  j++;
		  }
	  }
  }
}

//*****************************************************************************
// Pass in values used for calibration data files
void CGeminiModifiers::InitCalibrationValues(bool useFiles, char *txFile, char *rxFile)
{
  m_useCalibrationFiles = useFiles;
  
#ifdef _WIN32
  sprintf_s(m_txCalibrationFile, 300, txFile);
  sprintf_s(m_rxCalibrationFile, 300, rxFile);
#else
  snprintf(m_txCalibrationFile, 300, txFile);
  snprintf(m_rxCalibrationFile, 300, rxFile);
#endif
}

//*****************************************************************************
// The following calculates the modifiers for a given range
bool CGeminiModifiers::CalcXdModifiers(COMPLEX_SHORT* pData, CGeminiModifierVars* pMV, double range)
{
	// First calculate the bearing for each beam and extract the calibration from the cal tables

	int j     = 0;	
	int i     = 0;
	double r  = range;
	errno_t result;

	float   ampCal[256];
	float   rxCal[96][2];
	Complex<double> cal_d;

	FILE* fp;

  bool retValue = true;

  // Initialise calibration co-efficients
  for (i = 0; i < 256; i++)
    ampCal[i] = 1.0;

  // Are we using calibration files
  if (m_useCalibrationFiles)
  {

    // Read transmitter calibration file
    #ifdef _WIN32
	  result = fopen_s(&fp, m_txCalibrationFile, "r" );
	 #else
	   fp = fopen(m_txCalibrationFile, "r");
	   if(fp==NULL)
	   {
	   	result = -1;
	   }
	 #endif

    if (result == 0)
    {
	    for (i = 0 ; i < pMV->m_nBeams ; i++)
	    {
	    	#ifdef _WIN32
		   	fscanf_s(fp, "%f", &ampCal[i]);
		   #else
		   	fscanf(fp, "%f", &ampCal[i]);
		   #endif
      }
      
	    fclose(fp);
    }
    else
    {
    #ifdef _WIN32
      MessageBox(NULL, L"Error opening transmitter calibration file", L"Gemini Production Tool", MB_OK);
    #endif
    
      retValue = false;
    }
    	
    if (retValue)
    {
      // Read receiver calibration file
	    #ifdef _WIN32
	  	 	result = fopen_s(&fp, m_rxCalibrationFile, "r" );
	 	 #else
		 	fp = fopen(m_rxCalibrationFile, "r");
			if(fp==NULL)
			{
				result = -1;
			}
	    #endif
	    

      if (result == 0)
      {
	      for(i = 0 ; i < pMV->m_nChans ; i++)
	      {
	      	#ifdef _WIN32
		      	fscanf_s(fp, "%f,%f", &rxCal[i][0], &rxCal[i][1]);
		      #else
		      	fscanf(fp, "%f,%f", &rxCal[i][0], &rxCal[i][1]);
		      #endif
		      cal_d.real = rxCal[i][0];
		      cal_d.imag = rxCal[i][1];
		      pMV->m_pCal[i] = cal_d;
	      }

	      fclose(fp);
      }
      else
      {
      #ifdef _WIN32
        MessageBox(NULL, L"Error opening receiver calibration file", L"Gemini Production Tool", MB_OK);
    	#endif
    
        retValue = false;
      }
    }
  }
  
  if (retValue)
  {	
	  for (int chan = 0; chan < pMV->m_nChans; chan++)
	  {					
		  double xc = pMV->m_ieSpacing * ((double)chan - pMV->m_centreChan);	

		  for (int b = 0; b < pMV->m_nBeams; b++)
		  {			
			  double theta  = pMV->m_pTheta[b];			
			  double x      = r * sin(theta);
			  double y      = r * cos(theta);
			  double rp     = ML_SQR(x - xc) + ML_SQR(y);
  			
			  rp = sqrt(rp); // the range to the focusing point from the element

			  double rDelta = rp - r; 
			  double w      = ML_2_PI * rDelta / pMV->m_lambda; // in radian wavelengths
  			
			  Complex<double> cd(sin(w), cos(w));

#if 0			
			  cd *= pMV->m_pCal[j++];

			  COMPLEX_SHORT qi;
  			
			  qi.Q = (short)((double)(16383.0 * cd.realPart()));
			  qi.I = (short)((double)(16383.0 * cd.imagPart()));
#else
        // We are having trouble linking the Complex class in the dll environment
        Complex<double> output;
        
        output.real = cd.real*pMV->m_pCal[j].real - cd.imag*pMV->m_pCal[j].imag;
        output.imag = cd.real*pMV->m_pCal[j].imag + cd.imag*pMV->m_pCal[j].real;
        j++;

			  COMPLEX_SHORT qi;
  			
			  qi.Q = (short)((double)(16383.0 * output.realPart()) * ampCal[b]);
			  qi.I = (short)((double)(16383.0 * output.imagPart()) * ampCal[b]);
#endif

			  // mask the top bits

			  qi.Q &= 0x7fff;
			  qi.I &= 0x7fff;
  			
			  *pData++ = qi;			
		  }
	  }	
  }
  
  return retValue;
}

//*****************************************************************************
// Optimal focus ranges calculated assuming a speed of sound of 1500m/s
// When translated into lines then the line numbers should always be even
// to allow one way focusing to be used

void CGeminiModifiers::CalculateOptimalFocusRanges(CGeminiModifierVars* pMV)
{
	CGeminiFocusRangeTable* pFRT = &m_frt;
	
	double targetPhaseDelta = 0.28; // radians
	double sos              = 1500.0;
	double freq             = pMV->m_rxFreq;
	double lambda           = sos / freq;	
	double nElements        = pMV->m_nChans;
	double arrayLen         = pMV->m_ieSpacing * (nElements - 1);
	double hal              = arrayLen / 2;
	double farFieldRange    = ML_SQR(arrayLen) / lambda;
	double rowRange         = sos / pMV->m_scanFreq / 2;	
	int    nNearfieldRows   = (int)(farFieldRange / rowRange);
	
	nNearfieldRows *= 2; // to ensure that we calculate all table entries

	double* pD;

	pD = (double *)malloc(nNearfieldRows * sizeof(double));

  if (pD)
  {
  
	  int lastNDelta  = 0;
	  int frtPtr      = 0;	

	  for (int row = 0; row < nNearfieldRows && frtPtr < pFRT->m_nFtabs ; row++)
	  {
		  double r = sos / pMV->m_scanFreq / 2 * row;	

		  r += ( rowRange / 2 );

		  pD[row] =  hal/2 * sqrt( ML_SQR(hal) + ML_SQR(r) );
		  pD[row] += (ML_SQR(r) / 2 * log(hal + sqrt( ML_SQR(hal) + ML_SQR(r) )));
		  pD[row] -= (r * hal);
		  pD[row] -= (ML_SQR(r) / 2 * log(r));
		  pD[row] *= (2 / sos);
		  pD[row] /= (2 * hal);			 // we now have the time error
		  pD[row] *= (ML_2_PI * freq); // we now have the mean phase error over the array in radians

		  int nDelta = (int)(ML_Round(pD[row] / targetPhaseDelta)); // the number of delta phases

		  if (nDelta < pFRT->m_nFtabs+1)
		  {
			  if (nDelta && nDelta != lastNDelta) // generate a focus row
			  {
				  pFRT->m_rangeLine[frtPtr] = row;				
				  pFRT->m_rangeM[frtPtr++]  = r;
				  lastNDelta = nDelta;
			  }
		  }						
	  }	

	  pFRT->m_rangeM[frtPtr - 1] = 10e8;			// set to the far field
  	
	  // Set any remaining range line entries to 0 to inhibit transfer
	  while (frtPtr < GEM_MAX_FTABS)	
		  pFRT->m_rangeLine[frtPtr++] = 0;	
  	
		delete(pD);
    pD = NULL;
  }
}

//*****************************************************************************
// The top bit of each of the Q and I values is used to control the forward and reverse CQI indexing in
// the beamform roots required for time delay beamforming
void CGeminiModifiers::GenerateTDBFlags(COMPLEX_SHORT* pData, CGeminiModifierVars* pMV, double range)
{
	double scanRateM      = pMV->m_sos / pMV->m_scanFreq; // the scan rate in metres 
	double scanRateLambda = scanRateM/pMV->m_lambda;      // the scan rate in terms of wavelengths
	unsigned chanSize     = pMV->m_nBeams;							  // each channel contains m_nBeams	

	for (int chanHalf = 0; chanHalf < 2; chanHalf ++)
	{
		int startChan,endChan,chanStep; 

		if (chanHalf == 0)
		{
			startChan = pMV->m_nChans / 2;
			endChan   = pMV->m_nChans;
			chanStep  = 1;
		} 
		else
		{
			startChan = pMV->m_nChans / 2 - 1;
			endChan   = -1;
			chanStep  = -1;
		}
		
		int chan = startChan;

		while (chan != endChan)
		{
			for (int beamHalf = 0; beamHalf < 2; beamHalf++)
			{
				int lineStep = 0x0000;

				if (chanHalf != beamHalf) // the step flag is negative
					lineStep = 0x8000;
				
				int startBeam, endBeam, beamStep;

				if (beamHalf == 0) // deal with the right hand beams first
				{
					startBeam = pMV->m_nBeams / 2; 
					endBeam   = pMV->m_nBeams;
					beamStep  = 1;
				} 
				else
				{
					startBeam = pMV->m_nBeams / 2 - 1;
					endBeam   = -1;
					beamStep  = -1;
				}

				int beam          = startBeam;
				int lastLineDelta = 0;
				
				while (beam != endBeam)
				{
					double xc     = pMV->m_ieSpacing * ((double)chan - pMV->m_centreChan);				
					double theta  = pMV->m_pTheta[beam];			
					double x      = range*sin(theta);
					double y      = range*cos(theta);
					double rp     = ML_SQR(x-xc) + ML_SQR(y);
					
					rp = sqrt(rp);													 // the range to the focusing point from the element
					
					double rDelta       = range-rp;
					double lambdaDelta  = rDelta/pMV->m_lambda; // in wavelengths
					
					int lineDelta = (int)(ML_ABS(lambdaDelta) / (scanRateLambda / 2));

					if (lineDelta != lastLineDelta)
					{
						lastLineDelta = lineDelta;

						// we need to jump a line whenever lineDelta is an odd number

						if (lineDelta % 2 == 1)
						{
							COMPLEX_SHORT* qi = pData + (pMV->m_nBeams * chan) + beam;													
							qi->Q |= lineStep;
							qi->I |= 0x8000;
						}
					}
					beam += beamStep;		
				}
			}			
			chan += chanStep;
		}
	}
}

//*****************************************************************************
// Modifiers need to be translated prior to storing in Flash
// This is due to the technique used when transferring from ZBT to root BRAMS in the beamformer
void CGeminiModifiers::TranslateModifiers(CGeminiModifierVars* pMV, COMPLEX_SHORT* pUntranslated, COMPLEX_SHORT* pTranslated)
{
  // First copy the focus range table entries
	
	unsigned chanSize = pMV->m_nBeams; // each channel contains m_nBeams
	unsigned tabSize  = pMV->m_nBeams * pMV->m_nChans;
	int nBeams        = pMV->m_nBeams;
	int nChans        = pMV->m_nChans;
  int f;
  int c;
  
  COMPLEX_SHORT* pIn;
	COMPLEX_SHORT* pOut = pTranslated;// + m_frt.m_nFtabs;
	
	// First insert the frt entries into the first nChans entries of pTranslated
	for (f = 0; f < m_frt.m_nFtabs; f++)
	{
		COMPLEX_SHORT cs;
		cs.Q = 0;

		if (f < m_frt.m_nFtabs - 1)		
		{
			short s1 = m_frt.m_rangeLine[f+1];
			short s2 = m_frt.m_rangeLine[f+1];

			s1 = (s1 << 8) & 0xff00;
			s2 = (s2 >> 8) & 0x00ff;

			cs.I = s1 | s2;
		}
		else
			cs.I = (short)0xffff; // the last frt entry needs to be set to 0xffff

		*pOut++ = cs;
	}

	for (f = m_frt.m_nFtabs; f < pMV->m_nChans; f++)
	{
		COMPLEX_SHORT cs;

		cs.Q = 0;
		cs.I = 0;

		*pOut++ = cs;
	}

	for (int tab = 0; tab < m_frt.m_nFtabs; tab++)
	{	
		for (int half = 0; half < 2; half++) // we need to send the low beams first followed by the high ones	
		{
			for (int b = 0; b < nBeams/2; b++)
			{		
				int beam;

				if (half == 0)
					beam = nBeams/2 - 1 - b;
				else
					beam = b + nBeams/2;

				// do the Q values first - extracting 2 Q values at a time
				int chan = 0;
				unsigned short u;

				for (c = 0; c < nChans/2; c++)			// equivalent to num roots
				{
					COMPLEX_SHORT qi;	
					// Get a pointer into the data
					// Byte order needs swapping for Gemini
					// and then words also need swapping
					pIn = pUntranslated + (tabSize * tab) + (chanSize * chan++) + beam;
					u = pIn->Q & 0x00ff;
					u <<= 8;
					u |= ((pIn->Q >> 8) & 0x00ff);
					qi.I = u;

					pIn = pUntranslated + (tabSize * tab) + (chanSize * chan++) + beam;
					u = pIn->Q & 0x00ff;;
					u <<= 8;
					u |= ((pIn->Q >> 8) & 0x00ff);
					qi.Q = u;

					*pOut++ = qi;		
				}

				// and then the I values
				// do the Q values first - extracting 2 Q values at a time

				chan = 0;

				for (c = 0; c < nChans/2; c++)			// equivalent to num roots
				{
					COMPLEX_SHORT qi;						
					// Get a pointer into the data
					pIn = pUntranslated +  (tabSize * tab) + (chanSize * chan++) + beam;
					u = pIn->I & 0x00ff;
					u <<= 8;
					u |= ((pIn->I >> 8) & 0x00ff);
					qi.I = u;

					pIn = pUntranslated +  (tabSize * tab) + (chanSize * chan++) + beam;
					u = pIn->I & 0x00ff;
					u <<= 8;
					u |= ((pIn->I >> 8) & 0x00ff);
					qi.Q = u;
				
					*pOut++ = qi;						
				}
			}
		}
	}
}

//*****************************************************************************
// EOF
//*****************************************************************************
