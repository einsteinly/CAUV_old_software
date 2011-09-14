# The code to include from the Gemini SDK src folder
set (FILES
	  GeminiComms
	  GeminiModifiers
	  gemininetwork
	  )
	  
if(WIN32)
	list (APPEND FILES Serial SerialPortEnum stdafx)
endif(WIN32)
