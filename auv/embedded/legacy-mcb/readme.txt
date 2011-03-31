source files are in src
microchip projects are in Motor Control and Sensor PIC. If opening this on a new computer MPLAB uses absoulte references in its project files
	and will complain about files not being found. Easiest way to "get round" this is to delete the workspace file (.mcw), open the project (.mcp) in a text
	editor, find and replace the old code directory with the new one (look down the bottom of the file), save then open with MPLAB and recreated the workspace
	file if desired.
there are some netbeans projects for the code here too, but I (Alex Ridge, anr32) haven't used them