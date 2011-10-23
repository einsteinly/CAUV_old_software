/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#include <iostream>
#include <limits>
#include "sonar.h"

using namespace std;

int main(int argc, char* argv[])
{
	Sonar sonar("/dev/ttyUSB0");
	DisplaySonarObserver obs;

	cout << "Initialising sonar" << endl;
	sonar.addObserver(&obs);
	sonar.init();

	cin.clear();
	cin.ignore(numeric_limits<streamsize>::max(), '\n');

	cout << "Exiting..." << endl;
	return 0;
}

