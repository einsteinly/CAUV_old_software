/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
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

