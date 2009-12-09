#include <exception>
#include <stdexcept>
#include <iostream>

int main()
{
	try{
		throw std::runtime_error("this is what happened");
	}
	catch (std::exception& e) {
		std::cout << e.what(); // print what happened
	}
	
}