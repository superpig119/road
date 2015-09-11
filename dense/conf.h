#include <fstream>
#include <string>
#include <iostream>

using namespace std;

#ifndef CONF_H_
#define CONF_H_

class Conf
{
public:
	string nodeFilePath;
	string edgeFilePath;
	string trajectoryFilePath;
	string speedFilePath;
	int readConf();
};
#endif
