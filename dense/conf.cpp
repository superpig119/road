#include "conf.h"

int Conf::readConf()
{
	fstream ifile("configure");
	if(!ifile)
	{
		cout << "Cannot open configure file" << endl;
		return 1;
	}
	string s;
	while(ifile >> s)
	{
		if(s == "nodeFile")
			ifile >> nodeFilePath;
		else if(s == "edgeFile")
			ifile >> edgeFilePath;
		else if(s == "trajectoryFile")
			ifile >> trajectoryFilePath;
		else if(s == "speedFile")
			ifile >> speedFilePath;
	}

	return 0;
}
