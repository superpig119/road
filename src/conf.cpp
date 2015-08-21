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
	ifile >> s;
	if(s == "datapath")
		ifile >> filepath;

	return 0;
}
