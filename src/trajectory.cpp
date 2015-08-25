#include "trajectory.h"
#include <fstream>
#include <dirent.h>
#include <vector>

int Trajectory::readRawTrajectory()
{
	if(cf.readConf())
	{
		return -1;
	}

	DIR *dp;
	struct dirent *dirp;
	string dirname = cf.rawTrajectoryFolder;

	if((dp = opendir(dirname.c_str())) == NULL)
	{
		cout << "Connot open the Raw Trajectory Folder" << endl;
	}
						
	stringstream ss;
	string stmp;
	vector<string> vFile;
	vector<string>::iterator ivFile;
	while((dirp = readdir(dp)) != NULL)
	{
		ss.clear();
		ss.str("");
		ss << dirp->d_name;
		ss >> stmp;
		if(stmp.find(".txt") != string::npos)
			vFile.push_back(stmp);
	}

/*	for(ivFile = vFile.begin(); ivFile != vFile.end(); ivFile++)
	{
		cout << *ivFile << endl;
	}
*/	cout << "File Number:" << vFile.size() << endl;

	closedir(dp);

	readRawTrajectoryFile(vFile[0]);
}


void Trajectory::readRawTrajectoryFile(string filename)
{
	cout << "filename:" << filename << endl;
	string taxiID = filename;
	taxiID = taxiID.substr(0, taxiID.find("."));
	filename = cf.rawTrajectoryFolder + filename;
	cout << "Filename:" << filename << endl;
	ifstream ifile(filename.c_str());
	
	string stmp;
	while(ifile >> stmp)
		cout << stmp << endl;
}
