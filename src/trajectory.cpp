#include "trajectory.h"
#include <fstream>
#include <dirent.h>
#include <vector>
#include <iomanip>

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
	while((dirp = readdir(dp)) != NULL)
	{
		ss.clear();
		ss.str("");
		ss << dirp->d_name;
		ss >> stmp;
		if(stmp.find(".txt") != string::npos)
			vFile.push_back(stmp);
	}

/*	int i = 0;
	cout << "File Number:" << vFile.size() << endl;
*/
	ivFile = vFile.begin();
	count = 0;
	closedir(dp);

//	testTrajectory();
//	readRawTrajectoryFile(vFile[0]);
    return 0;
}

bool Trajectory::readNextTrajectory()
{
	if(ivFile == vFile.end())
		return false;
	cout << endl << "Analyzing File NO." << count << endl;
	count++;
	readRawTrajectoryFile(*ivFile);
	ivFile++;

	return true;
}

void Trajectory::readRawTrajectoryFile(string filename)
{
    string taxiID = filename;
	taxiID = taxiID.substr(0, taxiID.find("."));
	filename = cf.rawTrajectoryFolder + filename;
	ifstream ifile(filename.c_str());
	
	string stmp1, stmp2;
	int i = -1;
	struct tm ts;
	while(ifile >> stmp1)
	{
		if(stmp1 == "#")
		{
			taxiTrajectory tt;
			i++;
			ifile >> stmp1;	//trajectory ID
			ifile >> stmp1;	//taxi ID
			ts = wrapTime(ifile);
			tt.sTime = mktime(&ts);
			ts = wrapTime(ifile);
			tt.eTime = mktime(&ts);
			ifile >> tt.distance;
			ifile >> stmp1; //km
			vTrajectory.push_back(tt);
			continue;
		}
		trajectoryUnit tu;
		ts = wrapTime(ifile);
		tu.t = mktime(&ts);
		ifile >> tu.x;
		ifile >> tu.y;
		vTrajectory[i].vTU.push_back(tu);
		ifile >> stmp1;
	}

}

struct tm Trajectory::wrapTime(ifstream &ifile)
{
	int i;
	int hour, year;
	struct tm t;
	string stmp;
	ifile >> i;
	t.tm_mon = i - 1;
	ifile >> t.tm_mday;
	ifile >> year;
	t.tm_year = year - 1900;
	ifile >> hour;
	ifile >> t.tm_min;
	ifile >> t.tm_sec;
	ifile >> stmp;
	if(hour == 12)
		hour = 0;
	if(stmp == "PM")
		hour += 12;
	t.tm_hour = hour;

	return t;
}

void Trajectory::testTrajectory()
{
	vector<taxiTrajectory>::iterator ivT;
	vector<trajectoryUnit>::iterator ivTU;
	for(ivT = vTrajectory.begin(); ivT != vTrajectory.end(); ivT++)
	{
		cout << ctime(&(*ivT).sTime);
		cout << ctime(&(*ivT).eTime);
		cout << "vTU size:" << (*ivT).vTU.size() << endl;
		for(ivTU = (*ivT).vTU.begin(); ivTU != (*ivT).vTU.end(); ivTU++)
		{
			cout << ctime(&(*ivTU).t) << "\t" << setprecision(15) << (*ivTU).x << "\t" << (*ivTU).y << endl;
			
		}
	}
}

