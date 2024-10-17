#include <PluginPositionAndTorqueTwinCat.h>
#include <cstdlib>
#include <csignal>
#include <windows.h>
#include <GetTime.h>
#include <iomanip> 

bool endMain;

void SigintHandler(int sig)
{
	endMain = true;;
}

int main(void)
{
	endMain = false;
	signal(SIGINT, SigintHandler);

	PluginPositionAndTorqueTwinCat plugin;
	std::vector<std::string> dofName;
	dofName.push_back("r_ankle");
	dofName.push_back("l_ankle");
	dofName.push_back("r_knee");
	dofName.push_back("l_knee");
	dofName.push_back("r_hip");
	dofName.push_back("l_hip");

	std::vector<double> torque;
	torque.push_back(0);
	torque.push_back(1);
	torque.push_back(2);
	torque.push_back(3);

	plugin.setDofName(dofName);
	plugin.setDirectory("Test");
	plugin.setRecord(true);
	plugin.init(350);

	while (!endMain)
	{
		double timeInint = rtb::getTime();
		double time = plugin.GetAngleTimeStamp();
		std::cout << std::setprecision(16) << rtb::getTime() << " a" << std::endl << std::flush;
		std::map<std::string, double> temp = plugin.GetDataMap();
		std::cout << std::setprecision(16) << rtb::getTime() - timeInint << " b" << std::endl << std::flush;
		//std::cout << temp["ankleR"] << " : " << temp["ankleL"] << std::endl << std::flush;
		std::map<std::string, double> tempT = plugin.GetDataMapTorque();
		std::cout << std::setprecision(16) << rtb::getTime() - timeInint << " : " << tempT["r_ankle"] << " : " << tempT["l_ankle"] << std::endl << std::flush;
		plugin.setDofTorque(torque);
		std::cout << std::setprecision(16) << rtb::getTime() - timeInint << " c" << std::endl << std::flush;
		//Sleep(100);
	}
	plugin.stop();

	return 0;
}

