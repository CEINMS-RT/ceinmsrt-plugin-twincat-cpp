#include <PluginPositionAndTorqueTwinCat.h>

PluginPositionAndTorqueTwinCat::PluginPositionAndTorqueTwinCat() : record_(false), outDirectory_("output"), threadStop_(false)
{

}

PluginPositionAndTorqueTwinCat::~PluginPositionAndTorqueTwinCat()
{

}

void PluginPositionAndTorqueTwinCat::init(std::string& executableXMLFileName)
{
	ExecutionXmlReader executionCfg(executableXMLFileName);
	init(atoi(executionCfg.getComsumerPort().c_str()));
}

void PluginPositionAndTorqueTwinCat::init(int portno)
{

	client_ = new tcAdsClient(portno);//350
	std::string varInputs = "NMS_Controller.Input.CEINMS_In";
	std::string varOutputs = "NMS_Controller.Output.CEINMS_Out";
	varNameVect_.resize(2);
	varNameVect_[VarName::Inputs] = client_->getVariableHandle(&varInputs[0], varInputs.size());
	varNameVect_[VarName::Outputs] = client_->getVariableHandle(&varOutputs[0], varOutputs.size());

	if (varNameVect_[VarName::Inputs] == -1 || varNameVect_[VarName::Outputs] == -1 )
		exit(-1);
	if (record_)
	{
		logger_ = new OpenSimFileLogger<double>(outDirectory_);
		logger_->addLog(Logger::IK, dofName_);
		logger_->addLog(Logger::ID, dofName_);
		logger_->addLogCSV("RandomSignal");
		logger_->addLogCSV("Trigger");
	}

	timeStamp_ = rtb::getTime();

	ethercatThread_ = new std::thread(&PluginPositionAndTorqueTwinCat::ethercatThread, this);
}


void PluginPositionAndTorqueTwinCat::ethercatThread()
{
	double timeLocal;
	std::map<std::string, double> ikDataLocal;
	std::map<std::string, double> idDataLocal;

	while (!threadStop_)
	{

		int length = 2; //Lankle, Rankle
		std::vector<double> temp;
		std::vector<double> dataIK, dataSaveIK;
		std::vector<double> dataID, dataSaveID;
		dataIK.resize(length);
		dataID.resize(length);
		temp.resize(6);
		double randSignal, trigger;

		timeLocal = rtb::getTime();

		client_->read(varNameVect_[VarName::Outputs], &temp[0], 6 * sizeof(double));
		dataIK[0] = temp[1]; // Right Ankle Position
		dataIK[1] = temp[0]; // Left Ankle Position
		dataID[0] = temp[3]; // Right Ankle Torque
		dataID[1] = temp[2];// Left Ankle Torque
		randSignal = temp[4];
		trigger = temp[5];
		for (std::vector<std::string>::const_iterator it = dofName_.begin(); it != dofName_.end(); it++)
		{

			if (*it == "ankle_angle_r")
			{
				dataSaveIK.push_back(dataIK[0]);// + M_PI / 2); For the achilles the position have to be inverse and  add pi/2
					ikDataLocal[*it] = dataIK[0];// + M_PI / 2;
				dataSaveID.push_back(dataID[0]);
				idDataLocal[*it] = dataID[0];
			}
			else if (*it == "ankle_angle_l")
			{
				dataSaveIK.push_back(dataIK[1]);
				ikDataLocal[*it] = dataIK[1];
				dataSaveID.push_back(dataID[1]);
				idDataLocal[*it] = dataID[1];
			}
			/*else if (*it == "knee_angle_l")
			{
				ikDataLocal[*it] = -1.552; // sitting position for the left
				dataSaveIK.push_back(-1.552);
				idDataLocal[*it] = 0;
				dataSaveID.push_back(0);
			}*/
			else
			{
				ikDataLocal[*it] = 0;
				dataSaveIK.push_back(0);
				idDataLocal[*it] = 0;
				dataSaveID.push_back(0);
			}
		}

		mtxEthercat_.lock();
		dataAngleEthercat_ = ikDataLocal;
		dataTorqueEthercat_ = idDataLocal;
		mtxEthercat_.unlock();
		mtxTime_.lock();
		timeStampEthercat_ = timeLocal;
		mtxTime_.unlock();

		if (record_)
		{

			logger_->log(Logger::IK, timeLocal, dataSaveIK);
			logger_->log(Logger::ID, timeLocal, dataSaveID);

			logger_->logCSV("RandomSignal", timeLocal, randSignal);
			logger_->logCSV("Trigger", timeLocal, trigger);
		}

	}
}

void PluginPositionAndTorqueTwinCat::setDofTorque(const std::vector<double>& dofTorque)
{
	int length = 2; //Lankle, Rankle
	// TODO ADD gain
	std::vector<double> temp;
	temp.resize(2);
	for (std::vector<std::string>::const_iterator it = dofName_.begin(); it != dofName_.end(); it++)
	{
		int cpt = std::distance<std::vector<std::string>::const_iterator>(dofName_.begin(), it);
		if (*it == "ankle_angle_l")
		{
			temp[1] = dofTorque[cpt];
			//client_->write(varNameVect_[VarName::TorqueControlLeftAnkle], &temp[cpt], sizeof(double));
			//std::cout << "ankle_angle_l" << temp[cpt] << std::endl << std::flush;
		}
		if (*it == "ankle_angle_r")
		{
			temp[0] = dofTorque[cpt];
			//client_->write(varNameVect_[VarName::TorqueControlRightAnkle], &temp[cpt], sizeof(double));
			//std::cout << "ankle_angle_r" << temp[cpt] << std::endl << std::flush;
		}
	}
	if (dofTorque.size() >= 2)
		client_->write(varNameVect_[VarName::Inputs], &temp[0], 2*sizeof(double));
}

const std::map<std::string, double>& PluginPositionAndTorqueTwinCat::GetDataMap()
{
	mtxEthercat_.lock();
	dataAngle_ = dataAngleEthercat_;
	mtxEthercat_.unlock();
	return dataAngle_;
}

const std::map<std::string, double>& PluginPositionAndTorqueTwinCat::GetDataMapTorque()
{
	mtxEthercat_.lock();
	dataTorque_ = dataTorqueEthercat_;
	mtxEthercat_.unlock();

	return dataTorque_;
}

const double& PluginPositionAndTorqueTwinCat::GetAngleTimeStamp()
{
	mtxTime_.lock();
	timeStamp_ = timeStampEthercat_;
	mtxTime_.unlock();
	return timeStamp_;
}

void PluginPositionAndTorqueTwinCat::stop()
{
	threadStop_ = true;
	ethercatThread_->join();
	delete ethercatThread_;
	if (record_)
	{
		logger_->stop();
		delete logger_;
	}
	for (std::vector<unsigned long>::const_iterator it = varNameVect_.begin(); it != varNameVect_.end(); it++)
		client_->releaseVariableHandle(*it);
	client_->disconnect();
	delete client_;
}

void PluginPositionAndTorqueTwinCat::setDirectory(std::string outDirectory, std::string inDirectory)
{
	outDirectory_ = outDirectory;
}

#ifdef UNIX
extern "C" AngleAndComsumerPlugin* create() {
	return new PluginPositionAndTorqueTwinCat;
}

extern "C" void destroy(AngleAndComsumerPlugin* p) {
	delete p;
}
#endif

#ifdef WIN32 // __declspec (dllexport) id important for dynamic loading
extern "C" __declspec (dllexport) AngleAndComsumerPlugin* __cdecl create() {
	return new PluginPositionAndTorqueTwinCat;
}

extern "C" __declspec (dllexport) void __cdecl destroy(AngleAndComsumerPlugin* p) {
	delete p;
}
#endif