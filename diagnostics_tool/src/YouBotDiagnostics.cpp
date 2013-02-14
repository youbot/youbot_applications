#include "YouBotDiagnostics.h"

using namespace youbot;
using namespace std;

YouBotDiagnostics::YouBotDiagnostics(std::fstream * file, int noOfBase, int noOfArm)
{
	slaveTypMap.insert( pair<string,string>( ARM_CONTROLLER,    string("ARM-Joint"	) ) );
	slaveTypMap.insert( pair<string,string>( BASE_CONTROLLER,   string("BASE-Joint" ) ) );
	slaveTypMap.insert( pair<string,string>( ARM_MASTER_BOARD,  string("ARM-Master" ) ) );
	slaveTypMap.insert( pair<string,string>( BASE_MASTER_BOARD, string("BASE-Master") ) );

	ethercatMaster = 0;

	statusOK			  = true;
	checkMasterboards	  = true;
	quickRestartTestOK	  = true;
	checkControllerboards = true;

	inputNumberOfBase = noOfBase;
	inputNumberOfArm  = noOfArm;

	noOfAllEtherCATSlaves	 = 0;
	noOfBaseMasterBoards     = 0;
	noOfBaseSlaveControllers = 0;
	noOfArmMasterBoards      = 0;
	noOfArmSlaveControllers  = 0;

	configFileName = "youbot-ethercat.cfg";
	outputFile = file;
}

YouBotDiagnostics::~YouBotDiagnostics() {
	if (ethercatMaster) {
		EthercatMaster::destroy();
		ethercatMaster = 0;
	}
}

void YouBotDiagnostics::startDiagnostics()
{
	init();

	getAllTopologicalInformation();
	showJointsTopologicalInformation();

	checkNumberOfMasterboards();
	checkNumberOfControllerboards();

	testInitialization();
	testFunctionOfControllerboards();
	testFunctionManually();

	quickRestartTest();

	showSummary();
}


void YouBotDiagnostics::init()
{
	//check if the arm is in home position
	if (inputNumberOfArm > 0)
		checkHomepositionOfArm();

	// check if an EthercatMaster is available
	try
	{
		string path(YOUBOT_CONFIGURATIONS_DIR);
		
		if ( path.at(path.length() - 1 ) != '/')
			configFileName = "/" + configFileName;
		
		ethercatMaster = &EthercatMaster::getInstance(configFileName, YOUBOT_CONFIGURATIONS_DIR);
	}
	catch(exception& ex)
	{
		string compStr(ex.what());
		size_t found = compStr.find("No slaves found");

		if (found != string::npos)
		{
			outputStream << ex.what() << " Check the ethernetcable" << endl;
			outputStream << " * Is the cable connected to the ethernet-interface of your master?" << endl;
			outputStream << " * Is the cable connected to the baseboard of the slaves?" << endl;

		}

		found = compStr.find("No socket connection on eth");

		if (found != string::npos)
		{
			outputStream << ex.what() << " available. Check the following:" << endl;
			outputStream << " * Have you executed the program as root?" << endl;
			outputStream << " * Have you set the right interfacename in the ethercat-config file:?" << endl;
			outputStream << "   --> e.g.: eth0, eth1 ..." << endl;
		}

		if (found == string::npos)
		{
			outputStream << "Exception: " << ex.what() << endl;
		}

		stdOutputAndFile(outputStream);
		*outputFile << separator;
	}

	//get all slaves information of the etherCAT-BUS-topolgy
	if (ethercatMaster)
		ethercatMaster->getEthercatDiagnosticInformation(etherCATSlaves);

	if(etherCATSlaves.size() < 1)
	{
		cout << "There was an Exception. See description above!\n";
		outputFile->close();
		exit(-1);
	}
	else
	{
		noOfAllEtherCATSlaves = etherCATSlaves.size();
		
		motorActivity.resize(noOfAllEtherCATSlaves + 1);
		motorActivity[0] = true;
	}
}


void YouBotDiagnostics::getAllTopologicalInformation()
{
	bool reset = true;

	unsigned jointNo  = 0;
	unsigned jointCfg = 0;

	//joint konfiguration variables
	JointName jName;
	GearRatio gearRatio;
	EncoderTicksPerRound ticksPerRound;
	InverseMovementDirection inverseDir;
	JointLimits jLimits;
	FirmwareVersion firmwareTypeVersion;
    
	ticksPerRound.setParameter(4000);
	int controllerType;
    string firmwareVersion;

	*outputFile << "----- Topological Information of all slaves: -----" << endl;

	for (int i = 0, slaveCtr = 1; i < etherCATSlaves.size(); i++, slaveCtr++)
	{
		//formatted output:
		*outputFile << "slave no "	<< setw(2) << slaveCtr << ":\t"	<< slaveTypMap[etherCATSlaves[i].name];

//		if( slaveTypMap[etherCATSlaves[i].name] == "BASE-Master")
//			*outputFile << ": " << etherCATSlaves[i].name;
//		else
			*outputFile << ":\t" << etherCATSlaves[i].name;

		if ( slaveTypMap[etherCATSlaves[i].name] == "BASE-Master" || slaveTypMap[etherCATSlaves[i].name] == "ARM-Master")
			*outputFile << "\t\t--> parent: ";
		else
			*outputFile << "\t--> parent: ";

		*outputFile << setw(2) << etherCATSlaves[i].parent << "\t--> connections: " << setw(2) << (int) etherCATSlaves[i].topology << endl;


		//create the joint map
		if (reset)
			jointCfg = 1;
		else
			jointCfg++;
		
		if ( !strcmp( etherCATSlaves[i].name, BASE_MASTER_BOARD) )
		{
			reset = true;
			noOfBaseMasterBoards++;
			motorActivity[slaveCtr] = true;
		}
		if ( !strcmp( etherCATSlaves[i].name, ARM_MASTER_BOARD ) )
		{
			reset = true;
			noOfArmMasterBoards++;
			motorActivity[slaveCtr] = true;
		}

		if ( !strcmp( etherCATSlaves[i].name, BASE_CONTROLLER  ) )
		{
			noOfBaseSlaveControllers++;
			jointNo++;
			reset = false;

			TopologyMap::iterator it = youBotJointMap.insert( pair<int, JointTopology>( slaveCtr, JointTopology() ) ).first;
			it->second.relatedUnit          = string("BASE");
			it->second.relatedMasterBoard   = string(BASE_MASTER_BOARD);
			it->second.controllerTyp        = string(BASE_CONTROLLER);
			it->second.masterBoardNo        = noOfBaseMasterBoards;
			it->second.slaveNo              = slaveCtr;
			it->second.jointTopologyNo      = jointNo;
			it->second.jointConfigNo        = jointCfg;
			it->second.joint                = new YouBotJoint(jointNo);

			it->second.joint->getConfigurationParameter(firmwareTypeVersion);
			firmwareTypeVersion.getParameter(controllerType, firmwareVersion);
			it->second.firmwareVersion = atof(firmwareVersion.c_str());
			
			stringstream name;
			name << "joint_" << jointNo;
			jName.setParameter( name.str() );

			gearRatio.setParameter(364.0/9405.0);
			inverseDir.setParameter(false);

			it->second.joint->setConfigurationParameter(jName);
			it->second.joint->setConfigurationParameter(gearRatio);
			it->second.joint->setConfigurationParameter(ticksPerRound);
			it->second.joint->setConfigurationParameter(inverseDir);
		}

		if( !strcmp( etherCATSlaves[i].name, ARM_CONTROLLER   ) )
		{
			noOfArmSlaveControllers++;
			jointNo++;
			reset = false;

			TopologyMap::iterator it = youBotJointMap.insert( pair<int, JointTopology>( slaveCtr, JointTopology() ) ).first;
			it->second.relatedUnit		  = string("MANIPULATOR");
			it->second.relatedMasterBoard = string(ARM_MASTER_BOARD);
			it->second.controllerTyp	  = string(ARM_CONTROLLER);
			it->second.masterBoardNo	  = noOfArmMasterBoards;
			it->second.slaveNo			  = slaveCtr;
			it->second.jointTopologyNo	  = jointNo;
			it->second.jointConfigNo      = jointCfg;
			it->second.joint		      = new YouBotJoint(jointNo);

			it->second.joint->getConfigurationParameter(firmwareTypeVersion);
			firmwareTypeVersion.getParameter(controllerType, firmwareVersion);
			it->second.firmwareVersion = atof(firmwareVersion.c_str());
			
			stringstream name;
			name << "joint_" << jointNo;
			jName.setParameter( name.str() );


			switch (jointCfg)
			{
				case 1:
					gearRatio.setParameter(1.0/156.0);
					inverseDir.setParameter(true);
					jLimits.setParameter(-580000, -1000, true);
					break;

				case 2:
					gearRatio.setParameter(1.0/156.0);
					inverseDir.setParameter(true);
					jLimits.setParameter(-260000, -1000, true);
					break;

				case 3:
					gearRatio.setParameter(1.0/100.0);
					inverseDir.setParameter(true);
					jLimits.setParameter(-320000, -1000, true);
					break;

				case 4:
					gearRatio.setParameter(1.0/ 71.0);
					inverseDir.setParameter(true);
					jLimits.setParameter(-155000, -1000, true);
					break;

				case 5:
					gearRatio.setParameter(1.0/ 71.0);
					inverseDir.setParameter(true);
					jLimits.setParameter(-255000, -1000, true);
					break;
			}


			it->second.joint->setConfigurationParameter(jName);
			it->second.joint->setConfigurationParameter(ticksPerRound);
			it->second.joint->setConfigurationParameter(inverseDir);
			it->second.joint->setConfigurationParameter(jLimits);
			it->second.joint->setConfigurationParameter(gearRatio);
		}
	}

	*outputFile << separator;
	*outputFile << "Number of ALL slaves:           "  << etherCATSlaves.size() << endl;
	*outputFile << "Number of JOINT slaves:         "  << youBotJointMap.size()  << endl;
	*outputFile << "Number of BaseMasterBoards:     "  << noOfBaseMasterBoards << endl;
	*outputFile << "Number of BaseSlaveControllers: "  << noOfBaseSlaveControllers << endl;
	*outputFile << "Number of ArmMasterBoards:      "  << noOfArmMasterBoards << endl;
	*outputFile << "Number of ArmSlaveControllers:  "  << noOfArmSlaveControllers << endl;
	*outputFile << separator;
}


void YouBotDiagnostics::showJointsTopologicalInformation()
{
	*outputFile << "----- Topological Information of joints: -----" << endl;
	// list joints topology
	for (TopologyMap::iterator it = youBotJointMap.begin(); it != youBotJointMap.end(); it++)
	{
		*outputFile << "youBot-unit:    " << it->second.relatedUnit		   << endl;
		*outputFile << "MasterBoardtyp: " << it->second.relatedMasterBoard << endl;
		*outputFile << "SlaveTyp:       " << it->second.controllerTyp      << endl;
		*outputFile << "MasterBoardNo:  " << it->second.masterBoardNo   << endl;
		*outputFile << "SlaveNo:        " << it->second.slaveNo         << endl;
		*outputFile << "JointNo:        " << it->second.jointTopologyNo << endl;
		*outputFile << "Firmware:       " << it->second.firmwareVersion << endl;

		TopologyMap::iterator tmpIt = it;

		if ( ++tmpIt !=  youBotJointMap.end() )
			*outputFile << separator_;

	}
	outputStream << separator;
	stdOutputAndFile(outputStream);
}

void YouBotDiagnostics::checkNumberOfMasterboards()
{
	*outputFile << "----- Compare number of found base-/arm-masterboard(s) with users input -----" << endl;
	if (noOfArmMasterBoards < inputNumberOfArm)
	{
		outputStream << "Not all ARM-MASTER-BOARDS according to users input could be found" << endl;
		outputStream << "Found  ARM(S): " << noOfArmMasterBoards << endl;
		outputStream << "Stated ARM(S): " << inputNumberOfArm << endl;
		outputStream << "Check if the ethernet cable for the arm(s) is (are) connected or the input value is correct" << endl;
		stdOutputAndFile(outputStream);

		checkMasterboards = false;
		sleep(3);
	}

	if (noOfBaseMasterBoards < inputNumberOfBase)
	{
		if(!checkMasterboards)
			outputStream << separator_;

		outputStream << "Not all BASE-MASTER-BOARDS according to users input could be found" << endl;
		outputStream << "Found  BASE(S): " << noOfBaseMasterBoards << endl;
		outputStream << "Stated BASE(S): " << inputNumberOfBase << endl;
		outputStream << "Check if the ethernet cable for the base(s) is (are) connected or the input value is correct!" << endl;
		stdOutputAndFile(outputStream);

		checkMasterboards = false;
		sleep(3);
	}

	if (noOfArmMasterBoards > inputNumberOfArm || noOfBaseMasterBoards > inputNumberOfBase)
	{
		outputStream << " --> More Masterboards found than specified. Check your input value\n";
		checkMasterboards = false;
	}
	
	if (checkMasterboards)
		outputStream << " --> Number of masterboards equates to user input\n";

	outputStream << separator;
	stdOutputAndFile(outputStream);
	
	if (noOfArmMasterBoards > inputNumberOfArm)
		checkHomepositionOfArm();
}


void YouBotDiagnostics::checkNumberOfControllerboards()
{
	*outputFile << "----- Compare number of controllerboards -----" << endl;
	
	// check if all controllers could be found
	if ( ( (noOfBaseMasterBoards*NUMBER_OF_BASE_JOINTS) != noOfBaseSlaveControllers ) || ( (noOfArmMasterBoards*NUMBER_OF_ARM_JOINTS) != noOfArmSlaveControllers ) )
	{
		checkControllerboards = false;
		
		if ( (noOfBaseMasterBoards*NUMBER_OF_BASE_JOINTS) != noOfBaseSlaveControllers )
			outputStream << "Not all slave controller for the base could be found" << endl;
		if ( (noOfArmMasterBoards*NUMBER_OF_ARM_JOINTS) != noOfArmSlaveControllers )
			outputStream << "Not all slave controller for the arm could be found" << endl;

		outputStream << separator_;
		outputStream << "----- Check, which controllers couldn't be found: -----" << endl;

		stdOutputAndFile(outputStream);

		//Find out, which controllers couldn't be detected
		int baseCounter = 0;
		int armCounter  = 0;

		for(vector<ec_slavet>::iterator it = etherCATSlaves.begin(); it != etherCATSlaves.end(); it++)
		{
			if ( !strcmp( it->name, BASE_MASTER_BOARD) )
			{
				baseCounter++;

				for (int i = 1; i <= NUMBER_OF_BASE_JOINTS; i++)
				{
					if ( strcmp( (it+i)->name, BASE_CONTROLLER) )
					{
						outputStream << "controller of WHEEL #" << i <<" from Base #" << baseCounter << " was not detected" << endl;
						i = NUMBER_OF_BASE_JOINTS + 1;
					}
				}
			}

			if ( !strcmp( it->name, ARM_MASTER_BOARD) )
			{
				armCounter++;

				for (int i = 1; i <= NUMBER_OF_ARM_JOINTS; i++)
				{
					if ( strcmp( (it+i)->name, ARM_CONTROLLER) )
					{
						outputStream << "controller of AXIS  #" << i << " from ARM #" << armCounter << " was not detected" << endl;
						i = NUMBER_OF_ARM_JOINTS + 1;
					}
				}
			}
		}

		outputStream << separator;
		stdOutputAndFile(outputStream);
	}
}

void YouBotDiagnostics::testInitialization()
{
    InitializeJoint doInitialization;
    bool isInitialized = false;
    int noInitialization = 0;
	
	YouBotJoint *activeJoint;

	for (TopologyMap::iterator it = youBotJointMap.begin(); it != youBotJointMap.end(); it++ )
	{
		activeJoint = it->second.joint;
		
		ClearMotorControllerTimeoutFlag clearTimeoutFlag;
		activeJoint->setConfigurationParameter(clearTimeoutFlag);

		doInitialization.setParameter(false);
		activeJoint->getConfigurationParameter(doInitialization);
		doInitialization.getParameter(isInitialized);

		if (!isInitialized) 
		{
			doInitialization.setParameter(true);

			ethercatMaster->AutomaticReceiveOn(false);
			activeJoint->setConfigurationParameter(doInitialization);
			ethercatMaster->AutomaticReceiveOn(true);

			unsigned int statusFlags = 0;

			// check for the next 5 sec if the joints are commutated
			for (unsigned int u = 1; u <= 5000; u++) 
			{
				activeJoint->getStatus(statusFlags);

				if (statusFlags & INITIALIZED)
				{
					SLEEP_MILLISEC(1000);
					break;
				}
				
				SLEEP_MILLISEC(1);
			}
		}

		// the controller likes it
		SLEEP_MILLISEC(10);
			
		doInitialization.setParameter(false);
		ethercatMaster->AutomaticReceiveOn(false);
		activeJoint->getConfigurationParameter(doInitialization);
		ethercatMaster->AutomaticReceiveOn(true);
		doInitialization.getParameter(isInitialized);
		
		it->second.isInitilized = isInitialized;
	
		//show initialization status
		*outputFile  << "Commutation Status of joint #" << it->second.jointConfigNo << " of " 
				      << it->second.masterBoardNo << "." << it->second.relatedUnit << ": " 
					  << it->second.isInitilized << endl;
		
	}
	
	*outputFile << separator << endl;
}

void YouBotDiagnostics::testFunctionOfControllerboards()
{
	YouBotJoint * activeJoint;

	int slvCtr;
	vector<ec_slavet>::iterator it;

	outputStream << "----- Check functionality of the found joints: -----" << endl;
	stdOutputAndFile(outputStream);

	for( it= etherCATSlaves.begin(), slvCtr = 1 ; it != etherCATSlaves.end(); it++, slvCtr++)
	{
		float oldAngle = 0, newAngle = 0, difAngle = 0;

		JointSensedAngle	  msrAngle;
		JointSensedCurrent	  msrCurrent;
		JointAngleSetpoint	  cmdAngle;
		JointVelocitySetpoint cmdSpeed;
		JointCurrentSetpoint  cmdCurrent;
	
		
		cmdCurrent.current = 0 * ampere;
		unsigned int statusAndError = 0;

		if( !strcmp(it->name, BASE_CONTROLLER) )
		{
			outputStream <<"Testing etherCATSlave # " << slvCtr << " (i.e. joint # " << youBotJointMap[slvCtr].jointTopologyNo  << ")";
			outputStream <<" <=> WHEEL " << youBotJointMap[slvCtr].jointConfigNo << " of BASE " << youBotJointMap[slvCtr].masterBoardNo  << endl;
			stdOutputAndFile(outputStream);

			*outputFile << separator_;

			activeJoint = youBotJointMap[slvCtr].joint;

			cmdSpeed.angularVelocity = 3 * radian_per_second;
			activeJoint->setData(cmdSpeed);

			activeJoint->getData(msrAngle);
			oldAngle = msrAngle.angle.value();
			
			//check functionality for 5 seconds;
			for(int i = 0; i < 5; i++)
			{
				sleep(1);
				ethercatMaster->AutomaticReceiveOn(false);
				activeJoint->getData(msrAngle);
				ethercatMaster->AutomaticReceiveOn(true);
			}

			newAngle = msrAngle.angle.value();
			difAngle = newAngle - oldAngle;
			
			*outputFile << "difference of position: " << difAngle << endl;
			
			// same procedure backward
			cmdSpeed.angularVelocity = -3 * radian_per_second;
			activeJoint->setData(cmdSpeed);

			activeJoint->getData(msrAngle);
			oldAngle = msrAngle.angle.value();
			
			for(int i = 0; i < 5; i++)
			{
				sleep(1);
				ethercatMaster->AutomaticReceiveOn(false);
				activeJoint->getData(msrAngle);
				ethercatMaster->AutomaticReceiveOn(true);

				if ( i == 4 )
				{
					//make motor movable
					activeJoint->setData(cmdCurrent);
				}
			}
			
			newAngle = msrAngle.angle.value();
			difAngle = newAngle - oldAngle;
			
			*outputFile << "difference of position: " << difAngle << endl;
		}

		if( !strcmp(it->name, ARM_CONTROLLER) )
		{
			outputStream <<"Testing etherCATSlave # " << slvCtr << " (i.e. joint # " << youBotJointMap[slvCtr].jointTopologyNo << ")";
			outputStream <<" <=> AXIS " << youBotJointMap[slvCtr].jointConfigNo << " of MANIPULATOR " << youBotJointMap[slvCtr].masterBoardNo  << endl;
			stdOutputAndFile(outputStream);

			*outputFile << separator_;

			activeJoint = youBotJointMap[slvCtr].joint;
			
			//now set velocity mode and check position changes
			cmdSpeed.angularVelocity = 0.2 * radian_per_second;
			activeJoint->setData(cmdSpeed);

			activeJoint->getData(msrAngle);
			oldAngle = msrAngle.angle.value();
			
			//run velocitiy mode for "repeatLimit" seconds;
			int repeatLimit = 3;
			for (int i = 0; i < repeatLimit; i++)
			{
				sleep(1);
				ethercatMaster->AutomaticReceiveOn(false);
				activeJoint->getData(msrAngle);
				activeJoint->getData(msrCurrent);
				ethercatMaster->AutomaticReceiveOn(true);
				
				newAngle = msrAngle.angle.value();
				difAngle = newAngle - oldAngle;
				oldAngle = newAngle;

				*outputFile << "difference of position: "  << setw(6) << difAngle << " and current: " << msrCurrent.current.value() <<endl;
				
				//set axis back to homeposition
				if ( i == (repeatLimit-1) )
				{
					cmdAngle.angle = 0.025 * radian;
					activeJoint->setData(cmdAngle);
				}
			}
		}
		
		if( !strcmp(it->name, BASE_CONTROLLER) || !strcmp(it->name, ARM_CONTROLLER) )
		{
			if (difAngle == 0)
			{
				motorActivity[slvCtr] = false;
				//make the non working joint moveable
				activeJoint->setData(cmdCurrent);
				outputStream << "STATUS: ERROR\n";
			}
			else
			{
				motorActivity[slvCtr] = true;
				outputStream << "STATUS: OK\n";
			}

			activeJoint->getStatus(statusAndError);
			outputStream << "STATUSCODE: " << statusAndError << endl;

			cout << "DONE\n";
			stdOutputAndFile(outputStream);
			cout << separator_;
			*outputFile << separator;
		}
		
		//give the last joint the chance to drive back
		if( slvCtr == etherCATSlaves.size() )
			sleep(3);
	}
}


void YouBotDiagnostics::testFunctionManually()
{
	float actualPos = 0;
	float startPos  = 0;
	int   repeat    = 10;
	bool  clrscr    = true;

	YouBotJoint * manualJoint;

	stdOutputAndFile(outputStream);

	for (int i = 1; i < motorActivity.size(); i++)
	{
		JointSensedAngle msrAngle;
		ErrorAndStatus	 checkErrorFlag;
		unsigned int	 error  = 0;

		if (!motorActivity[i])
		{
			statusOK = false;

			TopologyMap::iterator it = youBotJointMap.find(i);

			manualJoint = it->second.joint;

			manualJoint->getData(msrAngle);
			actualPos = startPos = msrAngle.angle.value();

			manualJoint->getConfigurationParameter(checkErrorFlag);
			checkErrorFlag.getParameter(error);

			if (clrscr)
			{
				std::system("clear");
				clrscr = false;
				outputStream << "----- Moving the joints manually -----" << endl;
			}

			string jointTyp	= "";

			if ( it->second.relatedUnit == "BASE" )
				jointTyp = "WHEEL";
			else
				jointTyp = "AXIS";

			outputStream << "etherCATSlave # " << it->second.slaveNo
						 << " ( i.e. joint # "  << it->second.jointTopologyNo
						 << " ) was not moving" << endl;

			outputStream << "MOVE "	<< jointTyp	     << " "
						 << it->second.jointConfigNo   << " OF "
						 << it->second.relatedUnit   << " "
						 << it->second.masterBoardNo << " MANUALLY"
						 << endl;

			stdOutputAndFile(outputStream);

			boost::thread t(YouBotDiagnostics::waitAndPrint,repeat);

			int  ctr		  = 0;
			bool checkChanges = false;

			while ( (ctr < repeat) && !checkChanges )
			{
				manualJoint->getData(msrAngle);
				actualPos	= msrAngle.angle.value();

				if(actualPos != startPos)
					checkChanges = true;
				else
					checkChanges = false;

				ctr++;
				sleep(1);
			}

			t.join();

			cout << "DONE" << endl;

			if (checkChanges)
				outputStream << "RESULT: controller of joint # " << it->second.jointTopologyNo << " reacts on manual movement!" << endl;
			else
				outputStream << "RESULT: controller of joint # " << it->second.jointTopologyNo << " doesn't show any reaction on manual movement" << endl;

			stdOutputAndFile(outputStream);

			*outputFile << "--> pos difference: "<< actualPos  << endl << "--> errorcode: " << error << endl;

			if ( i == ( motorActivity.size() - 1 ) )
				outputStream << separator;
			else
				outputStream << separator_;

			stdOutputAndFile(outputStream);
		}
	}
}


void YouBotDiagnostics::quickRestartTest()
{
//Now restart the master a few times quickly and check, if all slaves can be found again.
	outputStream << "----- Make quick restart test: -----" << endl;
	stdOutputAndFile(outputStream);

	etherCATSlaves.clear();

	bool quickRestartTestOK = true;

	for (int i = 0; i < 3; i++)
	{
		*outputFile << "Number of all slaves before "<< i+1 <<". restart: " << noOfAllEtherCATSlaves << endl;
		if (ethercatMaster) {
			EthercatMaster::destroy();
			ethercatMaster = 0;
		}
		ethercatMaster = &EthercatMaster::getInstance(configFileName, YOUBOT_CONFIGURATIONS_DIR);
		if (ethercatMaster)
			ethercatMaster->getEthercatDiagnosticInformation(etherCATSlaves);
		*outputFile << "Number of all slaves after restart: " << etherCATSlaves.size() << endl;

		if (i != 2)
			*outputFile << separator_;

		if (noOfAllEtherCATSlaves != etherCATSlaves.size() )
			quickRestartTestOK = false;
	}
}


void YouBotDiagnostics::showSummary()
{
		outputStream << separator << "Diagnoses-test finished" << endl;
	if (statusOK && quickRestartTestOK && checkMasterboards && checkControllerboards)
		outputStream << "STATUS SUMMARY: Everything looks OK\n";
	else if (!quickRestartTestOK)
		outputStream << "STATUS SUMMARY: Quickstart failed\n";
	else if (!checkMasterboards)
		outputStream << "STATUS SUMMARY: Not all Masterboard(s) could be found\n"
					 << "Check ethernet(-cable) connection from masterboard to masterboard\n"
					 << "Or the specified value was wrong\n";
	else if (!checkControllerboards)
		outputStream << "STATUS SUMMARY: Not all Controllerboard(s) could be found\n" 
					 << "See output-file for more informations\n";
	else
		outputStream << "STATUS SUMMARY: Errors occured\n";

	stdOutputAndFile(outputStream);
}

void YouBotDiagnostics::waitAndPrint(int x)
{
	int repeat = x*10;
	int countdown = x;

	cout << "\033[?25l";

	for (int i = 0; i < repeat; i++)
	{
		cout << "You got " << setw(2) << countdown << " seconds for moving manually: ";

		if(i % 4 == 1)
			cout << "\\";
		if(i % 4 == 2)
			cout << "|";
		if(i % 4 == 3)
			cout << "/";
		if(i % 4 == 0)
			cout << "-";

		if ( !(i%10) )
			countdown--;

		cout.flush();
		cout << "\r";

		usleep(100000);
	}

	cout << "\033[?25h";
	cout << "" << endl;
}


string YouBotDiagnostics::makeLowerCase(string & convert)
{
   for( int i = 0; i<convert.length(); i++)
   {
      convert[i] = tolower(convert[i]);
   }
   return convert;
}


void YouBotDiagnostics::clrStream(stringstream & stream)
{
	stream.str("");
	stream.clear();
}


void YouBotDiagnostics::stdOutputAndFile(stringstream & stream)
{
	cout << stream.str();
	*outputFile << stream.str();

	clrStream(stream);
}

bool YouBotDiagnostics::checkHomepositionOfArm(void)
{
	bool   checkInput = false;
	string askStartposition;

	cout << "Does the ARM stand at startposition? [Y]es/[N]o: ";

	while ( !checkInput )
	{
		cin  >> askStartposition;
		cin.get(); //to delete the last ENTER from the buffer
		makeLowerCase(askStartposition);

		if ( (askStartposition == "n") || (askStartposition == "no")  )
		{
			cout << "Move arm to startposition and press ENTER" << endl;
			cout << "If the arm is blocking, repower it and restart the program!" << endl;
			cin.get();

			checkInput = true;
		}
		else
		{
			if ( (askStartposition != "yes") && (askStartposition != "y") )
				cout << "Please type (Y)ES or (N)O" << endl;
			else
				checkInput = true;
		}
	}
}
