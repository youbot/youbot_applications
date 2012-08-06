/******************************************************************************
* Copyright (c) 2011
* Locomotec
*
* Author:
* Özgür Sen
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and BSD license. The dual-license implies that users of this
* code may choose which terms they prefer.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Locomotec nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 2.1 of the
* License, or (at your option) any later version or the BSD license.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL and BSD license along with this program.
*
******************************************************************************/

#ifndef YOUBOTDIAGNOSTICS_H
#define	YOUBOTDIAGNOSTICS_H

#include <fstream>
#include <boost/thread.hpp>

#include "youbot/EthercatMaster.hpp"
#include "youbot/YouBotJoint.hpp"


#define NUMBER_OF_BASE_JOINTS 4
#define NUMBER_OF_ARM_JOINTS  5

#define ARM_CONTROLLER		"TMCM-KR-841"
#define BASE_CONTROLLER		"TMCM-174"
#define ARM_MASTER_BOARD	"KR-843"
#define BASE_MASTER_BOARD	"KR-845"

const string separator("=========================================================\n");
const string separator_("–––––––––––––––––––––––––––––––––––––––\n");

typedef struct {
	string relatedUnit;          // manipulator or base part
	string relatedMasterBoard;   // ARM_MASTER_BOARD or BASE_MASTER_BOARD
	string controllerTyp;        // ARM_CONTROLLER   or BASE_CONTROLLER
	double firmwareVersion;
	bool   isInitilized;
	int masterBoardNo;
	int slaveNo;				 // All slaves
	int jointTopologyNo;		 // only slaves with input/output buffer
	int jointConfigNo;           // Configuration number of a joint according to one Arm or Base; i.e. wheel number from 1 to 4 or axis number from 1 to 5
	youbot::YouBotJoint * joint;

} JointTopology;

typedef std::map<int, JointTopology> TopologyMap;


class YouBotDiagnostics {
public:
	YouBotDiagnostics(std::fstream * file, int noOfBase = 0, int noOfArm = 0);
	~YouBotDiagnostics();

	void startDiagnostics();

	// some local function
	static void waitAndPrint(int x);

private:
	void init();

	// Get topological information of all slaves
	void getAllTopologicalInformation();

	// Show topological information of joints
	void showJointsTopologicalInformation();

	// Check if the number of found masterboards is equal to the input
	void checkNumberOfMasterboards();

	// Check if the number of found controller is correct
	void checkNumberOfControllerboards();

	// Test if commutation and initialisation is possible
	void testInitialization();
	
	// Test the functionality of the controller boards and motors
	void testFunctionOfControllerboards();

	// check the controller by moving the motors manually, if they didn't show any reaction before
	void testFunctionManually();
	
	// make a quick restart test
	void quickRestartTest();
	
	// show summary
	void showSummary();

	// some local functions
	void clrStream(std::stringstream & stream);

	void stdOutputAndFile(std::stringstream & stream);

	std::string makeLowerCase(string & convert);
	
	bool checkHomepositionOfArm(void);

private:
	// these are the slaves of the youBot connected with a motor (= youBot joint); i.e. input and output is available
	// if a slave with IO is found, the internal enumeration of the joints position
	// and other topological informatin will be saved
	TopologyMap youBotJointMap;

	// to save the controller/motor activity
	// insert one more element to have easy access via etherCAT-slaves number
	// the first element (position 0) is true and is just a placeholder
	vector<bool> motorActivity;

	bool statusOK;
	bool checkMasterboards;
	bool quickRestartTestOK;
	bool checkControllerboards;

	int inputNumberOfBase;
	int inputNumberOfArm;

	std::stringstream outputStream;

	// instance of the EthercatMaster.
	// It is a pointer because multiple instances are created and destroyed for testing purposes
	youbot::EthercatMasterInterface* ethercatMaster;

	// save the slave types in a map
	map<string, string> slaveTypMap;

	// save all slave informations from SOEM-driver
	vector<ec_slavet> etherCATSlaves;

	//number of ALL found slaves in the etherCAT-topology
	unsigned noOfAllEtherCATSlaves;

	// number of the different boards
	unsigned noOfBaseMasterBoards;
	unsigned noOfBaseSlaveControllers;
	unsigned noOfArmMasterBoards;
	unsigned noOfArmSlaveControllers;

	std::string configFileName;
	std::fstream * outputFile;

};
#endif	/* YOUBOTDIAGNOSTICS_H */
