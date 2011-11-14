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

#include "YouBotJoypad.hpp"

using namespace std;

YouBotJoypad::YouBotJoypad(int numberOfAxis, int numberOfBalls, int numberOfHat, int numberOfButton)
	   : noOfAxis(numberOfAxis), noOfBalls(numberOfBalls), noOfHat(numberOfHat), noOfButton(numberOfButton)
{
	lastHatPosition		= 0;
	longSpeedPrefix		= 0;
	transSpeedPrefix	= 0;
	rotSpeedPrefix		= 0;
	baseSpeedStep		= 0.1;
	baseSpeedRegulator	= 0.1;

	armExisting		= false;
	baseExisting	= false;

	configfile = NULL;
}


int YouBotJoypad::init(std::string joypadCfgName, std::string path)
{
	//check if manipulator hardware exists
	try
	{
		youBotArm  = new youbot::YouBotManipulator("youbot-manipulator", YOUBOT_CONFIGURATIONS_DIR);
		youBotArm->doJointCommutation();
    
   	youBotArm->calibrateManipulator();
		armExisting = true;
	}
	catch ( exception& ex )
	{
		std::cout << "ARM Exception: " << ex.what() << endl;
		armExisting = false;
	}

	//check if base hardware exists
	try
	{
		youBotBase = new youbot::YouBotBase("youbot-base", YOUBOT_CONFIGURATIONS_DIR);
    youBotBase->doJointCommutation();
		baseExisting = true;

		quantity<angular_acceleration> angAcc;
		youbot::MotorAcceleration acceleration;
		acceleration.setParameter( angAcc.from_value(100000.0) );

//		for (int i = 1; i <= 4; i++)
//		{
//			youBotBase->getBaseJoint(i).setConfigurationParameter(acceleration);
//			sleep(1);
//		}
	}
	catch ( exception& ex )
	{
		std::cout << "BASE Exception: " << ex.what() << endl;
		baseExisting = false;
	}

	//load configuration file
//	std::string joypadfilename;
//	joypadfilename = path;
//	joypadfilename.append(joypadCfgName);
//	joypadfilename.append(".cfg");
	configfile = new youbot::ConfigFile(joypadCfgName, path);

	
	if( !configfile )
	{
		cout << "Could not load file: " << path << joypadCfgName << " Check if it exists and the path is correct"<<endl;
		return -1;
	}
	
	//Set Base-Configuration
	if ( configfile->sectionExists("BASE_SETUP") )
	{
		configfile->readInto(baseSpeedStep,   "BASE_SETUP", "BaseSpeedStep");
		configfile->readInto(speedUpButton,   "BASE_SETUP", "SpeedUpButton");
		configfile->readInto(speedDownButton, "BASE_SETUP", "SpeedDownButton");
	}
	else
		cout << "No \"BASE_SETUP\" section found in the config-file\n";

	//Set Stop-Configuratoin
	if (configfile->sectionExists("STOP_APPLICATION") )
		configfile->readInto(stopButton, "STOP_APPLICATION", "StopButton");
	else
		cout << "Insert STOP_APPLICATION section into the config-file\n";

	//Set Gripper-Configuration
	if ( configfile->sectionExists("GRIPPER_SETUP") )
	{
		 configfile->readInto(gripperCloseButton, "GRIPPER_SETUP", "CloseButton");
		 configfile->readInto(gripperOpenButton,  "GRIPPER_SETUP", "OpenButton");
	}
	else
		cout << "No \"GRIPPER\" section found in the config-file\n";

	// Set Hat-Configuration
	setHatConfiguration("HAT_UP"       , HAT_UP);
	setHatConfiguration("HAT_RIGHTUP"  , HAT_RIGHTUP);
	setHatConfiguration("HAT_RIGHT"    , HAT_RIGHT);
	setHatConfiguration("HAT_RIGHTDOWN", HAT_RIGHTDOWN);
	setHatConfiguration("HAT_DOWN"     , HAT_DOWN);
	setHatConfiguration("HAT_LEFTDOWN" , HAT_LEFTDOWN);
	setHatConfiguration("HAT_LEFT"     , HAT_LEFT);
	setHatConfiguration("HAT_LEFTUP"   , HAT_LEFTUP);

	//Set Button-Configuration
	for ( int i = 0; i < noOfButton; i++ )
	{		
		stringstream buttonNoStream;
		buttonNoStream << "BUTTON_" << i;
		string buttonNo = buttonNoStream.str();

		if ( configfile->sectionExists( buttonNo.c_str() ) )
			getComponentConfiguration(&buttonMap, i, buttonNo.c_str());
	}
	
	//Set Axis-Configuration
	for ( int i = 0; i < noOfAxis; i++ )
	{
		stringstream axisNoStream;
		string axis = "";

		axisNoStream << "AXIS_" << i << "_NEGATIV";
		axis = axisNoStream.str();
		
		if ( configfile->sectionExists( axis.c_str() ) )
			getComponentConfiguration(&axisMap, i*2, axis.c_str());
		
		axisNoStream.str("");
		axisNoStream.clear();
		axisNoStream << "AXIS_" << i << "_POSITIV";
		axis = axisNoStream.str();
		
		if  ( configfile->sectionExists( axis.c_str() ) )
			getComponentConfiguration(&axisMap, i*2+1, axis.c_str());
	}

	return 0;
}


int YouBotJoypad::axisEvent(int axisIndex, int axisValue)
{
	int mapKey = axisIndex*2;

	if (axisValue > 0)
		mapKey++;

	axisIt = axisMap.find(mapKey);

	if (axisIt == axisMap.end() )
		return -1;
	else
	{
//		return activateComponent(axisIt, (bool)axisValue);
		if (axisIt->second.ptrArmComponent)
			return moveArm(axisIt->second.ptrArmComponent,   (bool)axisValue );

		if (axisIt->second.ptrBaseComponent)
			return moveBase(axisIt->second.ptrBaseComponent, (bool)axisValue );
	}
}


int YouBotJoypad::hatEvent( int hatIndex, int hatPosition)
{
	
	hatIt = hatMap.find(lastHatPosition);
	
	if (hatIt != hatMap.end() )
	{
		//activateComponent(hatIt, false);
		if (hatIt->second.ptrArmComponent)
			moveArm(hatIt->second.ptrArmComponent,   false );

		if (hatIt->second.ptrBaseComponent)
			moveBase(hatIt->second.ptrBaseComponent, false );
	}

	if (hatPosition == 0)
	{
		lastHatPosition = 0;
	}
	else
	{
		hatIt = hatMap.find(hatPosition);

		if (hatIt != hatMap.end() )
		{
			//activateComponent(hatIt, true);
			if (hatIt->second.ptrArmComponent)
				moveArm(hatIt->second.ptrArmComponent,   true );

			if (hatIt->second.ptrBaseComponent)
				moveBase(hatIt->second.ptrBaseComponent, true );
		}

		lastHatPosition = hatPosition;
	}

	return 0;
}


int YouBotJoypad::buttonPressedEvent(int buttonIndex)
{
	if( buttonIndex == speedDownButton )
	{
		if( (int(baseSpeedRegulator * 1000)/1000.0)  > 0 )
			baseSpeedRegulator-= baseSpeedStep;

		moveBase(NULL, NULL);
	}

	if( buttonIndex == speedUpButton )
	{
		if( baseSpeedRegulator < 1.2 )
			baseSpeedRegulator+= baseSpeedStep;

		moveBase(NULL, NULL);
	}

	if (buttonIndex == gripperOpenButton)
	{
		barSpacing.barSpacing = 0.023 * meter;
		if(armExisting)
			youBotArm->getArmGripper().setData(barSpacing);
	}

	if (buttonIndex == gripperCloseButton)
	{
		barSpacing.barSpacing = 0.0 * meter;
		if(armExisting)
			youBotArm->getArmGripper().setData(barSpacing);
	}

	if (buttonIndex == stopButton)
	{
		exit(0);
	}

	buttonIt = buttonMap.find(buttonIndex);

	if (buttonIt == buttonMap.end())
		return -1;
	else
	{
		//return activateComponent(buttonIt, true);
		if (buttonIt->second.ptrArmComponent)
			return moveArm(buttonIt->second.ptrArmComponent,   true );

		if (buttonIt->second.ptrBaseComponent)
			return moveBase(buttonIt->second.ptrBaseComponent, true );
	}

}


int YouBotJoypad::buttonReleasedEvent(int buttonIndex)
{
	buttonIt = buttonMap.find(buttonIndex);

	if (buttonIt == buttonMap.end() )
		return -1;
	else
	{
		//return activateComponent(buttonIt, false);
		if (buttonIt->second.ptrArmComponent)
			return moveArm(buttonIt->second.ptrArmComponent,   false );

		if (buttonIt->second.ptrBaseComponent)
			return moveBase(buttonIt->second.ptrBaseComponent, false );
	}
}


int YouBotJoypad::setHatConfiguration(const char * hatPosition, int hatPositionValue)
{
	if (configfile->sectionExists( std::string(hatPosition) ) )
		return getComponentConfiguration(&hatMap, hatPositionValue, std::string(hatPosition));
	else
		return -1;
}


int YouBotJoypad::getComponentConfiguration(inputInterfaceMap * interfaceMap, int mapKey, std::string section)
{
	string compType;
	configfile->readInto(compType, section, string("ComponentType"));

	if ( strcmp("base", compType.c_str() ) == 0)
		getBaseConfiguration(interfaceMap, mapKey, section );

	if ( strcmp("arm", compType.c_str() ) == 0)
		getArmConfiguration(interfaceMap, mapKey, section  );

	return 0;
}


int YouBotJoypad::getArmConfiguration( inputInterfaceMap * interfaceMap, int mapKey , std::string section)
{
	int   jNo;
	float jSpeed;
	int	  jPrefix;
	
	configfile->readInto(jNo,     section, string("ArmJointNo"));
	configfile->readInto(jSpeed,  section, string("ArmSpeed"));
	configfile->readInto(jPrefix, section, string("RotationPrefix"));

	inputInterfaceMap::iterator it = interfaceMap->insert( pair<int,RemoteControlableComponent>( mapKey, RemoteControlableComponent() ) ).first;
	it->second.ptrArmComponent = new ArmMoveInformation;
	it->second.ptrArmComponent->armJointNo     = jNo;
	it->second.ptrArmComponent->armSpeed       = jSpeed;
	it->second.ptrArmComponent->rotationPrefix = jPrefix;
	it->second.ptrBaseComponent = NULL;

	return 0;
}


int YouBotJoypad::getBaseConfiguration( inputInterfaceMap * interfaceMap, int mapKey , std::string section)
{
	int   longPref;
	float transPref;
	int	  rotPref;
	
	configfile->readInto(longPref,  section, string("LongitudinalPrefix"));
	configfile->readInto(transPref, section, string("TransversalPrefix"));
	configfile->readInto(rotPref,   section, string("RotationPrefix"));


	inputInterfaceMap::iterator it = interfaceMap->insert( pair<int,RemoteControlableComponent>( mapKey, RemoteControlableComponent() ) ).first;
	it->second.ptrBaseComponent = new BaseMoveInformation;
	it->second.ptrBaseComponent->longitudinalPrefix	= longPref;
	it->second.ptrBaseComponent->transversalPrefix	= transPref;
	it->second.ptrBaseComponent->rotationPrefix		= rotPref;
	it->second.ptrArmComponent = NULL;

	return 0;
}


//int YouBotJoypad::activateComponent(inputInterfaceMap::iterator it, bool pressedState)
//{
//	if (it->second.ptrArmComponent)
//		return moveArm(it->second.ptrArmComponent,   pressedState );
//
//	if (it->second.ptrBaseComponent)
//		return moveBase(it->second.ptrBaseComponent, pressedState );
//}

int YouBotJoypad::moveBase(BaseMoveInformation * baseInfo, bool pressedState)
{
	if(baseInfo != NULL)
	{
		if(pressedState)
		{
			if(baseInfo->longitudinalPrefix != 0)
				longSpeedPrefix = baseInfo->longitudinalPrefix;

			if(baseInfo->transversalPrefix != 0)
				transSpeedPrefix = baseInfo->transversalPrefix;

			if(baseInfo->rotationPrefix != 0)
				rotSpeedPrefix = baseInfo->rotationPrefix;
		}
		else
		{
			if(baseInfo->longitudinalPrefix != 0)
				longSpeedPrefix = 0.0;

			if(baseInfo->transversalPrefix != 0)
				transSpeedPrefix = 0.0;

			if(baseInfo->rotationPrefix != 0)
				rotSpeedPrefix = 0.0;
		}
	}

	longitudinalVelocity = longSpeedPrefix  * baseSpeedRegulator * meter_per_second;
	transversalVelocity	 = transSpeedPrefix * baseSpeedRegulator * meter_per_second;
	angularVelocity		 = rotSpeedPrefix   * baseSpeedRegulator * M_PI *radian_per_second;

	try
	{
		if(baseExisting)
		{
			youBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
//			quantity<si::velocity> msrlongitudinalVelocity;
//			quantity<si::velocity> msrtransversalVelocity;
//			quantity<si::angular_velocity> mrsangularVelocity;
//			youBotBase->getBaseVelocity(msrlongitudinalVelocity, msrtransversalVelocity, mrsangularVelocity);
//
//			cout << "Longspeed cmd: " << longitudinalVelocity.value() << endl;
//			cout << "Longspeed msr: " << msrlongitudinalVelocity.value() << endl;
		}

	}
	catch(exception& ex)
	{
		cout << "BaseWHAT: " << ex.what() << endl;
	}

	return 0;
}


int YouBotJoypad::moveArm(ArmMoveInformation * armInfo, bool pressedState)
{
	if(armExisting)
	{
		if(pressedState)
		{
			jVelocity.angularVelocity = armInfo->rotationPrefix * armInfo->armSpeed * radian_per_second;
			youBotArm->getArmJoint(armInfo->armJointNo).setData(jVelocity);
		}
		else
		{
//			youBotArm->getArmJoint(armInfo->armJointNo).getData(sensedAngle);
//			setAngle.angle	   = sensedAngle.angle;
			jVelocity.angularVelocity = 0.0 * radian_per_second;
			
			try
			{
//					youBotArm->getArmJoint(armInfo->armJointNo).setData(setAngle);
					youBotArm->getArmJoint(armInfo->armJointNo).setData(jVelocity);
			}
			catch( exception& ex)
			{
				cout << "WHAT: " << ex.what() << endl;
			}
		}
	}
	
	return 0;
}
