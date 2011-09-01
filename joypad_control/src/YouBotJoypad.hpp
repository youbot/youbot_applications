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

#ifndef JOYPAD_HPP
#define	JOYPAD_HPP

#include <iostream>
#include <vector>
#include <string>
#include <map>

#include <stdint.h>

#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"

class YouBotJoypad
{
	// enums and structs to save the state of the joypad
	enum HAT_STATE {
		HAT_UP		  = 0x01,
		HAT_RIGHT	  = 0x02,
		HAT_DOWN	  = 0x04,
		HAT_LEFT	  = 0x08,
		HAT_RIGHTUP	  = (HAT_RIGHT | HAT_UP),   //0x03
		HAT_RIGHTDOWN = (HAT_RIGHT | HAT_DOWN), //0x06
		HAT_LEFTUP    = (HAT_LEFT | HAT_UP),	//0x09
		HAT_LEFTDOWN  = (HAT_LEFT | HAT_DOWN)	//0x12
	};

	typedef struct {
		int longitudinalPrefix;
		int transversalPrefix;
		int rotationPrefix;
	} BaseMoveInformation;

	typedef struct {
		int	  armJointNo;
		float armSpeed;
		int	  rotationPrefix;
	} ArmMoveInformation;

	typedef struct {
		ArmMoveInformation  * ptrArmComponent;
		BaseMoveInformation * ptrBaseComponent;
	} RemoteControlableComponent;

	typedef map<int, RemoteControlableComponent > inputInterfaceMap;

public:

	///c-tor
	///@param numberOfAxis   The number of found axis    from a joypad
	///@param numberOfBalls  The number of found balls   from a joypadjoypad_config
	///@param numberOfHat    The number of found hats    from a joypad
	///@param numberOfButton The number of found buttons from a joypad
	YouBotJoypad(int numberOfAxis, int numberOfBalls, int numberOfHat, int numberOfButton);
	~YouBotJoypad(){};

	///Initializes the youBot (base and arm) and the joypad
	///@param joypadCfgName The name of the configuration file for the joypad
	///@param path          The name of the path, where the configurationfiles can be found
	int init(std::string joypadCfgName = "youbot-joypad", std::string path = CONFIG_DIR);

	///Handles axis events
	///@param axisIndex The joypad axis index
	///@param axisValue The value of the moved axis
	int axisEvent(int axisIndex,   int axisValue);

	///Handles hat events
	///@param hatIndex The joypad hat index
	///@param hatValue The hat position value --> up, down etc.
	int hatEvent (int hatIndex, int hatPosition);

	///Sets the state for button down events
	///@param buttonIndex The joypad button index, which was pressed
	int buttonPressedEvent( int buttonIndex);

	///Sets the state for button up events
	///@param buttonIndex The joypad button index, which was released
	int buttonReleasedEvent(int buttonIndex);

private:

	///calls the following move methods
//	int activateComponent(inputInterfaceMap::iterator it, bool pressedState);

	///moves the base according to the base-movement-information
	///@param baseState Holds the information about how to move the base
	///@param pressedReleasedState When a hat, button or axis was pressed, its true, otherwise false (released)
	int moveBase(BaseMoveInformation * baseInfo, bool pressedState);
	
	///moves the arm according to the arm-movement information
	///@param armState Holds the information about how to move the arm
	///@param pressedReleasedState When a hat, button or axis was pressed, its true, otherwise false (released)
	int moveArm(ArmMoveInformation * armInfo, bool pressedState);

	///Sets the configuration for the hat input interface
	///@param hatPosition The hat-position, which is going to be configured
	int setHatConfiguration(const char * hatPosition, int hatPositionValue);


	/// calls the following getXConfiguration methods
	int getComponentConfiguration(inputInterfaceMap * interfaceMap, int mapKey, std::string section);
	///get arm configuration from the configfile
	///@param interfaceMap The map of a certain input interface from the joypad --> button, axis or hat
	///@param confFile A configfile with a section, which was set before
	int getArmConfiguration(inputInterfaceMap * interfaceMap, int mapKey, std::string section);

	///get base configuration from the configfile
	///@param interfaceMap The map which saves the states of a certain input interface from the joypad --> button, axis or hat
	///@param confFile A configfile with a section, which was set before
	int getBaseConfiguration(inputInterfaceMap * interfaceMap, int mapKey, std::string section);

	
private:

	youbot::YouBotBase		  * youBotBase;
	youbot::YouBotManipulator * youBotArm;

	youbot::ConfigFile* configfile;

//	bool hatTrigger;
	bool armExisting;
	bool baseExisting;
	bool gripperExisting;

	// To save the number of the joypad input interfaces; e.g. number of available axis
	int noOfAxis, noOfBalls, noOfHat, noOfButton;
	
	// To save the last hat position, before pressing a new one or releasing the hat input device
	int lastHatPosition;
	
	// To save the information about a component, that should react on an input event
	inputInterfaceMap hatMap; 
	inputInterfaceMap::iterator hatIt;

	inputInterfaceMap axisMap;
	inputInterfaceMap::iterator axisIt;
	
	inputInterfaceMap buttonMap;
	inputInterfaceMap::iterator buttonIt;
	
	// variables used to controll the base
	float longSpeedPrefix;
	float transSpeedPrefix;
	float rotSpeedPrefix;
	float baseSpeedStep;
	float baseSpeedRegulator;

	quantity<si::velocity> longitudinalVelocity;
	quantity<si::velocity> transversalVelocity;
    quantity<si::angular_velocity> angularVelocity;

	// variables used to get and set jointAngles or set velocity of the arm
    youbot::JointSensedAngle   sensedAngle;
    youbot::JointAngleSetpoint setAngle;
    youbot::JointVelocitySetpoint jVelocity;

	// to open or close the gripper
    youbot::GripperBarSpacingSetPoint barSpacing;


	//for saving globally valid button configurations
	unsigned speedUpButton;
	unsigned speedDownButton;
	unsigned stopButton;
	unsigned gripperOpenButton;
	unsigned gripperCloseButton;
	
};
#endif	/* JOYPAD_HPP */

