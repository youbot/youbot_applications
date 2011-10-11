/****************************************************************
 *
 * Copyright (c) 2011
 * All rights reserved.
 *
 * Hochschule Bonn-Rhein-Sieg
 * University of Applied Sciences
 * Computer Science Department
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author : Rhama Dwiputra
 * Contributor : Azamat Shakhimardanov
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * This sofware is published under a dual-license: GNU Lesser General Public 
 * License LGPL 2.1 and ASL2.0 license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Hochschule Bonn-Rhein-Sieg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the ASL2.0 license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL and the ASL2.0 license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and ASL2.0 license along with this program.
 *
 ****************************************************************/

#include <string>
#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"
#include "bluetooth/bluetooth.h"
#include "bluetooth/rfcomm.h"
#include "androidCommandEncrypt.h"
#include "androidCommandEncrypt.hpp"

using namespace youbot;

int main() {

	/* configuration flags for different system configuration (e.g. base without arm)*/
	bool youBotHasBase = false;
	bool youBotHasArm = false;

	/* create handles for youBot base and manipulator (if available) */
	YouBotBase* androidYouBotBase = 0;

	try {
		androidYouBotBase = new YouBotBase("youbot-base", YOUBOT_CONFIGURATIONS_DIR);
		androidYouBotBase->doJointCommutation();
		youBotHasBase = true;
	} catch (std::exception& e) {
		LOG(warning) << e.what();
		youBotHasBase = false;
	}

	//helloWorld();

	/*
	* Variable for the base.
	* Here "boost units" is used to set values in OODL, that means you have to set a value and a unit.
	*/
	quantity<si::velocity> longitudinalVelocity = 0 * meter_per_second;
	quantity<si::velocity> transversalVelocity = 0 * meter_per_second;
	quantity<si::angular_velocity> angularVelocity = 0 * radian_per_second;

	/*Variable for bluetooth communication*/
	int bytes_read, client, s;
	char buffer[1024];
	bool connected = true;
	string command;
	AndroidBaseMovement androidBaseMovement;
	struct sockaddr_rc loc_address = { 0 }, rem_address = { 0 };

	/*creating socket*/
	socklen_t opt = sizeof(rem_address);
	s = socket(AF_BLUETOOTH,SOCK_STREAM, BTPROTO_RFCOMM);
	if(s < 0)
		perror("Error opening socket");

	/*binding socket*/
	loc_address.rc_family = AF_BLUETOOTH;
	loc_address.rc_channel = (uint8_t) 1;
	bind(s, (struct sockaddr *)&loc_address, sizeof(loc_address));

	/*waiting for connection*/
	printf("waiting for connection ... \n");
	listen(s,1);
	client = accept(s, (struct sockaddr *)&rem_address, &opt);
	ba2str( &rem_address.rc_bdaddr, buffer);

	/* connection accepted */
	fprintf(stderr, "accepted connection from %s \n", buffer);
	printf("receiving command \n");

	while(connected){
		memset(buffer, 0, sizeof(buffer));
		bytes_read = read(client, buffer, sizeof(buffer));
		if( bytes_read > 0){
			/* executing command */
			command.assign(buffer);
			//printf("command received [%s] \n", buffer);
			if(command == "onPause"){
				close(client);
				printf("Android paused, waiting connection ... \n");
				listen(s,1);
				client = accept(s, (struct sockaddr *)&rem_address, &opt);
				ba2str( &rem_address.rc_bdaddr, buffer);
				fprintf(stderr, " accepted connection from %s \n", buffer);
				printf("receiving command \n");
			}else if (command == "onDestroy"){
				connected = false;
			}else{
				androidBaseMovement = encryptCommand(command);
				longitudinalVelocity = androidBaseMovement.longitudinalVelocity * SPEED_MAX_LONGITUDINAL * meter_per_second;
				transversalVelocity = androidBaseMovement.transversalVelocity * SPEED_MAX_TRANSVERSAL * meter_per_second;
				angularVelocity = androidBaseMovement.angularVelocity * SPEED_MAX_ANGULAR * radian_per_second;
			}
		}

		printf("command received --> ");
		cout << "Longitudinal :" << androidBaseMovement.longitudinalVelocity;
		cout << ", Transversal :" << androidBaseMovement.transversalVelocity;
		cout << ", angular :" << androidBaseMovement.angularVelocity << endl;
		/* set youBotBase speed */
		if(youBotHasBase){
			androidYouBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
		}

	}

	/* clean up */
	if (androidYouBotBase) {
		delete androidYouBotBase;
		androidYouBotBase = 0;
	}

	LOG(info) << "Done.";

	return 0;
}

