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

#include <signal.h>

#include "YouBotDiagnostics.h"

using namespace std;
using namespace youbot;

void signalHandler(int signal);

fstream file;

/*
 * Diagnostics tool to check if the youBot components are fully functional
 */
int main(int argc, char** argv)
{
	signal(SIGINT, signalHandler);
	
	if (argc > 1)
		file.open(argv[1], ios::out);
	else
		file.open("diagnose.dgs", ios::out);

	int numberOfBase = 0;
	int numberOfArm  = 0;

	// ask for the youBot-Configuration
	cout << "How many bases are connected to the etherCAT-master: ";
	cin  >> numberOfBase;
	cout << "How many arms  are connected to the etherCAT-master: ";
	cin  >> numberOfArm;

	file << "USER INPUT: Number of Base(s) connected to the etherCAT-master: " << numberOfBase << endl;
	file << "USER INPUT: Number of Arm(s)  connected to the etherCAT-master: " << numberOfArm << endl;
	file << separator;
	
	YouBotDiagnostics diagObj(&file, numberOfBase, numberOfArm);
	
	diagObj.startDiagnostics();

	file.close();
	return 0;
}

void signalHandler(int signal)
{
	file.close();
	exit(signal);
}

