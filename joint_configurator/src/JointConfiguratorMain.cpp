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
 * Author:
 * Jan Paulus, Nico Hochgeschwender, Michael Reckhaus, Azamat Shakhimardanov
 * Supervised by:
 * Gerhard K. Kraetzschmar
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * This sofware is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
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
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ****************************************************************/


#include <cmath>
#include <limits>
#include <iostream>
#include <vector>
#include <signal.h>
#include "youbot/YouBotManipulator.hpp"
#include "youbot/YouBotBase.hpp"
#include "JointConfigurator.hpp"


using namespace std;
using namespace youbot;

void menu() {

  //  if (ArmOrBase == ARM) {
  //    std::cout << std::endl << "Manipulator Joint " << jointNumber << " | " << jointName  << std::endl;
  //    std::cout << "Controller Type: " << controllerType << " Firmware version: " << version << std::endl;
  //  } else if (ArmOrBase == BASE) {
  //    std::cout << std::endl << "Base Joint " << jointNumber << " | " << jointName  << std::endl;
  //    std::cout << "Controller Type: " << controllerType << " Firmware version: " << version << std::endl;
  //  }

  std::cout << "===========================================================" << std::endl;
  std::cout << "1 = show parameters from joint and config file" << std::endl;
  std::cout << "2 = set config file values to joint" << std::endl;
  std::cout << "3 = store config file values to joint" << std::endl;
  std::cout << "===========================================================" << std::endl;
  std::cout << "4 = show password protected parameters" << std::endl;
  std::cout << "5 = enter password" << std::endl;
  std::cout << "6 = set password protected parameters" << std::endl;
  std::cout << "7 = store password protected parameters" << std::endl;
  std::cout << "===========================================================" << std::endl;
  std::cout << "8 = show read only parameters" << std::endl;
  std::cout << "0 = quit" << std::endl;
  std::cout << "===========================================================" << std::endl;
  std::cout << ": " << std::flush;
}

bool running = true;

void sigintHandler(int signal) {
  running = false;
  std::cout << std::endl << " Interrupt!" << std::endl;
}

int main(int argc, char *argv[]) {

  //  signal(SIGINT, sigintHandler);

  try {

    if (argc < 3 || argc > 5) {
      std::cout << "Usage:   sudo ./JointConfigurator MODULE JOINTNUMBER [CONFIGFILE] " << std::endl;
      std::cout << "Example: sudo ./JointConfigurator base 1" << std::endl;
      std::cout << "         sudo ./JointConfigurator arm 1 joint-parameter.cfg" << std::endl;
      return 0;
    }

    part baseOrArm;
    std::string arg2 = argv[1];

    if (arg2 == "base") {
      baseOrArm = BASE;
    } else if (arg2 == "arm") {
      baseOrArm = ARM;
    } else {
      std::cout << "Usage:   sudo ./JointConfigurator MODULE JOINTNUMBER [CONFIGFILE] " << std::endl;
      std::cout << "Example: sudo ./JointConfigurator base 1" << std::endl;
      std::cout << "         sudo ./JointConfigurator arm 1 arm.cfg arm-1-protected.cfg" << std::endl;
      return 0;
    }

    int jointNo = 0;
    jointNo = atoi(argv[2]);

    std::string configfile;
    std::string configfileProtected; 

    if (argc >= 4) {
      configfile = argv[3];
    }

    if (argc >= 5) {
      configfileProtected = argv[4];
    }

    std::string configfilepath = "../config";

    configfilepath = std::string(CONFIG_DIR);

    YouBotJoint* joint;
    YouBotBase* myYouBotBase = NULL;
    YouBotManipulator* myYouBotManipulator = NULL;

    if (baseOrArm == ARM) {
       myYouBotManipulator = new YouBotManipulator("youbot-manipulator", YOUBOT_CONFIGURATIONS_DIR);
      joint = &(myYouBotManipulator->getArmJoint(jointNo));
    } else if (baseOrArm == BASE) {
       myYouBotBase = new YouBotBase("youbot-base", YOUBOT_CONFIGURATIONS_DIR);
      joint = &(myYouBotBase->getBaseJoint(jointNo));
    } else {
      throw "unable to create joint";
    }

    JointConfigurator helper(joint, configfilepath, configfile, configfileProtected);


    char ch = 'x';

    menu();

    while (ch != '0') {

      ch = cin.get();
      switch (ch) {
        case '1':
          helper.readParameters();
          menu();
          break;
        case '2':
          helper.setParametersToJoint();
          menu();
          break;
        case '3':
          helper.storeParametersToJoint();
          menu();
          break;
        case '4':
          helper.readPasswordProtectedParameters();
          menu();
          break;
        case '5':
          helper.getPassword();
          menu();
          break;
        case '6':
          helper.setProtectedParametersToJoint();
          menu();
          break;
        case '7':
          helper.storeProtectedParametersToJoint();
          menu();
          break;
        case '8':
          helper.readReadOnlyParameters();
          menu();
          break;
        default:
          break;

      }
    }

    if(myYouBotManipulator != NULL)
      delete myYouBotManipulator;
    
    if(myYouBotBase != NULL)
      delete myYouBotBase;
    
   // EthercatMasterFactory::destroy();

  } catch (std::exception& e) {
    std::cout << e.what() << std::endl;
  } catch (...) {
    std::cout << "unhandled exception" << std::endl;
  }


  return 0;
}
