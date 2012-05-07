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

#include "youbot/YouBotBase.hpp"
#include "JointConfigurator.hpp"

using namespace std;
using namespace youbot;

void menu() {

  std::cout << "===========================================================" << std::endl;
  std::cout << "1 = show parameters from the arm joints and config file" << std::endl;
  std::cout << "2 = set config file values to joint" << std::endl;
  std::cout << "3 = store config file values to joint" << std::endl;
  std::cout << "===========================================================" << std::endl;
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


    if (argc < 1 || argc > 2) {
      std::cout << "Usage:   sudo ./JointConfigurator CONFIGFILE PATH " << std::endl;
      std::cout << "Example: sudo ./JointConfigurator ../config" << std::endl;
      return 0;
    }


    std::string configfilepath = "../config";

    configfilepath = std::string(CONFIG_DIR);

    if (argc == 2) {
      configfilepath = argv[1];
    }


    YouBotBase myYouBotBase("/youbot-base", YOUBOT_CONFIGURATIONS_DIR);


    JointConfigurator joint1(&(myYouBotBase.getBaseJoint(1)), configfilepath, "base.cfg", "");
    JointConfigurator joint2(&(myYouBotBase.getBaseJoint(2)), configfilepath, "base.cfg", "");
    JointConfigurator joint3(&(myYouBotBase.getBaseJoint(3)), configfilepath, "base.cfg", "");
    JointConfigurator joint4(&(myYouBotBase.getBaseJoint(4)), configfilepath, "base.cfg", "");
    


    char ch = 'x';

    menu();


    while (ch != '0') {

      ch = cin.get();
      switch (ch) {
        case '1':
          joint1.readParameters();
          joint2.readParameters();
          joint3.readParameters();
          joint4.readParameters();
          menu();
          break;
        case '2':
          joint1.setParametersToJoint();
          joint2.setParametersToJoint();
          joint3.setParametersToJoint();
          joint4.setParametersToJoint();
          menu();
          break;
        case '3':
          joint1.storeParametersToJoint();
          joint2.storeParametersToJoint();
          joint3.storeParametersToJoint();
          joint4.storeParametersToJoint();
          menu();
          break;
        default:
          break;

      }
    }

   // EthercatMasterFactory::destroy();

  } catch (std::exception& e) {
    std::cout << e.what() << std::endl;
  } catch (...) {
    std::cout << "unhandled exception" << std::endl;
  }


  return 0;
}
