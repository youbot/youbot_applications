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


#include "JointConfigurator.hpp"

JointConfigurator::JointConfigurator(YouBotJoint* youbotjoint, std::string configpath, std::string configname, std::string configNameProtected) {

  ParameterRead = false;
  ProtectedParameterRead = false;
  configfile = NULL;
  configfilePP = NULL;

  if (configpath.at(configpath.length() - 1) != '/') {
    configpath.append("/");
  }

  if (configname == "") {
    UseParameter = false;
  } else {
    configfile = new ConfigFile(configname, configpath);
    UseParameter = true;
  }

  if (configNameProtected == "") {
    UseProtectedParameter = false;
  } else {
    configfilePP = new ConfigFile(configNameProtected, configpath);
    UseProtectedParameter = true;
  }

  this->joint = youbotjoint;
  FirmwareVersion firmwareVersion;

  joint->getConfigurationParameter(firmwareVersion);
  firmwareVersion.getParameter(controllerType, version);

  JointName jName;
  joint->getConfigurationParameter(jName);
  jName.getParameter(jointName);

  std::cout << std::setprecision(15);


  double firmwareVer = 0;
  int controller = 0;
  if (UseParameter) {
    configfile->readInto(firmwareVer, "Joint_Type", "FirmwareVersion");
    configfile->readInto(controller, "Joint_Type", "ControllerType");
    if ((version != firmwareVer) || (controller != controllerType)) {
      UseParameter = false;
      throw std::runtime_error("The configuration file for the joint parameter contain the wrong controller type or firmware version!");
      delete configfile;
    }
  }

  if (UseProtectedParameter) {
    configfilePP->readInto(firmwareVer, "Joint_Type", "FirmwareVersion");
    configfilePP->readInto(controller, "Joint_Type", "ControllerType");
    if (!(AreSame(version, firmwareVer) && controller == controllerType)) {
      UseProtectedParameter = false;
      throw std::runtime_error("The configuration file for the protected joint parameter contain the wrong controller type or firmware version!");
      delete configfile;
      delete configfilePP;
    }
  }

}

JointConfigurator::~JointConfigurator() {
  delete configfile;
  delete configfilePP;
}

bool JointConfigurator::AreSame(double A, double B) {
  return std::fabs(A - B) < 0.001;
}

void JointConfigurator::readParameters() {
  double dummy;

  if (!UseParameter) {
    std::cout << "There is no configuration file provided for the parameters!" << std::endl;
    return;
  }

  std::cout << std::endl << "===========================================================" << std::endl;
  std::cout << "Joint: " << jointName << std::endl;
  std::cout << "Controller Type: " << controllerType << " Firmware version: " << version << std::endl << std::endl;

  joint->getConfigurationParameter(MaximumPositioningVelocity_Parameter);
  MaximumPositioningVelocity_Parameter.getParameter(MaximumPositioningVelocity_actual);
  configfile->readInto(dummy, "Joint_Parameter", "MaximumPositioningVelocity");
  MaximumPositioningVelocity_file = dummy * radian_per_second;
  if (!AreSame(MaximumPositioningVelocity_actual.value(), MaximumPositioningVelocity_file.value())) {
    std::cout << "MaximumPositioningVelocity \t\t\t\tactual: " << MaximumPositioningVelocity_actual << " \tNEW VALUE: " << MaximumPositioningVelocity_file << std::endl;
  } else {
    std::cout << "MaximumPositioningVelocity \t\t\t\tactual: " << MaximumPositioningVelocity_actual << std::endl;
  }

  joint->getConfigurationParameter(MotorAcceleration_Parameter);
  MotorAcceleration_Parameter.getParameter(MotorAcceleration_actual);
  configfile->readInto(dummy, "Joint_Parameter", "MotorAcceleration");
  MotorAcceleration_file = dummy * radian_per_second / second;
  if (!AreSame(MotorAcceleration_actual.value(), MotorAcceleration_file.value())) {
    std::cout << "MotorAcceleration           \t\t\t\tactual: " << MotorAcceleration_actual << " \tNEW VALUE: " << MotorAcceleration_file << std::endl;
  } else {
    std::cout << "MotorAcceleration           \t\t\t\tactual: " << MotorAcceleration_actual << std::endl;
  }

  joint->getConfigurationParameter(RampGeneratorSpeedAndPositionControl_Parameter);
  RampGeneratorSpeedAndPositionControl_Parameter.getParameter(RampGeneratorSpeedAndPositionControl_actual);
  configfile->readInto(dummy, "Joint_Parameter", "RampGeneratorSpeedAndPositionControl");
  RampGeneratorSpeedAndPositionControl_file = dummy;
  if (!AreSame(RampGeneratorSpeedAndPositionControl_actual, RampGeneratorSpeedAndPositionControl_file)) {
    std::cout << "RampGeneratorSpeedAndPositionControl \t\t\tactual: " << RampGeneratorSpeedAndPositionControl_actual << " \tNEW VALUE: " << RampGeneratorSpeedAndPositionControl_file << std::endl;
  } else {
    std::cout << "RampGeneratorSpeedAndPositionControl \t\t\tactual: " << RampGeneratorSpeedAndPositionControl_actual << std::endl;
  }

  joint->getConfigurationParameter(PositionControlSwitchingThreshold_Parameter);
  PositionControlSwitchingThreshold_Parameter.getParameter(PositionControlSwitchingThreshold_actual);
  configfile->readInto(dummy, "Joint_Parameter", "PositionControlSwitchingThreshold");
  PositionControlSwitchingThreshold_file = dummy * radian_per_second;
  if (!AreSame(PositionControlSwitchingThreshold_actual.value(), PositionControlSwitchingThreshold_file.value())) {
    std::cout << "PositionControlSwitchingThreshold \t\t\tactual: " << PositionControlSwitchingThreshold_actual << " \tNEW VALUE: " << PositionControlSwitchingThreshold_file << std::endl;
  } else {
    std::cout << "PositionControlSwitchingThreshold \t\t\tactual: " << PositionControlSwitchingThreshold_actual << std::endl;
  }

  joint->getConfigurationParameter(SpeedControlSwitchingThreshold_Parameter);
  SpeedControlSwitchingThreshold_Parameter.getParameter(SpeedControlSwitchingThreshold_actual);
  configfile->readInto(dummy, "Joint_Parameter", "SpeedControlSwitchingThreshold");
  SpeedControlSwitchingThreshold_file = dummy * radian_per_second;
  if (!AreSame(SpeedControlSwitchingThreshold_actual.value(), SpeedControlSwitchingThreshold_file.value())) {
    std::cout << "SpeedControlSwitchingThreshold \t\t\t\tactual: " << SpeedControlSwitchingThreshold_actual << " \tNEW VALUE: " << SpeedControlSwitchingThreshold_file << std::endl;
  } else {
    std::cout << "SpeedControlSwitchingThreshold \t\t\t\tactual: " << SpeedControlSwitchingThreshold_actual << std::endl;
  }

  joint->getConfigurationParameter(CurrentControlSwitchingThreshold_Parameter);
  CurrentControlSwitchingThreshold_Parameter.getParameter(CurrentControlSwitchingThreshold_actual);
  configfile->readInto(dummy, "Joint_Parameter", "CurrentControlSwitchingThreshold");
  CurrentControlSwitchingThreshold_file = dummy * radian_per_second;
  if (!AreSame(CurrentControlSwitchingThreshold_actual.value(), CurrentControlSwitchingThreshold_file.value())) {
    std::cout << "CurrentControlSwitchingThreshold  \t\t\tactual: " << CurrentControlSwitchingThreshold_actual << " \tNEW VALUE: " << CurrentControlSwitchingThreshold_file << std::endl;
  } else {
    std::cout << "CurrentControlSwitchingThreshold  \t\t\tactual: " << CurrentControlSwitchingThreshold_actual << std::endl;
  }

  joint->getConfigurationParameter(PParameterFirstParametersPositionControl_Parameter);
  PParameterFirstParametersPositionControl_Parameter.getParameter(PParameterFirstParametersPositionControl_actual);
  configfile->readInto(dummy, "Joint_Parameter", "PParameterFirstParametersPositionControl");
  PParameterFirstParametersPositionControl_file = dummy;
  if (!AreSame(PParameterFirstParametersPositionControl_actual, PParameterFirstParametersPositionControl_file)) {
    std::cout << "PParameterFirstParametersPositionControl \t\tactual: " << PParameterFirstParametersPositionControl_actual << " \tNEW VALUE: " << PParameterFirstParametersPositionControl_file << std::endl;
  } else {
    std::cout << "PParameterFirstParametersPositionControl \t\tactual: " << PParameterFirstParametersPositionControl_actual << std::endl;
  }

  joint->getConfigurationParameter(IParameterFirstParametersPositionControl_Parameter);
  IParameterFirstParametersPositionControl_Parameter.getParameter(IParameterFirstParametersPositionControl_actual);
  configfile->readInto(dummy, "Joint_Parameter", "IParameterFirstParametersPositionControl");
  IParameterFirstParametersPositionControl_file = dummy;
  if (!AreSame(IParameterFirstParametersPositionControl_actual, IParameterFirstParametersPositionControl_file)) {
    std::cout << "IParameterFirstParametersPositionControl \t\tactual: " << IParameterFirstParametersPositionControl_actual << " \tNEW VALUE: " << IParameterFirstParametersPositionControl_file << std::endl;
  } else {
    std::cout << "IParameterFirstParametersPositionControl \t\tactual: " << IParameterFirstParametersPositionControl_actual << std::endl;
  }

  joint->getConfigurationParameter(DParameterFirstParametersPositionControl_Parameter);
  DParameterFirstParametersPositionControl_Parameter.getParameter(DParameterFirstParametersPositionControl_actual);
  configfile->readInto(dummy, "Joint_Parameter", "DParameterFirstParametersPositionControl");
  DParameterFirstParametersPositionControl_file = dummy;
  if (!AreSame(DParameterFirstParametersPositionControl_actual, DParameterFirstParametersPositionControl_file)) {
    std::cout << "DParameterFirstParametersPositionControl \t\tactual: " << DParameterFirstParametersPositionControl_actual << " \tNEW VALUE: " << DParameterFirstParametersPositionControl_file << std::endl;
  } else {
    std::cout << "DParameterFirstParametersPositionControl \t\tactual: " << DParameterFirstParametersPositionControl_actual << std::endl;
  }

  joint->getConfigurationParameter(IClippingParameterFirstParametersPositionControl_Parameter);
  IClippingParameterFirstParametersPositionControl_Parameter.getParameter(IClippingParameterFirstParametersPositionControl_actual);
  configfile->readInto(dummy, "Joint_Parameter", "IClippingParameterFirstParametersPositionControl");
  IClippingParameterFirstParametersPositionControl_file = dummy;
  if (!AreSame(IClippingParameterFirstParametersPositionControl_actual, IClippingParameterFirstParametersPositionControl_file)) {
    std::cout << "IClippingParameterFirstParametersPositionControl \tactual: " << IClippingParameterFirstParametersPositionControl_actual << " \tNEW VALUE: " << IClippingParameterFirstParametersPositionControl_file << std::endl;
  } else {
    std::cout << "IClippingParameterFirstParametersPositionControl \tactual: " << IClippingParameterFirstParametersPositionControl_actual << std::endl;
  }


  joint->getConfigurationParameter(PParameterFirstParametersSpeedControl_Parameter);
  PParameterFirstParametersSpeedControl_Parameter.getParameter(PParameterFirstParametersSpeedControl_actual);
  configfile->readInto(dummy, "Joint_Parameter", "PParameterFirstParametersSpeedControl");
  PParameterFirstParametersSpeedControl_file = dummy;
  if (!AreSame(PParameterFirstParametersSpeedControl_actual, PParameterFirstParametersSpeedControl_file)) {
    std::cout << "PParameterFirstParametersSpeedControl \t\t\tactual: " << PParameterFirstParametersSpeedControl_actual << " \tNEW VALUE: " << PParameterFirstParametersSpeedControl_file << std::endl;
  } else {
    std::cout << "PParameterFirstParametersSpeedControl \t\t\tactual: " << PParameterFirstParametersSpeedControl_actual << std::endl;
  }

  joint->getConfigurationParameter(IParameterFirstParametersSpeedControl_Parameter);
  IParameterFirstParametersSpeedControl_Parameter.getParameter(IParameterFirstParametersSpeedControl_actual);
  configfile->readInto(dummy, "Joint_Parameter", "IParameterFirstParametersSpeedControl");
  IParameterFirstParametersSpeedControl_file = dummy;
  if (!AreSame(IParameterFirstParametersSpeedControl_actual, IParameterFirstParametersSpeedControl_file)) {
    std::cout << "IParameterFirstParametersSpeedControl \t\t\tactual: " << IParameterFirstParametersSpeedControl_actual << " \tNEW VALUE: " << IParameterFirstParametersSpeedControl_file << std::endl;
  } else {
    std::cout << "IParameterFirstParametersSpeedControl \t\t\tactual: " << IParameterFirstParametersSpeedControl_actual << std::endl;
  }

  joint->getConfigurationParameter(DParameterFirstParametersSpeedControl_Parameter);
  DParameterFirstParametersSpeedControl_Parameter.getParameter(DParameterFirstParametersSpeedControl_actual);
  configfile->readInto(dummy, "Joint_Parameter", "DParameterFirstParametersSpeedControl");
  DParameterFirstParametersSpeedControl_file = dummy;
  if (!AreSame(DParameterFirstParametersSpeedControl_actual, DParameterFirstParametersSpeedControl_file)) {
    std::cout << "DParameterFirstParametersSpeedControl \t\t\tactual: " << DParameterFirstParametersSpeedControl_actual << " \tNEW VALUE: " << DParameterFirstParametersSpeedControl_file << std::endl;
  } else {
    std::cout << "DParameterFirstParametersSpeedControl \t\t\tactual: " << DParameterFirstParametersSpeedControl_actual << std::endl;
  }

  joint->getConfigurationParameter(IClippingParameterFirstParametersSpeedControl_Parameter);
  IClippingParameterFirstParametersSpeedControl_Parameter.getParameter(IClippingParameterFirstParametersSpeedControl_actual);
  configfile->readInto(dummy, "Joint_Parameter", "IClippingParameterFirstParametersSpeedControl");
  IClippingParameterFirstParametersSpeedControl_file = dummy;
  if (!AreSame(IClippingParameterFirstParametersSpeedControl_actual, IClippingParameterFirstParametersSpeedControl_file)) {
    std::cout << "IClippingParameterFirstParametersSpeedControl \t\tactual: " << IClippingParameterFirstParametersSpeedControl_actual << " \tNEW VALUE: " << IClippingParameterFirstParametersSpeedControl_file << std::endl;
  } else {
    std::cout << "IClippingParameterFirstParametersSpeedControl \t\tactual: " << IClippingParameterFirstParametersSpeedControl_actual << std::endl;
  }

  joint->getConfigurationParameter(PParameterFirstParametersCurrentControl_Parameter);
  PParameterFirstParametersCurrentControl_Parameter.getParameter(PParameterFirstParametersCurrentControl_actual);
  configfile->readInto(dummy, "Joint_Parameter", "PParameterFirstParametersCurrentControl");
  PParameterFirstParametersCurrentControl_file = dummy;
  if (!AreSame(PParameterFirstParametersCurrentControl_actual, PParameterFirstParametersCurrentControl_file)) {
    std::cout << "PParameterFirstParametersCurrentControl \t\tactual: " << PParameterFirstParametersCurrentControl_actual << " \tNEW VALUE: " << PParameterFirstParametersCurrentControl_file << std::endl;
  } else {
    std::cout << "PParameterFirstParametersCurrentControl \t\tactual: " << PParameterFirstParametersCurrentControl_actual << std::endl;
  }

  joint->getConfigurationParameter(IParameterFirstParametersCurrentControl_Parameter);
  IParameterFirstParametersCurrentControl_Parameter.getParameter(IParameterFirstParametersCurrentControl_actual);
  configfile->readInto(dummy, "Joint_Parameter", "IParameterFirstParametersCurrentControl");
  IParameterFirstParametersCurrentControl_file = dummy;
  if (!AreSame(IParameterFirstParametersCurrentControl_actual, IParameterFirstParametersCurrentControl_file)) {
    std::cout << "IParameterFirstParametersCurrentControl \t\tactual: " << IParameterFirstParametersCurrentControl_actual << " \tNEW VALUE: " << IParameterFirstParametersCurrentControl_file << std::endl;
  } else {
    std::cout << "IParameterFirstParametersCurrentControl \t\tactual: " << IParameterFirstParametersCurrentControl_actual << std::endl;
  }

  joint->getConfigurationParameter(DParameterFirstParametersCurrentControl_Parameter);
  DParameterFirstParametersCurrentControl_Parameter.getParameter(DParameterFirstParametersCurrentControl_actual);
  configfile->readInto(dummy, "Joint_Parameter", "DParameterFirstParametersCurrentControl");
  DParameterFirstParametersCurrentControl_file = dummy;
  if (!AreSame(DParameterFirstParametersCurrentControl_actual, DParameterFirstParametersCurrentControl_file)) {
    std::cout << "DParameterFirstParametersCurrentControl \t\tactual: " << DParameterFirstParametersCurrentControl_actual << " \tNEW VALUE: " << DParameterFirstParametersCurrentControl_file << std::endl;
  } else {
    std::cout << "DParameterFirstParametersCurrentControl \t\tactual: " << DParameterFirstParametersCurrentControl_actual << std::endl;
  }

  joint->getConfigurationParameter(IClippingParameterFirstParametersCurrentControl_Parameter);
  IClippingParameterFirstParametersCurrentControl_Parameter.getParameter(IClippingParameterFirstParametersCurrentControl_actual);
  configfile->readInto(dummy, "Joint_Parameter", "IClippingParameterFirstParametersCurrentControl");
  IClippingParameterFirstParametersCurrentControl_file = dummy;
  if (!AreSame(IClippingParameterFirstParametersCurrentControl_actual, IClippingParameterFirstParametersCurrentControl_file)) {
    std::cout << "IClippingParameterFirstParametersCurrentControl \tactual: " << IClippingParameterFirstParametersCurrentControl_actual << " \tNEW VALUE: " << IClippingParameterFirstParametersCurrentControl_file << std::endl;
  } else {
    std::cout << "IClippingParameterFirstParametersCurrentControl \tactual: " << IClippingParameterFirstParametersCurrentControl_actual << std::endl;
  }

  joint->getConfigurationParameter(PParameterSecondParametersPositionControl_Parameter);
  PParameterSecondParametersPositionControl_Parameter.getParameter(PParameterSecondParametersPositionControl_actual);
  configfile->readInto(dummy, "Joint_Parameter", "PParameterSecondParametersPositionControl");
  PParameterSecondParametersPositionControl_file = dummy;
  if (!AreSame(PParameterSecondParametersPositionControl_actual, PParameterSecondParametersPositionControl_file)) {
    std::cout << "PParameterSecondParametersPositionControl \t\tactual: " << PParameterSecondParametersPositionControl_actual << " \tNEW VALUE: " << PParameterSecondParametersPositionControl_file << std::endl;
  } else {
    std::cout << "PParameterSecondParametersPositionControl \t\tactual: " << PParameterSecondParametersPositionControl_actual << std::endl;
  }

  joint->getConfigurationParameter(IParameterSecondParametersPositionControl_Parameter);
  IParameterSecondParametersPositionControl_Parameter.getParameter(IParameterSecondParametersPositionControl_actual);
  configfile->readInto(dummy, "Joint_Parameter", "IParameterSecondParametersPositionControl");
  IParameterSecondParametersPositionControl_file = dummy;
  if (!AreSame(IParameterSecondParametersPositionControl_actual, IParameterSecondParametersPositionControl_file)) {
    std::cout << "IParameterSecondParametersPositionControl \t\tactual: " << IParameterSecondParametersPositionControl_actual << " \tNEW VALUE: " << IParameterSecondParametersPositionControl_file << std::endl;
  } else {
    std::cout << "IParameterSecondParametersPositionControl \t\tactual: " << IParameterSecondParametersPositionControl_actual << std::endl;
  }

  joint->getConfigurationParameter(DParameterSecondParametersPositionControl_Parameter);
  DParameterSecondParametersPositionControl_Parameter.getParameter(DParameterSecondParametersPositionControl_actual);
  configfile->readInto(dummy, "Joint_Parameter", "DParameterSecondParametersPositionControl");
  DParameterSecondParametersPositionControl_file = dummy;
  if (!AreSame(DParameterSecondParametersPositionControl_actual, DParameterSecondParametersPositionControl_file)) {
    std::cout << "DParameterSecondParametersPositionControl \t\tactual: " << DParameterSecondParametersPositionControl_actual << " \tNEW VALUE: " << DParameterSecondParametersPositionControl_file << std::endl;
  } else {
    std::cout << "DParameterSecondParametersPositionControl \t\tactual: " << DParameterSecondParametersPositionControl_actual << std::endl;
  }

  joint->getConfigurationParameter(IClippingParameterSecondParametersPositionControl_Parameter);
  IClippingParameterSecondParametersPositionControl_Parameter.getParameter(IClippingParameterSecondParametersPositionControl_actual);
  configfile->readInto(dummy, "Joint_Parameter", "IClippingParameterSecondParametersPositionControl");
  IClippingParameterSecondParametersPositionControl_file = dummy;
  if (!AreSame(IClippingParameterSecondParametersPositionControl_actual, IClippingParameterSecondParametersPositionControl_file)) {
    std::cout << "IClippingParameterSecondParametersPositionControl \tactual: " << IClippingParameterSecondParametersPositionControl_actual << " \tNEW VALUE: " << IClippingParameterSecondParametersPositionControl_file << std::endl;
  } else {
    std::cout << "IClippingParameterSecondParametersPositionControl \tactual: " << IClippingParameterSecondParametersPositionControl_actual << std::endl;
  }

  joint->getConfigurationParameter(PParameterSecondParametersSpeedControl_Parameter);
  PParameterSecondParametersSpeedControl_Parameter.getParameter(PParameterSecondParametersSpeedControl_actual);
  configfile->readInto(dummy, "Joint_Parameter", "PParameterSecondParametersSpeedControl");
  PParameterSecondParametersSpeedControl_file = dummy;
  if (!AreSame(PParameterSecondParametersSpeedControl_actual, PParameterSecondParametersSpeedControl_file)) {
    std::cout << "PParameterSecondParametersSpeedControl \t\t\tactual: " << PParameterSecondParametersSpeedControl_actual << " \tNEW VALUE: " << PParameterSecondParametersSpeedControl_file << std::endl;
  } else {
    std::cout << "PParameterSecondParametersSpeedControl \t\t\tactual: " << PParameterSecondParametersSpeedControl_actual << std::endl;
  }

  joint->getConfigurationParameter(IParameterSecondParametersSpeedControl_Parameter);
  IParameterSecondParametersSpeedControl_Parameter.getParameter(IParameterSecondParametersSpeedControl_actual);
  configfile->readInto(dummy, "Joint_Parameter", "IParameterSecondParametersSpeedControl");
  IParameterSecondParametersSpeedControl_file = dummy;
  if (!AreSame(IParameterSecondParametersSpeedControl_actual, IParameterSecondParametersSpeedControl_file)) {
    std::cout << "IParameterSecondParametersSpeedControl \t\t\tactual: " << IParameterSecondParametersSpeedControl_actual << " \tNEW VALUE: " << IParameterSecondParametersSpeedControl_file << std::endl;
  } else {
    std::cout << "IParameterSecondParametersSpeedControl \t\t\tactual: " << IParameterSecondParametersSpeedControl_actual << std::endl;
  }

  joint->getConfigurationParameter(DParameterSecondParametersSpeedControl_Parameter);
  DParameterSecondParametersSpeedControl_Parameter.getParameter(DParameterSecondParametersSpeedControl_actual);
  configfile->readInto(dummy, "Joint_Parameter", "DParameterSecondParametersSpeedControl");
  DParameterSecondParametersSpeedControl_file = dummy;
  if (!AreSame(DParameterSecondParametersSpeedControl_actual, DParameterSecondParametersSpeedControl_file)) {
    std::cout << "DParameterSecondParametersSpeedControl \t\t\tactual: " << DParameterSecondParametersSpeedControl_actual << " \tNEW VALUE: " << DParameterSecondParametersSpeedControl_file << std::endl;
  } else {
    std::cout << "DParameterSecondParametersSpeedControl \t\t\tactual: " << DParameterSecondParametersSpeedControl_actual << std::endl;
  }

  joint->getConfigurationParameter(IClippingParameterSecondParametersSpeedControl_Parameter);
  IClippingParameterSecondParametersSpeedControl_Parameter.getParameter(IClippingParameterSecondParametersSpeedControl_actual);
  configfile->readInto(dummy, "Joint_Parameter", "IClippingParameterSecondParametersSpeedControl");
  IClippingParameterSecondParametersSpeedControl_file = dummy;
  if (!AreSame(IClippingParameterSecondParametersSpeedControl_actual, IClippingParameterSecondParametersSpeedControl_file)) {
    std::cout << "IClippingParameterSecondParametersSpeedControl \t\tactual: " << IClippingParameterSecondParametersSpeedControl_actual << " \tNEW VALUE: " << IClippingParameterSecondParametersSpeedControl_file << std::endl;
  } else {
    std::cout << "IClippingParameterSecondParametersSpeedControl \t\tactual: " << IClippingParameterSecondParametersSpeedControl_actual << std::endl;
  }
  joint->getConfigurationParameter(PParameterSecondParametersCurrentControl_Parameter);
  PParameterSecondParametersCurrentControl_Parameter.getParameter(PParameterSecondParametersCurrentControl_actual);
  configfile->readInto(dummy, "Joint_Parameter", "PParameterSecondParametersCurrentControl");
  PParameterSecondParametersCurrentControl_file = dummy;
  if (!AreSame(PParameterSecondParametersCurrentControl_actual, PParameterSecondParametersCurrentControl_file)) {
    std::cout << "PParameterSecondParametersCurrentControl     \t\tactual: " << PParameterSecondParametersCurrentControl_actual << " \tNEW VALUE: " << PParameterSecondParametersCurrentControl_file << std::endl;
  } else {
    std::cout << "PParameterSecondParametersCurrentControl     \t\tactual: " << PParameterSecondParametersCurrentControl_actual << std::endl;
  }

  joint->getConfigurationParameter(IParameterSecondParametersCurrentControl_Parameter);
  IParameterSecondParametersCurrentControl_Parameter.getParameter(IParameterSecondParametersCurrentControl_actual);
  configfile->readInto(dummy, "Joint_Parameter", "IParameterSecondParametersCurrentControl");
  IParameterSecondParametersCurrentControl_file = dummy;
  if (!AreSame(IParameterSecondParametersCurrentControl_actual, IParameterSecondParametersCurrentControl_file)) {
    std::cout << "IParameterSecondParametersCurrentControl \t\tactual: " << IParameterSecondParametersCurrentControl_actual << " \tNEW VALUE: " << IParameterSecondParametersCurrentControl_file << std::endl;
  } else {
    std::cout << "IParameterSecondParametersCurrentControl \t\tactual: " << IParameterSecondParametersCurrentControl_actual << std::endl;
  }

  joint->getConfigurationParameter(DParameterSecondParametersCurrentControl_Parameter);
  DParameterSecondParametersCurrentControl_Parameter.getParameter(DParameterSecondParametersCurrentControl_actual);
  configfile->readInto(dummy, "Joint_Parameter", "DParameterSecondParametersCurrentControl");
  DParameterSecondParametersCurrentControl_file = dummy;
  if (!AreSame(DParameterSecondParametersCurrentControl_actual, DParameterSecondParametersCurrentControl_file)) {
    std::cout << "DParameterSecondParametersCurrentControl \t\tactual: " << DParameterSecondParametersCurrentControl_actual << " \tNEW VALUE: " << DParameterSecondParametersCurrentControl_file << std::endl;
  } else {
    std::cout << "DParameterSecondParametersCurrentControl \t\tactual: " << DParameterSecondParametersCurrentControl_actual << std::endl;
  }

  joint->getConfigurationParameter(IClippingParameterSecondParametersCurrentControl_Parameter);
  IClippingParameterSecondParametersCurrentControl_Parameter.getParameter(IClippingParameterSecondParametersCurrentControl_actual);
  configfile->readInto(dummy, "Joint_Parameter", "IClippingParameterSecondParametersCurrentControl");
  IClippingParameterSecondParametersCurrentControl_file = dummy;
  if (!AreSame(IClippingParameterSecondParametersCurrentControl_actual, IClippingParameterSecondParametersCurrentControl_file)) {
    std::cout << "IClippingParameterSecondParametersCurrentControl \tactual: " << IClippingParameterSecondParametersCurrentControl_actual << " \tNEW VALUE: " << IClippingParameterSecondParametersCurrentControl_file << std::endl;
  } else {
    std::cout << "IClippingParameterSecondParametersCurrentControl \tactual: " << IClippingParameterSecondParametersCurrentControl_actual << std::endl;
  }

  joint->getConfigurationParameter(MaximumVelocityToSetPosition_Parameter);
  MaximumVelocityToSetPosition_Parameter.getParameter(MaximumVelocityToSetPosition_actual);
  configfile->readInto(dummy, "Joint_Parameter", "MaximumVelocityToSetPosition");
  MaximumVelocityToSetPosition_file = dummy * radian_per_second;
  if (!AreSame(MaximumVelocityToSetPosition_actual.value(), MaximumVelocityToSetPosition_file.value())) {
    std::cout << "MaximumVelocityToSetPosition \t\t\t\tactual: " << MaximumVelocityToSetPosition_actual << " \tNEW VALUE: " << MaximumVelocityToSetPosition_file << std::endl;
  } else {
    std::cout << "MaximumVelocityToSetPosition \t\t\t\tactual: " << MaximumVelocityToSetPosition_actual << std::endl;
  }

  joint->getConfigurationParameter(PositionTargetReachedDistance_Parameter);
  PositionTargetReachedDistance_Parameter.getParameter(PositionTargetReachedDistance_actual);
  configfile->readInto(dummy, "Joint_Parameter", "PositionTargetReachedDistance");
  PositionTargetReachedDistance_file = dummy;
  if (PositionTargetReachedDistance_actual != PositionTargetReachedDistance_file) {
    std::cout << "PositionTargetReachedDistance \t\t\t\tactual: " << PositionTargetReachedDistance_actual << " \tNEW VALUE: " << PositionTargetReachedDistance_file << std::endl;
  } else {
    std::cout << "PositionTargetReachedDistance \t\t\t\tactual: " << PositionTargetReachedDistance_actual << std::endl;
  }

  joint->getConfigurationParameter(VelocityThresholdForHallFX_Parameter);
  VelocityThresholdForHallFX_Parameter.getParameter(VelocityThresholdForHallFX_actual);
  configfile->readInto(dummy, "Joint_Parameter", "VelocityThresholdForHallFX");
  VelocityThresholdForHallFX_file = dummy * radian_per_second;
  if (!AreSame(VelocityThresholdForHallFX_actual.value(), VelocityThresholdForHallFX_file.value())) {
    std::cout << "VelocityThresholdForHallFX \t\t\t\tactual: " << VelocityThresholdForHallFX_actual << " \tNEW VALUE: " << VelocityThresholdForHallFX_file << std::endl;
  } else {
    std::cout << "VelocityThresholdForHallFX \t\t\t\tactual: " << VelocityThresholdForHallFX_actual << std::endl;
  }

  ParameterRead = true;
}

void JointConfigurator::readPasswordProtectedParameters() {

  if (!UseProtectedParameter) {
    std::cout << "There is no configuration file provided for the protected parameters!" << std::endl;
    return;
  }

  std::cout << "===================== Password Protected Parameters =====================" << std::endl;
  std::cout << "Joint: " << jointName << std::endl;
  std::cout << "Controller Type: " << controllerType << " Firmware version: " << version << std::endl << std::endl;

  double dummy;

  joint->getConfigurationParameter(PWMLimit_Parameter);
  PWMLimit_Parameter.getParameter(PWMLimit_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "PWMLimit");
  PWMLimit_file = dummy;
  if (PWMLimit_actual != PWMLimit_file) {
    std::cout << "PWMLimit                  \t\t\t\tactual: " << PWMLimit_actual << " \tNEW VALUE: " << PWMLimit_file << std::endl;
  } else {
    std::cout << "PWMLimit                  \t\t\t\tactual: " << PWMLimit_actual << std::endl;
  }

  joint->getConfigurationParameter(MaximumMotorCurrent_Parameter);
  MaximumMotorCurrent_Parameter.getParameter(MaximumMotorCurrent_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "MaximumMotorCurrent");
  MaximumMotorCurrent_file = dummy * ampere;
  if (!AreSame(MaximumMotorCurrent_actual.value(), MaximumMotorCurrent_file.value())) {
    std::cout << "MaximumMotorCurrent      \t\t\t\tactual: " << MaximumMotorCurrent_actual << " \tNEW VALUE: " << MaximumMotorCurrent_file << std::endl;
  } else {
    std::cout << "MaximumMotorCurrent      \t\t\t\tactual: " << MaximumMotorCurrent_actual << std::endl;
  }

  joint->getConfigurationParameter(ThermalWindingTimeConstant_Parameter);
  ThermalWindingTimeConstant_Parameter.getParameter(ThermalWindingTimeConstant_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "ThermalWindingTimeConstant");
  ThermalWindingTimeConstant_file = dummy * si::seconds;
  if (!AreSame(ThermalWindingTimeConstant_actual.value(), ThermalWindingTimeConstant_file.value())) {
    std::cout << "ThermalWindingTimeConstant \t\t\t\tactual: " << ThermalWindingTimeConstant_actual << " \tNEW VALUE: " << ThermalWindingTimeConstant_file << std::endl;
  } else {
    std::cout << "ThermalWindingTimeConstant \t\t\t\tactual: " << ThermalWindingTimeConstant_actual << std::endl;
  }

  joint->getConfigurationParameter(I2tLimit_Parameter);
  I2tLimit_Parameter.getParameter(I2tLimit_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "I2tLimit");
  I2tLimit_file = dummy;
  if (!AreSame(I2tLimit_actual, I2tLimit_file)) {
    std::cout << "I2tLimit \t\t\t\t\t\tactual: " << I2tLimit_actual << " \tNEW VALUE: " << I2tLimit_file << std::endl;
  } else {
    std::cout << "I2tLimit \t\t\t\t\t\tactual: " << I2tLimit_actual << std::endl;
  }




  joint->getConfigurationParameter(PIDControlTime_Parameter);
  PIDControlTime_Parameter.getParameter(PIDControlTime_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "PIDControlTime");
  PIDControlTime_file = dummy * si::seconds;
  if (!AreSame(PIDControlTime_actual.value(), PIDControlTime_file.value())) {
    std::cout << "PIDControlTime           \t\t\t\tactual: " << PIDControlTime_actual << " \tNEW VALUE: " << PIDControlTime_file << std::endl;
  } else {
    std::cout << "PIDControlTime           \t\t\t\tactual: " << PIDControlTime_actual << std::endl;
  }

  joint->getConfigurationParameter(CurrentControlLoopDelay_Parameter);
  CurrentControlLoopDelay_Parameter.getParameter(CurrentControlLoopDelay_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "CurrentControlLoopDelay");
  CurrentControlLoopDelay_file = dummy * si::seconds;
  if (!AreSame(CurrentControlLoopDelay_actual.value(), CurrentControlLoopDelay_file.value())) {
    std::cout << "CurrentControlLoopDelay  \t\t\t\tactual: " << CurrentControlLoopDelay_actual << " \tNEW VALUE: " << CurrentControlLoopDelay_file << std::endl;
  } else {
    std::cout << "CurrentControlLoopDelay  \t\t\t\tactual: " << CurrentControlLoopDelay_actual << std::endl;
  }

  joint->getConfigurationParameter(PWMHysteresis_Parameter);
  PWMHysteresis_Parameter.getParameter(PWMHysteresis_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "PWMHysteresis");
  PWMHysteresis_file = dummy;
  if (!AreSame(PWMHysteresis_actual, PWMHysteresis_file)) {
    std::cout << "PWMHysteresis \t\t\t\t\t\tactual: " << PWMHysteresis_actual << " \tNEW VALUE: " << PWMHysteresis_file << std::endl;
  } else {
    std::cout << "PWMHysteresis \t\t\t\t\t\tactual: " << PWMHysteresis_actual << std::endl;
  }

  joint->getConfigurationParameter(CommutationMode_Parameter);
  CommutationMode_Parameter.getParameter(CommutationMode_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "CommutationMode");
  CommutationMode_file = dummy;
  if (!AreSame(CommutationMode_actual, CommutationMode_file)) {
    std::cout << "CommutationMode \t\t\t\t\tactual: " << CommutationMode_actual << " \tNEW VALUE: " << CommutationMode_file << std::endl;
  } else {
    std::cout << "CommutationMode \t\t\t\t\tactual: " << CommutationMode_actual << std::endl;
  }

  joint->getConfigurationParameter(SetEncoderCounterZeroAtNextNChannel_Parameter);
  SetEncoderCounterZeroAtNextNChannel_Parameter.getParameter(SetEncoderCounterZeroAtNextNChannel_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "SetEncoderCounterZeroAtNextNChannel");
  SetEncoderCounterZeroAtNextNChannel_file = dummy;
  if (!AreSame(SetEncoderCounterZeroAtNextNChannel_actual, SetEncoderCounterZeroAtNextNChannel_file)) {
    std::cout << "SetEncoderCounterZeroAtNextNChannel \t\t\tactual: " << SetEncoderCounterZeroAtNextNChannel_actual << " \tNEW VALUE: " << SetEncoderCounterZeroAtNextNChannel_file << std::endl;
  } else {
    std::cout << "SetEncoderCounterZeroAtNextNChannel \t\t\tactual: " << SetEncoderCounterZeroAtNextNChannel_actual << std::endl;
  }

  joint->getConfigurationParameter(SetEncoderCounterZeroAtNextSwitch_Parameter);
  SetEncoderCounterZeroAtNextSwitch_Parameter.getParameter(SetEncoderCounterZeroAtNextSwitch_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "SetEncoderCounterZeroAtNextSwitch");
  SetEncoderCounterZeroAtNextSwitch_file = dummy;
  if (!AreSame(SetEncoderCounterZeroAtNextSwitch_actual, SetEncoderCounterZeroAtNextSwitch_file)) {
    std::cout << "SetEncoderCounterZeroAtNextSwitch  \t\t\tactual: " << SetEncoderCounterZeroAtNextSwitch_actual << " \tNEW VALUE: " << SetEncoderCounterZeroAtNextSwitch_file << std::endl;
  } else {
    std::cout << "SetEncoderCounterZeroAtNextSwitch  \t\t\tactual: " << SetEncoderCounterZeroAtNextSwitch_actual << std::endl;
  }

  joint->getConfigurationParameter(SetEncoderCounterZeroOnlyOnce_Parameter);
  SetEncoderCounterZeroOnlyOnce_Parameter.getParameter(SetEncoderCounterZeroOnlyOnce_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "SetEncoderCounterZeroOnlyOnce");
  SetEncoderCounterZeroOnlyOnce_file = dummy;
  if (!AreSame(SetEncoderCounterZeroOnlyOnce_actual, SetEncoderCounterZeroOnlyOnce_file)) {
    std::cout << "SetEncoderCounterZeroOnlyOnce  \t\t\t\tactual: " << SetEncoderCounterZeroOnlyOnce_actual << " \tNEW VALUE: " << SetEncoderCounterZeroOnlyOnce_file << std::endl;
  } else {
    std::cout << "SetEncoderCounterZeroOnlyOnce  \t\t\t\tactual: " << SetEncoderCounterZeroOnlyOnce_actual << std::endl;
  }

  joint->getConfigurationParameter(EncoderStopSwitch_Parameter);
  EncoderStopSwitch_Parameter.getParameter(EncoderStopSwitch_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "EncoderStopSwitch");
  EncoderStopSwitch_file = dummy;
  if (!AreSame(EncoderStopSwitch_actual, EncoderStopSwitch_file)) {
    std::cout << "EncoderStopSwitch  \t\t\t\t\tactual: " << EncoderStopSwitch_actual << " \tNEW VALUE: " << EncoderStopSwitch_file << std::endl;
  } else {
    std::cout << "EncoderStopSwitch  \t\t\t\t\tactual: " << EncoderStopSwitch_actual << std::endl;
  }

  joint->getConfigurationParameter(StopSwitchPolarity_Parameter);
  StopSwitchPolarity_Parameter.getParameter(StopSwitchPolarity_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "StopSwitchPolarity");
  StopSwitchPolarity_file = dummy;
  if (!AreSame(StopSwitchPolarity_actual, StopSwitchPolarity_file)) {
    std::cout << "StopSwitchPolarity \t\t\t\t\tactual: " << StopSwitchPolarity_actual << " \tNEW VALUE: " << StopSwitchPolarity_file << std::endl;
  } else {
    std::cout << "StopSwitchPolarity \t\t\t\t\tactual: " << StopSwitchPolarity_actual << std::endl;
  }

  joint->getConfigurationParameter(CommutationMotorCurrent_Parameter);
  CommutationMotorCurrent_Parameter.getParameter(CommutationMotorCurrent_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "CommutationMotorCurrent");
  CommutationMotorCurrent_file = dummy * ampere;
  if (!AreSame(CommutationMotorCurrent_actual.value(), CommutationMotorCurrent_file.value())) {
    std::cout << "CommutationMotorCurrent \t\t\t\tactual: " << CommutationMotorCurrent_actual << " \tNEW VALUE: " << CommutationMotorCurrent_file << std::endl;
  } else {
    std::cout << "CommutationMotorCurrent \t\t\t\tactual: " << CommutationMotorCurrent_actual << std::endl;
  }

  joint->getConfigurationParameter(MassInertiaConstant_Parameter);
  MassInertiaConstant_Parameter.getParameter(MassInertiaConstant_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "MassInertiaConstant");
  MassInertiaConstant_file = dummy;
  if (!AreSame(MassInertiaConstant_actual, MassInertiaConstant_file)) {
    std::cout << "MassInertiaConstant \t\t\t\t\tactual: " << MassInertiaConstant_actual << " \tNEW VALUE: " << MassInertiaConstant_file << std::endl;
  } else {
    std::cout << "MassInertiaConstant \t\t\t\t\tactual: " << MassInertiaConstant_actual << std::endl;
  }

  joint->getConfigurationParameter(BEMFConstant_Parameter);
  BEMFConstant_Parameter.getParameter(BEMFConstant_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "BEMFConstant");
  BEMFConstant_file = dummy;
  if (!AreSame(BEMFConstant_actual, BEMFConstant_file)) {
    std::cout << "BEMFConstant \t\t\t\t\t\tactual: " << BEMFConstant_actual << " \tNEW VALUE: " << BEMFConstant_file << std::endl;
  } else {
    std::cout << "BEMFConstant \t\t\t\t\t\tactual: " << BEMFConstant_actual << std::endl;
  }

  joint->getConfigurationParameter(SineInitializationVelocity_Parameter);
  SineInitializationVelocity_Parameter.getParameter(SineInitializationVelocity_actual);
  configfilePP->readInto(SineInitializationVelocity_file, "Joint_Parameter", "SineInitializationVelocity");
  if (SineInitializationVelocity_actual != SineInitializationVelocity_file) {
    std::cout << "SineInitializationVelocity \t\t\t\tactual: " << SineInitializationVelocity_actual << " \tNEW VALUE: " << SineInitializationVelocity_file << std::endl;
  } else {
    std::cout << "SineInitializationVelocity \t\t\t\tactual: " << SineInitializationVelocity_actual << std::endl;
  }

  joint->getConfigurationParameter(CommutationCompensationClockwise_Parameter);
  CommutationCompensationClockwise_Parameter.getParameter(CommutationCompensationClockwise_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "CommutationCompensationClockwise");
  CommutationCompensationClockwise_file = dummy;
  if (!AreSame(CommutationCompensationClockwise_actual, CommutationCompensationClockwise_file)) {
    std::cout << "CommutationCompensationClockwise \t\t\tactual: " << CommutationCompensationClockwise_actual << " \tNEW VALUE: " << CommutationCompensationClockwise_file << std::endl;
  } else {
    std::cout << "CommutationCompensationClockwise \t\t\tactual: " << CommutationCompensationClockwise_actual << std::endl;
  }

  joint->getConfigurationParameter(CommutationCompensationCounterClockwise_Parameter);
  CommutationCompensationCounterClockwise_Parameter.getParameter(CommutationCompensationCounterClockwise_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "CommutationCompensationCounterClockwise");
  CommutationCompensationCounterClockwise_file = dummy;
  if (!AreSame(CommutationCompensationCounterClockwise_actual, CommutationCompensationCounterClockwise_file)) {
    std::cout << "CommutationCompensationCounterClockwise \t\tactual: " << CommutationCompensationCounterClockwise_actual << " \tNEW VALUE: " << CommutationCompensationCounterClockwise_file << std::endl;
  } else {
    std::cout << "CommutationCompensationCounterClockwise \t\tactual: " << CommutationCompensationCounterClockwise_actual << std::endl;
  }

  joint->getConfigurationParameter(InitSineDelay_Parameter);
  InitSineDelay_Parameter.getParameter(InitSineDelay_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "InitSineDelay");
  InitSineDelay_file = dummy * si::seconds;
  if (!AreSame(InitSineDelay_actual.value(), InitSineDelay_file.value())) {
    std::cout << "InitSineDelay \t\t\t\t\t\tactual: " << InitSineDelay_actual << " \tNEW VALUE: " << InitSineDelay_file << std::endl;
  } else {
    std::cout << "InitSineDelay \t\t\t\t\t\tactual: " << InitSineDelay_actual << std::endl;
  }

  joint->getConfigurationParameter(ActivateOvervoltageProtection_Parameter);
  ActivateOvervoltageProtection_Parameter.getParameter(ActivateOvervoltageProtection_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "ActivateOvervoltageProtection");
  ActivateOvervoltageProtection_file = dummy;
  if (!AreSame(ActivateOvervoltageProtection_actual, ActivateOvervoltageProtection_file)) {
    std::cout << "ActivateOvervoltageProtection \t\t\t\tactual: " << ActivateOvervoltageProtection_actual << " \tNEW VALUE: " << ActivateOvervoltageProtection_file << std::endl;
  } else {
    std::cout << "ActivateOvervoltageProtection \t\t\t\tactual: " << ActivateOvervoltageProtection_actual << std::endl;
  }

  joint->getConfigurationParameter(SineCompensationFactor_Parameter);
  SineCompensationFactor_Parameter.getParameter(SineCompensationFactor_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "SineCompensationFactor");
  SineCompensationFactor_file = dummy;
  if (!AreSame(SineCompensationFactor_actual, SineCompensationFactor_file)) {
    std::cout << "SineCompensationFactor \t\t\t\t\tactual: " << SineCompensationFactor_actual << " \tNEW VALUE: " << SineCompensationFactor_file << std::endl;
  } else {
    std::cout << "SineCompensationFactor \t\t\t\t\tactual: " << SineCompensationFactor_actual << std::endl;
  }

  joint->getConfigurationParameter(EncoderResolution_Parameter);
  EncoderResolution_Parameter.getParameter(EncoderResolution_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "EncoderResolution");
  EncoderResolution_file = dummy;
  if (!AreSame(EncoderResolution_actual, EncoderResolution_file)) {
    std::cout << "EncoderResolution \t\t\t\t\tactual: " << EncoderResolution_actual << " \tNEW VALUE: " << EncoderResolution_file << std::endl;
  } else {
    std::cout << "EncoderResolution \t\t\t\t\tactual: " << EncoderResolution_actual << std::endl;
  }

  joint->getConfigurationParameter(HallSensorPolarityReversal_Parameter);
  HallSensorPolarityReversal_Parameter.getParameter(HallSensorPolarityReversal_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "HallSensorPolarityReversal");
  HallSensorPolarityReversal_file = dummy;
  if (!AreSame(HallSensorPolarityReversal_actual, HallSensorPolarityReversal_file)) {
    std::cout << "HallSensorPolarityReversal \t\t\t\tactual: " << HallSensorPolarityReversal_actual << " \tNEW VALUE: " << HallSensorPolarityReversal_file << std::endl;
  } else {
    std::cout << "HallSensorPolarityReversal \t\t\t\tactual: " << HallSensorPolarityReversal_actual << std::endl;
  }

  joint->getConfigurationParameter(InitializationMode_Parameter);
  InitializationMode_Parameter.getParameter(InitializationMode_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "InitializationMode");
  InitializationMode_file = dummy;
  if (!AreSame(InitializationMode_actual, InitializationMode_file)) {
    std::cout << "InitializationMode \t\t\t\t\tactual: " << InitializationMode_actual << " \tNEW VALUE: " << InitializationMode_file << std::endl;
  } else {
    std::cout << "InitializationMode \t\t\t\t\tactual: " << InitializationMode_actual << std::endl;
  }

  joint->getConfigurationParameter(MotorCoilResistance_Parameter);
  MotorCoilResistance_Parameter.getParameter(MotorCoilResistance_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "MotorCoilResistance");
  MotorCoilResistance_file = dummy * ohm;
  if (!AreSame(MotorCoilResistance_actual.value(), MotorCoilResistance_file.value())) {
    std::cout << "MotorCoilResistance \t\t\t\t\tactual: " << MotorCoilResistance_actual << " \tNEW VALUE: " << MotorCoilResistance_file << std::endl;
  } else {
    std::cout << "MotorCoilResistance \t\t\t\t\tactual: " << MotorCoilResistance_actual << std::endl;
  }

  joint->getConfigurationParameter(MotorPoles_Parameter);
  MotorPoles_Parameter.getParameter(MotorPoles_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "MotorPoles");
  MotorPoles_file = dummy;
  if (!AreSame(MotorPoles_actual, MotorPoles_file)) {
    std::cout << "MotorPoles \t\t\t\t\t\tactual: " << MotorPoles_actual << " \tNEW VALUE: " << MotorPoles_file << std::endl;
  } else {
    std::cout << "MotorPoles \t\t\t\t\t\tactual: " << MotorPoles_actual << std::endl;
  }

  joint->getConfigurationParameter(PWMSchemeBlockCommutation_Parameter);
  PWMSchemeBlockCommutation_Parameter.getParameter(PWMSchemeBlockCommutation_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "PWMSchemeBlockCommutation");
  PWMSchemeBlockCommutation_file = dummy;
  if (!AreSame(PWMSchemeBlockCommutation_actual, PWMSchemeBlockCommutation_file)) {
    std::cout << "PWMSchemeBlockCommutation \t\t\t\tactual: " << PWMSchemeBlockCommutation_actual << " \tNEW VALUE: " << PWMSchemeBlockCommutation_file << std::endl;
  } else {
    std::cout << "PWMSchemeBlockCommutation \t\t\t\tactual: " << PWMSchemeBlockCommutation_actual << std::endl;
  }

  joint->getConfigurationParameter(ReversingEncoderDirection_Parameter);
  ReversingEncoderDirection_Parameter.getParameter(ReversingEncoderDirection_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "ReversingEncoderDirection");
  ReversingEncoderDirection_file = dummy;
  if (!AreSame(ReversingEncoderDirection_actual, ReversingEncoderDirection_file)) {
    std::cout << "ReversingEncoderDirection \t\t\t\tactual: " << ReversingEncoderDirection_actual << " \tNEW VALUE: " << ReversingEncoderDirection_file << std::endl;
  } else {
    std::cout << "ReversingEncoderDirection \t\t\t\tactual: " << ReversingEncoderDirection_actual << std::endl;
  }

  joint->getConfigurationParameter(MotorControllerTimeout_Parameter);
  MotorControllerTimeout_Parameter.getParameter(MotorControllerTimeout_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "MotorControllerTimeout");
  MotorControllerTimeout_file = dummy * second;
  if (!AreSame(MotorControllerTimeout_actual.value(), MotorControllerTimeout_file.value())) {
    std::cout << "MotorControllerTimeout \t\t\t\t\tactual: " << MotorControllerTimeout_actual << " \tNEW VALUE: " << MotorControllerTimeout_file << std::endl;
  } else {
    std::cout << "MotorControllerTimeout \t\t\t\t\tactual: " << MotorControllerTimeout_actual << std::endl;
  }

  joint->getConfigurationParameter(MotorHaltedVelocity_Parameter);
  MotorHaltedVelocity_Parameter.getParameter(MotorHaltedVelocity_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "MotorHaltedVelocity");
  MotorHaltedVelocity_file = dummy;
  if (!AreSame(MotorHaltedVelocity_actual, MotorHaltedVelocity_file)) {
    std::cout << "MotorHaltedVelocity \t\t\t\t\tactual: " << MotorHaltedVelocity_actual << " \tNEW VALUE: " << MotorHaltedVelocity_file << std::endl;
  } else {
    std::cout << "MotorHaltedVelocity \t\t\t\t\tactual: " << MotorHaltedVelocity_actual << std::endl;
  }



  ProtectedParameterRead = true;
}

//=======================READ ONLY =============================

void JointConfigurator::readReadOnlyParameters() {

  std::cout << "===================== Read Only Parameters =====================" << std::endl;
  std::cout << "Joint: " << jointName << std::endl;
  std::cout << "Controller Type: " << controllerType << " Firmware version: " << version << std::endl << std::endl;

  joint->getConfigurationParameter(OperationalTime_Parameter);
  OperationalTime_Parameter.getParameter(OperationalTime_actual);
  std::cout << "OperationalTime \t\t\t\t\tactual: " << OperationalTime_actual << std::endl;

  joint->getConfigurationParameter(ActualMotorVoltage_Parameter);
  ActualMotorVoltage_Parameter.getParameter(ActualMotorVoltage_actual);
  std::cout << "ActualMotorVoltage \t\t\t\t\tactual: " << ActualMotorVoltage_actual << std::endl;

  joint->getConfigurationParameter(ActualPWMDutyCycle_Parameter);
  ActualPWMDutyCycle_Parameter.getParameter(ActualPWMDutyCycle_actual);
  std::cout << "ActualPWMDutyCycle \t\t\t\t\tactual: " << ActualPWMDutyCycle_actual << std::endl;

  joint->getConfigurationParameter(ActualCommutationOffset_Parameter);
  ActualCommutationOffset_Parameter.getParameter(ActualCommutationOffset_actual);
  std::cout << "ActualCommutationOffset \t\t\t\tactual: " << ActualCommutationOffset_actual << std::endl;

  joint->getConfigurationParameter(PositionError_Parameter);
  PositionError_Parameter.getParameter(PositionError_actual);
  std::cout << "PositionError \t\t\t\t\t\tactual: " << PositionError_actual << std::endl;

  joint->getConfigurationParameter(PositionErrorSum_Parameter);
  PositionErrorSum_Parameter.getParameter(PositionErrorSum_actual);
  std::cout << "PositionErrorSum \t\t\t\t\tactual: " << PositionErrorSum_actual << std::endl;

  joint->getConfigurationParameter(VelocityError_Parameter);
  VelocityError_Parameter.getParameter(VelocityError_actual);
  std::cout << "VelocityError \t\t\t\t\t\tactual: " << VelocityError_actual << std::endl;

  joint->getConfigurationParameter(VelocityErrorSum_Parameter);
  VelocityErrorSum_Parameter.getParameter(VelocityErrorSum_actual);
  std::cout << "VelocityErrorSum \t\t\t\t\tactual: " << VelocityErrorSum_actual << std::endl;

  joint->getConfigurationParameter(CurrentError_Parameter);
  CurrentError_Parameter.getParameter(CurrentError_actual);
  std::cout << "CurrentError \t\t\t\t\t\tactual: " << CurrentError_actual << std::endl;

  joint->getConfigurationParameter(CurrentErrorSum_Parameter);
  CurrentErrorSum_Parameter.getParameter(CurrentErrorSum_actual);
  std::cout << "CurrentErrorSum \t\t\t\t\tactual: " << CurrentErrorSum_actual << std::endl;

  joint->getConfigurationParameter(RampGeneratorSpeed_Parameter);
  RampGeneratorSpeed_Parameter.getParameter(RampGeneratorSpeed_actual);
  std::cout << "RampGeneratorSpeed \t\t\t\t\tactual: " << RampGeneratorSpeed_actual << std::endl;

  joint->getConfigurationParameter(I2tSum_Parameter);
  I2tSum_Parameter.getParameter(I2tSum_actual);
  std::cout << "I2tSum \t\t\t\t\t\t\tactual: " << I2tSum_actual << std::endl;

  joint->getConfigurationParameter(I2tExceedCounter_Parameter);
  I2tExceedCounter_Parameter.getParameter(I2tExceedCounter_actual);
  std::cout << "I2tExceedCounter \t\t\t\t\tactual: " << I2tExceedCounter_actual << std::endl;

  joint->getConfigurationParameter(ActualMotorDriverTemperature_Parameter);
  ActualMotorDriverTemperature_Parameter.getParameter(ActualMotorDriverTemperature_actual);
  std::cout << "ActualMotorDriverTemperature \t\t\t\tactual: " << ActualMotorDriverTemperature_actual << std::endl;

  //  joint->getConfigurationParameter(ActualModuleSupplyCurrent_Parameter);
  //  ActualModuleSupplyCurrent_Parameter.getParameter(ActualModuleSupplyCurrent_actual);
  //  std::cout << "ActualModuleSupplyCurrent \t\t\t\tactual: " << ActualModuleSupplyCurrent_actual << std::endl;


}

void JointConfigurator::setParametersToJoint() {
  if (!ParameterRead) {
    std::cout << "The Joint Parameters have to been read before hand!" << std::endl;
    return;
  }

  std::cout << "Setting parameters for Joint: " << jointName << std::endl;

  if (!AreSame(MaximumPositioningVelocity_actual.value(), MaximumPositioningVelocity_file.value())) {
    MaximumPositioningVelocity_Parameter.setParameter(MaximumPositioningVelocity_file);
    joint->setConfigurationParameter(MaximumPositioningVelocity_Parameter);
    std::cout << "MaximumPositioningVelocity set to: " << MaximumPositioningVelocity_file << std::endl;
  }

  if (!AreSame(MotorAcceleration_actual.value(), MotorAcceleration_file.value())) {
    MotorAcceleration_Parameter.setParameter(MotorAcceleration_file);
    joint->setConfigurationParameter(MotorAcceleration_Parameter);
    std::cout << "MotorAcceleration set to: " << MotorAcceleration_file << std::endl;
  }

  if (!AreSame(PositionControlSwitchingThreshold_actual.value(), PositionControlSwitchingThreshold_file.value())) {
    PositionControlSwitchingThreshold_Parameter.setParameter(PositionControlSwitchingThreshold_file);
    joint->setConfigurationParameter(PositionControlSwitchingThreshold_Parameter);
    std::cout << "PositionControlSwitchingThreshold set to: " << PositionControlSwitchingThreshold_file << std::endl;
  }

  if (!AreSame(SpeedControlSwitchingThreshold_actual.value(), SpeedControlSwitchingThreshold_file.value())) {
    SpeedControlSwitchingThreshold_Parameter.setParameter(SpeedControlSwitchingThreshold_file);
    joint->setConfigurationParameter(SpeedControlSwitchingThreshold_Parameter);
    std::cout << "SpeedControlSwitchingThreshold set to: " << SpeedControlSwitchingThreshold_file << std::endl;
  }

  if (!AreSame(CurrentControlSwitchingThreshold_actual.value(), CurrentControlSwitchingThreshold_file.value())) {
    CurrentControlSwitchingThreshold_Parameter.setParameter(CurrentControlSwitchingThreshold_file);
    joint->setConfigurationParameter(CurrentControlSwitchingThreshold_Parameter);
    std::cout << "CurrentControlSwitchingThreshold set to: " << CurrentControlSwitchingThreshold_file << std::endl;
  }

  if (!AreSame(RampGeneratorSpeedAndPositionControl_actual, RampGeneratorSpeedAndPositionControl_file)) {
    RampGeneratorSpeedAndPositionControl_Parameter.setParameter(RampGeneratorSpeedAndPositionControl_file);
    joint->setConfigurationParameter(RampGeneratorSpeedAndPositionControl_Parameter);
    std::cout << "RampGeneratorSpeedAndPositionControl set to: " << RampGeneratorSpeedAndPositionControl_file << std::endl;
  }

  if (!AreSame(PParameterFirstParametersPositionControl_actual, PParameterFirstParametersPositionControl_file)) {
    PParameterFirstParametersPositionControl_Parameter.setParameter(PParameterFirstParametersPositionControl_file);
    joint->setConfigurationParameter(PParameterFirstParametersPositionControl_Parameter);
    std::cout << "PParameterFirstParametersPositionControl set to: " << PParameterFirstParametersPositionControl_file << std::endl;
  }

  if (!AreSame(IParameterFirstParametersPositionControl_actual, IParameterFirstParametersPositionControl_file)) {
    IParameterFirstParametersPositionControl_Parameter.setParameter(IParameterFirstParametersPositionControl_file);
    joint->setConfigurationParameter(IParameterFirstParametersPositionControl_Parameter);
    std::cout << "IParameterFirstParametersPositionControl set to: " << IParameterFirstParametersPositionControl_file << std::endl;
  }

  if (!AreSame(DParameterFirstParametersPositionControl_actual, DParameterFirstParametersPositionControl_file)) {
    DParameterFirstParametersPositionControl_Parameter.setParameter(DParameterFirstParametersPositionControl_file);
    joint->setConfigurationParameter(DParameterFirstParametersPositionControl_Parameter);
    std::cout << "DParameterFirstParametersPositionControl set to: " << DParameterFirstParametersPositionControl_file << std::endl;
  }

  if (!AreSame(IClippingParameterFirstParametersPositionControl_actual, IClippingParameterFirstParametersPositionControl_file)) {
    IClippingParameterFirstParametersPositionControl_Parameter.setParameter(IClippingParameterFirstParametersPositionControl_file);
    joint->setConfigurationParameter(IClippingParameterFirstParametersPositionControl_Parameter);
    std::cout << "IClippingParameterFirstParametersPositionControl set to: " << IClippingParameterFirstParametersPositionControl_file << std::endl;
  }

  if (!AreSame(PParameterFirstParametersSpeedControl_actual, PParameterFirstParametersSpeedControl_file)) {
    PParameterFirstParametersSpeedControl_Parameter.setParameter(PParameterFirstParametersSpeedControl_file);
    joint->setConfigurationParameter(PParameterFirstParametersSpeedControl_Parameter);
    std::cout << "PParameterFirstParametersSpeedControl set to: " << PParameterFirstParametersSpeedControl_file << std::endl;
  }

  if (!AreSame(IParameterFirstParametersSpeedControl_actual, IParameterFirstParametersSpeedControl_file)) {
    IParameterFirstParametersSpeedControl_Parameter.setParameter(IParameterFirstParametersSpeedControl_file);
    joint->setConfigurationParameter(IParameterFirstParametersSpeedControl_Parameter);
    std::cout << "IParameterFirstParametersSpeedControl set to: " << IParameterFirstParametersSpeedControl_file << std::endl;
  }

  if (!AreSame(DParameterFirstParametersSpeedControl_actual, DParameterFirstParametersSpeedControl_file)) {
    DParameterFirstParametersSpeedControl_Parameter.setParameter(DParameterFirstParametersSpeedControl_file);
    joint->setConfigurationParameter(DParameterFirstParametersSpeedControl_Parameter);
    std::cout << "DParameterFirstParametersSpeedControl set to: " << DParameterFirstParametersSpeedControl_file << std::endl;
  }

  if (!AreSame(IClippingParameterFirstParametersSpeedControl_actual, IClippingParameterFirstParametersSpeedControl_file)) {
    IClippingParameterFirstParametersSpeedControl_Parameter.setParameter(IClippingParameterFirstParametersSpeedControl_file);
    joint->setConfigurationParameter(IClippingParameterFirstParametersSpeedControl_Parameter);
    std::cout << "IClippingParameterFirstParametersSpeedControl set to: " << IClippingParameterFirstParametersSpeedControl_file << std::endl;
  }


  if (!AreSame(PParameterFirstParametersCurrentControl_actual, PParameterFirstParametersCurrentControl_file)) {
    PParameterFirstParametersCurrentControl_Parameter.setParameter(PParameterFirstParametersCurrentControl_file);
    joint->setConfigurationParameter(PParameterFirstParametersCurrentControl_Parameter);
    std::cout << "PParameterFirstParametersCurrentControl set to: " << PParameterFirstParametersCurrentControl_file << std::endl;
  }

  if (!AreSame(IParameterFirstParametersCurrentControl_actual, IParameterFirstParametersCurrentControl_file)) {
    IParameterFirstParametersCurrentControl_Parameter.setParameter(IParameterFirstParametersCurrentControl_file);
    joint->setConfigurationParameter(IParameterFirstParametersCurrentControl_Parameter);
    std::cout << "IParameterFirstParametersCurrentControl set to: " << IParameterFirstParametersCurrentControl_file << std::endl;
  }


  if (!AreSame(DParameterFirstParametersCurrentControl_actual, DParameterFirstParametersCurrentControl_file)) {
    DParameterFirstParametersCurrentControl_Parameter.setParameter(DParameterFirstParametersCurrentControl_file);
    joint->setConfigurationParameter(DParameterFirstParametersCurrentControl_Parameter);
    std::cout << "DParameterFirstParametersCurrentControl set to: " << DParameterFirstParametersCurrentControl_file << std::endl;
  }

  if (!AreSame(IClippingParameterFirstParametersCurrentControl_actual, IClippingParameterFirstParametersCurrentControl_file)) {
    IClippingParameterFirstParametersCurrentControl_Parameter.setParameter(IClippingParameterFirstParametersCurrentControl_file);
    joint->setConfigurationParameter(IClippingParameterFirstParametersCurrentControl_Parameter);
    std::cout << "IClippingParameterFirstParametersCurrentControl set to: " << IClippingParameterFirstParametersCurrentControl_file << std::endl;
  }

  if (!AreSame(PParameterSecondParametersPositionControl_actual, PParameterSecondParametersPositionControl_file)) {
    PParameterSecondParametersPositionControl_Parameter.setParameter(PParameterSecondParametersPositionControl_file);
    joint->setConfigurationParameter(PParameterSecondParametersPositionControl_Parameter);
    std::cout << "PParameterSecondParametersPositionControl set to: " << PParameterSecondParametersPositionControl_file << std::endl;
  }


  if (!AreSame(IParameterSecondParametersPositionControl_actual, IParameterSecondParametersPositionControl_file)) {
    IParameterSecondParametersPositionControl_Parameter.setParameter(IParameterSecondParametersPositionControl_file);
    joint->setConfigurationParameter(IParameterSecondParametersPositionControl_Parameter);
    std::cout << "IParameterSecondParametersPositionControl set to: " << IParameterSecondParametersPositionControl_file << std::endl;
  }

  if (!AreSame(DParameterSecondParametersPositionControl_actual, DParameterSecondParametersPositionControl_file)) {
    DParameterSecondParametersPositionControl_Parameter.setParameter(DParameterSecondParametersPositionControl_file);
    joint->setConfigurationParameter(DParameterSecondParametersPositionControl_Parameter);
    std::cout << "DParameterSecondParametersPositionControl set to: " << DParameterSecondParametersPositionControl_file << std::endl;
  }

  if (!AreSame(IClippingParameterSecondParametersPositionControl_actual, IClippingParameterSecondParametersPositionControl_file)) {
    IClippingParameterSecondParametersPositionControl_Parameter.setParameter(IClippingParameterSecondParametersPositionControl_file);
    joint->setConfigurationParameter(IClippingParameterSecondParametersPositionControl_Parameter);
    std::cout << "IClippingParameterSecondParametersPositionControl set to: " << IClippingParameterSecondParametersPositionControl_file << std::endl;
  }

  if (!AreSame(PParameterSecondParametersSpeedControl_actual, PParameterSecondParametersSpeedControl_file)) {
    PParameterSecondParametersSpeedControl_Parameter.setParameter(PParameterSecondParametersSpeedControl_file);
    joint->setConfigurationParameter(PParameterSecondParametersSpeedControl_Parameter);
    std::cout << "PParameterSecondParametersSpeedControl set to: " << PParameterSecondParametersSpeedControl_file << std::endl;
  }

  if (!AreSame(IParameterSecondParametersSpeedControl_actual, IParameterSecondParametersSpeedControl_file)) {
    IParameterSecondParametersSpeedControl_Parameter.setParameter(IParameterSecondParametersSpeedControl_file);
    joint->setConfigurationParameter(IParameterSecondParametersSpeedControl_Parameter);
    std::cout << "IParameterSecondParametersSpeedControl set to: " << IParameterSecondParametersSpeedControl_file << std::endl;
  }

  if (!AreSame(DParameterSecondParametersSpeedControl_actual, DParameterSecondParametersSpeedControl_file)) {
    DParameterSecondParametersSpeedControl_Parameter.setParameter(DParameterSecondParametersSpeedControl_file);
    joint->setConfigurationParameter(DParameterSecondParametersSpeedControl_Parameter);
    std::cout << "DParameterSecondParametersSpeedControl set to: " << DParameterSecondParametersSpeedControl_file << std::endl;
  }

  if (!AreSame(IClippingParameterSecondParametersSpeedControl_actual, IClippingParameterSecondParametersSpeedControl_file)) {
    IClippingParameterSecondParametersSpeedControl_Parameter.setParameter(IClippingParameterSecondParametersSpeedControl_file);
    joint->setConfigurationParameter(IClippingParameterSecondParametersSpeedControl_Parameter);
    std::cout << "IClippingParameterSecondParametersSpeedControl set to: " << IClippingParameterSecondParametersSpeedControl_file << std::endl;
  }

  if (!AreSame(PParameterSecondParametersCurrentControl_actual, PParameterSecondParametersCurrentControl_file)) {
    PParameterSecondParametersCurrentControl_Parameter.setParameter(PParameterSecondParametersCurrentControl_file);
    joint->setConfigurationParameter(PParameterSecondParametersCurrentControl_Parameter);
    std::cout << "PParameterSecondParametersCurrentControl set to: " << PParameterSecondParametersCurrentControl_file << std::endl;
  }

  if (!AreSame(IParameterSecondParametersCurrentControl_actual, IParameterSecondParametersCurrentControl_file)) {
    IParameterSecondParametersCurrentControl_Parameter.setParameter(IParameterSecondParametersCurrentControl_file);
    joint->setConfigurationParameter(IParameterSecondParametersCurrentControl_Parameter);
    std::cout << "IParameterSecondParametersCurrentControl set to: " << IParameterSecondParametersCurrentControl_file << std::endl;
  }

  if (!AreSame(DParameterSecondParametersCurrentControl_actual, DParameterSecondParametersCurrentControl_file)) {
    DParameterSecondParametersCurrentControl_Parameter.setParameter(DParameterSecondParametersCurrentControl_file);
    joint->setConfigurationParameter(DParameterSecondParametersCurrentControl_Parameter);
    std::cout << "DParameterSecondParametersCurrentControl set to: " << DParameterSecondParametersCurrentControl_file << std::endl;
  }

  if (!AreSame(IClippingParameterSecondParametersCurrentControl_actual, IClippingParameterSecondParametersCurrentControl_file)) {
    IClippingParameterSecondParametersCurrentControl_Parameter.setParameter(IClippingParameterSecondParametersCurrentControl_file);
    joint->setConfigurationParameter(IClippingParameterSecondParametersCurrentControl_Parameter);
    std::cout << "IClippingParameterSecondParametersCurrentControl set to: " << IClippingParameterSecondParametersCurrentControl_file << std::endl;
  }

  if (!AreSame(MaximumVelocityToSetPosition_actual.value(), MaximumVelocityToSetPosition_file.value())) {
    MaximumVelocityToSetPosition_Parameter.setParameter(MaximumVelocityToSetPosition_file);
    joint->setConfigurationParameter(MaximumVelocityToSetPosition_Parameter);
    std::cout << "MaximumVelocityToSetPosition set to: " << MaximumVelocityToSetPosition_file << std::endl;
  }

  if (!AreSame(PositionTargetReachedDistance_actual, PositionTargetReachedDistance_file)) {
    PositionTargetReachedDistance_Parameter.setParameter(PositionTargetReachedDistance_file);
    joint->setConfigurationParameter(PositionTargetReachedDistance_Parameter);
    std::cout << "PositionTargetReachedDistance set to: " << PositionTargetReachedDistance_file << std::endl;
  }

  if (!AreSame(VelocityThresholdForHallFX_actual.value(), VelocityThresholdForHallFX_file.value())) {
    VelocityThresholdForHallFX_Parameter.setParameter(VelocityThresholdForHallFX_file);
    joint->setConfigurationParameter(VelocityThresholdForHallFX_Parameter);
    std::cout << "VelocityThresholdForHallFX set to: " << VelocityThresholdForHallFX_file << std::endl;
  }

  std::cout << "Parameters set for Joint: " << jointName << std::endl << std::endl;

}

void JointConfigurator::setProtectedParametersToJoint() {
  if (!ProtectedParameterRead) {
    std::cout << "The protected Joint Parameters have to been read before hand!" << std::endl;
    return;
  }
  try {

    std::cout << "Setting parameters for Joint: " << jointName << std::endl;

    if (!AreSame(PWMLimit_actual, PWMLimit_file)) {
      PWMLimit_Parameter.setParameter(PWMLimit_file);
      joint->setConfigurationParameter(PWMLimit_Parameter);
      std::cout << "PWMLimit set to: " << PWMLimit_file << std::endl;
    }

    if (!AreSame(MaximumMotorCurrent_actual.value(), MaximumMotorCurrent_file.value())) {
      MaximumMotorCurrent_Parameter.setParameter(MaximumMotorCurrent_file);
      joint->setConfigurationParameter(MaximumMotorCurrent_Parameter);
      std::cout << "MaximumMotorCurrent set to: " << MaximumMotorCurrent_file << std::endl;
    }

    if (!AreSame(PIDControlTime_actual.value(), PIDControlTime_file.value())) {
      PIDControlTime_Parameter.setParameter(PIDControlTime_file);
      joint->setConfigurationParameter(PIDControlTime_Parameter);
      std::cout << "PIDControlTime set to: " << PIDControlTime_file << std::endl;
    }

    if (!AreSame(CurrentControlLoopDelay_actual.value(), CurrentControlLoopDelay_file.value())) {
      CurrentControlLoopDelay_Parameter.setParameter(CurrentControlLoopDelay_file);
      joint->setConfigurationParameter(CurrentControlLoopDelay_Parameter);
      std::cout << "CurrentControlLoopDelay set to: " << CurrentControlLoopDelay_file << std::endl;
    }

    if (!AreSame(PWMHysteresis_actual, PWMHysteresis_file)) {
      PWMHysteresis_Parameter.setParameter(PWMHysteresis_file);
      joint->setConfigurationParameter(PWMHysteresis_Parameter);
      std::cout << "PWMHysteresis set to: " << PWMHysteresis_file << std::endl;
    }

    if (!AreSame(SetEncoderCounterZeroAtNextNChannel_actual, SetEncoderCounterZeroAtNextNChannel_file)) {
      SetEncoderCounterZeroAtNextNChannel_Parameter.setParameter(SetEncoderCounterZeroAtNextNChannel_file);
      joint->setConfigurationParameter(SetEncoderCounterZeroAtNextNChannel_Parameter);
      std::cout << "SetEncoderCounterZeroAtNextNChannel set to: " << SetEncoderCounterZeroAtNextNChannel_file << std::endl;
    }

    if (!AreSame(SetEncoderCounterZeroAtNextSwitch_actual, SetEncoderCounterZeroAtNextSwitch_file)) {
      SetEncoderCounterZeroAtNextSwitch_Parameter.setParameter(SetEncoderCounterZeroAtNextSwitch_file);
      joint->setConfigurationParameter(SetEncoderCounterZeroAtNextSwitch_Parameter);
      std::cout << "SetEncoderCounterZeroAtNextSwitch set to: " << SetEncoderCounterZeroAtNextSwitch_file << std::endl;
    }

    if (!AreSame(SetEncoderCounterZeroOnlyOnce_actual, SetEncoderCounterZeroOnlyOnce_file)) {
      SetEncoderCounterZeroOnlyOnce_Parameter.setParameter(SetEncoderCounterZeroOnlyOnce_file);
      joint->setConfigurationParameter(SetEncoderCounterZeroOnlyOnce_Parameter);
      std::cout << "SetEncoderCounterZeroOnlyOnce set to: " << SetEncoderCounterZeroOnlyOnce_file << std::endl;
    }

    if (!AreSame(EncoderStopSwitch_actual, EncoderStopSwitch_file)) {
      EncoderStopSwitch_Parameter.setParameter(EncoderStopSwitch_file);
      joint->setConfigurationParameter(EncoderStopSwitch_Parameter);
      std::cout << "EncoderStopSwitch set to: " << EncoderStopSwitch_file << std::endl;
    }

    if (!AreSame(StopSwitchPolarity_actual, StopSwitchPolarity_file)) {
      StopSwitchPolarity_Parameter.setParameter(StopSwitchPolarity_file);
      joint->setConfigurationParameter(StopSwitchPolarity_Parameter);
      std::cout << "StopSwitchPolarity set to: " << StopSwitchPolarity_file << std::endl;
    }

    if (!AreSame(CommutationMotorCurrent_actual.value(), CommutationMotorCurrent_file.value())) {
      CommutationMotorCurrent_Parameter.setParameter(CommutationMotorCurrent_file);
      joint->setConfigurationParameter(CommutationMotorCurrent_Parameter);
      std::cout << "CommutationMotorCurrent set to: " << CommutationMotorCurrent_file << std::endl;
    }

    if (!AreSame(MassInertiaConstant_actual, MassInertiaConstant_file)) {
      MassInertiaConstant_Parameter.setParameter(MassInertiaConstant_file);
      joint->setConfigurationParameter(MassInertiaConstant_Parameter);
      std::cout << "MassInertiaConstant set to: " << MassInertiaConstant_file << std::endl;
    }

    if (!AreSame(BEMFConstant_actual, BEMFConstant_file)) {
      BEMFConstant_Parameter.setParameter(BEMFConstant_file);
      joint->setConfigurationParameter(BEMFConstant_Parameter);
      std::cout << "BEMFConstant set to: " << BEMFConstant_file << std::endl;
    }

    if (!AreSame(SineInitializationVelocity_actual, SineInitializationVelocity_file)) {
      SineInitializationVelocity_Parameter.setParameter(SineInitializationVelocity_file);
      joint->setConfigurationParameter(SineInitializationVelocity_Parameter);
      std::cout << "SineInitializationVelocity set to: " << SineInitializationVelocity_file << std::endl;
    }

    if (!AreSame(CommutationCompensationClockwise_actual, CommutationCompensationClockwise_file)) {
      CommutationCompensationClockwise_Parameter.setParameter(CommutationCompensationClockwise_file);
      joint->setConfigurationParameter(CommutationCompensationClockwise_Parameter);
      std::cout << "CommutationCompensationClockwise set to: " << CommutationCompensationClockwise_file << std::endl;
    }

    if (!AreSame(CommutationCompensationCounterClockwise_actual, CommutationCompensationCounterClockwise_file)) {
      CommutationCompensationCounterClockwise_Parameter.setParameter(CommutationCompensationCounterClockwise_file);
      joint->setConfigurationParameter(CommutationCompensationCounterClockwise_Parameter);
      std::cout << "CommutationCompensationCounterClockwise set to: " << CommutationCompensationCounterClockwise_file << std::endl;
    }

    if (!AreSame(InitSineDelay_actual.value(), InitSineDelay_file.value())) {
      InitSineDelay_Parameter.setParameter(InitSineDelay_file);
      joint->setConfigurationParameter(InitSineDelay_Parameter);
      std::cout << "InitSineDelay set to: " << InitSineDelay_file << std::endl;
    }

    if (!AreSame(ActivateOvervoltageProtection_actual, ActivateOvervoltageProtection_file)) {
      ActivateOvervoltageProtection_Parameter.setParameter(ActivateOvervoltageProtection_file);
      joint->setConfigurationParameter(ActivateOvervoltageProtection_Parameter);
      std::cout << "ActivateOvervoltageProtection set to: " << ActivateOvervoltageProtection_file << std::endl;
    }

    if (!AreSame(SineCompensationFactor_actual, SineCompensationFactor_file)) {
      SineCompensationFactor_Parameter.setParameter(SineCompensationFactor_file);
      joint->setConfigurationParameter(SineCompensationFactor_Parameter);
      std::cout << "SineCompensationFactor set to: " << SineCompensationFactor_file << std::endl;
    }

    if (!AreSame(CommutationMode_actual, CommutationMode_file)) {
      CommutationMode_Parameter.setParameter(CommutationMode_file);
      joint->setConfigurationParameter(CommutationMode_Parameter);
      std::cout << "CommutationMode set to: " << CommutationMode_file << std::endl;
    }

    if (!AreSame(EncoderResolution_actual, EncoderResolution_file)) {
      EncoderResolution_Parameter.setParameter(EncoderResolution_file);
      joint->setConfigurationParameter(EncoderResolution_Parameter);
      std::cout << "EncoderResolution set to: " << EncoderResolution_file << std::endl;
    }

    if (!AreSame(HallSensorPolarityReversal_actual, HallSensorPolarityReversal_file)) {
      HallSensorPolarityReversal_Parameter.setParameter(HallSensorPolarityReversal_file);
      joint->setConfigurationParameter(HallSensorPolarityReversal_Parameter);
      std::cout << "HallSensorPolarityReversal set to: " << HallSensorPolarityReversal_file << std::endl;
    }

    if (!AreSame(InitializationMode_actual, InitializationMode_file)) {
      InitializationMode_Parameter.setParameter(InitializationMode_file);
      joint->setConfigurationParameter(InitializationMode_Parameter);
      std::cout << "InitializationMode set to: " << InitializationMode_file << std::endl;
    }

    if (!AreSame(MotorCoilResistance_actual.value(), MotorCoilResistance_file.value())) {
      MotorCoilResistance_Parameter.setParameter(MotorCoilResistance_file);
      joint->setConfigurationParameter(MotorCoilResistance_Parameter);
      std::cout << "MotorCoilResistance set to: " << MotorCoilResistance_file << std::endl;
    }

    if (!AreSame(MotorPoles_actual, MotorPoles_file)) {
      MotorPoles_Parameter.setParameter(MotorPoles_file);
      joint->setConfigurationParameter(MotorPoles_Parameter);
      std::cout << "MotorPoles set to: " << MotorPoles_file << std::endl;
    }

    if (!AreSame(PWMSchemeBlockCommutation_actual, PWMSchemeBlockCommutation_file)) {
      PWMSchemeBlockCommutation_Parameter.setParameter(PWMSchemeBlockCommutation_file);
      joint->setConfigurationParameter(PWMSchemeBlockCommutation_Parameter);
      std::cout << "PWMSchemeBlockCommutation set to: " << PWMSchemeBlockCommutation_file << std::endl;
    }

    if (!AreSame(ReversingEncoderDirection_actual, ReversingEncoderDirection_file)) {
      ReversingEncoderDirection_Parameter.setParameter(ReversingEncoderDirection_file);
      joint->setConfigurationParameter(ReversingEncoderDirection_Parameter);
      std::cout << "ReversingEncoderDirection set to: " << ReversingEncoderDirection_file << std::endl;
    }

    if (!AreSame(ThermalWindingTimeConstant_actual.value(), ThermalWindingTimeConstant_file.value())) {
      ThermalWindingTimeConstant_Parameter.setParameter(ThermalWindingTimeConstant_file);
      joint->setConfigurationParameter(ThermalWindingTimeConstant_Parameter);
      std::cout << "ThermalWindingTimeConstant set to: " << ThermalWindingTimeConstant_file << std::endl;
    }

    if (!AreSame(I2tLimit_actual, I2tLimit_file)) {
      I2tLimit_Parameter.setParameter(I2tLimit_file);
      joint->setConfigurationParameter(I2tLimit_Parameter);
      std::cout << "I2tLimit set to: " << I2tLimit_file << std::endl;
    }

    if (!AreSame(MotorControllerTimeout_actual.value(), MotorControllerTimeout_file.value())) {
      MotorControllerTimeout_Parameter.setParameter(MotorControllerTimeout_file);
      joint->setConfigurationParameter(MotorControllerTimeout_Parameter);
      std::cout << "MotorControllerTimeout set to: " << MotorControllerTimeout_file << std::endl;
    }

    if (!AreSame(MotorHaltedVelocity_actual, MotorHaltedVelocity_file)) {
      MotorHaltedVelocity_Parameter.setParameter(MotorHaltedVelocity_file);
      joint->setConfigurationParameter(MotorHaltedVelocity_Parameter);
      std::cout << "MotorHaltedVelocity set to: " << MotorHaltedVelocity_file << std::endl;
    }


    std::cout << "Protected Parameters set for Joint: " << jointName << std::endl << std::endl;

  } catch (JointParameterException& e) {

  }
}

void JointConfigurator::storeParametersToJoint() {

  if (!ParameterRead) {
    std::cout << "The Joint Parameters have to been read before hand!" << std::endl;
    return;
  }

  std::cout << "Storing parameters for Joint: " << jointName << std::endl;

  if (!AreSame(MaximumPositioningVelocity_actual.value(), MaximumPositioningVelocity_file.value())) {
    MaximumPositioningVelocity_Parameter.setParameter(MaximumPositioningVelocity_file);
    joint->storeConfigurationParameterPermanent(MaximumPositioningVelocity_Parameter);
    std::cout << "MaximumPositioningVelocity stored0 to: " << MaximumPositioningVelocity_file << std::endl;
  }

  if (!AreSame(MotorAcceleration_actual.value(), MotorAcceleration_file.value())) {
    MotorAcceleration_Parameter.setParameter(MotorAcceleration_file);
    joint->storeConfigurationParameterPermanent(MotorAcceleration_Parameter);
    std::cout << "MotorAcceleration stored with: " << MotorAcceleration_file << std::endl;
  }

  if (!AreSame(PositionControlSwitchingThreshold_actual.value(), PositionControlSwitchingThreshold_file.value())) {
    PositionControlSwitchingThreshold_Parameter.setParameter(PositionControlSwitchingThreshold_file);
    joint->storeConfigurationParameterPermanent(PositionControlSwitchingThreshold_Parameter);
    std::cout << "PositionControlSwitchingThreshold stored with: " << PositionControlSwitchingThreshold_file << std::endl;
  }

  if (!AreSame(SpeedControlSwitchingThreshold_actual.value(), SpeedControlSwitchingThreshold_file.value())) {
    SpeedControlSwitchingThreshold_Parameter.setParameter(SpeedControlSwitchingThreshold_file);
    joint->storeConfigurationParameterPermanent(SpeedControlSwitchingThreshold_Parameter);
    std::cout << "SpeedControlSwitchingThreshold stored with: " << SpeedControlSwitchingThreshold_file << std::endl;
  }

  if (!AreSame(CurrentControlSwitchingThreshold_actual.value(), CurrentControlSwitchingThreshold_file.value())) {
    CurrentControlSwitchingThreshold_Parameter.setParameter(CurrentControlSwitchingThreshold_file);
    joint->storeConfigurationParameterPermanent(CurrentControlSwitchingThreshold_Parameter);
    std::cout << "CurrentControlSwitchingThreshold stored with: " << CurrentControlSwitchingThreshold_file << std::endl;
  }

  if (!AreSame(RampGeneratorSpeedAndPositionControl_actual, RampGeneratorSpeedAndPositionControl_file)) {
    RampGeneratorSpeedAndPositionControl_Parameter.setParameter(RampGeneratorSpeedAndPositionControl_file);
    joint->storeConfigurationParameterPermanent(RampGeneratorSpeedAndPositionControl_Parameter);
    std::cout << "RampGeneratorSpeedAndPositionControl stored with: " << RampGeneratorSpeedAndPositionControl_file << std::endl;
  }

  if (!AreSame(PParameterFirstParametersPositionControl_actual, PParameterFirstParametersPositionControl_file)) {
    PParameterFirstParametersPositionControl_Parameter.setParameter(PParameterFirstParametersPositionControl_file);
    joint->storeConfigurationParameterPermanent(PParameterFirstParametersPositionControl_Parameter);
    std::cout << "PParameterFirstParametersPositionControl stored with: " << PParameterFirstParametersPositionControl_file << std::endl;
  }

  if (!AreSame(IParameterFirstParametersPositionControl_actual, IParameterFirstParametersPositionControl_file)) {
    IParameterFirstParametersPositionControl_Parameter.setParameter(IParameterFirstParametersPositionControl_file);
    joint->storeConfigurationParameterPermanent(IParameterFirstParametersPositionControl_Parameter);
    std::cout << "IParameterFirstParametersPositionControl stored with: " << IParameterFirstParametersPositionControl_file << std::endl;
  }

  if (!AreSame(DParameterFirstParametersPositionControl_actual, DParameterFirstParametersPositionControl_file)) {
    DParameterFirstParametersPositionControl_Parameter.setParameter(DParameterFirstParametersPositionControl_file);
    joint->storeConfigurationParameterPermanent(DParameterFirstParametersPositionControl_Parameter);
    std::cout << "DParameterFirstParametersPositionControl stored with: " << DParameterFirstParametersPositionControl_file << std::endl;
  }

  if (!AreSame(IClippingParameterFirstParametersPositionControl_actual, IClippingParameterFirstParametersPositionControl_file)) {
    IClippingParameterFirstParametersPositionControl_Parameter.setParameter(IClippingParameterFirstParametersPositionControl_file);
    joint->storeConfigurationParameterPermanent(IClippingParameterFirstParametersPositionControl_Parameter);
    std::cout << "IClippingParameterFirstParametersPositionControl stored with: " << IClippingParameterFirstParametersPositionControl_file << std::endl;
  }

  if (!AreSame(PParameterFirstParametersSpeedControl_actual, PParameterFirstParametersSpeedControl_file)) {
    PParameterFirstParametersSpeedControl_Parameter.setParameter(PParameterFirstParametersSpeedControl_file);
    joint->storeConfigurationParameterPermanent(PParameterFirstParametersSpeedControl_Parameter);
    std::cout << "PParameterFirstParametersSpeedControl stored with: " << PParameterFirstParametersSpeedControl_file << std::endl;
  }

  if (!AreSame(IParameterFirstParametersSpeedControl_actual, IParameterFirstParametersSpeedControl_file)) {
    IParameterFirstParametersSpeedControl_Parameter.setParameter(IParameterFirstParametersSpeedControl_file);
    joint->storeConfigurationParameterPermanent(IParameterFirstParametersSpeedControl_Parameter);
    std::cout << "IParameterFirstParametersSpeedControl stored with: " << IParameterFirstParametersSpeedControl_file << std::endl;
  }

  if (!AreSame(DParameterFirstParametersSpeedControl_actual, DParameterFirstParametersSpeedControl_file)) {
    DParameterFirstParametersSpeedControl_Parameter.setParameter(DParameterFirstParametersSpeedControl_file);
    joint->storeConfigurationParameterPermanent(DParameterFirstParametersSpeedControl_Parameter);
    std::cout << "DParameterFirstParametersSpeedControl stored with: " << DParameterFirstParametersSpeedControl_file << std::endl;
  }

  if (!AreSame(IClippingParameterFirstParametersSpeedControl_actual, IClippingParameterFirstParametersSpeedControl_file)) {
    IClippingParameterFirstParametersSpeedControl_Parameter.setParameter(IClippingParameterFirstParametersSpeedControl_file);
    joint->storeConfigurationParameterPermanent(IClippingParameterFirstParametersSpeedControl_Parameter);
    std::cout << "IClippingParameterFirstParametersSpeedControl stored with: " << IClippingParameterFirstParametersSpeedControl_file << std::endl;
  }


  if (!AreSame(PParameterFirstParametersCurrentControl_actual, PParameterFirstParametersCurrentControl_file)) {
    PParameterFirstParametersCurrentControl_Parameter.setParameter(PParameterFirstParametersCurrentControl_file);
    joint->storeConfigurationParameterPermanent(PParameterFirstParametersCurrentControl_Parameter);
    std::cout << "PParameterFirstParametersCurrentControl stored with: " << PParameterFirstParametersCurrentControl_file << std::endl;
  }

  if (!AreSame(IParameterFirstParametersCurrentControl_actual, IParameterFirstParametersCurrentControl_file)) {
    IParameterFirstParametersCurrentControl_Parameter.setParameter(IParameterFirstParametersCurrentControl_file);
    joint->storeConfigurationParameterPermanent(IParameterFirstParametersCurrentControl_Parameter);
    std::cout << "IParameterFirstParametersCurrentControl stored with: " << IParameterFirstParametersCurrentControl_file << std::endl;
  }


  if (!AreSame(DParameterFirstParametersCurrentControl_actual, DParameterFirstParametersCurrentControl_file)) {
    DParameterFirstParametersCurrentControl_Parameter.setParameter(DParameterFirstParametersCurrentControl_file);
    joint->storeConfigurationParameterPermanent(DParameterFirstParametersCurrentControl_Parameter);
    std::cout << "DParameterFirstParametersCurrentControl stored with: " << DParameterFirstParametersCurrentControl_file << std::endl;
  }

  if (!AreSame(IClippingParameterFirstParametersCurrentControl_actual, IClippingParameterFirstParametersCurrentControl_file)) {
    IClippingParameterFirstParametersCurrentControl_Parameter.setParameter(IClippingParameterFirstParametersCurrentControl_file);
    joint->storeConfigurationParameterPermanent(IClippingParameterFirstParametersCurrentControl_Parameter);
    std::cout << "IClippingParameterFirstParametersCurrentControl stored with: " << IClippingParameterFirstParametersCurrentControl_file << std::endl;
  }

  if (!AreSame(PParameterSecondParametersPositionControl_actual, PParameterSecondParametersPositionControl_file)) {
    PParameterSecondParametersPositionControl_Parameter.setParameter(PParameterSecondParametersPositionControl_file);
    joint->storeConfigurationParameterPermanent(PParameterSecondParametersPositionControl_Parameter);
    std::cout << "PParameterSecondParametersPositionControl stored with: " << PParameterSecondParametersPositionControl_file << std::endl;
  }


  if (!AreSame(IParameterSecondParametersPositionControl_actual, IParameterSecondParametersPositionControl_file)) {
    IParameterSecondParametersPositionControl_Parameter.setParameter(IParameterSecondParametersPositionControl_file);
    joint->storeConfigurationParameterPermanent(IParameterSecondParametersPositionControl_Parameter);
    std::cout << "IParameterSecondParametersPositionControl stored with: " << IParameterSecondParametersPositionControl_file << std::endl;
  }

  if (!AreSame(DParameterSecondParametersPositionControl_actual, DParameterSecondParametersPositionControl_file)) {
    DParameterSecondParametersPositionControl_Parameter.setParameter(DParameterSecondParametersPositionControl_file);
    joint->storeConfigurationParameterPermanent(DParameterSecondParametersPositionControl_Parameter);
    std::cout << "DParameterSecondParametersPositionControl stored with: " << DParameterSecondParametersPositionControl_file << std::endl;
  }

  if (!AreSame(IClippingParameterSecondParametersPositionControl_actual, IClippingParameterSecondParametersPositionControl_file)) {
    IClippingParameterSecondParametersPositionControl_Parameter.setParameter(IClippingParameterSecondParametersPositionControl_file);
    joint->storeConfigurationParameterPermanent(IClippingParameterSecondParametersPositionControl_Parameter);
    std::cout << "IClippingParameterSecondParametersPositionControl stored with: " << IClippingParameterSecondParametersPositionControl_file << std::endl;
  }

  if (!AreSame(PParameterSecondParametersSpeedControl_actual, PParameterSecondParametersSpeedControl_file)) {
    PParameterSecondParametersSpeedControl_Parameter.setParameter(PParameterSecondParametersSpeedControl_file);
    joint->storeConfigurationParameterPermanent(PParameterSecondParametersSpeedControl_Parameter);
    std::cout << "PParameterSecondParametersSpeedControl stored with: " << PParameterSecondParametersSpeedControl_file << std::endl;
  }

  if (!AreSame(IParameterSecondParametersSpeedControl_actual, IParameterSecondParametersSpeedControl_file)) {
    IParameterSecondParametersSpeedControl_Parameter.setParameter(IParameterSecondParametersSpeedControl_file);
    joint->storeConfigurationParameterPermanent(IParameterSecondParametersSpeedControl_Parameter);
    std::cout << "IParameterSecondParametersSpeedControl stored with: " << IParameterSecondParametersSpeedControl_file << std::endl;
  }

  if (!AreSame(DParameterSecondParametersSpeedControl_actual, DParameterSecondParametersSpeedControl_file)) {
    DParameterSecondParametersSpeedControl_Parameter.setParameter(DParameterSecondParametersSpeedControl_file);
    joint->storeConfigurationParameterPermanent(DParameterSecondParametersSpeedControl_Parameter);
    std::cout << "DParameterSecondParametersSpeedControl stored with: " << DParameterSecondParametersSpeedControl_file << std::endl;
  }

  if (!AreSame(IClippingParameterSecondParametersSpeedControl_actual, IClippingParameterSecondParametersSpeedControl_file)) {
    IClippingParameterSecondParametersSpeedControl_Parameter.setParameter(IClippingParameterSecondParametersSpeedControl_file);
    joint->storeConfigurationParameterPermanent(IClippingParameterSecondParametersSpeedControl_Parameter);
    std::cout << "IClippingParameterSecondParametersSpeedControl stored with: " << IClippingParameterSecondParametersSpeedControl_file << std::endl;
  }

  if (!AreSame(PParameterSecondParametersCurrentControl_actual, PParameterSecondParametersCurrentControl_file)) {
    PParameterSecondParametersCurrentControl_Parameter.setParameter(PParameterSecondParametersCurrentControl_file);
    joint->storeConfigurationParameterPermanent(PParameterSecondParametersCurrentControl_Parameter);
    std::cout << "PParameterSecondParametersCurrentControl stored with: " << PParameterSecondParametersCurrentControl_file << std::endl;
  }

  if (!AreSame(IParameterSecondParametersCurrentControl_actual, IParameterSecondParametersCurrentControl_file)) {
    IParameterSecondParametersCurrentControl_Parameter.setParameter(IParameterSecondParametersCurrentControl_file);
    joint->storeConfigurationParameterPermanent(IParameterSecondParametersCurrentControl_Parameter);
    std::cout << "IParameterSecondParametersCurrentControl stored with: " << IParameterSecondParametersCurrentControl_file << std::endl;
  }

  if (!AreSame(DParameterSecondParametersCurrentControl_actual, DParameterSecondParametersCurrentControl_file)) {
    DParameterSecondParametersCurrentControl_Parameter.setParameter(DParameterSecondParametersCurrentControl_file);
    joint->storeConfigurationParameterPermanent(DParameterSecondParametersCurrentControl_Parameter);
    std::cout << "DParameterSecondParametersCurrentControl stored with: " << DParameterSecondParametersCurrentControl_file << std::endl;
  }

  if (!AreSame(IClippingParameterSecondParametersCurrentControl_actual, IClippingParameterSecondParametersCurrentControl_file)) {
    IClippingParameterSecondParametersCurrentControl_Parameter.setParameter(IClippingParameterSecondParametersCurrentControl_file);
    joint->storeConfigurationParameterPermanent(IClippingParameterSecondParametersCurrentControl_Parameter);
    std::cout << "IClippingParameterSecondParametersCurrentControl stored with: " << IClippingParameterSecondParametersCurrentControl_file << std::endl;
  }

  if (!AreSame(MaximumVelocityToSetPosition_actual.value(), MaximumVelocityToSetPosition_file.value())) {
    MaximumVelocityToSetPosition_Parameter.setParameter(MaximumVelocityToSetPosition_file);
    joint->storeConfigurationParameterPermanent(MaximumVelocityToSetPosition_Parameter);
    std::cout << "MaximumVelocityToSetPosition stored with: " << MaximumVelocityToSetPosition_file << std::endl;
  }

  if (!AreSame(PositionTargetReachedDistance_actual, PositionTargetReachedDistance_file)) {
    PositionTargetReachedDistance_Parameter.setParameter(PositionTargetReachedDistance_file);
    joint->storeConfigurationParameterPermanent(PositionTargetReachedDistance_Parameter);
    std::cout << "PositionTargetReachedDistance stored with: " << PositionTargetReachedDistance_file << std::endl;
  }

  if (!AreSame(VelocityThresholdForHallFX_actual.value(), VelocityThresholdForHallFX_file.value())) {
    VelocityThresholdForHallFX_Parameter.setParameter(VelocityThresholdForHallFX_file);
    joint->storeConfigurationParameterPermanent(VelocityThresholdForHallFX_Parameter);
    std::cout << "VelocityThresholdForHallFX stored with: " << VelocityThresholdForHallFX_file << std::endl;
  }

  std::cout << "Parameters stored for Joint: " << jointName << std::endl << std::endl;
}

void JointConfigurator::storeProtectedParametersToJoint() {

  if (!ProtectedParameterRead) {
    std::cout << "The protected Joint Parameters have to been read before hand!" << std::endl;
    return;
  }
  try {

    std::cout << "Storing parameters for Joint: " << jointName << std::endl;

    if (!AreSame(PWMLimit_actual, PWMLimit_file)) {
      PWMLimit_Parameter.setParameter(PWMLimit_file);
      joint->setConfigurationParameter(PWMLimit_Parameter);
      std::cout << "PWMLimit set to: " << PWMLimit_file << std::endl;
    }

    if (!AreSame(MaximumMotorCurrent_actual.value(), MaximumMotorCurrent_file.value())) {
      MaximumMotorCurrent_Parameter.setParameter(MaximumMotorCurrent_file);
      joint->setConfigurationParameter(MaximumMotorCurrent_Parameter);
      std::cout << "MaximumMotorCurrent set to: " << MaximumMotorCurrent_file << std::endl;
    }

    if (!AreSame(PIDControlTime_actual.value(), PIDControlTime_file.value())) {
      PIDControlTime_Parameter.setParameter(PIDControlTime_file);
      joint->setConfigurationParameter(PIDControlTime_Parameter);
      std::cout << "PIDControlTime set to: " << PIDControlTime_file << std::endl;
    }

    if (!AreSame(CurrentControlLoopDelay_actual.value(), CurrentControlLoopDelay_file.value())) {
      CurrentControlLoopDelay_Parameter.setParameter(CurrentControlLoopDelay_file);
      joint->setConfigurationParameter(CurrentControlLoopDelay_Parameter);
      std::cout << "CurrentControlLoopDelay set to: " << CurrentControlLoopDelay_file << std::endl;
    }

    if (!AreSame(PWMHysteresis_actual, PWMHysteresis_file)) {
      PWMHysteresis_Parameter.setParameter(PWMHysteresis_file);
      joint->setConfigurationParameter(PWMHysteresis_Parameter);
      std::cout << "PWMHysteresis set to: " << PWMHysteresis_file << std::endl;
    }

    if (!AreSame(SetEncoderCounterZeroAtNextNChannel_actual, SetEncoderCounterZeroAtNextNChannel_file)) {
      SetEncoderCounterZeroAtNextNChannel_Parameter.setParameter(SetEncoderCounterZeroAtNextNChannel_file);
      joint->setConfigurationParameter(SetEncoderCounterZeroAtNextNChannel_Parameter);
      std::cout << "SetEncoderCounterZeroAtNextNChannel set to: " << SetEncoderCounterZeroAtNextNChannel_file << std::endl;
    }

    if (!AreSame(SetEncoderCounterZeroAtNextSwitch_actual, SetEncoderCounterZeroAtNextSwitch_file)) {
      SetEncoderCounterZeroAtNextSwitch_Parameter.setParameter(SetEncoderCounterZeroAtNextSwitch_file);
      joint->setConfigurationParameter(SetEncoderCounterZeroAtNextSwitch_Parameter);
      std::cout << "SetEncoderCounterZeroAtNextSwitch set to: " << SetEncoderCounterZeroAtNextSwitch_file << std::endl;
    }

    if (!AreSame(SetEncoderCounterZeroOnlyOnce_actual, SetEncoderCounterZeroOnlyOnce_file)) {
      SetEncoderCounterZeroOnlyOnce_Parameter.setParameter(SetEncoderCounterZeroOnlyOnce_file);
      joint->setConfigurationParameter(SetEncoderCounterZeroOnlyOnce_Parameter);
      std::cout << "SetEncoderCounterZeroOnlyOnce set to: " << SetEncoderCounterZeroOnlyOnce_file << std::endl;
    }

    if (!AreSame(EncoderStopSwitch_actual, EncoderStopSwitch_file)) {
      EncoderStopSwitch_Parameter.setParameter(EncoderStopSwitch_file);
      joint->setConfigurationParameter(EncoderStopSwitch_Parameter);
      std::cout << "EncoderStopSwitch set to: " << EncoderStopSwitch_file << std::endl;
    }

    if (!AreSame(StopSwitchPolarity_actual, StopSwitchPolarity_file)) {
      StopSwitchPolarity_Parameter.setParameter(StopSwitchPolarity_file);
      joint->setConfigurationParameter(StopSwitchPolarity_Parameter);
      std::cout << "StopSwitchPolarity set to: " << StopSwitchPolarity_file << std::endl;
    }

    if (!AreSame(CommutationMotorCurrent_actual.value(), CommutationMotorCurrent_file.value())) {
      CommutationMotorCurrent_Parameter.setParameter(CommutationMotorCurrent_file);
      joint->setConfigurationParameter(CommutationMotorCurrent_Parameter);
      std::cout << "CommutationMotorCurrent set to: " << CommutationMotorCurrent_file << std::endl;
    }

    if (!AreSame(MassInertiaConstant_actual, MassInertiaConstant_file)) {
      MassInertiaConstant_Parameter.setParameter(MassInertiaConstant_file);
      joint->setConfigurationParameter(MassInertiaConstant_Parameter);
      std::cout << "MassInertiaConstant set to: " << MassInertiaConstant_file << std::endl;
    }

    if (!AreSame(BEMFConstant_actual, BEMFConstant_file)) {
      BEMFConstant_Parameter.setParameter(BEMFConstant_file);
      joint->setConfigurationParameter(BEMFConstant_Parameter);
      std::cout << "BEMFConstant set to: " << BEMFConstant_file << std::endl;
    }

    if (!AreSame(SineInitializationVelocity_actual, SineInitializationVelocity_file)) {
      SineInitializationVelocity_Parameter.setParameter(SineInitializationVelocity_file);
      joint->setConfigurationParameter(SineInitializationVelocity_Parameter);
      std::cout << "SineInitializationVelocity set to: " << SineInitializationVelocity_file << std::endl;
    }

    if (!AreSame(CommutationCompensationClockwise_actual, CommutationCompensationClockwise_file)) {
      CommutationCompensationClockwise_Parameter.setParameter(CommutationCompensationClockwise_file);
      joint->setConfigurationParameter(CommutationCompensationClockwise_Parameter);
      std::cout << "CommutationCompensationClockwise set to: " << CommutationCompensationClockwise_file << std::endl;
    }

    if (!AreSame(CommutationCompensationCounterClockwise_actual, CommutationCompensationCounterClockwise_file)) {
      CommutationCompensationCounterClockwise_Parameter.setParameter(CommutationCompensationCounterClockwise_file);
      joint->setConfigurationParameter(CommutationCompensationCounterClockwise_Parameter);
      std::cout << "CommutationCompensationCounterClockwise set to: " << CommutationCompensationCounterClockwise_file << std::endl;
    }

    if (!AreSame(InitSineDelay_actual.value(), InitSineDelay_file.value())) {
      InitSineDelay_Parameter.setParameter(InitSineDelay_file);
      joint->setConfigurationParameter(InitSineDelay_Parameter);
      std::cout << "InitSineDelay set to: " << InitSineDelay_file << std::endl;
    }

    if (!AreSame(ActivateOvervoltageProtection_actual, ActivateOvervoltageProtection_file)) {
      ActivateOvervoltageProtection_Parameter.setParameter(ActivateOvervoltageProtection_file);
      joint->setConfigurationParameter(ActivateOvervoltageProtection_Parameter);
      std::cout << "ActivateOvervoltageProtection set to: " << ActivateOvervoltageProtection_file << std::endl;
    }

    if (!AreSame(SineCompensationFactor_actual, SineCompensationFactor_file)) {
      SineCompensationFactor_Parameter.setParameter(SineCompensationFactor_file);
      joint->setConfigurationParameter(SineCompensationFactor_Parameter);
      std::cout << "SineCompensationFactor set to: " << SineCompensationFactor_file << std::endl;
    }

    if (!AreSame(CommutationMode_actual, CommutationMode_file)) {
      CommutationMode_Parameter.setParameter(CommutationMode_file);
      joint->setConfigurationParameter(CommutationMode_Parameter);
      std::cout << "CommutationMode set to: " << CommutationMode_file << std::endl;
    }

    if (!AreSame(EncoderResolution_actual, EncoderResolution_file)) {
      EncoderResolution_Parameter.setParameter(EncoderResolution_file);
      joint->setConfigurationParameter(EncoderResolution_Parameter);
      std::cout << "EncoderResolution set to: " << EncoderResolution_file << std::endl;
    }

    if (!AreSame(HallSensorPolarityReversal_actual, HallSensorPolarityReversal_file)) {
      HallSensorPolarityReversal_Parameter.setParameter(HallSensorPolarityReversal_file);
      joint->setConfigurationParameter(HallSensorPolarityReversal_Parameter);
      std::cout << "HallSensorPolarityReversal set to: " << HallSensorPolarityReversal_file << std::endl;
    }

    if (!AreSame(InitializationMode_actual, InitializationMode_file)) {
      InitializationMode_Parameter.setParameter(InitializationMode_file);
      joint->setConfigurationParameter(InitializationMode_Parameter);
      std::cout << "InitializationMode set to: " << InitializationMode_file << std::endl;
    }

    if (!AreSame(MotorCoilResistance_actual.value(), MotorCoilResistance_file.value())) {
      MotorCoilResistance_Parameter.setParameter(MotorCoilResistance_file);
      joint->setConfigurationParameter(MotorCoilResistance_Parameter);
      std::cout << "MotorCoilResistance set to: " << MotorCoilResistance_file << std::endl;
    }

    if (!AreSame(MotorPoles_actual, MotorPoles_file)) {
      MotorPoles_Parameter.setParameter(MotorPoles_file);
      joint->setConfigurationParameter(MotorPoles_Parameter);
      std::cout << "MotorPoles set to: " << MotorPoles_file << std::endl;
    }

    if (!AreSame(PWMSchemeBlockCommutation_actual, PWMSchemeBlockCommutation_file)) {
      PWMSchemeBlockCommutation_Parameter.setParameter(PWMSchemeBlockCommutation_file);
      joint->setConfigurationParameter(PWMSchemeBlockCommutation_Parameter);
      std::cout << "PWMSchemeBlockCommutation set to: " << PWMSchemeBlockCommutation_file << std::endl;
    }

    if (!AreSame(ReversingEncoderDirection_actual, ReversingEncoderDirection_file)) {
      ReversingEncoderDirection_Parameter.setParameter(ReversingEncoderDirection_file);
      joint->setConfigurationParameter(ReversingEncoderDirection_Parameter);
      std::cout << "ReversingEncoderDirection set to: " << ReversingEncoderDirection_file << std::endl;
    }

    if (!AreSame(ThermalWindingTimeConstant_actual.value(), ThermalWindingTimeConstant_file.value())) {
      ThermalWindingTimeConstant_Parameter.setParameter(ThermalWindingTimeConstant_file);
      joint->setConfigurationParameter(ThermalWindingTimeConstant_Parameter);
      std::cout << "ThermalWindingTimeConstant set to: " << ThermalWindingTimeConstant_file << std::endl;
    }

    if (!AreSame(I2tLimit_actual, I2tLimit_file)) {
      I2tLimit_Parameter.setParameter(I2tLimit_file);
      joint->setConfigurationParameter(I2tLimit_Parameter);
      std::cout << "I2tLimit set to: " << I2tLimit_file << std::endl;
    }

    if (!AreSame(MotorControllerTimeout_actual.value(), MotorControllerTimeout_file.value())) {
      MotorControllerTimeout_Parameter.setParameter(MotorControllerTimeout_file);
      joint->setConfigurationParameter(MotorControllerTimeout_Parameter);
      std::cout << "MotorControllerTimeout set to: " << MotorControllerTimeout_file << std::endl;
    }

    if (!AreSame(MotorHaltedVelocity_actual, MotorHaltedVelocity_file)) {
      MotorHaltedVelocity_Parameter.setParameter(MotorHaltedVelocity_file);
      joint->setConfigurationParameter(MotorHaltedVelocity_Parameter);
      std::cout << "MotorHaltedVelocity set to: " << MotorHaltedVelocity_file << std::endl;
    }



    std::cout << "Protected Parameters stored for Joint: " << jointName << std::endl << std::endl;
  } catch (JointParameterException& e) {

  }
}

void JointConfigurator::getPassword() {

  int password;
  std::cout << "Please enter the Password: " << std::endl;
  std::cin >> password;
  ApproveProtectedParameters approveParameters;
  approveParameters.setParameter(password);
  joint->setConfigurationParameter(approveParameters);

}
