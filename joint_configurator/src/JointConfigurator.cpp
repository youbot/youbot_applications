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
#include "youbot/YouBotJointParameter.hpp"
#include "youbot/YouBotGripper.hpp"
#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"
#include <vector>
#include <sstream>
#include <boost/limits.hpp>
#include "generic/Logger.hpp"
#include "generic/Units.hpp"
#include "generic/Time.hpp"
#include "generic/Exceptions.hpp"
#include "generic-joint/JointParameter.hpp"
#include "youbot/YouBotJointParameterReadOnly.hpp"

enum part {
  BASE,
  ARM
};


using namespace std;
using namespace youbot;

class JointConfigurator {
public:

  JointConfigurator(part baseOrArm, int jointNumber, std::string configname = "joint-parameter.cfg", std::string configNameProtected = "protected-joint-parameter.cfg");

  ~JointConfigurator();
  void readParameters();
  void readPasswordProtectedParameters();
  void readReadOnlyParameters();
  void setParametersToJoint();
  void storeParametersToJoint();
  void setProtectedParametersToJoint();
  void storeProtectedParametersToJoint();
  void getPassword();
  void menu();

private:

  bool AreSame(double A, double B);
  YouBotJoint* joint;
  ConfigFile* configfile;
  ConfigFile* configfilePP;
  bool ParameterRead;
  bool ProtectedParameterRead;
  part ArmOrBase;
  int jointNumber;
  int controllerType;
  double version;
  std::string jointName;
  YouBotManipulator* myYouBotManipulator;
  YouBotBase* myYouBotBase;

  MaximumPositioningVelocity MaximumPositioningVelocity_Parameter;
  quantity<angular_velocity> MaximumPositioningVelocity_actual;
  quantity<angular_velocity> MaximumPositioningVelocity_file;

  PWMLimit PWMLimit_Parameter;
  unsigned int PWMLimit_actual;
  unsigned int PWMLimit_file;

  MaximumMotorCurrent MaximumMotorCurrent_Parameter;
  quantity<current> MaximumMotorCurrent_actual;
  quantity<current> MaximumMotorCurrent_file;

  MaximumVelocityToSetPosition MaximumVelocityToSetPosition_Parameter;
  quantity<angular_velocity> MaximumVelocityToSetPosition_actual;
  quantity<angular_velocity> MaximumVelocityToSetPosition_file;

  SpeedControlSwitchingThreshold SpeedControlSwitchingThreshold_Parameter;
  quantity<angular_velocity> SpeedControlSwitchingThreshold_actual;
  quantity<angular_velocity> SpeedControlSwitchingThreshold_file;

  ClearTargetDistance ClearTargetDistance_Parameter;
  int ClearTargetDistance_actual;
  int ClearTargetDistance_file;

  PositionTargetReachedDistance PositionTargetReachedDistance_Parameter;
  int PositionTargetReachedDistance_actual;
  int PositionTargetReachedDistance_file;

  MotorAcceleration MotorAcceleration_Parameter;
  quantity<angular_acceleration> MotorAcceleration_actual;
  quantity<angular_acceleration> MotorAcceleration_file;

  PositionControlSwitchingThreshold PositionControlSwitchingThreshold_Parameter;
  quantity<angular_velocity> PositionControlSwitchingThreshold_actual;
  quantity<angular_velocity> PositionControlSwitchingThreshold_file;

  PParameterFirstParametersPositionControl PParameterFirstParametersPositionControl_Parameter;
  int PParameterFirstParametersPositionControl_actual;
  int PParameterFirstParametersPositionControl_file;

  IParameterFirstParametersPositionControl IParameterFirstParametersPositionControl_Parameter;
  int IParameterFirstParametersPositionControl_actual;
  int IParameterFirstParametersPositionControl_file;

  DParameterFirstParametersPositionControl DParameterFirstParametersPositionControl_Parameter;
  int DParameterFirstParametersPositionControl_actual;
  int DParameterFirstParametersPositionControl_file;

  PIDControlTime PIDControlTime_Parameter;
  quantity<si::time> PIDControlTime_actual;
  quantity<si::time> PIDControlTime_file;

  CurrentControlLoopDelay CurrentControlLoopDelay_Parameter;
  quantity<si::time> CurrentControlLoopDelay_actual;
  quantity<si::time> CurrentControlLoopDelay_file;

  IClippingParameterFirstParametersPositionControl IClippingParameterFirstParametersPositionControl_Parameter;
  int IClippingParameterFirstParametersPositionControl_actual;
  int IClippingParameterFirstParametersPositionControl_file;

  PWMHysteresis PWMHysteresis_Parameter;
  int PWMHysteresis_actual;
  int PWMHysteresis_file;

  ClearISumIfPWMReachesMaximum ClearISumIfPWMReachesMaximum_Parameter;
  bool ClearISumIfPWMReachesMaximum_actual;
  bool ClearISumIfPWMReachesMaximum_file;

  PParameterFirstParametersSpeedControl PParameterFirstParametersSpeedControl_Parameter;
  int PParameterFirstParametersSpeedControl_actual;
  int PParameterFirstParametersSpeedControl_file;

  IParameterFirstParametersSpeedControl IParameterFirstParametersSpeedControl_Parameter;
  int IParameterFirstParametersSpeedControl_actual;
  int IParameterFirstParametersSpeedControl_file;

  DParameterFirstParametersSpeedControl DParameterFirstParametersSpeedControl_Parameter;
  int DParameterFirstParametersSpeedControl_actual;
  int DParameterFirstParametersSpeedControl_file;

  IClippingParameterFirstParametersSpeedControl IClippingParameterFirstParametersSpeedControl_Parameter;
  int IClippingParameterFirstParametersSpeedControl_actual;
  int IClippingParameterFirstParametersSpeedControl_file;

  RampGeneratorSpeedAndPositionControl RampGeneratorSpeedAndPositionControl_Parameter;
  bool RampGeneratorSpeedAndPositionControl_actual;
  bool RampGeneratorSpeedAndPositionControl_file;

  SetEncoderCounterZeroAtNextNChannel SetEncoderCounterZeroAtNextNChannel_Parameter;
  bool SetEncoderCounterZeroAtNextNChannel_actual;
  bool SetEncoderCounterZeroAtNextNChannel_file;

  SetEncoderCounterZeroAtNextSwitch SetEncoderCounterZeroAtNextSwitch_Parameter;
  bool SetEncoderCounterZeroAtNextSwitch_actual;
  bool SetEncoderCounterZeroAtNextSwitch_file;

  SetEncoderCounterZeroOnlyOnce SetEncoderCounterZeroOnlyOnce_Parameter;
  bool SetEncoderCounterZeroOnlyOnce_actual;
  bool SetEncoderCounterZeroOnlyOnce_file;

  EncoderStopSwitch EncoderStopSwitch_Parameter;
  unsigned int EncoderStopSwitch_actual;
  unsigned int EncoderStopSwitch_file;

  ActualCommutationOffset ActualCommutationOffset_Parameter;
  int ActualCommutationOffset_actual;
  int ActualCommutationOffset_file;

  StopSwitchPolarity StopSwitchPolarity_Parameter;
  unsigned int StopSwitchPolarity_actual;
  unsigned int StopSwitchPolarity_file;

  PParameterFirstParametersCurrentControl PParameterFirstParametersCurrentControl_Parameter;
  int PParameterFirstParametersCurrentControl_actual;
  int PParameterFirstParametersCurrentControl_file;

  IParameterFirstParametersCurrentControl IParameterFirstParametersCurrentControl_Parameter;
  int IParameterFirstParametersCurrentControl_actual;
  int IParameterFirstParametersCurrentControl_file;

  DParameterFirstParametersCurrentControl DParameterFirstParametersCurrentControl_Parameter;
  int DParameterFirstParametersCurrentControl_actual;
  int DParameterFirstParametersCurrentControl_file;

  IClippingParameterFirstParametersCurrentControl IClippingParameterFirstParametersCurrentControl_Parameter;
  int IClippingParameterFirstParametersCurrentControl_actual;
  int IClippingParameterFirstParametersCurrentControl_file;

  PParameterSecondParametersCurrentControl PParameterSecondParametersCurrentControl_Parameter;
  int PParameterSecondParametersCurrentControl_actual;
  int PParameterSecondParametersCurrentControl_file;

  IParameterSecondParametersCurrentControl IParameterSecondParametersCurrentControl_Parameter;
  int IParameterSecondParametersCurrentControl_actual;
  int IParameterSecondParametersCurrentControl_file;

  DParameterSecondParametersCurrentControl DParameterSecondParametersCurrentControl_Parameter;
  int DParameterSecondParametersCurrentControl_actual;
  int DParameterSecondParametersCurrentControl_file;

  IClippingParameterSecondParametersCurrentControl IClippingParameterSecondParametersCurrentControl_Parameter;
  int IClippingParameterSecondParametersCurrentControl_actual;
  int IClippingParameterSecondParametersCurrentControl_file;

  CurrentControlSwitchingThreshold CurrentControlSwitchingThreshold_Parameter;
  quantity<angular_velocity> CurrentControlSwitchingThreshold_actual;
  quantity<angular_velocity> CurrentControlSwitchingThreshold_file;

  CommutationMotorCurrent CommutationMotorCurrent_Parameter;
  quantity<current> CommutationMotorCurrent_actual;
  quantity<current> CommutationMotorCurrent_file;

  PParameterSecondParametersPositionControl PParameterSecondParametersPositionControl_Parameter;
  int PParameterSecondParametersPositionControl_actual;
  int PParameterSecondParametersPositionControl_file;

  IParameterSecondParametersPositionControl IParameterSecondParametersPositionControl_Parameter;
  int IParameterSecondParametersPositionControl_actual;
  int IParameterSecondParametersPositionControl_file;

  DParameterSecondParametersPositionControl DParameterSecondParametersPositionControl_Parameter;
  int DParameterSecondParametersPositionControl_actual;
  int DParameterSecondParametersPositionControl_file;

  IClippingParameterSecondParametersPositionControl IClippingParameterSecondParametersPositionControl_Parameter;
  int IClippingParameterSecondParametersPositionControl_actual;
  int IClippingParameterSecondParametersPositionControl_file;

  PParameterSecondParametersSpeedControl PParameterSecondParametersSpeedControl_Parameter;
  int PParameterSecondParametersSpeedControl_actual;
  int PParameterSecondParametersSpeedControl_file;

  IParameterSecondParametersSpeedControl IParameterSecondParametersSpeedControl_Parameter;
  int IParameterSecondParametersSpeedControl_actual;
  int IParameterSecondParametersSpeedControl_file;

  DParameterSecondParametersSpeedControl DParameterSecondParametersSpeedControl_Parameter;
  int DParameterSecondParametersSpeedControl_actual;
  int DParameterSecondParametersSpeedControl_file;

  IClippingParameterSecondParametersSpeedControl IClippingParameterSecondParametersSpeedControl_Parameter;
  int IClippingParameterSecondParametersSpeedControl_actual;
  int IClippingParameterSecondParametersSpeedControl_file;

  MassInertiaConstant MassInertiaConstant_Parameter;
  int MassInertiaConstant_actual;
  int MassInertiaConstant_file;

  BEMFConstant BEMFConstant_Parameter;
  int BEMFConstant_actual;
  int BEMFConstant_file;

  SineInitializationVelocity SineInitializationVelocity_Parameter;
  int SineInitializationVelocity_actual;
  int SineInitializationVelocity_file;

  CommutationCompensationClockwise CommutationCompensationClockwise_Parameter;
  int CommutationCompensationClockwise_actual;
  int CommutationCompensationClockwise_file;

  CommutationCompensationCounterClockwise CommutationCompensationCounterClockwise_Parameter;
  int CommutationCompensationCounterClockwise_actual;
  int CommutationCompensationCounterClockwise_file;

  InitSineDelay InitSineDelay_Parameter;
  quantity<si::time> InitSineDelay_actual;
  quantity<si::time> InitSineDelay_file;

  ActivateOvervoltageProtection ActivateOvervoltageProtection_Parameter;
  bool ActivateOvervoltageProtection_actual;
  bool ActivateOvervoltageProtection_file;

  MaximumPWMChangePerPIDInterval MaximumPWMChangePerPIDInterval_Parameter;
  int MaximumPWMChangePerPIDInterval_actual;
  int MaximumPWMChangePerPIDInterval_file;

  SineCompensationFactor SineCompensationFactor_Parameter;
  int SineCompensationFactor_actual;
  int SineCompensationFactor_file;

  EncoderNullPolarity EncoderNullPolarity_Parameter;
  bool EncoderNullPolarity_actual;
  bool EncoderNullPolarity_file;

  MotorContollerGearRatio MotorContollerGearRatio_Parameter;
  unsigned int MotorContollerGearRatio_actual;
  unsigned int MotorContollerGearRatio_file;

  CommutationMode CommutationMode_Parameter;
  unsigned int CommutationMode_actual;
  unsigned int CommutationMode_file;

  EncoderResolution EncoderResolution_Parameter;
  unsigned int EncoderResolution_actual;
  unsigned int EncoderResolution_file;

  HallSensorPolarityReversal HallSensorPolarityReversal_Parameter;
  bool HallSensorPolarityReversal_actual;
  bool HallSensorPolarityReversal_file;

  InitializationMode InitializationMode_Parameter;
  int InitializationMode_actual;
  int InitializationMode_file;

  MotorCoilResistance MotorCoilResistance_Parameter;
  quantity<resistance> MotorCoilResistance_actual;
  quantity<resistance> MotorCoilResistance_file;

  MotorPoles MotorPoles_Parameter;
  unsigned int MotorPoles_actual;
  unsigned int MotorPoles_file;

  PIDControllerState PIDControllerState_Parameter;
  int PIDControllerState_actual;
  int PIDControllerState_file;

  PWMSchemeBlockCommutation PWMSchemeBlockCommutation_Parameter;
  unsigned int PWMSchemeBlockCommutation_actual;
  unsigned int PWMSchemeBlockCommutation_file;

  ReversingEncoderDirection ReversingEncoderDirection_Parameter;
  bool ReversingEncoderDirection_actual;
  bool ReversingEncoderDirection_file;

  OperationalTime OperationalTime_Parameter;
  quantity<si::time> OperationalTime_actual;
  quantity<si::time> OperationalTime_file;

  ThermalWindingTimeConstant ThermalWindingTimeConstant_Parameter;
  quantity<si::time> ThermalWindingTimeConstant_actual;
  quantity<si::time> ThermalWindingTimeConstant_file;

  I2tLimit I2tLimit_Parameter;
  unsigned int I2tLimit_actual;
  unsigned int I2tLimit_file;

  I2tExceedCounter I2tExceedCounter_Parameter;
  unsigned int I2tExceedCounter_actual;
  unsigned int I2tExceedCounter_file;

  ActualMotorVoltage ActualMotorVoltage_Parameter;
  unsigned int ActualMotorVoltage_actual;
  ActualPWMDutyCycle ActualPWMDutyCycle_Parameter;
  unsigned int ActualPWMDutyCycle_actual;
  PositionError PositionError_Parameter;
 quantity<plane_angle> PositionError_actual;
  PositionErrorSum PositionErrorSum_Parameter;
  quantity<plane_angle> PositionErrorSum_actual;
  VelocityError VelocityError_Parameter;
  quantity<si::angular_velocity> VelocityError_actual;
  VelocityErrorSum VelocityErrorSum_Parameter;
  quantity<si::angular_velocity> VelocityErrorSum_actual;
  RampGeneratorSpeed RampGeneratorSpeed_Parameter;
  quantity<si::angular_velocity> RampGeneratorSpeed_actual;
  I2tSum I2tSum_Parameter;
  unsigned int I2tSum_actual;
  
  MotorControllerTimeout MotorControllerTimeout_Parameter;
  quantity<si::time> MotorControllerTimeout_actual;
  quantity<si::time> MotorControllerTimeout_file;

};

JointConfigurator::JointConfigurator(part baseOrArm, int jointNumber, std::string configname, std::string configNameProtected) {

  ParameterRead = false;
  ProtectedParameterRead = false;
  configfile = new ConfigFile(configname, std::string(CONFIG_DIR));
  configfilePP = new ConfigFile(configNameProtected, std::string(CONFIG_DIR));
  this->ArmOrBase = baseOrArm;
  this->jointNumber = jointNumber;

  if (baseOrArm == ARM) {
    myYouBotManipulator = new YouBotManipulator("youbot-manipulator", YOUBOT_CONFIGURATIONS_DIR);
    joint = &(myYouBotManipulator->getArmJoint(jointNumber));
  } else if (baseOrArm == BASE) {
    myYouBotBase = new YouBotBase("youbot-base", YOUBOT_CONFIGURATIONS_DIR);
    joint = &(myYouBotBase->getBaseJoint(jointNumber));
  } else {
    throw "unable to create joint";
  }

  FirmwareVersion firmwareVersion;

  joint->getConfigurationParameter(firmwareVersion);
  firmwareVersion.getParameter(controllerType, version);
  
  JointName jName;
  joint->getConfigurationParameter(jName);
  jName.getParameter(jointName);

}

JointConfigurator::~JointConfigurator() {
  delete configfile;
  delete myYouBotManipulator;
  delete myYouBotBase;
}

bool JointConfigurator::AreSame(double A, double B) {
  return std::fabs(A - B) < 0.01;
}

void JointConfigurator::readParameters() {
  double dummy;

  std::cout << std::endl;

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
  ParameterRead = true;
}

void JointConfigurator::readPasswordProtectedParameters() {
  std::cout << "===================== Password Protected Parameters =====================" << std::endl;

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

  joint->getConfigurationParameter(MaximumVelocityToSetPosition_Parameter);
  MaximumVelocityToSetPosition_Parameter.getParameter(MaximumVelocityToSetPosition_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "MaximumVelocityToSetPosition");
  MaximumVelocityToSetPosition_file = dummy * radian_per_second;
  if (!AreSame(MaximumVelocityToSetPosition_actual.value(), MaximumVelocityToSetPosition_file.value())) {
    std::cout << "MaximumVelocityToSetPosition \t\t\t\tactual: " << MaximumVelocityToSetPosition_actual << " \tNEW VALUE: " << MaximumVelocityToSetPosition_file << std::endl;
  } else {
    std::cout << "MaximumVelocityToSetPosition \t\t\t\tactual: " << MaximumVelocityToSetPosition_actual << std::endl;
  }

  joint->getConfigurationParameter(ClearTargetDistance_Parameter);
  ClearTargetDistance_Parameter.getParameter(ClearTargetDistance_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "ClearTargetDistance");
  ClearTargetDistance_file = dummy;
  if (ClearTargetDistance_actual != ClearTargetDistance_file) {
    std::cout << "ClearTargetDistance         \t\t\t\tactual: " << ClearTargetDistance_actual << " \tNEW VALUE: " << ClearTargetDistance_file << std::endl;
  } else {
    std::cout << "ClearTargetDistance         \t\t\t\tactual: " << ClearTargetDistance_actual << std::endl;
  }

  joint->getConfigurationParameter(PositionTargetReachedDistance_Parameter);
  PositionTargetReachedDistance_Parameter.getParameter(PositionTargetReachedDistance_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "PositionTargetReachedDistance");
  PositionTargetReachedDistance_file = dummy;
  if (PositionTargetReachedDistance_actual != PositionTargetReachedDistance_file) {
    std::cout << "PositionTargetReachedDistance \t\t\t\tactual: " << PositionTargetReachedDistance_actual << " \tNEW VALUE: " << PositionTargetReachedDistance_file << std::endl;
  } else {
    std::cout << "PositionTargetReachedDistance \t\t\t\tactual: " << PositionTargetReachedDistance_actual << std::endl;
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

  joint->getConfigurationParameter(ClearISumIfPWMReachesMaximum_Parameter);
  ClearISumIfPWMReachesMaximum_Parameter.getParameter(ClearISumIfPWMReachesMaximum_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "ClearISumIfPWMReachesMaximum");
  ClearISumIfPWMReachesMaximum_file = dummy;
  if (!AreSame(ClearISumIfPWMReachesMaximum_actual, ClearISumIfPWMReachesMaximum_file)) {
    std::cout << "ClearISumIfPWMReachesMaximum   \t\t\t\tactual: " << ClearISumIfPWMReachesMaximum_actual << " \tNEW VALUE: " << ClearISumIfPWMReachesMaximum_file << std::endl;
  } else {
    std::cout << "ClearISumIfPWMReachesMaximum   \t\t\t\tactual: " << ClearISumIfPWMReachesMaximum_actual << std::endl;
  }

  joint->getConfigurationParameter(PIDControllerState_Parameter);
  PIDControllerState_Parameter.getParameter(PIDControllerState_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "PIDControllerState");
  PIDControllerState_file = dummy;
  if (!AreSame(PIDControllerState_actual, PIDControllerState_file)) {
    std::cout << "PIDControllerState \t\t\t\t\tactual: " << PIDControllerState_actual << " \tNEW VALUE: " << PIDControllerState_file << std::endl;
  } else {
    std::cout << "PIDControllerState \t\t\t\t\tactual: " << PIDControllerState_actual << std::endl;
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

  joint->getConfigurationParameter(MotorContollerGearRatio_Parameter);
  MotorContollerGearRatio_Parameter.getParameter(MotorContollerGearRatio_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "MotorContollerGearRatio");
  MotorContollerGearRatio_file = dummy;
  if (!AreSame(MotorContollerGearRatio_actual, MotorContollerGearRatio_file)) {
    std::cout << "MotorContollerGearRatio \t\t\t\tactual: " << MotorContollerGearRatio_actual << " \tNEW VALUE: " << MotorContollerGearRatio_file << std::endl;
  } else {
    std::cout << "MotorContollerGearRatio \t\t\t\tactual: " << MotorContollerGearRatio_actual << std::endl;
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
  configfilePP->readInto(dummy, "Joint_Parameter", "SineInitializationVelocity");
  SineInitializationVelocity_file = dummy;
  if (!AreSame(SineInitializationVelocity_actual, SineInitializationVelocity_file)) {
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

  joint->getConfigurationParameter(MaximumPWMChangePerPIDInterval_Parameter);
  MaximumPWMChangePerPIDInterval_Parameter.getParameter(MaximumPWMChangePerPIDInterval_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "MaximumPWMChangePerPIDInterval");
  MaximumPWMChangePerPIDInterval_file = dummy;
  if (!AreSame(MaximumPWMChangePerPIDInterval_actual, MaximumPWMChangePerPIDInterval_file)) {
    std::cout << "MaximumPWMChangePerPIDInterval \t\t\t\tactual: " << MaximumPWMChangePerPIDInterval_actual << " \tNEW VALUE: " << MaximumPWMChangePerPIDInterval_file << std::endl;
  } else {
    std::cout << "MaximumPWMChangePerPIDInterval \t\t\t\tactual: " << MaximumPWMChangePerPIDInterval_actual << std::endl;
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

  joint->getConfigurationParameter(EncoderNullPolarity_Parameter);
  EncoderNullPolarity_Parameter.getParameter(EncoderNullPolarity_actual);
  configfilePP->readInto(dummy, "Joint_Parameter", "EncoderNullPolarity");
  EncoderNullPolarity_file = dummy;
  if (!AreSame(EncoderNullPolarity_actual, EncoderNullPolarity_file)) {
    std::cout << "EncoderNullPolarity \t\t\t\t\tactual: " << EncoderNullPolarity_actual << " \tNEW VALUE: " << EncoderNullPolarity_file << std::endl;
  } else {
    std::cout << "EncoderNullPolarity \t\t\t\t\tactual: " << EncoderNullPolarity_actual << std::endl;
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

  
  
  
  ProtectedParameterRead = true;
}

//=======================READ ONLY =============================

void JointConfigurator::readReadOnlyParameters() {
  
  std::cout << "===================== Read Only Parameters =====================" << std::endl;

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
  
  joint->getConfigurationParameter(RampGeneratorSpeed_Parameter);
  RampGeneratorSpeed_Parameter.getParameter(RampGeneratorSpeed_actual);
  std::cout << "RampGeneratorSpeed \t\t\t\t\tactual: " << RampGeneratorSpeed_actual << std::endl;
  
  joint->getConfigurationParameter(I2tSum_Parameter);
  I2tSum_Parameter.getParameter(I2tSum_actual);
  std::cout << "I2tSum \t\t\t\t\t\t\tactual: " << I2tSum_actual << std::endl;
  
  joint->getConfigurationParameter(I2tExceedCounter_Parameter);
  I2tExceedCounter_Parameter.getParameter(I2tExceedCounter_actual);
  std::cout << "I2tExceedCounter \t\t\t\t\tactual: " << I2tExceedCounter_actual << std::endl;

}

void JointConfigurator::setParametersToJoint() {
  if (!ParameterRead) {
    std::cout << "The Joint Parameters have to been read before hand!" << std::endl;
    return;
  }
  MaximumPositioningVelocity_Parameter.setParameter(MaximumPositioningVelocity_file);
  joint->setConfigurationParameter(MaximumPositioningVelocity_Parameter);

  MotorAcceleration_Parameter.setParameter(MotorAcceleration_file);
  joint->setConfigurationParameter(MotorAcceleration_Parameter);

  PositionControlSwitchingThreshold_Parameter.setParameter(PositionControlSwitchingThreshold_file);
  joint->setConfigurationParameter(PositionControlSwitchingThreshold_Parameter);

  SpeedControlSwitchingThreshold_Parameter.setParameter(SpeedControlSwitchingThreshold_file);
  joint->setConfigurationParameter(SpeedControlSwitchingThreshold_Parameter);

  CurrentControlSwitchingThreshold_Parameter.setParameter(CurrentControlSwitchingThreshold_file);
  joint->setConfigurationParameter(CurrentControlSwitchingThreshold_Parameter);

  RampGeneratorSpeedAndPositionControl_Parameter.setParameter(RampGeneratorSpeedAndPositionControl_file);
  joint->setConfigurationParameter(RampGeneratorSpeedAndPositionControl_Parameter);

  PParameterFirstParametersPositionControl_Parameter.setParameter(PParameterFirstParametersPositionControl_file);
  joint->setConfigurationParameter(PParameterFirstParametersPositionControl_Parameter);

  IParameterFirstParametersPositionControl_Parameter.setParameter(IParameterFirstParametersPositionControl_file);
  joint->setConfigurationParameter(IParameterFirstParametersPositionControl_Parameter);

  DParameterFirstParametersPositionControl_Parameter.setParameter(DParameterFirstParametersPositionControl_file);
  joint->setConfigurationParameter(DParameterFirstParametersPositionControl_Parameter);

  IClippingParameterFirstParametersPositionControl_Parameter.setParameter(IClippingParameterFirstParametersPositionControl_file);
  joint->setConfigurationParameter(IClippingParameterFirstParametersPositionControl_Parameter);

  PParameterFirstParametersSpeedControl_Parameter.setParameter(PParameterFirstParametersSpeedControl_file);
  joint->setConfigurationParameter(PParameterFirstParametersSpeedControl_Parameter);

  IParameterFirstParametersSpeedControl_Parameter.setParameter(IParameterFirstParametersSpeedControl_file);
  joint->setConfigurationParameter(IParameterFirstParametersSpeedControl_Parameter);

  DParameterFirstParametersSpeedControl_Parameter.setParameter(DParameterFirstParametersSpeedControl_file);
  joint->setConfigurationParameter(DParameterFirstParametersSpeedControl_Parameter);

  IClippingParameterFirstParametersSpeedControl_Parameter.setParameter(IClippingParameterFirstParametersSpeedControl_file);
  joint->setConfigurationParameter(IClippingParameterFirstParametersSpeedControl_Parameter);

  PParameterFirstParametersCurrentControl_Parameter.setParameter(PParameterFirstParametersCurrentControl_file);
  joint->setConfigurationParameter(PParameterFirstParametersCurrentControl_Parameter);

  IParameterFirstParametersCurrentControl_Parameter.setParameter(IParameterFirstParametersCurrentControl_file);
  joint->setConfigurationParameter(IParameterFirstParametersCurrentControl_Parameter);

  DParameterFirstParametersCurrentControl_Parameter.setParameter(DParameterFirstParametersCurrentControl_file);
  joint->setConfigurationParameter(DParameterFirstParametersCurrentControl_Parameter);

  IClippingParameterFirstParametersCurrentControl_Parameter.setParameter(IClippingParameterFirstParametersCurrentControl_file);
  joint->setConfigurationParameter(IClippingParameterFirstParametersCurrentControl_Parameter);

  PParameterSecondParametersPositionControl_Parameter.setParameter(PParameterSecondParametersPositionControl_file);
  joint->setConfigurationParameter(PParameterSecondParametersPositionControl_Parameter);

  IParameterSecondParametersPositionControl_Parameter.setParameter(IParameterSecondParametersPositionControl_file);
  joint->setConfigurationParameter(IParameterSecondParametersPositionControl_Parameter);

  DParameterSecondParametersPositionControl_Parameter.setParameter(DParameterSecondParametersPositionControl_file);
  joint->setConfigurationParameter(DParameterSecondParametersPositionControl_Parameter);

  IClippingParameterSecondParametersPositionControl_Parameter.setParameter(IClippingParameterSecondParametersPositionControl_file);
  joint->setConfigurationParameter(IClippingParameterSecondParametersPositionControl_Parameter);

  PParameterSecondParametersSpeedControl_Parameter.setParameter(PParameterSecondParametersSpeedControl_file);
  joint->setConfigurationParameter(PParameterSecondParametersSpeedControl_Parameter);

  IParameterSecondParametersSpeedControl_Parameter.setParameter(IParameterSecondParametersSpeedControl_file);
  joint->setConfigurationParameter(IParameterSecondParametersSpeedControl_Parameter);

  DParameterSecondParametersSpeedControl_Parameter.setParameter(DParameterSecondParametersSpeedControl_file);
  joint->setConfigurationParameter(DParameterSecondParametersSpeedControl_Parameter);

  IClippingParameterSecondParametersSpeedControl_Parameter.setParameter(IClippingParameterSecondParametersSpeedControl_file);
  joint->setConfigurationParameter(IClippingParameterSecondParametersSpeedControl_Parameter);

  PParameterSecondParametersCurrentControl_Parameter.setParameter(PParameterSecondParametersCurrentControl_file);
  joint->setConfigurationParameter(PParameterSecondParametersCurrentControl_Parameter);

  IParameterSecondParametersCurrentControl_Parameter.setParameter(IParameterSecondParametersCurrentControl_file);
  joint->setConfigurationParameter(IParameterSecondParametersCurrentControl_Parameter);

  DParameterSecondParametersCurrentControl_Parameter.setParameter(DParameterSecondParametersCurrentControl_file);
  joint->setConfigurationParameter(DParameterSecondParametersCurrentControl_Parameter);

  IClippingParameterSecondParametersCurrentControl_Parameter.setParameter(IClippingParameterSecondParametersCurrentControl_file);
  joint->setConfigurationParameter(IClippingParameterSecondParametersCurrentControl_Parameter);


  std::cout << "Parameters set!" << std::endl;
}

void JointConfigurator::setProtectedParametersToJoint() {
  if (!ProtectedParameterRead) {
    std::cout << "The protected Joint Parameters have to been read before hand!" << std::endl;
    return;
  }
  try {

    PWMLimit_Parameter.setParameter(PWMLimit_file);
    joint->setConfigurationParameter(PWMLimit_Parameter);

    MaximumMotorCurrent_Parameter.setParameter(MaximumMotorCurrent_file);
    joint->setConfigurationParameter(MaximumMotorCurrent_Parameter);

    MaximumVelocityToSetPosition_Parameter.setParameter(MaximumVelocityToSetPosition_file);
    joint->setConfigurationParameter(MaximumVelocityToSetPosition_Parameter);

    ClearTargetDistance_Parameter.setParameter(ClearTargetDistance_file);
    joint->setConfigurationParameter(ClearTargetDistance_Parameter);

    PositionTargetReachedDistance_Parameter.setParameter(PositionTargetReachedDistance_file);
    joint->setConfigurationParameter(PositionTargetReachedDistance_Parameter);

    PIDControlTime_Parameter.setParameter(PIDControlTime_file);
    joint->setConfigurationParameter(PIDControlTime_Parameter);

    CurrentControlLoopDelay_Parameter.setParameter(CurrentControlLoopDelay_file);
    joint->setConfigurationParameter(CurrentControlLoopDelay_Parameter);

    PWMHysteresis_Parameter.setParameter(PWMHysteresis_file);
    joint->setConfigurationParameter(PWMHysteresis_Parameter);

    ClearISumIfPWMReachesMaximum_Parameter.setParameter(ClearISumIfPWMReachesMaximum_file);
    joint->setConfigurationParameter(ClearISumIfPWMReachesMaximum_Parameter);

    SetEncoderCounterZeroAtNextNChannel_Parameter.setParameter(SetEncoderCounterZeroAtNextNChannel_file);
    joint->setConfigurationParameter(SetEncoderCounterZeroAtNextNChannel_Parameter);

    SetEncoderCounterZeroAtNextSwitch_Parameter.setParameter(SetEncoderCounterZeroAtNextSwitch_file);
    joint->setConfigurationParameter(SetEncoderCounterZeroAtNextSwitch_Parameter);

    SetEncoderCounterZeroOnlyOnce_Parameter.setParameter(SetEncoderCounterZeroOnlyOnce_file);
    joint->setConfigurationParameter(SetEncoderCounterZeroOnlyOnce_Parameter);

    EncoderStopSwitch_Parameter.setParameter(EncoderStopSwitch_file);
    joint->setConfigurationParameter(EncoderStopSwitch_Parameter);

    StopSwitchPolarity_Parameter.setParameter(StopSwitchPolarity_file);
    joint->setConfigurationParameter(StopSwitchPolarity_Parameter);

    CommutationMotorCurrent_Parameter.setParameter(CommutationMotorCurrent_file);
    joint->setConfigurationParameter(CommutationMotorCurrent_Parameter);

    MassInertiaConstant_Parameter.setParameter(MassInertiaConstant_file);
    joint->setConfigurationParameter(MassInertiaConstant_Parameter);

    BEMFConstant_Parameter.setParameter(BEMFConstant_file);
    joint->setConfigurationParameter(BEMFConstant_Parameter);

    SineInitializationVelocity_Parameter.setParameter(SineInitializationVelocity_file);
    joint->setConfigurationParameter(SineInitializationVelocity_Parameter);

    CommutationCompensationClockwise_Parameter.setParameter(CommutationCompensationClockwise_file);
    joint->setConfigurationParameter(CommutationCompensationClockwise_Parameter);

    CommutationCompensationCounterClockwise_Parameter.setParameter(CommutationCompensationCounterClockwise_file);
    joint->setConfigurationParameter(CommutationCompensationCounterClockwise_Parameter);

    InitSineDelay_Parameter.setParameter(InitSineDelay_file);
    joint->setConfigurationParameter(InitSineDelay_Parameter);

    ActivateOvervoltageProtection_Parameter.setParameter(ActivateOvervoltageProtection_file);
    joint->setConfigurationParameter(ActivateOvervoltageProtection_Parameter);

    MaximumPWMChangePerPIDInterval_Parameter.setParameter(MaximumPWMChangePerPIDInterval_file);
    joint->setConfigurationParameter(MaximumPWMChangePerPIDInterval_Parameter);

    SineCompensationFactor_Parameter.setParameter(SineCompensationFactor_file);
    joint->setConfigurationParameter(SineCompensationFactor_Parameter);

    EncoderNullPolarity_Parameter.setParameter(EncoderNullPolarity_file);
    joint->setConfigurationParameter(EncoderNullPolarity_Parameter);

    MotorContollerGearRatio_Parameter.setParameter(MotorContollerGearRatio_file);
    joint->setConfigurationParameter(MotorContollerGearRatio_Parameter);

    CommutationMode_Parameter.setParameter(CommutationMode_file);
    joint->setConfigurationParameter(CommutationMode_Parameter);

    EncoderResolution_Parameter.setParameter(EncoderResolution_file);
    joint->setConfigurationParameter(EncoderResolution_Parameter);

    HallSensorPolarityReversal_Parameter.setParameter(HallSensorPolarityReversal_file);
    joint->setConfigurationParameter(HallSensorPolarityReversal_Parameter);

    InitializationMode_Parameter.setParameter(InitializationMode_file);
    joint->setConfigurationParameter(InitializationMode_Parameter);

    MotorCoilResistance_Parameter.setParameter(MotorCoilResistance_file);
    joint->setConfigurationParameter(MotorCoilResistance_Parameter);

    MotorPoles_Parameter.setParameter(MotorPoles_file);
    joint->setConfigurationParameter(MotorPoles_Parameter);

    PIDControllerState_Parameter.setParameter(PIDControllerState_file);
    joint->setConfigurationParameter(PIDControllerState_Parameter);

    PWMSchemeBlockCommutation_Parameter.setParameter(PWMSchemeBlockCommutation_file);
    joint->setConfigurationParameter(PWMSchemeBlockCommutation_Parameter);

    ReversingEncoderDirection_Parameter.setParameter(ReversingEncoderDirection_file);
    joint->setConfigurationParameter(ReversingEncoderDirection_Parameter);

    ThermalWindingTimeConstant_Parameter.setParameter(ThermalWindingTimeConstant_file);
    joint->setConfigurationParameter(ThermalWindingTimeConstant_Parameter);

    I2tLimit_Parameter.setParameter(I2tLimit_file);
    joint->setConfigurationParameter(I2tLimit_Parameter);
    
    MotorControllerTimeout_Parameter.setParameter(MotorControllerTimeout_file);
    joint->setConfigurationParameter(MotorControllerTimeout_Parameter);
    

    std::cout << "Protected Parameters set!" << std::endl;

  } catch (JointParameterException& e) {

  }
}

void JointConfigurator::storeParametersToJoint() {

  if (!ParameterRead) {
    std::cout << "The Joint Parameters have to been read before hand!" << std::endl;
    return;
  }

  MaximumPositioningVelocity_Parameter.setParameter(MaximumPositioningVelocity_file);
  joint->storeConfigurationParameterPermanent(MaximumPositioningVelocity_Parameter);

  MotorAcceleration_Parameter.setParameter(MotorAcceleration_file);
  joint->storeConfigurationParameterPermanent(MotorAcceleration_Parameter);

  PositionControlSwitchingThreshold_Parameter.setParameter(PositionControlSwitchingThreshold_file);
  joint->storeConfigurationParameterPermanent(PositionControlSwitchingThreshold_Parameter);

  SpeedControlSwitchingThreshold_Parameter.setParameter(SpeedControlSwitchingThreshold_file);
  joint->storeConfigurationParameterPermanent(SpeedControlSwitchingThreshold_Parameter);

  CurrentControlSwitchingThreshold_Parameter.setParameter(CurrentControlSwitchingThreshold_file);
  joint->storeConfigurationParameterPermanent(CurrentControlSwitchingThreshold_Parameter);

  RampGeneratorSpeedAndPositionControl_Parameter.setParameter(RampGeneratorSpeedAndPositionControl_file);
  joint->storeConfigurationParameterPermanent(RampGeneratorSpeedAndPositionControl_Parameter);

  PParameterFirstParametersPositionControl_Parameter.setParameter(PParameterFirstParametersPositionControl_file);
  joint->storeConfigurationParameterPermanent(PParameterFirstParametersPositionControl_Parameter);

  IParameterFirstParametersPositionControl_Parameter.setParameter(IParameterFirstParametersPositionControl_file);
  joint->storeConfigurationParameterPermanent(IParameterFirstParametersPositionControl_Parameter);

  DParameterFirstParametersPositionControl_Parameter.setParameter(DParameterFirstParametersPositionControl_file);
  joint->storeConfigurationParameterPermanent(DParameterFirstParametersPositionControl_Parameter);

  IClippingParameterFirstParametersPositionControl_Parameter.setParameter(IClippingParameterFirstParametersPositionControl_file);
  joint->storeConfigurationParameterPermanent(IClippingParameterFirstParametersPositionControl_Parameter);

  PParameterFirstParametersSpeedControl_Parameter.setParameter(PParameterFirstParametersSpeedControl_file);
  joint->storeConfigurationParameterPermanent(PParameterFirstParametersSpeedControl_Parameter);

  IParameterFirstParametersSpeedControl_Parameter.setParameter(IParameterFirstParametersSpeedControl_file);
  joint->storeConfigurationParameterPermanent(IParameterFirstParametersSpeedControl_Parameter);

  DParameterFirstParametersSpeedControl_Parameter.setParameter(DParameterFirstParametersSpeedControl_file);
  joint->storeConfigurationParameterPermanent(DParameterFirstParametersSpeedControl_Parameter);

  IClippingParameterFirstParametersSpeedControl_Parameter.setParameter(IClippingParameterFirstParametersSpeedControl_file);
  joint->storeConfigurationParameterPermanent(IClippingParameterFirstParametersSpeedControl_Parameter);

  PParameterFirstParametersCurrentControl_Parameter.setParameter(PParameterFirstParametersCurrentControl_file);
  joint->storeConfigurationParameterPermanent(PParameterFirstParametersCurrentControl_Parameter);

  IParameterFirstParametersCurrentControl_Parameter.setParameter(IParameterFirstParametersCurrentControl_file);
  joint->storeConfigurationParameterPermanent(IParameterFirstParametersCurrentControl_Parameter);

  DParameterFirstParametersCurrentControl_Parameter.setParameter(DParameterFirstParametersCurrentControl_file);
  joint->storeConfigurationParameterPermanent(DParameterFirstParametersCurrentControl_Parameter);

  IClippingParameterFirstParametersCurrentControl_Parameter.setParameter(IClippingParameterFirstParametersCurrentControl_file);
  joint->storeConfigurationParameterPermanent(IClippingParameterFirstParametersCurrentControl_Parameter);

  PParameterSecondParametersPositionControl_Parameter.setParameter(PParameterSecondParametersPositionControl_file);
  joint->storeConfigurationParameterPermanent(PParameterSecondParametersPositionControl_Parameter);

  IParameterSecondParametersPositionControl_Parameter.setParameter(IParameterSecondParametersPositionControl_file);
  joint->storeConfigurationParameterPermanent(IParameterSecondParametersPositionControl_Parameter);

  DParameterSecondParametersPositionControl_Parameter.setParameter(DParameterSecondParametersPositionControl_file);
  joint->storeConfigurationParameterPermanent(DParameterSecondParametersPositionControl_Parameter);

  IClippingParameterSecondParametersPositionControl_Parameter.setParameter(IClippingParameterSecondParametersPositionControl_file);
  joint->storeConfigurationParameterPermanent(IClippingParameterSecondParametersPositionControl_Parameter);

  PParameterSecondParametersSpeedControl_Parameter.setParameter(PParameterSecondParametersSpeedControl_file);
  joint->storeConfigurationParameterPermanent(PParameterSecondParametersSpeedControl_Parameter);

  IParameterSecondParametersSpeedControl_Parameter.setParameter(IParameterSecondParametersSpeedControl_file);
  joint->storeConfigurationParameterPermanent(IParameterSecondParametersSpeedControl_Parameter);

  DParameterSecondParametersSpeedControl_Parameter.setParameter(DParameterSecondParametersSpeedControl_file);
  joint->storeConfigurationParameterPermanent(DParameterSecondParametersSpeedControl_Parameter);

  IClippingParameterSecondParametersSpeedControl_Parameter.setParameter(IClippingParameterSecondParametersSpeedControl_file);
  joint->storeConfigurationParameterPermanent(IClippingParameterSecondParametersSpeedControl_Parameter);

  PParameterSecondParametersCurrentControl_Parameter.setParameter(PParameterSecondParametersCurrentControl_file);
  joint->storeConfigurationParameterPermanent(PParameterSecondParametersCurrentControl_Parameter);

  IParameterSecondParametersCurrentControl_Parameter.setParameter(IParameterSecondParametersCurrentControl_file);
  joint->storeConfigurationParameterPermanent(IParameterSecondParametersCurrentControl_Parameter);

  DParameterSecondParametersCurrentControl_Parameter.setParameter(DParameterSecondParametersCurrentControl_file);

  joint->storeConfigurationParameterPermanent(DParameterSecondParametersCurrentControl_Parameter);

  IClippingParameterSecondParametersCurrentControl_Parameter.setParameter(IClippingParameterSecondParametersCurrentControl_file);
  joint->storeConfigurationParameterPermanent(IClippingParameterSecondParametersCurrentControl_Parameter);


  std::cout << "Parameters stored!" << std::endl;
}

void JointConfigurator::storeProtectedParametersToJoint() {

  if (!ProtectedParameterRead) {
    std::cout << "The protected Joint Parameters have to been read before hand!" << std::endl;
    return;
  }
  try {

    PWMLimit_Parameter.setParameter(PWMLimit_file);
    joint->storeConfigurationParameterPermanent(PWMLimit_Parameter);

    MaximumMotorCurrent_Parameter.setParameter(MaximumMotorCurrent_file);
    joint->storeConfigurationParameterPermanent(MaximumMotorCurrent_Parameter);

    MaximumVelocityToSetPosition_Parameter.setParameter(MaximumVelocityToSetPosition_file);
    joint->storeConfigurationParameterPermanent(MaximumVelocityToSetPosition_Parameter);

    ClearTargetDistance_Parameter.setParameter(ClearTargetDistance_file);
    joint->storeConfigurationParameterPermanent(ClearTargetDistance_Parameter);

    PositionTargetReachedDistance_Parameter.setParameter(PositionTargetReachedDistance_file);
    joint->storeConfigurationParameterPermanent(PositionTargetReachedDistance_Parameter);

    PIDControlTime_Parameter.setParameter(PIDControlTime_file);
    joint->storeConfigurationParameterPermanent(PIDControlTime_Parameter);

    CurrentControlLoopDelay_Parameter.setParameter(CurrentControlLoopDelay_file);
    joint->storeConfigurationParameterPermanent(CurrentControlLoopDelay_Parameter);

    PWMHysteresis_Parameter.setParameter(PWMHysteresis_file);
    joint->storeConfigurationParameterPermanent(PWMHysteresis_Parameter);

    ClearISumIfPWMReachesMaximum_Parameter.setParameter(ClearISumIfPWMReachesMaximum_file);
    joint->storeConfigurationParameterPermanent(ClearISumIfPWMReachesMaximum_Parameter);

    SetEncoderCounterZeroAtNextNChannel_Parameter.setParameter(SetEncoderCounterZeroAtNextNChannel_file);
    joint->storeConfigurationParameterPermanent(SetEncoderCounterZeroAtNextNChannel_Parameter);

    SetEncoderCounterZeroAtNextSwitch_Parameter.setParameter(SetEncoderCounterZeroAtNextSwitch_file);
    joint->storeConfigurationParameterPermanent(SetEncoderCounterZeroAtNextSwitch_Parameter);

    SetEncoderCounterZeroOnlyOnce_Parameter.setParameter(SetEncoderCounterZeroOnlyOnce_file);
    joint->storeConfigurationParameterPermanent(SetEncoderCounterZeroOnlyOnce_Parameter);

    EncoderStopSwitch_Parameter.setParameter(EncoderStopSwitch_file);
    joint->storeConfigurationParameterPermanent(EncoderStopSwitch_Parameter);

    StopSwitchPolarity_Parameter.setParameter(StopSwitchPolarity_file);
    joint->storeConfigurationParameterPermanent(StopSwitchPolarity_Parameter);

    CommutationMotorCurrent_Parameter.setParameter(CommutationMotorCurrent_file);
    joint->storeConfigurationParameterPermanent(CommutationMotorCurrent_Parameter);

    MassInertiaConstant_Parameter.setParameter(MassInertiaConstant_file);
    joint->storeConfigurationParameterPermanent(MassInertiaConstant_Parameter);

    BEMFConstant_Parameter.setParameter(BEMFConstant_file);
    joint->storeConfigurationParameterPermanent(BEMFConstant_Parameter);

    SineInitializationVelocity_Parameter.setParameter(SineInitializationVelocity_file);
    joint->storeConfigurationParameterPermanent(SineInitializationVelocity_Parameter);

    CommutationCompensationClockwise_Parameter.setParameter(CommutationCompensationClockwise_file);
    joint->storeConfigurationParameterPermanent(CommutationCompensationClockwise_Parameter);

    CommutationCompensationCounterClockwise_Parameter.setParameter(CommutationCompensationCounterClockwise_file);
    joint->storeConfigurationParameterPermanent(CommutationCompensationCounterClockwise_Parameter);

    InitSineDelay_Parameter.setParameter(InitSineDelay_file);
    joint->storeConfigurationParameterPermanent(InitSineDelay_Parameter);

    ActivateOvervoltageProtection_Parameter.setParameter(ActivateOvervoltageProtection_file);
    joint->storeConfigurationParameterPermanent(ActivateOvervoltageProtection_Parameter);

    MaximumPWMChangePerPIDInterval_Parameter.setParameter(MaximumPWMChangePerPIDInterval_file);
    joint->storeConfigurationParameterPermanent(MaximumPWMChangePerPIDInterval_Parameter);

    SineCompensationFactor_Parameter.setParameter(SineCompensationFactor_file);
    joint->storeConfigurationParameterPermanent(SineCompensationFactor_Parameter);

    EncoderNullPolarity_Parameter.setParameter(EncoderNullPolarity_file);
    joint->storeConfigurationParameterPermanent(EncoderNullPolarity_Parameter);

    MotorContollerGearRatio_Parameter.setParameter(MotorContollerGearRatio_file);
    joint->storeConfigurationParameterPermanent(MotorContollerGearRatio_Parameter);

    CommutationMode_Parameter.setParameter(CommutationMode_file);
    joint->storeConfigurationParameterPermanent(CommutationMode_Parameter);

    EncoderResolution_Parameter.setParameter(EncoderResolution_file);
    joint->storeConfigurationParameterPermanent(EncoderResolution_Parameter);

    HallSensorPolarityReversal_Parameter.setParameter(HallSensorPolarityReversal_file);
    joint->storeConfigurationParameterPermanent(HallSensorPolarityReversal_Parameter);

    InitializationMode_Parameter.setParameter(InitializationMode_file);
    joint->storeConfigurationParameterPermanent(InitializationMode_Parameter);

    MotorCoilResistance_Parameter.setParameter(MotorCoilResistance_file);
    joint->storeConfigurationParameterPermanent(MotorCoilResistance_Parameter);

    MotorPoles_Parameter.setParameter(MotorPoles_file);
    joint->storeConfigurationParameterPermanent(MotorPoles_Parameter);

    PIDControllerState_Parameter.setParameter(PIDControllerState_file);
    joint->storeConfigurationParameterPermanent(PIDControllerState_Parameter);

    PWMSchemeBlockCommutation_Parameter.setParameter(PWMSchemeBlockCommutation_file);
    joint->storeConfigurationParameterPermanent(PWMSchemeBlockCommutation_Parameter);

    ReversingEncoderDirection_Parameter.setParameter(ReversingEncoderDirection_file);
    joint->storeConfigurationParameterPermanent(ReversingEncoderDirection_Parameter);

    ThermalWindingTimeConstant_Parameter.setParameter(ThermalWindingTimeConstant_file);
    joint->storeConfigurationParameterPermanent(ThermalWindingTimeConstant_Parameter);

    I2tLimit_Parameter.setParameter(I2tLimit_file);
    joint->storeConfigurationParameterPermanent(I2tLimit_Parameter);
    
    std::cout << "Protected Parameters stored!" << std::endl;
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

void JointConfigurator::menu() {

  if (ArmOrBase == ARM) {
    std::cout << std::endl << "Manipulator Joint " << jointNumber << " | " << jointName  << std::endl;
    std::cout << "Controller Type: " << controllerType << " Firmware version: " << version << std::endl;
  } else if (ArmOrBase == BASE) {
    std::cout << std::endl << "Base Joint " << jointNumber << " | " << jointName  << std::endl;
    std::cout << "Controller Type: " << controllerType << " Firmware version: " << version << std::endl;
  }

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
      std::cout << "         sudo ./JointConfigurator arm 1 joint-parameter.cfg" << std::endl;
      return 0;
    }

    int jointNo = 0;
    jointNo = atoi(argv[2]);

    std::string configfile = "joint-parameter.cfg";
    std::string configfileProtected = "protected-joint-parameter.cfg";

    if (argc >= 4) {
      configfile = argv[3];
    }

    if (argc >= 5) {
      configfileProtected = argv[4];
    }

    JointConfigurator helper(baseOrArm, jointNo, configfile, configfileProtected);


    char ch = 'x';

    helper.menu();

    while (ch != '0') {

      ch = cin.get();
      switch (ch) {
        case '1':
          helper.readParameters();
          helper.menu();
          break;
        case '2':
          helper.setParametersToJoint();
          helper.menu();
          break;
        case '3':
          helper.storeParametersToJoint();
          helper.menu();
          break;
        case '4':
          helper.readPasswordProtectedParameters();
          helper.menu();
          break;
        case '5':
          helper.getPassword();
          helper.menu();
          break;
        case '6':
          helper.setProtectedParametersToJoint();
          helper.menu();
          break;
        case '7':
          helper.storeProtectedParametersToJoint();
          helper.menu();
          break;
        case '8':
          helper.readReadOnlyParameters();
          helper.menu();
          break;
        default:
          break;

      }
    }


  } catch (std::exception& e) {
    std::cout << e.what() << std::endl;
  } catch (...) {
    std::cout << "unhandled exception" << std::endl;
  }


  return 0;
}
