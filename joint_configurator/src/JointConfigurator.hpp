#ifndef JOINTCONFIGURATOR_H
#define JOINTCONFIGURATOR_H
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

  JointConfigurator(YouBotJoint* youbotjoint, std::string configpath, std::string configname, std::string configNameProtected);

  ~JointConfigurator();
  void readParameters();
  void readPasswordProtectedParameters();
  void readReadOnlyParameters();
  void setParametersToJoint();
  void storeParametersToJoint();
  void setProtectedParametersToJoint();
  void storeProtectedParametersToJoint();
  void getPassword();

private:

  bool AreSame(double A, double B);
  YouBotJoint* joint;
  ConfigFile* configfile;
  ConfigFile* configfilePP;
  bool ParameterRead;
  bool ProtectedParameterRead;
  bool UseProtectedParameter;
  bool UseParameter;
  int controllerType;
  std::string version;
  std::string jointName;

  MaximumPositioningVelocity MaximumPositioningVelocity_Parameter;
  quantity<angular_velocity> MaximumPositioningVelocity_actual;
  quantity<angular_velocity> MaximumPositioningVelocity_file;

  MaximumMotorCurrent MaximumMotorCurrent_Parameter;
  quantity<current> MaximumMotorCurrent_actual;
  quantity<current> MaximumMotorCurrent_file;

  MaximumVelocityToSetPosition MaximumVelocityToSetPosition_Parameter;
  quantity<angular_velocity> MaximumVelocityToSetPosition_actual;
  quantity<angular_velocity> MaximumVelocityToSetPosition_file;

  SpeedControlSwitchingThreshold SpeedControlSwitchingThreshold_Parameter;
  quantity<angular_velocity> SpeedControlSwitchingThreshold_actual;
  quantity<angular_velocity> SpeedControlSwitchingThreshold_file;

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

  PParameterCurrentControl PParameterCurrentControl_Parameter;
  int PParameterCurrentControl_actual;
  int PParameterCurrentControl_file;

  IParameterCurrentControl IParameterCurrentControl_Parameter;
  int IParameterCurrentControl_actual;
  int IParameterCurrentControl_file;

  DParameterCurrentControl DParameterCurrentControl_Parameter;
  int DParameterCurrentControl_actual;
  int DParameterCurrentControl_file;

  IClippingParameterCurrentControl IClippingParameterCurrentControl_Parameter;
  int IClippingParameterCurrentControl_actual;
  int IClippingParameterCurrentControl_file;

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

  InitSineDelay InitSineDelay_Parameter;
  quantity<si::time> InitSineDelay_actual;
  quantity<si::time> InitSineDelay_file;

  ActivateOvervoltageProtection ActivateOvervoltageProtection_Parameter;
  bool ActivateOvervoltageProtection_actual;
  bool ActivateOvervoltageProtection_file;

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
  
  MotorHaltedVelocity MotorHaltedVelocity_Parameter;
  int MotorHaltedVelocity_actual;
  int MotorHaltedVelocity_file;

  ActualMotorVoltage ActualMotorVoltage_Parameter;
  quantity<electric_potential> ActualMotorVoltage_actual;
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
  ActualMotorDriverTemperature ActualMotorDriverTemperature_Parameter;
  quantity<celsius::temperature> ActualMotorDriverTemperature_actual;
  
  ActualModuleSupplyCurrent ActualModuleSupplyCurrent_Parameter;
  quantity<si::current> ActualModuleSupplyCurrent_actual;
  
  MotorControllerTimeout MotorControllerTimeout_Parameter;
  quantity<si::time> MotorControllerTimeout_actual;
  quantity<si::time> MotorControllerTimeout_file;
  
  CurrentError CurrentError_Parameter;
  quantity<si::current> CurrentError_actual;
          
  CurrentErrorSum CurrentErrorSum_Parameter;
  quantity<si::current> CurrentErrorSum_actual;
  
  quantity<si::angular_velocity> VelocityThresholdForHallFX_actual;
  quantity<si::angular_velocity> VelocityThresholdForHallFX_file;
  VelocityThresholdForHallFX VelocityThresholdForHallFX_Parameter;

};


#endif