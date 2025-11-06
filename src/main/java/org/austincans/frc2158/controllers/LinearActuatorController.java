// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// oooooooooooo ooooooooo.     .oooooo.        ooooooooo.              .o8                     .   
// `888'     `8 `888   `Y88.  d8P'  `Y8b       `888   `Y88.           "888                   .o8   
//  888          888   .d88' 888                888   .d88'  .ooooo.   888oooo.   .ooooo.  .o888oo 
//  888oooo8     888ooo88P'  888                888ooo88P'  d88' `88b  d88' `88b d88' `88b   888   
//  888    "     888`88b.    888                888`88b.    888   888  888   888 888   888   888   
//  888          888  `88b.  `88b    ooo        888  `88b.  888   888  888   888 888   888   888 . 
// o888o        o888o  o888o  `Y8bood8P'       o888o  o888o `Y8bod8P'  `Y8bod8P' `Y8bod8P'   "888" 
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
//
// Updated on Wednesday November 05, 2025 at 17:57:47 CST

package org.austincans.frc2158.controllers;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

// @author Austin Cans Robotics Team 2158
public class LinearActuatorController {
  private static final int linearActuatorDeviceID = 13;
  private static SparkMax m_linearActuatorMotor;
  private static LinearActuatorController m_linearActuatorController = null;
  private SparkMaxConfig linearActuatorMotorConfig;

  //
  public LinearActuatorController() {
    m_linearActuatorController = getInstance();
  }

  //
  public LinearActuatorController getInstance() {
    if (m_linearActuatorController == null) {
      m_linearActuatorMotor = new SparkMax(linearActuatorDeviceID, MotorType.kBrushless);
      linearActuatorMotorConfig = new SparkMaxConfig();
      linearActuatorMotorConfig.softLimit.forwardSoftLimit(38.5).forwardSoftLimitEnabled(true);
      linearActuatorMotorConfig.softLimit.reverseSoftLimit(0.0).reverseSoftLimitEnabled(true);
      m_linearActuatorMotor.configure(
          linearActuatorMotorConfig,
          ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
    }
    return m_linearActuatorController;
  }

  public void goUp() {
    m_linearActuatorMotor.set(0.12);
  }

  public void goDown() {
    m_linearActuatorMotor.set(-0.12);
  }
}
