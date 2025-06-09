// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

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
