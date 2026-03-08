// Copyright (c) 2026 the ausTIN CANs
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

package org.austincans.lib.subsystems.pivot;

import java.util.List;

import org.austincans.lib.configs.ArmConfig;
import org.austincans.lib.encoders.AbsoluteEncoderFactory;
import org.austincans.lib.encoders.CANcoderIO;
import org.austincans.lib.motors.MotorFactory;
import org.austincans.lib.motors.MotorIOTalonFXS;
import org.austincans.lib.subsystems.selfcheck.HardwareFault;
import org.austincans.lib.subsystems.selfcheck.SelfChecking;
import org.austincans.lib.subsystems.selfcheck.SelfCheckingTalonFXS;

// TODO: have SelfChecking as generic
public class PivotIOTalonFXSCANcoder extends PivotIOReal<MotorIOTalonFXS, CANcoderIO> {
  private SelfCheckingTalonFXS motorSelfCheck;

  public PivotIOTalonFXSCANcoder(PivotConfig armConfig) {
    super(
      MotorFactory.createTalonFXSWithFusedCANcoder(armConfig.motorConfig(), armConfig.absoluteEncoderConfig().deviceId()),
      AbsoluteEncoderFactory.createCANcoder(armConfig.absoluteEncoderConfig()));

    motorSelfCheck = new SelfCheckingTalonFXS("Arm Motor", motorIO.getMotor());

    motorIO.setEncoderPosition(absoluteEncoderIO.getAbsolutePositionRotations());
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    super.updateInputs(inputs);
  }

  @Override
  public void stop() {
    setVoltage(0);
  }

  @Override
  public List<HardwareFault> checkForFaults() {
    List<HardwareFault> faults = motorSelfCheck.checkForFaults();
    if (faults.isEmpty()) {
      return List.of();
    } else {
      return faults;
    }
  }

  public List<SelfChecking> getHardware() {
    return List.of(motorSelfCheck);
  }
}
