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
//
// Updated on Thursday February 05, 2026 at 19:51:21 CST

package org.austincans.frc2158.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.austincans.frc2158.Constants;

public class ArmIOSim implements ArmIO {
  private final DCMotor neoVortex = DCMotor.getNeoVortex(1);

  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          neoVortex,
          ArmConstants.armGearRatio,
          0.001,
          ArmConstants.armLength,
          0,
          ArmConstants.armMaxRad,
          true,
          ArmConstants.armStartingPosition);

  private double armAppliedVolts;

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    armSim.setInputVoltage(armAppliedVolts);
    armSim.update(Constants.loopPeriodSecs);

    inputs.position = armSim.getAngleRads();
    inputs.velocity = armSim.getVelocityRadPerSec();
    inputs.currentAmps = armSim.getCurrentDrawAmps();
    inputs.currentVoltage = armAppliedVolts;
  }

  @Override
  public void setVoltage(double volts) {
    armAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }
}
