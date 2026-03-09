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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.austincans.lib.Constants;
import org.austincans.lib.configs.PivotConfig;
import org.austincans.lib.motors.MotorIOFake;

public class PivotIOSim implements PivotIO {
  private final MotorIOFake motorIO;

  private final DCMotor neoVortex = DCMotor.getNeoVortex(1);

  private final SingleJointedArmSim armSim;
  private double armAppliedVolts;

  public PivotIOSim(PivotConfig pivotConfig) {
    motorIO = new MotorIOFake();

    armSim = new SingleJointedArmSim(
          neoVortex,
          pivotConfig.gearRatio(),
          0.001,
          pivotConfig.armLengthMeters(),
          pivotConfig.minAngleRad(),
          pivotConfig.maxAngleRad(),
          true,
          pivotConfig.startingAngleRad());
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    armSim.setInputVoltage(armAppliedVolts);
    armSim.update(Constants.loopPeriodSec);

    motorIO.updateInputs(inputs.motorInputs);
  }

  @Override
  public void setVoltage(double volts) {
    armAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    motorIO.setVoltage(armAppliedVolts);
  }

  @Override
  public void setPosition(double position) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
  }

  @Override
  public void setVelocity(double velocity) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setVelocity'");
  }

  @Override
  public void stop() {
    motorIO.setVoltage(0);
  }
}
