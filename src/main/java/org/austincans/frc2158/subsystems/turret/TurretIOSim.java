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

package org.austincans.frc2158.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.austincans.frc2158.Constants;

public class TurretIOSim implements TurretIO {
  private static final boolean inputVoltage = false; 

  private final DCMotor motorBase = DCMotor.getKrakenX60(1);
  private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              motorBase, TurretConstants.rotationalInertiaMetersSq, TurretConstants.gearing),
          motorBase);

  private double appliedVoltage = 0;

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    if (inputVoltage) {
      sim.setInputVoltage(appliedVoltage);
    }
    sim.update(Constants.loopPeriodSecs);

    inputs.positionRad = sim.getAngularPositionRad();
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.voltage = appliedVoltage;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double voltage) {
    appliedVoltage = voltage;
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    sim.setAngularVelocity(velocityRadPerSec);
  }

  @Override
  public void setDesiredAngle(Rotation2d angle) {
    sim.setAngle(angle.getRadians());
  }
}
