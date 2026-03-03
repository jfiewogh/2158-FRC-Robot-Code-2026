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

package org.austincans.frc2158.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.austincans.frc2158.math.Angle;

public class HoodIOSim implements HoodIO {
  private final DCMotor motorBase = DCMotor.getKrakenX60(1);
  private DCMotorSim motor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              motorBase, HoodConstants.rotationalInertiaMetersSq, HoodConstants.gearing),
          motorBase);

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.hoodAngleDeg = getAngle().toDegrees(0);
    inputs.temperatureCelsius = 0;
    inputs.motorPositionRad = getAngle().toRadians();
    inputs.velocityRadPerSec = motor.getAngularVelocityRadPerSec();
  }

  @Override
  public void setAngle(Angle angle) {
    motor.setAngle(Units.rotationsToRadians(angle.toRotations()));
  }

  @Override
  public void setAngleRotation2d(Rotation2d angle) {
    motor.setAngle(angle.getRadians());
  }

  @Override
  public Angle getAngle() {
    return Angle.fromRotations(motor.getAngularPositionRotations());
  }

  @Override
  public Rotation2d getAngleRotation2d() {
    return Rotation2d.fromRadians(motor.getAngularPositionRad());
  }

  @Override
  public boolean atAngle(Angle setpoint) {
    return setpoint.toRadians() == motor.getAngularPositionRad();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setInputVoltage(voltage);
  }
}
