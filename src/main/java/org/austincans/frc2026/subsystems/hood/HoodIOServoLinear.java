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

package org.austincans.frc2026.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import org.austincans.lib.controllers.LinearActuatorController;
import org.austincans.lib.math.Angle;

public class HoodIOServoLinear implements HoodIO {
  LinearActuatorController linearActuator;
  InterpolatingDoubleTreeMap angles = new InterpolatingDoubleTreeMap();
  Angle currentAngle = Angle.fromDegrees(0);

  public HoodIOServoLinear() {
    this.linearActuator = new LinearActuatorController(HoodConstants.linearActuatorID);
    angles.put(0.0, 0.0); // (Angles, Position)  Angles [0, 90], Position [0, 1]

    // if (HoodConstants.potentiometer == true);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.hoodAngleDeg = currentAngle.getRotation2d().getDegrees();
  }

  @Override
  public Angle getAngle() {
    return currentAngle;
  }

  @Override
  public void setAngle(Angle setPoint) {
    linearActuator.setSpeed(angles.get(setPoint.toDegrees()));
    currentAngle = Angle.fromDegrees(setPoint.toDegrees());
  }

  @Override
  public void setAngleRotation2d(Rotation2d angle) {
    linearActuator.setSpeed(angles.get(angle.getDegrees()));
    currentAngle = Angle.fromDegrees(angle.getDegrees());
  }

  @Override
  public Rotation2d getAngleRotation2d() {
    return Rotation2d.fromDegrees(currentAngle.toDegrees());
  }

  @Override
  public boolean atAngle(Angle setpoint) {
    return currentAngle.toDegrees() == setpoint.toDegrees();
  }

  @Override
  public void setVoltage(double voltage) {}
}
