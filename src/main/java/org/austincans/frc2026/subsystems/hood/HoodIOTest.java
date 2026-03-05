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
import org.austincans.frc2026.math.Angle;
import org.austincans.frc2026.simulation.RobotSimulation3D;
import org.ironmaple.simulation.MeshSimulation3D;

public class HoodIOTest implements HoodIO {
  private final RobotSimulation3D robotSimulation;

  private Rotation2d currentAngle = Rotation2d.kZero;
  private double velocityRadPerSec = 0;

  public HoodIOTest(RobotSimulation3D robotSimulation) {
    this.robotSimulation = robotSimulation;
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.hoodAngleDeg = currentAngle.getDegrees();
    inputs.velocityRadPerSec = velocityRadPerSec;
    MeshSimulation3D hood = robotSimulation.getHood();
    if (hood != null) {
      hood.setAngularVelocityRadPerSec(0, inputs.velocityRadPerSec, 0);
    }
  }

  @Override
  public void setAngle(Angle angle) {
    currentAngle = angle.getRotation2d();
  }

  @Override
  public void setAngleRotation2d(Rotation2d angle) {
    currentAngle = angle;
  }

  @Override
  public Angle getAngle() {
    return Angle.fromRotation2d(currentAngle);
  }

  @Override
  public Rotation2d getAngleRotation2d() {
    return currentAngle;
  }

  public boolean atAngle(Angle setpoint) {
    return currentAngle == setpoint.getRotation2d();
  }

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public void setVelocity(double velocityRadPerSec) {
    this.velocityRadPerSec = velocityRadPerSec;
  }
}
