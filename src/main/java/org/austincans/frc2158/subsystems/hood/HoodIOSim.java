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
// Updated on Thursday January 29, 2026 at 19:31:22 CST

package org.austincans.frc2158.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import org.austincans.frc2158.math.Angle;

/** Add your docs here. */
public class HoodIOSim implements HoodIO {
  public HoodIOSim() {}

  Angle angle = Angle.fromDegrees(0);

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.hoodAngleDeg = angle.getRotation2d().getDegrees();
  }

  @Override
  public Angle getAngle() {
    return angle;
  }

  @Override
  public void setAngle(Angle setPointDegrees) {
    angle = setPointDegrees;
  }

  @Override
  public void setAngleRotation2d(Rotation2d setPoint) {
    angle = Angle.fromDegrees(setPoint.getDegrees());
  }

  @Override
  public Rotation2d getAngleRotation2d() {
    return Rotation2d.fromDegrees(angle.toDegrees());
  }

  @Override
  public boolean atAngle(Angle setPoint) {
    return true;
  }
}
