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

package org.austincans.frc2158.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import org.austincans.frc2158.math.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public class HoodIOInputs {
    public double hoodAngleDeg;
    public double temperature;
    public double motorPosition;
    public double hoodRadsPerSec;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public Angle getAngle();

  public void setAngle(Angle setPointDegrees);

  default void runHoodMotor(double speed) {}

  default void applyVolts(double volts) {}

  public void setAngleRotation2d(Rotation2d angle);

  public Rotation2d getAngleRotation2d();

  public boolean atAngle(Angle setPoint);
}
