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

package org.austincans.lib.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

import org.austincans.lib.math.Angle;
import org.austincans.lib.motors.MotorIO.MotorIOInputs;
import org.austincans.lib.subsystems.selfcheck.HardwareFault;
import org.austincans.lib.subsystems.selfcheck.SelfChecking;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public class HoodIOInputs {
    public MotorIOInputs motorInputs;
    public double hoodAngleRad; //only for servo
  }

  public void updateInputs(HoodIOInputs inputs);

  public default void setVelocity(double velocityRadPerSec) {}

  public void setAngle(Angle angle);

  public void setAngleRotation2d(Rotation2d angle);

  public Angle getAngle();

  public Rotation2d getAngleRotation2d();

  public boolean atAngle(Angle setpoint);

  public void setVoltage(double voltage);

  public default List<HardwareFault> checkForFaults() {
    return List.of();
  }

  public default List<SelfChecking> getHardware() {
    return List.of();
  }
}
