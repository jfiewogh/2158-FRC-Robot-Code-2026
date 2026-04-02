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

package org.austincans.frc2026.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import org.austincans.frc2026.subsystems.selfcheck.HardwareFault;
import org.austincans.frc2026.subsystems.selfcheck.SelfChecking;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  class TurretIOInputs {
    public double positionRad = 0.0; // robot relative
    public double velocityRadPerSec = 0.0; // robot relative
    public double motorPositionRotations = 0.0;
    public double motorVelocityRotationsPerSec = 0.0;
    public double voltage = 0.0;
    public double currentAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public double encoderPosition = 0.0;
  }

  public void setVoltage(double voltage);

  public void setDesiredAngle(Rotation2d angle);

  public void setVelocity(double velocityRadPerSec);

  public void updateInputs(TurretIOInputs inputs);

  public void resetAngle(Rotation2d pose);

  public default List<HardwareFault> checkForFaults() {
    return List.of();
  }

  public default List<SelfChecking> getHardware() {
    return List.of();
  }
}
