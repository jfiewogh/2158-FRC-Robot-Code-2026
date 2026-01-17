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
// Updated on Friday January 16, 2026 at 12:36:02 CST

package org.austincans.frc2158.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import org.austincans.frc2158.subsystems.selfcheck.SubsystemFault;
import org.austincans.frc2158.util.ConfigUtil;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  class TurretIOInputs {
    public double positionDeg = 0.0;
    public double velocityDegPerSec = 0.0;
    public double voltage = 0.0;
    public double currentAmps = 0.0;
    public double temperatureC = 0.0;
  }

  /*
   * PID
   * kP: Proportional
   * kI: Integral
   * kD: Derivative
   */
  public double kP = ConfigUtil.getConfigDouble("turretkP", 0);
  public double kI = ConfigUtil.getConfigDouble("turretkI", 0);
  public double kD = ConfigUtil.getConfigDouble("turretkD", 0);
  public double kV = ConfigUtil.getConfigDouble("turretkV", 0);
  public double kS = ConfigUtil.getConfigDouble("turretkS", 0);
  public double kA = ConfigUtil.getConfigDouble("turretkA", 0);

  public void setVoltage(double voltage);

  public void setDesiredAngle(Rotation2d angle);

  public void setVelocity(double degPerSec);

  public void updateInputs(TurretIOInputs inputs);

  public default List<SubsystemFault> checkForFaults() {
    return null;
  }
}
