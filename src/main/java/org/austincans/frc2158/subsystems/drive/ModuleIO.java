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
// Updated on Wednesday January 28, 2026 at 19:56:47 CST

package org.austincans.frc2158.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import org.austincans.frc2158.subsystems.selfcheck.SubsystemFault;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  class ModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveSupplyCurrentAmps = 0.0;
    public double driveStatorCurrentAmps = 0.0;
    public double driveTorqueCurrentAmps = 0.0;

    public boolean turnConnected = false;
    public boolean turnEncoderConnected = false;
    public Rotation2d turnAbsolutePosition = Rotation2d.kZero;
    public Rotation2d turnPosition = Rotation2d.kZero;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnSupplyCurrentAmps = 0.0;
    public double turnStatorCurrentAmps = 0.0;
    public double turnTorqueCurrentAmps = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified open loop value. */
  default void setDriveOpenLoop(double output) {}

  /** Run the turn motor at the specified open loop value. */
  default void setTurnOpenLoop(double output) {}

  /** Run the drive motor at the specified velocity. */
  default void setDriveVelocity(double velocityRadPerSec) {}

  default void setDriveVelocity(double velocityRadPerSec, double feedforward) {}

  /** Run the turn motor to the specified rotation. */
  default void setTurnPosition(Rotation2d rotation) {}

  /** Set P, I, and D gains for closed loop control on drive motor. */
  public default void setDrivePID(double kP, double kI, double kD) {}

  /** Set P, I, and D gains for closed loop control on turn motor. */
  public default void setTurnPID(double kP, double kI, double kD) {}

  /** Set brake mode on drive motor */
  public default void setBrakeMode(boolean enabled) {}

  default List<SubsystemFault> checkForFaults() {
    return List.of();
  }
}
