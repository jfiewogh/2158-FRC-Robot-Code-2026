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

package org.austincans.frc2026.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;

import org.austincans.frc2026.subsystems.selfcheck.AdvancedSubsystem;
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
  }

  public static enum ModuleIOOutputMode {
    COAST,
    BRAKE,
    DRIVE,
    CHARACTERIZE
  }

  public static class ModuleIOOutputs {
    public ModuleIOOutputMode mode = ModuleIOOutputMode.COAST;
    public double driveVelocityRadPerSec = 0.0;
    public double driveFeedforward = 0.0;
    public double driveCharacterizationOutput = 0.0;
    public Rotation2d turnRotation = Rotation2d.kZero;
    public boolean turnNeutral = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  public default void applyOutputs(ModuleIOOutputs outputs) {}

  public void registerSelfCheckHardware(AdvancedSubsystem subsystem);
}