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

package org.austincans.lib.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;
import org.littletonrobotics.junction.Logger;

public class ArmBase extends AdvancedSubsystem {
  private final ArmIO armIO;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  
  protected ArmBase(ArmIO armIO) {
    this.armIO = armIO;
    armIO.getHardware().forEach(e -> registerHardware(e));
    registerCheckCommand(systemCheckCommand());
  }

  @Override
  public void periodic() {
    armIO.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    
    armIO.checkForFaults().forEach(this::addHardwareFault);
  }

  public void setAngle(Rotation2d angle) {
    armIO.setPosition(angle.getRadians());
  }

  public double getPositionRad() {
    return inputs.position;
  }

  public double getVelocityRadPerSec() {
    return inputs.velocity;
  }

  public Command setAngleCommand(Rotation2d rotation) {
    return Commands.runOnce(() -> setAngle(rotation));
  }

  public Command moveToAngleCommand(Rotation2d angle) {
    return Commands.runOnce(() -> armIO.setPosition(angle.getRadians()))
      .until(() -> Math.abs(angle.getRadians() - getPositionRad()) < Units.degreesToRadians(2));
  }

  public Command stopCommand() {
    return new InstantCommand(() -> armIO.setVoltage(0));
  }

  public Command moveAtVelocityCommand(double velocityDegPerSec) {
    return new InstantCommand(() -> armIO.setVoltage(velocityDegPerSec));
  }

  @Override
  protected Command systemCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(() -> armIO.setVelocity(2)),
            Commands.waitSeconds(0.5),
            Commands.runOnce(
                () -> {
                  if (getVelocityRadPerSec() < 0.1) {
                    addSubsystemFault(
                        getName(), "[System Check] Extension velocity measured too slow", false);
                  }
                  armIO.setVelocity(0);
                },
                this),
            Commands.runOnce(() -> setAngle(Rotation2d.fromDegrees(0)), this),
            Commands.waitSeconds(2.0),
            Commands.runOnce(
                () -> {
                  if (getPositionRad() < Units.degreesToRadians(-15) || 
                      getPositionRad() > Units.degreesToRadians(15)) {
                    addSubsystemFault(
                        getName(), "[System Check] Joint did not reach target position", false);
                  }
                },
                this),
            Commands.runOnce(() -> setAngle(Rotation2d.fromDegrees(90)), this),
            Commands.waitSeconds(2.0),
            Commands.runOnce(
                () -> {
                  if (getPositionRad() < Units.degreesToRadians(75) ||
                      getPositionRad() > Units.degreesToRadians(115)) {
                    addSubsystemFault(
                        getName(), "[System Check] Joint did not reach target position", false);
                  }
                },
                this))
        .andThen(
            Commands.runOnce(
                () -> {
                  armIO.setVoltage(0);
                },
                this));
  }
}
