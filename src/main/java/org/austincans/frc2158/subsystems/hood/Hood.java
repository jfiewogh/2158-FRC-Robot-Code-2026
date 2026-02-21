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
// Updated on Saturday February 21, 2026 at 16:10:33 CST

package org.austincans.frc2158.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.austincans.frc2158.math.Angle;
import org.austincans.frc2158.subsystems.selfcheck.AdvancedSubsystem;
import org.littletonrobotics.junction.Logger;

public class Hood extends AdvancedSubsystem {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  public Hood(HoodIO io) {
    this.io = io;
    io.getHardware().forEach(e -> registerHardware(e));
  }

  Command runHoodMotorCommand(Angle angle) {
    return Commands.run((() -> io.setAngle(angle)));
  }

  public void setAngle(Rotation2d angle) {
    io.setAngleRotation2d(angle);
  }

  public Rotation2d getAngle() {
    return io.getAngleRotation2d();
  }

  public boolean atAngle(Angle setpoint) {
    return io.atAngle(setpoint);
  }

  @Override
  public void periodic() {
    Logger.processInputs("Hood/Inputs", inputs);
    io.updateInputs(inputs);
  }

  public Command stopCommand() {
    return new InstantCommand(
        () -> {
          io.setAngle(Angle.fromDegrees(0));
        });
  }

  public Command systemCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  clearFaults();
                  io.setAngle(Angle.fromDegrees(10));
                },
                this),
            Commands.waitSeconds(0.1),
            Commands.runOnce(
                () -> {
                  if (inputs.hoodVelocityRadPerSec < 1) {
                    addFault("[System Check] Hood encoder velocity measured too slow", false, true);
                  }
                },
                this),
            Commands.run(
                    () -> {
                      io.setAngle(Angle.fromDegrees(20));
                    },
                    this)
                .withTimeout(1.0),
            Commands.runOnce(
                () -> {
                  if (inputs.hoodAngleDeg < 22 || inputs.hoodAngleDeg > 18) {
                    addFault("[System Check] Hood did not reach target position", false, true);
                  }
                },
                this),
            Commands.waitSeconds(0.25))
        .until(() -> getFaults().size() > 0)
        .andThen(Commands.runOnce(this::stopCommand, this));
  }
}
