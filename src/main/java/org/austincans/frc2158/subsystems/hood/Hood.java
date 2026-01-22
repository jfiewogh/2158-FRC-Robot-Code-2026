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
// Updated on Thursday January 22, 2026 at 17:29:37 CST

package org.austincans.frc2158.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.austincans.frc2158.math.Angle;
import org.austincans.frc2158.subsystems.selfcheck.AdvancedSubsystem;
import org.littletonrobotics.junction.Logger;

public class Hood extends AdvancedSubsystem {
  private HoodIO io;
  private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private Angle angle;

  public Hood(HoodIO io) {
    this.io = io;
  }

  Command runHoodMotorCommand(Angle angle) {
    return Commands.run((() -> io.setAngle(angle)));
  }

  public void setAngle(Rotation2d angle) {
    io.setAngleRotation2d(angle);
  }

  public void setAngle(Angle angle) {
    io.setAngle(angle);
  }

  public Angle getAngle() {
    return io.getAngle();
  }

  public void runHoodVoltage(double volts) {
    io.applyVolts(volts);
  }

  public Command stopCommand() {
    return new InstantCommand(
        () -> {
          io.runHoodMotor(0);
        });
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }

  public Command systemCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  clearFaults();
                  io.applyVolts(0);
                  io.setAngle(Angle.fromDegrees(10));
                },
                this),
            Commands.waitSeconds(0.1),
            Commands.runOnce(
                () -> {
                  if (inputs.hoodRadsPerSec < 1) {
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
