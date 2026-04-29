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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.austincans.lib.subsystems.hood.HoodIO.HoodIOInputs;
import org.austincans.lib.subsystems.hood.HoodIO.HoodIOOutputs;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;
import org.littletonrobotics.junction.Logger;

import com.google.protobuf.ByteString.Output;

public class Hood extends AdvancedSubsystem {
  private final HoodIO io;
  private final HoodIOInputs inputs = new HoodIOInputs();
  private final HoodIOOutputs outputs = new HoodIOOutputs();

  public Hood(HoodIO io) {
    this.io = io;
    registerCheckCommand(systemCheckCommand());
  }

  // TODO: Only IO with setVelocity implementation is HoodIOTest
  public void setVelocity(double velocityRadPerSec) {
    outputs.motorOutputs.setVelocity(RadiansPerSecond.of(velocityRadPerSec));
  }

  public void setAngle(Rotation2d angle) {
    outputs.motorOutputs.setPosition(Radians.of(angle.getRadians()));
  }

  public Rotation2d getAngle() {
    return new Rotation2d(inputs.motorInputs.position);
  }

  public boolean atAngle(Angle setpoint) {
    return inputs.motorInputs.position == setpoint;
  }

  @Override
  public void periodic() {
    // Logger.processInputs("Hood/Inputs", inputs);
    io.updateInputs(inputs);
  }

  public Command stopCommand() {
    return new InstantCommand(
        () -> {
          outputs.motorOutputs.setPosition(Degrees.of(0));
        });
  }

  public Command systemCheckCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              clearFaults();
               outputs.motorOutputs.setPosition(Degrees.of(10));
            },
            this),
        Commands.waitSeconds(0.1),
        Commands.runOnce(
            () -> {
              if (inputs.motorInputs.getPosition(Radians) < Math.PI * 2) {
                addSubsystemFault(
                    "HOOD", "[System Check] Hood encoder velocity measured too slow", false);
              }
            },
            this),
        Commands.run(
            () -> {
               outputs.motorOutputs.setPosition(Degrees.of(20));
            },
            this)
            .withTimeout(1.0),
        Commands.runOnce(
            () -> {
              if (inputs.motorInputs.getPosition(Degrees) < 22 || inputs.motorInputs.getPosition(Degrees) > 18) {
                addSubsystemFault(
                    "HOOD", "[System Check] Hood did not reach target position", false);
              }
            },
            this),
        Commands.waitSeconds(0.25))
        .andThen(Commands.runOnce(this::stopCommand, this));
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }
}
