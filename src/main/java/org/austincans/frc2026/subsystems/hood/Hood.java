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

package org.austincans.frc2026.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.austincans.frc2026.subsystems.hood.HoodIO.HoodIOOutputs;
import org.austincans.frc2026.subsystems.selfcheck.AdvancedSubsystem;

/** This is specifically for the 2026 robot */
public class Hood extends AdvancedSubsystem {
  public final boolean tuningModeEnabled = false;

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final HoodIOOutputs outputs = new HoodIOOutputs();

  public Hood(HoodIO io) {
    this.io = io;
    io.registerSelfCheckHardware(this);
    registerCheckCommand(systemCheckCommand());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // Logger.processInputs("Hood/Inputs", inputs);
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  public void setPulse(int pulseWidth) {
    outputs.pulseWidth = pulseWidth;
  }

  public void addToPulse(int addend) {
    setPulse(inputs.pulseWidth + addend);
  }

  public void setAngle(Rotation2d angle) {
    setPulse(HoodUtil.angleToPWM(angle));
  }

  /** This should only be used for testing */
  public Command setPulseCommand(int pulseWidth) {
    return Commands.runOnce(() -> setPulse(pulseWidth), this);
  }

  public Command addToPulseCommand(int addend) {
    return Commands.runOnce(() -> addToPulse(addend), this);
  }

  public Command stopCommand() {
    return Commands.runOnce(() -> setPulse(inputs.pulseWidth), this);
  }

  public Command systemCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  clearFaults();
                  setAngle(Rotation2d.fromDegrees(10));
                },
                this),
            Commands.waitSeconds(0.1),
            Commands.run(
                    () -> {
                      setAngle(Rotation2d.fromDegrees(20));
                    },
                    this)
                .withTimeout(1.0),
            Commands.runOnce(
                () -> {
                  if (inputs.angleDeg < 22 || inputs.angleDeg > 18) {
                    addSubsystemFault(
                        "HOOD", "[System Check] Hood did not reach target position", false);
                  }
                },
                this),
            Commands.waitSeconds(0.25))
        .andThen(Commands.runOnce(this::stopCommand, this));
  }
}
