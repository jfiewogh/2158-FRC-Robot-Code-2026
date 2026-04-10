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

package org.austincans.frc2026.subsystems.intake.deploy;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.austincans.frc2026.Constants;
import org.austincans.frc2026.subsystems.intake.deploy.DeployIO.DeployIOOutputs;
import org.austincans.frc2026.subsystems.selfcheck.AdvancedSubsystem;

public class Deploy extends AdvancedSubsystem {
  private final DeployIO io;
  private final DeployIOInputsAutoLogged inputs = new DeployIOInputsAutoLogged();
  private final DeployIOOutputs outputs = new DeployIOOutputs();

  public Deploy(DeployIO io) {
    this.io = io;
    io.registerSelfCheckHardware(this);
    registerCheckCommand(systemCheckCommand());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // Logger.processInputs("Intake/Deploy", inputs);
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  public void deploy() {
    outputs.voltage = 0.15 * Constants.maxVoltage;
  }

  public void retract() {
    outputs.voltage = -0.15 * Constants.maxVoltage;
  }

  public void stop() {
    outputs.voltage = 0.0;
  }

  @Override
  protected Command systemCheckCommand() {
    return Commands.none();
  }
}
