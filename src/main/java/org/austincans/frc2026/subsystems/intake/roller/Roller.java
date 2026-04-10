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

package org.austincans.frc2026.subsystems.intake.roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.austincans.frc2026.Constants;
import org.austincans.frc2026.Robot;
import org.austincans.frc2026.subsystems.intake.roller.RollerIO.RollerIOOutputs;
import org.austincans.frc2026.subsystems.selfcheck.AdvancedSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;

public class Roller extends AdvancedSubsystem {
  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();
  private final RollerIOOutputs outputs = new RollerIOOutputs();

  /** Should be not running on start */
  @AutoLogOutput public boolean running = false;

  public Roller(RollerIO io) {
    this.io = io;
    io.registerSelfCheckHardware(this);
    registerCheckCommand(systemCheckCommand());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // Logger.processInputs("Intake/Roller", inputs);
    Robot.batteryLogger.reportCurrentUsage(
        "Rollers",
        false,
        inputs.connected ? inputs.supplyCurrentAmps : 0.0,
        inputs.followerConnected ? inputs.followerSupplyCurrentAmps : 0.0);
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  public void runIntake() {
    running = true;
    outputs.voltage = 0.35 * Constants.maxVoltage;
  }

  public void runOuttake() {
    running = true;
    outputs.voltage = -0.35 * Constants.maxVoltage;
  }

  public void stop() {
    running = false;
    outputs.voltage = 0.0;
  }

  public void toggle() {
    if (running) {
      stop();
    } else {
      runIntake();
    }
  }

  @Override
  protected Command systemCheckCommand() {
    return Commands.none();
  }
}
