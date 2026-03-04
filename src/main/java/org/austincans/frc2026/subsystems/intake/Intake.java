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

package org.austincans.frc2026.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.austincans.frc2026.controllers.LaserCAN;
import org.austincans.frc2026.math.Angle;
import org.austincans.frc2026.subsystems.selfcheck.AdvancedSubsystem;
import org.littletonrobotics.junction.Logger;

public class Intake extends AdvancedSubsystem {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final LaserCAN laserCAN = new LaserCAN(IntakeConstants.laserCANID);

  public Intake(IntakeIO io) {
    this.io = io;
    io.getHardware().forEach(e -> registerHardware(e));
    registerCheckCommand(systemCheckCommand());
  }

  public boolean distanceSensorActive() {
    return laserCAN.getMeasurement().distance_mm < IntakeConstants.maxDistFromIntake;
  }

  public void setRelativeVelocity(double velocity) {
    io.setRelativeVelocity(velocity);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void setDeployAngle(Angle angle) {
    io.setDeployAngle(angle);
  }

  public double getVelocityRadPerSec() {
    return inputs.intakeVelocityRadPerSec;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    io.checkForFaults().forEach(this::addHardwareFault);
  }

  public Command systemCheckCommand() {
    return Commands.runOnce(() -> {});
  }

  public Command deployCommand() {
    return Commands.runOnce(() -> {
      io.deploy();
    },this);
  }

  public Command retractCommand() {
    return Commands.runOnce(() -> {
      io.retract();
    },this);
  }
}
