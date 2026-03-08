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

package org.austincans.frc2026.subsystems.led;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.austincans.frc2026.RobotContainer;
import org.austincans.frc2026.commands.TurretCommands;
import org.austincans.frc2026.subsystems.drive.Drive;
import org.austincans.frc2026.subsystems.flywheel.Flywheel;
import org.austincans.frc2026.subsystems.intake.Intake;
import org.austincans.frc2026.subsystems.led.LedIO.LedState;

public class Led extends SubsystemBase {
  private final LedIO io;
  private final LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();

  public Led(LedIO io) {
    this.io = io;
  }

  public void updateState() {
    if (Flywheel.isReadyToShoot) {
      io.setPattern(LedState.READYTOSHOOT);
    } else if (Flywheel.isShooting) {
      io.setPattern(LedState.SHOOTING);
    } else if (Drive.isDriving) {
      io.setPattern(LedState.DRIVING);
    } else if (Intake.deployed) {
      io.setPattern(LedState.INTAKED);
    } else if (TurretCommands.isShuttling) {
      io.setPattern(LedState.SHUTTLING);
      TurretCommands.isShuttling = false;
    } else {
      io.setPattern(LedState.IDLE);
    }
  }

  @Override
  public void periodic() {
    updateState();
    io.updateInputs(inputs);
  }
}
