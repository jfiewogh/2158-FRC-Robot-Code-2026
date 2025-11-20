// Copyright (c) 2025 the ausTIN CANs
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
// Updated on Thursday November 20, 2025 at 12:24:24 CST

package org.austincans.frc2158.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import org.austincans.frc2158.controllers.LaserCAN;
import org.austincans.frc2158.subsystems.selfcheck.AdvancedSubsystem;
import org.austincans.frc2158.util.ConfigUtil;

public class Intake extends AdvancedSubsystem {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final LaserCAN laserCAN = new LaserCAN(ConfigUtil.getConfigInt("intakeLaserCANID", 0));

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void setRelativeVelocity(double velocity) {
    io.setRelativeVelocity(velocity);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.checkForFaults().forEach(this::addFault);
  }

  public Command systemCheckCommand() {
    return null;
  }
}
