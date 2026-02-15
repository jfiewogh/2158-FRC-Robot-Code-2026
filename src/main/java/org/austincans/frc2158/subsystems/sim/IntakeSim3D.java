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
// Updated on Sunday February 15, 2026 at 11:44:15 CST

package org.austincans.frc2158.subsystems.sim;

import org.ironmaple.simulation.IntakeSimulation3D;

public class IntakeSim3D {
  private final IntakeSimulation3D intakeSimulation;

  public IntakeSim3D(IntakeSimulation3D intakeSimulation) {
    this.intakeSimulation = intakeSimulation;
  }

  public void update(double velocityRadPerSec) {
    if (velocityRadPerSec > 0) {
      intakeSimulation.startIntake();
    } else {
      intakeSimulation.stopIntake();
    }
  }
}
