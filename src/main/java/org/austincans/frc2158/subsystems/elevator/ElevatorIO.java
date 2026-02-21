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
// Updated on Saturday February 21, 2026 at 16:51:46 CST

package org.austincans.frc2158.subsystems.elevator;

import java.util.List;
import org.austincans.frc2158.subsystems.selfcheck.SelfChecking;
import org.austincans.frc2158.subsystems.selfcheck.SubsystemFault;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public class ElevatorIOInputs {
    boolean connected = false;
    double heightMeters = 0.0;
    double velocityMetersPerSec = 0.0;
    double voltage = 0.0;
    double current = 0.0;
    double temperatureC = 0.0;
  }

  public void updateInputs(ElevatorIOInputs inputs);

  public default void runVoltage(double volts) {}

  public default List<SubsystemFault> checkForFaults() {
    return List.of();
  }

  public default List<SelfChecking> getHardware() {
    return List.of();
  }
}
