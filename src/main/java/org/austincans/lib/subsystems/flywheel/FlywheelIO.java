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

package org.austincans.lib.subsystems.flywheel;

import org.austincans.lib.motors.MotorIO.MotorIOInputs;
import org.austincans.lib.motors.MotorIO.MotorIOOutputs;
import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;

public interface FlywheelIO {
  public class FlywheelIOInputs {
    public MotorIOInputs motorInputs; 
  }

  public class FlywheelIOOutputs {
    public MotorIOOutputs motorOutputs;
  }

  public void updateInputs(FlywheelIOInputs inputs);

  public void applyOutputs(FlywheelIOOutputs outputs);

  public void registerSelfCheckHardware(AdvancedSubsystem subsystem);
}
