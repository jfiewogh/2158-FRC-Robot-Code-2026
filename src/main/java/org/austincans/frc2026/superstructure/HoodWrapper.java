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

package org.austincans.frc2026.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;
import org.austincans.frc2026.subsystems.hood.Hood;
import org.austincans.lib.superstructure.SubsystemWrapper;

public class HoodWrapper extends SubsystemWrapper<Hood> {
  public HoodWrapper(Optional<Hood> subsystem) {
    super(subsystem);
  }

  public void setAngle(Rotation2d angle) {
    if (isPresent) {
      s.setAngle(angle);
    }
  }

  public Command setAngleCommand(Rotation2d angle) {
    return isPresent ? Commands.runOnce(() -> s.setAngle(angle), s) : Commands.none();
  }

  public Command setPulseCommand(int pulse) {
    return isPresent ? Commands.runOnce(() -> s.setPulse(pulse), s) : Commands.none();
  }

  public Command setLowestAngleCommand() {
    return setPulseCommand(1000);
  }

  public Command incrementCommand() {
    return isPresent ? s.addToPulseCommand(50) : Commands.none();
  }

  public Command decrementCommand() {
    return isPresent ? s.addToPulseCommand(-50) : Commands.none();
  }
}
