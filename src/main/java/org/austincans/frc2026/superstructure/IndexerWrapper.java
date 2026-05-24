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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;
import org.austincans.frc2026.subsystems.feeder.Indexer;
import org.austincans.lib.superstructure.SubsystemWrapper;

public class IndexerWrapper extends SubsystemWrapper<Indexer> {
  public IndexerWrapper(Optional<Indexer> subsystem) {
    super(subsystem);
  }

  public Command runCommand() {
    return isPresent ? Commands.runOnce(s::run, s) : Commands.none();
  }

  public Command stopCommand() {
    return isPresent ? Commands.runOnce(s::stop, s) : Commands.none();
  }

  public boolean atSetpoint() {
    return isPresent && s.atSetpoint();
  }

  public void run() {
    if (isPresent) {
      s.run();
    }
  }

  public void stop() {
    if (isPresent) {
      s.stop();
    }
  }
}
