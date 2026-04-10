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

package org.austincans.frc2026.subsystems.superstructure;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.austincans.frc2026.subsystems.feeder.Indexer;
import org.austincans.frc2026.subsystems.feeder.Spindexer;
import org.austincans.frc2026.subsystems.flywheel.Flywheel;

/** Commands that require more than one subsystem */
public class SuperstructureCommands {
  private SuperstructureCommands() {}

  public static void runFeederAndFlywheel(
      Flywheel flywheel, Indexer indexer, Spindexer spindexer, double flywheelVelocityRPM) {
    flywheel.setVelocity(RPM.of(flywheelVelocityRPM));
    indexer.run();
    if (flywheel.atSetpoint() && indexer.atSetpoint()) {
      spindexer.run();
    }
  }

  public static void stopFeederAndFlywheel(
      Flywheel flywheel, Indexer indexer, Spindexer spindexer) {
    flywheel.stopShooter();
    indexer.stop();
    spindexer.stop();
  }

  public static Command runFeederAndFlywheelCommand(
      Optional<Flywheel> flywheel,
      Optional<Indexer> indexer,
      Optional<Spindexer> spindexer,
      DoubleSupplier flywheelVelocityRPM) {
    if (flywheel.isEmpty() || indexer.isEmpty() || spindexer.isEmpty()) {
      return Commands.none();
    }
    Flywheel f = flywheel.get();
    Indexer i = indexer.get();
    Spindexer s = spindexer.get();
    return Commands.run(
            () -> runFeederAndFlywheel(f, i, s, flywheelVelocityRPM.getAsDouble()), f, i, s)
        .finallyDo(() -> stopFeederAndFlywheel(f, i, s));
  }
}
