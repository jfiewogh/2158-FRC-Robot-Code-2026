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
// Updated on Thursday February 19, 2026 at 20:08:44 CST

package org.ironmaple.simulation;

import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

/**
 * A context record that groups the primary simulation components (Drive and Arena).
 *
 * <p>This is designed to be passed around to subsystem simulations (IO layers) to avoid static
 * global access (dependency injection pattern).
 */
public record SimulationContext(
    AbstractDriveTrainSimulation driveTrainSimulation, SimulatedArena simulatedArena) {
  /**
   * Creates a new context with the given drive simulation and the singleton arena instance.
   *
   * @param driveTrainSimulation the drive train simulation
   * @return a new SimulationContext
   */
  public static SimulationContext of(AbstractDriveTrainSimulation driveTrainSimulation) {
    return new SimulationContext(driveTrainSimulation, SimulatedArena.getInstance());
  }
}
