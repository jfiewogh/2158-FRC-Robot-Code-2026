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
// Updated on Saturday February 21, 2026 at 16:10:33 CST

package org.ironmaple.simulation.physics.threading;

import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Interface for physics calculators that need to receive swerve module states from the main thread.
 *
 * <p>Implementing this interface allows the {@link PhysicsThread} to automatically update the
 * calculator with the captured state from {@link SimulationInputs}, ensuring thread-safe
 * synchronization.
 */
public interface ThreadedSwerveCalculator extends PhysicsCalculator {
  /**
   * Updates the captured states for the next tick.
   *
   * @param states The swerve module states from the beginning of the frame.
   */
  void setCapturedStates(SwerveModuleState[] states);
}
