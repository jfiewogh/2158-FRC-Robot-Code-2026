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
// Updated on Tuesday January 27, 2026 at 19:20:07 CST

package org.ironmaple.simulation.physics.threading;

import org.ironmaple.simulation.physics.PhysicsEngine;

/**
 * Interface for physics calculations that run on the physics thread.
 *
 * <p>Implementations of this interface are registered with {@link PhysicsThread} and have their
 * {@link #applyForces(PhysicsEngine)} method called each tick BEFORE the physics step.
 *
 * <p>This ensures that force calculations use the CURRENT physics state, not stale cached state,
 * eliminating feedback delay that causes oscillation.
 */
public interface PhysicsCalculator {

  /**
   * Apply forces to physics bodies.
   *
   * <p>Called on the physics thread each tick, immediately before {@code engine.step()}.
   * Implementations should:
   *
   * <ol>
   *   <li>Query current state directly from physics bodies
   *   <li>Calculate forces based on that current state
   *   <li>Apply forces to bodies
   * </ol>
   *
   * @param engine The physics engine (for raycasting, etc.)
   */
  void applyForces(PhysicsEngine engine);
}
