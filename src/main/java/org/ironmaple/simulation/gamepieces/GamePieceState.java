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
// Updated on Sunday February 22, 2026 at 14:08:53 CST

package org.ironmaple.simulation.gamepieces;

/**
 *
 *
 * <h2>Represents the state of a game piece in the simulation.</h2>
 */
public enum GamePieceState {
  /** Game piece is on the field (physics-simulated). */
  ON_FIELD,
  /** Game piece is held in an intake mechanism. */
  IN_INTAKE,
  /** Game piece is loaded in a shooter mechanism. */
  IN_SHOOTER,
  /** Game piece is in flight (projectile). */
  IN_FLIGHT,
  /** Game piece has been scored in a goal. */
  IN_GOAL
}
