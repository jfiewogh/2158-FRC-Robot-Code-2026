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

package org.ironmaple.simulation.gamepieces;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 *
 *
 * <h1>Interface for all Game pieces.</h1>
 *
 * <p>This class contains basic functions all game pieces will have so that game pieces of different
 * types can be used interchangeably in collision detection.
 */
public interface GamePiece {

  /**
   *
   *
   * <h2>Gives the pose3d of a game piece.</h2>
   *
   * @return The pose of this piece as a Pose3d.
   */
  Pose3d getPose3d();

  /**
   *
   *
   * <h2>Gives the string type of the current game piece.</h2>
   *
   * @return The game piece string type (ex "Coral", "Algae", "Note").
   */
  String getType();

  /**
   *
   *
   * <h2>Gives the velocity of the game piece.</h2>
   *
   * For grounded game pieces the z access velocity does not exist and so will be set to 0
   * automatically.
   *
   * @return The velocity of the game piece as a Translation3d.
   */
  Translation3d getVelocity3dMPS();

  /**
   *
   *
   * <h2>Internal function called to trigger a callback whenever this game piece is scored in a goal
   * </h2>
   */
  public void triggerHitTargeCallBack();

  /**
   *
   *
   * <h2>Gives wether or not the piece is "grounded".</h2>
   *
   * A grounded piece is likely a child of {@link GamePieceOnFieldSimulation} while a non grounded
   * piece is likely a child of{@link GamePieceProjectile}.
   *
   * @return wether or not the piece is grounded.
   */
  boolean isGrounded();
}
