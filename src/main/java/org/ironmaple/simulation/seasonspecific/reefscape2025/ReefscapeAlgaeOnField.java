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
// Updated on Sunday February 01, 2026 at 15:05:51 CST

package org.ironmaple.simulation.seasonspecific.reefscape2025;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.dyn4j.geometry.Circle;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

/**
 *
 *
 * <h1>Represents an ALGAE in the 2025 Reefscape game.</h1>
 *
 * <p>The Algae (<a href="https://www.andymark.com/products/frc-2024?sku=am-5602">am-5602</a>) is a
 * 16-inch (41 cm) rubber playground ball with a ±½ inch (~12 mm) diameter, featured as a game piece
 * in the 2025 Reefscape game.
 */
public class ReefscapeAlgaeOnField extends GamePieceOnFieldSimulation {
  public static final GamePieceInfo REEFSCAPE_ALGAE_INFO =
      new GamePieceInfo("Algae", new Circle(0.176), Inches.of(16), Kilograms.of(0.4), 1.8, 5, 0.8);

  public ReefscapeAlgaeOnField(Translation2d initialPosition) {
    super(REEFSCAPE_ALGAE_INFO, new Pose2d(initialPosition, new Rotation2d()));
  }
}
