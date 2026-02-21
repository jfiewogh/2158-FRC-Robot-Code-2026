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

package org.ironmaple.simulation.seasonspecific.reefscape2025;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import org.dyn4j.geometry.Rectangle;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

/**
 *
 *
 * <h1>Represents a CORAL in the 2025 Reefscape game.</h1>
 *
 * <p>The Coral (<a href="https://www.andymark.com/products/frc-2024?sku=am-5601">am-5601</a>) is an
 * 11 ⅞-inch (~30 cm) long piece of 4-inch diameter Schedule 40 Cellular (Foam) Core PVC pipe,
 * featured as a game piece in the 2025 Reefscape game.
 *
 * <p>The Coral has a 4-inch (~102 mm) inside diameter and a 4½-inch (~11 cm) outside diameter.
 */
public class ReefscapeCoralOnField extends GamePieceOnFieldSimulation {
  public static final GamePieceInfo REEFSCAPE_CORAL_INFO =
      new GamePieceInfo(
          "Coral", new Rectangle(0.3, 0.11), Meters.of(0.11), Kilograms.of(0.3), 2.8, 4, 0.3);

  public ReefscapeCoralOnField(Pose2d initialPose) {
    super(REEFSCAPE_CORAL_INFO, initialPose);
  }
}
