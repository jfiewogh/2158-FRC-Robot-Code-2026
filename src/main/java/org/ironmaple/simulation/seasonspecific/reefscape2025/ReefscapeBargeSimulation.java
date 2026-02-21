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

import static edu.wpi.first.units.Units.Centimeters;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.*;
import org.ironmaple.simulation.Goal;
import org.ironmaple.simulation.gamepieces.GamePiece;

/**
 *
 *
 * <h2>Simulates a <strong>BARGE</strong>s on the field.</h2>
 *
 * <p>This class simulates a <strong>BARGE</strong>s on the field where <strong>ALGAE</strong> can
 * be scored.
 */
public class ReefscapeBargeSimulation extends Goal {

  protected static final Translation3d redBargePose = new Translation3d(8.805, 2.1, 1.57);
  protected static final Translation3d blueBargePose = new Translation3d(8.805, 6.18, 1.57);
  StructPublisher<Pose3d> posePublisher;

  /**
   *
   *
   * <h2>Creates an barge of the specified color.</h2>
   *
   * @param arena The host arena of this barge.
   * @param isBlue Wether this is the blue barge or the red one.
   */
  public ReefscapeBargeSimulation(Arena2025Reefscape arena, boolean isBlue) {
    super(
        arena,
        Centimeters.of(112),
        Centimeters.of(343),
        Centimeters.of(100),
        "Algae",
        isBlue ? blueBargePose : redBargePose,
        isBlue,
        false);

    StructPublisher<Pose3d> heldAlgaePublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic(isBlue ? "BlueBarge" : "RedBarge", Pose3d.struct)
            .publish();
    heldAlgaePublisher.set(new Pose3d(position, new Rotation3d()));
  }

  @Override
  public void draw(List<Pose3d> algaePosesToDisplay) {
    for (int i = 0; i < gamePieceCount; i++) {
      algaePosesToDisplay.add(
          new Pose3d(
                  xyBox.getCenter().x,
                  xyBox.getCenter().y - xyBox.getHeight() / 2,
                  elevation.in(Units.Meters) + height.in(Units.Meters) / 2,
                  new Rotation3d())
              .plus(new Transform3d(0, i * 0.35, 0, new Rotation3d())));
    }
  }

  @Override
  protected boolean checkVel(GamePiece gamePiece) {
    return gamePiece.getVelocity3dMPS().getZ() < 0;
  }

  @Override
  protected void addPoints() {
    arena.addValueToMatchBreakdown(isBlue, "AlgaeInNet", 1);
    arena.addValueToMatchBreakdown(
        isBlue, "Auto/AlgaeScoredInAuto", DriverStation.isAutonomous() ? 1 : 0);

    arena.addToScore(isBlue, 4);
  }
}
