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

package org.ironmaple.simulation.seasonspecific.crescendo2024;

import static edu.wpi.first.units.Units.Centimeters;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import java.util.*;
import org.ironmaple.simulation.Goal;

/**
 *
 *
 * <h2>Simulates a <strong>AMPS</strong> on the field.</h2>
 *
 * <p>This class simulates the <strong>AMP</strong> on the field where <strong>NOTES</strong>s can
 * be scored.
 */
public class CrescendoAmp extends Goal {

  protected static final Translation3d blueAmpPose = new Translation3d(1.81, 8.3, 0.66);
  protected static final Translation3d redAmpPose = new Translation3d(14.7, 8.3, 0.66);

  protected final Arena2024Crescendo crescendoArena;

  StructPublisher<Pose3d> posePublisher;

  /**
   *
   *
   * <h2>Creates an Amp of the specified color.</h2>
   *
   * @param arena The host arena of this amp.
   * @param isBlue Wether this is the blue amp or the red one.
   */
  public CrescendoAmp(Arena2024Crescendo arena, boolean isBlue) {
    super(
        arena,
        Centimeters.of(61),
        Centimeters.of(30),
        Centimeters.of(46),
        "Note",
        isBlue ? blueAmpPose : redAmpPose,
        isBlue,
        false);

    crescendoArena = arena;

    setNeededAngle(new Rotation3d(0, Math.PI / 2, Math.PI / 2));

    StructPublisher<Pose3d> ampPublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic(isBlue ? "BlueAmp" : "RedAmp", Pose3d.struct)
            .publish();
    ampPublisher.set(new Pose3d(position, this.pieceAngle));
  }

  @Override
  protected void addPoints() {
    crescendoArena.addValueToMatchBreakdown(isBlue, "TotalNotesInAmp", 1.0);
    arena.addValueToMatchBreakdown(
        isBlue, "AmplifiedScore", crescendoArena.isAmped(isBlue) ? 2 : 0);

    crescendoArena.addAmpCharge(isBlue);
    arena.addToScore(isBlue, crescendoArena.isAmped(isBlue) ? 2 : 1);
  }

  @Override
  public void draw(List<Pose3d> drawList) {
    return;
  }
}
