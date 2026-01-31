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
// Updated on Saturday January 31, 2026 at 12:05:30 CST

package org.ironmaple.simulation.seasonspecific.crescendo2024;

import static edu.wpi.first.units.Units.Centimeters;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import java.util.*;
import org.ironmaple.simulation.Goal;
import org.ironmaple.simulation.gamepieces.GamePiece;

/**
 *
 *
 * <h2>Simulates a <strong>SPEAKER</strong>s on the field.</h2>
 *
 * <p>This class simulates the <strong>SPEAKER</strong>s on the field where <strong>NOTES</strong>s
 * can be scored. It may be amplified by using the {@link Arena2024Crescendo#activateAmp(boolean)}
 * function on the host arena.
 */
public class CrescendoSpeaker extends Goal {

  protected static final Translation3d redSpeakerPose = new Translation3d(16.52, 5.5825, 2.1);
  protected static final Translation3d blueSpeakerPose = new Translation3d(0, 5.5825, 2.1);
  StructPublisher<Pose3d> posePublisher;
  protected final Arena2024Crescendo crescendoArena;

  /**
   *
   *
   * <h2>Creates an Speaker of the specified color.</h2>
   *
   * @param arena The host arena of this speaker.
   * @param isBlue Wether this is the blue speaker or the red one.
   */
  public CrescendoSpeaker(Arena2024Crescendo arena, boolean isBlue) {
    super(
        arena,
        Centimeters.of(90),
        Centimeters.of(116.5),
        Centimeters.of(100),
        "Note",
        isBlue ? blueSpeakerPose : redSpeakerPose,
        isBlue,
        false);

    crescendoArena = arena;
    StructPublisher<Pose3d> speakerPosePublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic(isBlue ? "BlueSpeaker" : "RedSpeaker", Pose3d.struct)
            .publish();
    speakerPosePublisher.set(new Pose3d(position, new Rotation3d()));
  }

  @Override
  protected boolean checkVel(GamePiece gamePiece) {
    return gamePiece.getVelocity3dMPS().getZ() > 0;
  }

  @Override
  protected void addPoints() {
    arena.addValueToMatchBreakdown(isBlue, "TotalNotesInSpeaker", 1);
    arena.addValueToMatchBreakdown(
        isBlue, "AmplifiedScore", crescendoArena.isAmped(isBlue) ? 5 : 0);
    arena.addToScore(isBlue, crescendoArena.isAmped(isBlue) ? 5 : 2);
  }

  @Override
  public void draw(List<Pose3d> drawList) {
    return;
  }
}
