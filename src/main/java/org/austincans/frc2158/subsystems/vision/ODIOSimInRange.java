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

package org.austincans.frc2158.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.ArrayList;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena3D;

public class ODIOSimInRange implements ODIO {
  private final String gamePieceType;
  private final double maxDistanceMeters;
  private final Supplier<Pose3d> robotPoseSupplier;

  public ODIOSimInRange(
      String gamePieceType, double maxDistanceMeters, Supplier<Pose3d> robotPoseSupplier) {
    this.gamePieceType = gamePieceType;
    this.maxDistanceMeters = maxDistanceMeters;
    this.robotPoseSupplier = robotPoseSupplier;
  }

  private Pose3d[] getAllGamePiecePoses() {
    return SimulatedArena3D.getInstance().getGamePiecePosesArrayByType(gamePieceType);
  }

  private ArrayList<Pose3d> getGamePiecePosesInView() {
    Pose3d robotPose = robotPoseSupplier.get();

    ArrayList<Pose3d> gamePiecePoses = new ArrayList<Pose3d>();
    for (Pose3d fieldRelGamePiecePose : getAllGamePiecePoses()) {
      double distance =
          robotPose.getTranslation().getDistance(fieldRelGamePiecePose.getTranslation());
      if (distance < maxDistanceMeters) {
        gamePiecePoses.add(fieldRelGamePiecePose);
      }
    }
    return gamePiecePoses;
  }

  @Override
  public void updateInputs(ObjectDetectionIOInputs inputs) {
    ArrayList<Pose3d> gamePiecePosesArrayList = getGamePiecePosesInView();
    inputs.gamePiecePoses = gamePiecePosesArrayList.toArray(new Pose3d[0]);
    inputs.nearestGamePiecePose =
        inputs.gamePiecePoses.length > 0
            ? robotPoseSupplier.get().nearest(gamePiecePosesArrayList)
            : null;
  }
}
