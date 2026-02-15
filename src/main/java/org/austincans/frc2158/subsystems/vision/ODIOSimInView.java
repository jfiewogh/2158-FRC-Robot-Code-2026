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
// Updated on Sunday February 15, 2026 at 11:44:15 CST

package org.austincans.frc2158.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.function.Supplier;
import org.austincans.frc2158.subsystems.vision.VisionConstants.LimelightConfig;
import org.ironmaple.simulation.SimulatedArena3D;

// NOTE: This can see through obstacles on field
// This uses horizontal and vertical viewing angles
// It does take into account the actual camera dimensions
public class ODIOSimInView implements ODIO {
  private final String gamePieceType;
  private final LimelightConfig config;
  private final double maxDetectionDistanceMeters;
  private final Supplier<Pose3d> fieldRelRobotPoseSupplier;
  private final Supplier<Pose3d> robotRelCameraPoseSupplier;

  public ODIOSimInView(
      String gamePieceType,
      LimelightConfig config,
      double maxDetectionDistanceMeters,
      Supplier<Pose3d> fieldRelRobotPoseSupplier,
      Supplier<Pose3d> robotRelCameraPoseSupplier) {
    this.gamePieceType = gamePieceType;
    this.config = config;
    this.maxDetectionDistanceMeters = maxDetectionDistanceMeters;
    this.fieldRelRobotPoseSupplier = fieldRelRobotPoseSupplier;
    this.robotRelCameraPoseSupplier = robotRelCameraPoseSupplier;
  }

  private Pose3d[] getAllGamePiecePoses() {
    return SimulatedArena3D.getInstance().getGamePiecePosesArrayByType(gamePieceType);
  }

  private boolean checkPoseInView(Pose3d fieldRelGamePiecePose) {
    Pose3d robotRelCameraPose = robotRelCameraPoseSupplier.get();
    Pose3d fieldRelCameraPose =
        fieldRelRobotPoseSupplier
            .get()
            .plus(
                new Transform3d(
                    robotRelCameraPose.getTranslation(), robotRelCameraPose.getRotation()));
    Transform3d robotRelGamePieceTransform = fieldRelGamePiecePose.minus(fieldRelCameraPose);
    Translation3d robotRelGamePieceTranslation = robotRelGamePieceTransform.getTranslation();

    double x = robotRelGamePieceTranslation.getX();
    if (x > maxDetectionDistanceMeters) {
      return false;
    }
    double y = robotRelGamePieceTranslation.getY();
    double z = robotRelGamePieceTranslation.getZ();

    double yMaxRadius = x * Math.sin(config.getHorizontalViewingAngleRad());
    double zMaxRadius = x * Math.sin(config.getVerticalViewingAngleRad());

    // Checks if in rectangle
    return Math.abs(y) < yMaxRadius && Math.abs(z) < zMaxRadius;
  }

  private ArrayList<Pose3d> getGamePiecePosesInView() {
    ArrayList<Pose3d> gamePiecePoses = new ArrayList<Pose3d>();
    for (Pose3d fieldRelGamePiecePose : getAllGamePiecePoses()) {
      if (checkPoseInView(fieldRelGamePiecePose)) {
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
            ? fieldRelRobotPoseSupplier.get().nearest(gamePiecePosesArrayList)
            : null;
  }
}
