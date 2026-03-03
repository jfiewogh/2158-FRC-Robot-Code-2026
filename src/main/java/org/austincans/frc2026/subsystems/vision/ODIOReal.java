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

package org.austincans.frc2026.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import org.austincans.frc2026.subsystems.vision.LimelightHelpers.LimelightResults;
import org.austincans.frc2026.subsystems.vision.LimelightHelpers.LimelightTarget_Detector;
import org.austincans.frc2026.subsystems.vision.ObjectDetection.GamePieceType;
import org.austincans.frc2026.subsystems.vision.VisionConstants.LimelightConfig;

public class ODIOReal implements ODIO {
  private final String limelightName;
  private final GamePieceType gamePieceType;
  private final LimelightConfig config;

  // TODO: add to config
  private static final Pose3d robotRelCameraPose =
      new Pose3d(
          new Translation3d(0, 0, 0), // meters
          new Rotation3d(0, 0, 0));

  public ODIOReal(String limelightName, GamePieceType gamePieceType, LimelightConfig config) {
    this.limelightName = limelightName;
    this.gamePieceType = gamePieceType;
    this.config = config;
  }

  // Only works for ball game piece
  // TODO: Add version for other game pieces (coral, note)
  private Translation3d getTranslationFromDetectorResult(LimelightTarget_Detector result) {
    Rotation2d yaw = Rotation2d.fromDegrees(result.tx);
    Rotation2d pitch = Rotation2d.fromDegrees(result.ty);

    double areaPixels = result.ta * config.getWidthPixels() * config.getHeightPixels();
    double sideLengthPixels = Math.sqrt(areaPixels);

    double angularWidthRad =
        sideLengthPixels / config.getWidthPixels() * config.getHorizontalViewingAngleRad();

    double horizontalOffsetMetersPerAngleRad =
        gamePieceType.getRadiusMeters() * 2 / angularWidthRad;
    double yDisplacementMeters = yaw.getRadians() * horizontalOffsetMetersPerAngleRad;
    double xyDistanceMeters = yDisplacementMeters / Math.sin(yaw.getRadians());
    double xDisplacementMeters = xyDistanceMeters * Math.cos(yaw.getRadians());

    double angularHeightRad =
        sideLengthPixels / config.getHeightPixels() * config.getVerticalViewingAngleRad();
    double verticalOffsetPerAngleMeters = gamePieceType.getRadiusMeters() * 2 / angularHeightRad;
    double zDisplacementMeters = pitch.getRadians() * verticalOffsetPerAngleMeters;

    return new Translation3d(xDisplacementMeters, yDisplacementMeters, zDisplacementMeters);
  }

  private Pose3d getFieldRelCameraPose(Pose3d robotPose) {
    return robotPose.plus(
        new Transform3d(robotRelCameraPose.getTranslation(), robotRelCameraPose.getRotation()));
  }

  private ArrayList<Pose3d> getFieldRelGamePiecePoses() {
    LimelightResults latestResults = LimelightHelpers.getLatestResults(limelightName);
    LimelightTarget_Detector[] detectorResults = latestResults.targets_Detector;

    Pose3d robotPose = LimelightHelpers.getBotPose3d(limelightName);

    ArrayList<Pose3d> gamePiecePoses = new ArrayList<Pose3d>();

    for (LimelightTarget_Detector result : detectorResults) {
      if (result.className == gamePieceType.getClassName()) {
        Translation3d cameraRelativeGamePieceTranslation = getTranslationFromDetectorResult(result);

        Pose3d fieldRelativeCameraPose = getFieldRelCameraPose(robotPose);

        Pose3d fieldRelativeGamePiecePose =
            fieldRelativeCameraPose.plus(
                new Transform3d(cameraRelativeGamePieceTranslation, Rotation3d.kZero));
        gamePiecePoses.add(fieldRelativeGamePiecePose);
      }
    }
    return gamePiecePoses;
  }

  private Pose3d getNearestGamePiece() {
    Pose3d robotPose = LimelightHelpers.getBotPose3d(limelightName);
    ArrayList<Pose3d> fieldRelGamePiecePoses = getFieldRelGamePiecePoses();
    return fieldRelGamePiecePoses.size() > 0 ? robotPose.nearest(fieldRelGamePiecePoses) : null;
  }

  @Override
  public void updateInputs(ObjectDetectionIOInputs inputs) {
    inputs.gamePiecePoses = getFieldRelGamePiecePoses().toArray(new Pose3d[0]);
    inputs.nearestGamePiecePose = getNearestGamePiece();
  }
}
