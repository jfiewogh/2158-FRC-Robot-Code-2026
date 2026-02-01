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

package org.ironmaple.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;
import java.util.function.Supplier;

public class LegacyFieldMirroringUtils2024 {
  public static final double FIELD_WIDTH = 16.54;
  public static final double FIELD_HEIGHT = 8.21;

  public static final Translation3d SPEAKER_POSE_BLUE = new Translation3d(0.1, 5.55, 2.2);

  public static final Supplier<Translation2d> SPEAKER_POSITION_SUPPLIER =
      () -> toCurrentAllianceTranslation(SPEAKER_POSE_BLUE.toTranslation2d());

  public static Rotation2d toCurrentAllianceRotation(Rotation2d rotationAtBlueSide) {
    final Rotation2d yAxis = Rotation2d.fromDegrees(90),
        differenceFromYAxisAtBlueSide = rotationAtBlueSide.minus(yAxis),
        differenceFromYAxisNew =
            differenceFromYAxisAtBlueSide.times(isSidePresentedAsRed() ? -1 : 1);
    return yAxis.rotateBy(differenceFromYAxisNew);
  }

  public static Translation2d toCurrentAllianceTranslation(Translation2d translationAtBlueSide) {
    if (isSidePresentedAsRed())
      return new Translation2d(
          FIELD_WIDTH - translationAtBlueSide.getX(), translationAtBlueSide.getY());
    return translationAtBlueSide;
  }

  public static Translation3d toCurrentAllianceTranslation(Translation3d translation3dAtBlueSide) {
    final Translation2d translation3dAtCurrentAlliance =
        toCurrentAllianceTranslation(translation3dAtBlueSide.toTranslation2d());
    if (isSidePresentedAsRed())
      return new Translation3d(
          translation3dAtCurrentAlliance.getX(),
          translation3dAtCurrentAlliance.getY(),
          translation3dAtBlueSide.getZ());
    return translation3dAtBlueSide;
  }

  public static Pose2d toCurrentAlliancePose(Pose2d poseAtBlueSide) {
    return new Pose2d(
        toCurrentAllianceTranslation(poseAtBlueSide.getTranslation()),
        toCurrentAllianceRotation(poseAtBlueSide.getRotation()));
  }

  public static boolean isSidePresentedAsRed() {
    final Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get().equals(DriverStation.Alliance.Red);
  }

  public static Rotation2d getCurrentAllianceDriverStationFacing() {
    return switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)) {
      case Red -> new Rotation2d(Math.PI);
      case Blue -> new Rotation2d(0);
    };
  }
}
