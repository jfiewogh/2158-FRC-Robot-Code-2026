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
// Updated on Saturday February 21, 2026 at 12:49:35 CST

package org.ironmaple.utils;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

public class FieldMirroringUtils {
  public static final double FIELD_WIDTH = Inches.of(651).in(Meters);
  public static final double FIELD_HEIGHT = Inches.of(315).in(Meters);

  public static Rotation2d toCurrentAllianceRotation(Rotation2d rotationAtBlueSide) {
    return isSidePresentedAsRed() ? flip(rotationAtBlueSide) : rotationAtBlueSide;
  }

  public static Rotation2d flip(Rotation2d rotation) {
    return rotation.plus(Rotation2d.k180deg);
  }

  public static Translation2d toCurrentAllianceTranslation(Translation2d translationAtBlueSide) {
    return isSidePresentedAsRed() ? flip(translationAtBlueSide) : translationAtBlueSide;
  }

  public static Translation2d flip(Translation2d translation) {
    return new Translation2d(FIELD_WIDTH - translation.getX(), FIELD_HEIGHT - translation.getY());
  }

  public static Pose3d flip(Pose3d toFlip) {
    var flipped = flip(toFlip.getTranslation().toTranslation2d());
    return new Pose3d(flipped.getX(), flipped.getY(), toFlip.getZ(), toFlip.getRotation());
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
    return toCurrentAllianceRotation(Rotation2d.kZero);
  }
}
