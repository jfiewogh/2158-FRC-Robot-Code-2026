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

package org.austincans.frc2026.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import org.austincans.frc2026.subsystems.drive.Drive;
import org.austincans.frc2026.subsystems.drive.DriveConstants;

public class RebuiltFieldUtils {
  // TODO: Maybe move this to Constants file, different values should be based on what field is
  // chosen
  public static final double fieldLength = Units.inchesToMeters(651.22);
  public static final double fieldWidth = Units.inchesToMeters(317.69);

  private static double blueAllianceZoneX = Units.inchesToMeters(156.61);
  private static double redAllianceZoneX = Units.inchesToMeters(651.22 - 156.61);

  public static boolean isInAllianceZone(Alliance alliance, Pose2d robotPose) {
    return alliance == Alliance.Blue
        ? robotPose.getX() < blueAllianceZoneX
        : robotPose.getX() > redAllianceZoneX;
  }

  public static boolean poseInAllianceHalf(Pose3d pose) {
    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    return allianceOptional.isPresent() && allianceOptional.get() == Alliance.Blue
        ? pose.getX() < fieldLength / 2 - DriveConstants.driveBaseRadius * 1.2
        : pose.getX() > fieldLength / 2 + DriveConstants.driveBaseRadius * 1.2;
  }

  public static boolean robotInAllianceHalf(Drive drive) {
    return poseInAllianceHalf(drive.getPose3d());
  }
}
