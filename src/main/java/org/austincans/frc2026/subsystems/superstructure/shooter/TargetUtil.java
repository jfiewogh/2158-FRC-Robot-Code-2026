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

package org.austincans.frc2026.subsystems.superstructure.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.austincans.frc2026.commands.RebuiltFieldUtils;
import org.austincans.frc2026.subsystems.drive.Drive;

public class TargetUtil {
  private static double yOff = Units.inchesToMeters(158.32 - 25.62);

  public enum TrackingTargets {
    redHub(new Translation3d(11.938, 4.034536, 1.5748)),
    blueHub(new Translation3d(4.5974, 4.034536, 1.5748)),
    blueOutTrench(new Translation3d(4.625594, 0.632968, 0)),
    blueDepotTrench(new Translation3d(4.625594, 7.436358, 0)),
    redDepotTrench(new Translation3d(11.915394, 0.632968, 0)),
    redLeftTrench(new Translation3d(11.915394, 7.436358, 0)),

    redShuttleLeft(new Translation3d(RebuiltFieldUtils.fieldLength - 1, yOff, 0)),
    redShuttleRight(
        new Translation3d(
            RebuiltFieldUtils.fieldLength - 1, RebuiltFieldUtils.fieldWidth - yOff, 0)),
    blueShuttleRight(new Translation3d(1, yOff, 0)),
    blueShuttleLeft(new Translation3d(1, RebuiltFieldUtils.fieldWidth - yOff, 0));

    private final Translation3d targetTranslation3d;

    private TrackingTargets(
        Translation3d target) { // Constructor must be private or package-private
      this.targetTranslation3d = target;
    }

    public Translation3d getTranslation3d() { // Getter method
      return this.targetTranslation3d;
    }
  }

  private static TrackingTargets getClosestTarget(Pose3d pose, TrackingTargets[] possibleTargets) {
    Translation3d translation = pose.getTranslation();
    TrackingTargets closestTarget = possibleTargets[0];
    for (int i = 1; i < possibleTargets.length; i++) {
      TrackingTargets target = possibleTargets[i];
      double distance = translation.getDistance(target.getTranslation3d());
      double currentClosestDistance = translation.getDistance(closestTarget.getTranslation3d());
      if (distance < currentClosestDistance) {
        closestTarget = target;
      }
    }
    return closestTarget;
  }

  public static TrackingTargets getShuttleTarget(Drive drive, Alliance alliance) {
    Pose3d robotPose = new Pose3d(drive.getPose());
    TrackingTargets[] possibleTargets;
    if (alliance == Alliance.Blue) {
      possibleTargets =
          new TrackingTargets[] {TrackingTargets.blueShuttleLeft, TrackingTargets.blueShuttleRight};
    } else {
      possibleTargets =
          new TrackingTargets[] {TrackingTargets.redShuttleLeft, TrackingTargets.redShuttleRight};
    }
    return getClosestTarget(robotPose, possibleTargets);
  }
}
