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

package org.austincans.frc2026.superstructure.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.austincans.frc2026.RobotState;
import org.austincans.frc2026.subsystems.turret.TurretConstants;
import org.austincans.frc2026.superstructure.shooter.ShotCalculator.ShotInputs;
import org.austincans.frc2026.superstructure.shooter.ShotCalculator.ShotParameters;
import org.austincans.frc2026.superstructure.shooter.TargetUtil.TrackingTargets;

public class ShotUtil {
  private static final ShotCalculator hubShotCalculator;
  private static final ShotCalculator shuttleShotCalculator;

  static {
    hubShotCalculator =
        new ShotCalculator(
            0.02,
            ShotConstants.scoringHoodAngleMap,
            ShotConstants.scoringFlywheelSpeedMap,
            ShotConstants.scoringTimeOfFlightMap,
            TurretConstants.robotRelTurretTransform);
    shuttleShotCalculator =
        new ShotCalculator(
            0.02,
            ShotConstants.passingHoodAngleMap,
            ShotConstants.passingFlywheelSpeedMap,
            ShotConstants.passingTimeOfFlightMap,
            TurretConstants.robotRelTurretTransform);
  }

  public static ShotParameters getHubShotParameters() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
    ShotInputs inputs =
        new ShotInputs(
            new Pose3d(RobotState.getInstance().getEstimatedPose()),
            RobotState.getInstance().getRobotVelocity(),
            RobotState.getInstance().getFieldVelocity(),
            alliance == Alliance.Red
                ? TrackingTargets.redHub.getTranslation3d()
                : TrackingTargets.blueHub.getTranslation3d());
    return hubShotCalculator.calculate(inputs);
  }

  public static ShotParameters getShuttleShotParameters() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
    Pose3d fieldRelRobotPose = new Pose3d(RobotState.getInstance().getEstimatedPose());
    ShotInputs inputs =
        new ShotInputs(
            fieldRelRobotPose,
            RobotState.getInstance().getRobotVelocity(),
            RobotState.getInstance().getFieldVelocity(),
            TargetUtil.getShuttleTarget(fieldRelRobotPose, alliance).getTranslation3d());
    return shuttleShotCalculator.calculate(inputs);
  }
}
