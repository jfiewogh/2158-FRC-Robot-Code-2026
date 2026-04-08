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

package org.austincans.frc2026.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Function;
import org.austincans.frc2026.util.GeomUtil;
import org.littletonrobotics.junction.Logger;

// TODO: Move into lib
// TODO: Maybe make it work for z's between floor and hub height (ex. robot on bump)
/**
 * Used to calculate the parameters necessary for shooting based on maps of
 * empirically measured
 * data.
 *
 * <p>
 * Heavily based on z
 *
 * <ul>
 * <li>https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java
 * <li>https://github.com/eeveemara/frc-fire-control/blob/main/src/main/java/frc/firecontrol/ShotCalculator.java
 * </ul>
 */
public class ShotCalculator {
  private static final int iterations = 20;

  private final double phaseDelaySecs;

  /** Pitch relative to the shooter */
  private final InterpolatingTreeMap<Double, Rotation2d> pitchMap;

  private final InterpolatingDoubleTreeMap flywheelSpeedMap;
  private final InterpolatingDoubleTreeMap timeOfFlightMap;

  private final Transform3d robotRelTurretOffset;
  private Function<Rotation2d, Translation3d> turretRelShotOffsetFromPitchMap; // based on turret offset
  private Function<Rotation2d, Translation3d> turretRelShotOffsetFromYawMap; // based on turret offset

  /**
   * The key for all maps is XY distance in meters.
   *
   * @param phaseDelaySecs
   * @param pitchRadMap
   * @param flywheelSpeedMap     RPM
   * @param timeOfFlightMap      This is needed for needed for shooting on the
   *                             move
   * @param robotRelTurretOffset in meters and radians
   */
  public ShotCalculator(
      double phaseDelaySecs,
      InterpolatingTreeMap<Double, Rotation2d> pitchMap,
      InterpolatingDoubleTreeMap flywheelSpeedMap,
      InterpolatingDoubleTreeMap timeOfFlightMap,
      Transform3d robotRelTurretOffset,
      Function<Rotation2d, Translation3d> turretRelShotOffsetFromPitchMap,
      Function<Rotation2d, Translation3d> turretRelShotOffsetFromYawMap) {
    this.phaseDelaySecs = phaseDelaySecs;
    this.pitchMap = pitchMap;
    this.flywheelSpeedMap = flywheelSpeedMap;
    this.timeOfFlightMap = timeOfFlightMap;
    this.robotRelTurretOffset = robotRelTurretOffset;
    this.turretRelShotOffsetFromPitchMap = turretRelShotOffsetFromPitchMap;
    this.turretRelShotOffsetFromYawMap = turretRelShotOffsetFromYawMap;
  }

  public ShotCalculator(
      double phaseDelaySecs,
      InterpolatingTreeMap<Double, Rotation2d> pitchMap,
      InterpolatingDoubleTreeMap flywheelSpeedMap,
      InterpolatingDoubleTreeMap timeOfFlightMap,
      Transform3d robotRelTurretOffset) {
    this.phaseDelaySecs = phaseDelaySecs;
    this.pitchMap = pitchMap;
    this.flywheelSpeedMap = flywheelSpeedMap;
    this.timeOfFlightMap = timeOfFlightMap;
    this.robotRelTurretOffset = robotRelTurretOffset;
  }

  public record ShotInputs(
      Pose3d driveFieldRelPose3d,
      ChassisSpeeds driveRobotRelVelocities,
      ChassisSpeeds driveFieldRelVelocities,
      Translation3d targetFieldRelPosition3d) {
  }

  public record ShotParameters(
      Rotation2d hoodAngle,
      Rotation2d fieldRelTurretAngle,
      double flywheelSpeedRPM,
      Rotation2d driveFieldRelRotation2d) {
  }

  /** This only takes into account X and Y components, ignoring Z. */
  public ShotParameters calculate(ShotInputs inputs) {
    // TODO: update this so that inputs
    // include robot pose, turret offset (from function), hood offset (from
    // function)

    /* Calculate drive pose */
    Pose2d driveFieldRelPose2d = inputs.driveFieldRelPose3d.toPose2d();
    ChassisSpeeds driveRobotRelVelocities = inputs.driveRobotRelVelocities;
    // Account for phase delay
    // TODO: You can also try to take into account acceleration. This can be
    // achieved by storing
    // previous velocity and calculating the slope.
    driveFieldRelPose2d = driveFieldRelPose2d.exp(
        new Twist2d(
            driveRobotRelVelocities.vxMetersPerSecond * phaseDelaySecs,
            driveRobotRelVelocities.vyMetersPerSecond * phaseDelaySecs,
            driveRobotRelVelocities.omegaRadiansPerSecond * phaseDelaySecs));

    /* Calculate target */
    Translation2d targetFieldRelPosition2d = inputs.targetFieldRelPosition3d.toTranslation2d();
    Pose2d shotFieldRelPose2d = driveFieldRelPose2d.transformBy(GeomUtil.toTransform2d(robotRelTurretOffset));
    double shotToTargetDistance2d = targetFieldRelPosition2d.getDistance(driveFieldRelPose2d.getTranslation());

    /* Calculate field relative shot velocities */
    ChassisSpeeds driveFieldRelVelocities = inputs.driveFieldRelVelocities;
    Rotation2d driveRotation = driveFieldRelPose2d.getRotation();
    ChassisSpeeds shotFieldRelVelocities = GeomUtil.transformVelocity(
        driveFieldRelVelocities,
        robotRelTurretOffset.getTranslation().toTranslation2d(),
        driveRotation);

    /*
     * Account for shooter velocity to offset (lookahead pose, virtual target)
     * Utilizes fixed point iteration
     * We use this to estimate the distance and time of flight
     * since time of flight depends on distance from target,
     * offset depends on time of flight,
     * and our data is discrete
     */
    double timeOfFlightSecs = timeOfFlightMap.get(shotToTargetDistance2d);
    Pose2d lookaheadShotPose2d = shotFieldRelPose2d;
    double lookaheadShotToTargetDistance2d = shotToTargetDistance2d;

    for (int i = 0; i < iterations; i++) {
      timeOfFlightSecs = timeOfFlightMap.get(lookaheadShotToTargetDistance2d);
      Translation2d offset2d = new Translation2d(
          shotFieldRelVelocities.vxMetersPerSecond * timeOfFlightSecs,
          shotFieldRelVelocities.vyMetersPerSecond * timeOfFlightSecs);
      lookaheadShotPose2d = new Pose2d(
          shotFieldRelPose2d.getTranslation().plus(offset2d), shotFieldRelPose2d.getRotation());
      lookaheadShotToTargetDistance2d = targetFieldRelPosition2d.getDistance(lookaheadShotPose2d.getTranslation());
    }

    Logger.recordOutput("turret pose", shotFieldRelPose2d);
    Logger.recordOutput("shot pose", lookaheadShotPose2d);
    Logger.recordOutput("target pose", targetFieldRelPosition2d);

    // Account for shot position being off center
    // Pose2d lookaheadDrivePose2d =
    // lookaheadShotPose2d.transformBy(GeomUtil.toTransform2d(robotRelTurretOffset).inverse());

    /* Hood angle */
    Rotation2d hoodAngle = pitchMap.get(lookaheadShotToTargetDistance2d);

    /* Turret angle */
    Rotation2d turretAngle = targetFieldRelPosition2d.minus(lookaheadShotPose2d.getTranslation()).getAngle();

    /* Flywheel speed */
    double flywheelSpeedRPM = flywheelSpeedMap.get(lookaheadShotToTargetDistance2d);

    return new ShotParameters(
        hoodAngle, turretAngle, flywheelSpeedRPM, driveFieldRelPose2d.getRotation());
  }

  public static double getMinTimeOfFlight() {
    return ShotConstants.scoringTimeOfFlightMap.get(0.0);
  }

    public static double getMaxTimeOfFlight() {
    return ShotConstants.passingTimeOfFlightMap.get(100.0);
  }
}
