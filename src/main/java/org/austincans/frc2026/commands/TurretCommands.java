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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.austincans.frc2026.FieldConstants;
import org.austincans.frc2026.subsystems.drive.Drive;
import org.austincans.frc2026.subsystems.flywheel.Flywheel;
import org.austincans.frc2026.subsystems.flywheel.FlywheelConstants;
import org.austincans.frc2026.subsystems.turret.Turret;
import org.austincans.frc2026.subsystems.turret.TurretConstants;
import org.littletonrobotics.junction.Logger;

// TODO: move tracking targets to ShooterCommands, move this into turret subsystem directory
public class TurretCommands {
  public static double flywheelExitVelocityConstant = 1;
  public static double robotShootTime = 1;

  public static InterpolatingDoubleTreeMap distancesAnglesFromHub = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(FieldConstants.fieldLength / 6, 7 * Math.PI / 18),
      Map.entry(0.0, Math.PI / 2),
      Map.entry(FieldConstants.fieldLength / 3, Math.PI / 4));
  public static InterpolatingDoubleTreeMap distancesAnglesFromShuttlingPoint = InterpolatingDoubleTreeMap
      .ofEntries(Map.entry(5.0, 5.0), Map.entry(10.0, 10.0));

  // TODO: Add areas where you cannot shoot from (ex. in front of hub in neutral
  // zone, behind tower)

  static double yOff = Units.inchesToMeters(158.32 - 25.62);

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

  private static final Translation2d TURRET_OFFSET = new Translation2d(0, 0); // meters
  private static final double FLYWHEEL_SHOOT_SPEED = FlywheelConstants.maxRPM;

  private static final double TURRET_HEIGHT = TurretConstants.heightMeters; // meters
  public static boolean isShuttling = false;
  private static final double latency = 0.02;

  private TurretCommands() {
  }

  private static double getActualExitVelocityMetersPerSec(double flywheelVelocityRPM) {
    return Units.rotationsToRadians(flywheelVelocityRPM) / 60 * FlywheelConstants.radiusMeters;
  }

  /**
   * Only setter methods used for flywheel, turret, and hood. Only getter methods
   * used for drive.
   */
  public static double getHoodAngleFromDistance(
      double distance, InterpolatingDoubleTreeMap points) {
    return points.get(distance);
  }

  public static Translation2d getFutureTranslation(
      Translation2d currentTranslation, double time, Translation2d speeds) {
    return currentTranslation.plus(speeds.times(time));
  }

  public static double getRobotDistanceFromHub(Drive drive, Alliance alliance) {
    if (alliance == Alliance.Blue) {
      return drive
          .getPose3d()
          .getTranslation()
          .getDistance(TrackingTargets.blueHub.getTranslation3d());
    } else {
      return drive
          .getPose3d()
          .getTranslation()
          .getDistance(TrackingTargets.redHub.getTranslation3d());
    }
  }

  public static double getDistance(Translation2d translation1, Translation2d translation2) {
    return translation1.getDistance(translation2);
  }

  public static double getDistanceFromShuttlingPoint(Drive drive, TrackingTargets target) {
    return drive.getPose3d().getTranslation().getDistance(target.getTranslation3d());
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
      possibleTargets = new TrackingTargets[] { TrackingTargets.blueShuttleLeft, TrackingTargets.blueShuttleRight };
    } else {
      possibleTargets = new TrackingTargets[] { TrackingTargets.redShuttleLeft, TrackingTargets.redShuttleRight };
    }
    return getClosestTarget(robotPose, possibleTargets);
  }

  public static Command pointToHub(Optional<Turret> turret, Optional<Drive> drive) {
    if (turret.isEmpty() || drive.isEmpty()) {
      return Commands.none();
    }
    Turret t = turret.get();
    Drive d = drive.get();
    return Commands.run(
        () -> {
          Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
          TrackingTargets target = alliance == Alliance.Blue ? TrackingTargets.blueHub : TrackingTargets.redHub;
          Translation3d targetTranslation = target.getTranslation3d();
          Pose3d drivePose = d.getPose3d();

          // Transform3d applies Translation3d first, then Rotation3d
          Pose3d turretPose = drivePose.plus(TurretConstants.robotRelTurretTransform);
          Translation3d turretTranslation = turretPose.getTranslation();
          Logger.recordOutput("turret pose", turretPose);

          Rotation2d fieldRelAngle = targetTranslation
              .toTranslation2d()
              .minus(turretTranslation.toTranslation2d())
              .getAngle();

          // We need this for calculating
          Logger.recordOutput(
              "turret distance from hub",
              turretTranslation
                  .toTranslation2d()
                  .getDistance(targetTranslation.toTranslation2d()));

          t.setDesiredAngleFieldRelative(fieldRelAngle, d.getRotation());
        },
        t)
        .finallyDo(t::stop);
  }

  private static Rotation2d setpointJoystickTurretAngle = Rotation2d.kZero; // should be zero on start

  public static Command joystickTurretAngleCommand(
      Turret turret, DoubleSupplier vertSupplier, DoubleSupplier horizSupplier) {
    return Commands.run(
        () -> {
          if (Math.abs(vertSupplier.getAsDouble()) > 0.1
              && Math.abs(horizSupplier.getAsDouble()) > 0.1) {
            setpointJoystickTurretAngle = Rotation2d.fromRadians(
                Math.atan2(horizSupplier.getAsDouble(), vertSupplier.getAsDouble()));
            turret.setDesiredAngleRobotRelative(setpointJoystickTurretAngle);
          }
        },
        turret);
  }

  public static Command resetAngleCommand(Optional<Turret> turret) {
    return turret.map((t) -> t.resetAngle()).orElse(Commands.none());
  }

  public static Command setFieldRelTurretAngleCommand(Optional<Turret> turret, Rotation2d fieldRelAngle,
      Rotation2d driveFieldRelRotation) {
    return turret
        .map((t) -> Commands.runOnce(() -> t.setDesiredAngleFieldRelative(fieldRelAngle, driveFieldRelRotation), t))
        .orElse(Commands.none());
  }

  public static boolean atSetpoint(Optional<Turret> turret) {
        return turret.map((t) -> t.atSetpoint()).orElse(true);
    }
}
