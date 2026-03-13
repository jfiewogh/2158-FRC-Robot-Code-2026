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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;
import org.austincans.frc2026.Constants;
import org.austincans.frc2026.subsystems.drive.Drive;
import org.austincans.frc2026.subsystems.flywheel.Flywheel;
import org.austincans.frc2026.subsystems.flywheel.FlywheelConstants;
import org.austincans.frc2026.subsystems.hood.Hood;
import org.austincans.frc2026.subsystems.turret.Turret;
import org.austincans.frc2026.subsystems.turret.TurretConstants;
import org.austincans.frc2026.util.ShooterMath;
import org.littletonrobotics.junction.Logger;

public class TurretCommands {
  public enum TrackingTargets {
    redHub(new Translation3d(11.938, 4.034536, 1.5748)),
    blueHub(new Translation3d(4.5974, 4.034536, 1.5748)),
    blueOutTrench(new Translation3d(4.625594, 0.632968, 0)),
    blueDepotTrench(new Translation3d(4.625594, 7.436358, 0)),
    redDepotTrench(new Translation3d(11.915394, 0.632968, 0)),
    redLeftTrench(new Translation3d(11.915394, 7.436358, 0)),

    redShuttleLeft(new Translation3d(RebuiltFieldUtils.fieldLength - 1, 3, 0)),
    redShuttleRight(
        new Translation3d(RebuiltFieldUtils.fieldLength - 1, RebuiltFieldUtils.fieldWidth - 3, 0)),
    blueShuttleRight(new Translation3d(1, 3, 0)),
    blueShuttleLeft(new Translation3d(1, RebuiltFieldUtils.fieldWidth - 3, 0));

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

  private static final double TURRET_HEIGHT = TurretConstants.turretHeightMeters; // meters
  public static boolean isShuttling = false;
  private static final double latency = 0.02;

  private static double getActualExitVelocityMetersPerSec(double flywheelVelocityRPM) {
    return Units.rotationsToRadians(flywheelVelocityRPM) / 60 * FlywheelConstants.radiusMeters;
  }

  /**
   * Only setter methods used for flywheel, turret, and hood. Only getter methods used for drive.
   */
  public static void shootToTarget(
      Flywheel flywheel, Turret turret, Hood hood, Drive drive, TrackingTargets target) {
    Pose2d robotPose = drive.getPose();
    ChassisSpeeds robotSpeeds = drive.getChassisSpeeds();
    Drive.driveSpeedMultiplier = 0.5;

    double targetHeight = target.getTranslation3d().getZ();

    Translation2d futureRobotTranslation =
        robotPose
            .getTranslation()
            .plus(
                new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
                    .times(latency));
    Rotation2d futureRobotRotation =
        robotPose
            .getRotation()
            .plus(new Rotation2d(robotSpeeds.omegaRadiansPerSecond).times(latency));

    Translation2d turretTranslation =
        futureRobotTranslation.plus(TURRET_OFFSET.rotateBy(futureRobotRotation));

    flywheel.setRelativeVelocity(0);

    // TOTAL VALUES
    double totalVelocityMetersPerSec = getActualExitVelocityMetersPerSec(FLYWHEEL_SHOOT_SPEED);
    double totalXYDistanceMeters =
        turretTranslation.getDistance(target.getTranslation3d().toTranslation2d());
    double totalPitchRad =
        ShooterMath.calculateProjectileAngle(
            TURRET_HEIGHT, totalVelocityMetersPerSec, totalXYDistanceMeters, targetHeight);

    if (Double.isNaN(totalPitchRad)) {
      System.out.println("Cannot shoot into hub (1)");
    } else {
      Rotation2d totalYaw =
          Rotation2d.fromRadians(
              Math.atan2(
                  target.getTranslation3d().getY() - turretTranslation.getY(),
                  target.getTranslation3d().getX() - turretTranslation.getX()));
      double totalXYVelocity = totalVelocityMetersPerSec * Math.cos(totalPitchRad);
      double totalXVelocity = totalXYVelocity * totalYaw.getCos();
      double totalYVelocity = totalXYVelocity * totalYaw.getSin();

      // CALCULATE EFFECTIVE YAW (TURRET ANGLE)
      double effectiveSpeedX = totalXVelocity - robotSpeeds.vxMetersPerSecond;
      double effectiveSpeedY = totalYVelocity - robotSpeeds.vyMetersPerSecond;
      double effectiveSpeed = Math.hypot(effectiveSpeedX, effectiveSpeedY);
      Rotation2d turretAngle = Rotation2d.fromRadians(Math.atan2(effectiveSpeedY, effectiveSpeedX));

      // CALCULATE EFFECTIVE PITCH (HOOD ANGLE)
      double totalTime = totalXYDistanceMeters / totalXYVelocity;

      double hoodAngleRad =
          ShooterMath.calculateProjectileAngle(
              TURRET_HEIGHT, totalVelocityMetersPerSec, effectiveSpeed * totalTime, targetHeight);

      if (Double.isNaN(hoodAngleRad)) {
        System.out.println("Cannot shoot into hub (2)");
      } else if (Constants.isSim
          && (turretAngle.minus(futureRobotRotation).getRotations() > 2.75
              || turretAngle.minus(futureRobotRotation).getRotations() < -2.75)) {
        System.out.println("Cannot Rotate Turret to Position");
      } else {
        // Should run only if angle is possible
        flywheel.setRelativeVelocity(FLYWHEEL_SHOOT_SPEED);
        turret.setDesiredAngleFieldRelative(turretAngle, futureRobotRotation);
        Logger.recordOutput("turret field rel angle", turretAngle);
        hood.setAngle(Rotation2d.fromRadians(hoodAngleRad));
      }
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

  private static TrackingTargets getShuttleTarget(Drive drive, Alliance alliance) {
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

  private static void shuttle(
      Flywheel flywheel, Turret turret, Hood hood, Drive drive, Alliance alliance) {
    shootToTarget(flywheel, turret, hood, drive, getShuttleTarget(drive, alliance));
  }

  // TODO: Maybe make it so that robot does not shoot over hub
  public static Command shuttleCommand(Flywheel flywheel, Turret turret, Hood hood, Drive drive) {
    return Commands.run(
            () -> {
              Optional<Alliance> allianceOptional = DriverStation.getAlliance();
              if (allianceOptional.isPresent()) {
                shuttle(flywheel, turret, hood, drive, allianceOptional.get());
              }
            },
            flywheel,
            turret,
            hood)
        .until(flywheel::noGamePiecesToShoot)
        .finallyDo(
            () -> {
              flywheel.setRelativeVelocity(0);
              Drive.driveSpeedMultiplier = 1;
              isShuttling = false;
            });
  }

  private static void shootToHub(
      Flywheel flywheel, Turret turret, Hood hood, Drive drive, Alliance alliance) {
    shootToTarget(
        flywheel,
        turret,
        hood,
        drive,
        alliance == Alliance.Blue ? TrackingTargets.blueHub : TrackingTargets.redHub);
  }

  public static Command shootToHubCommand(
      Flywheel flywheel, Turret turret, Hood hood, Drive drive) {
    return Commands.run(
            () -> {
              Optional<Alliance> allianceOptional = DriverStation.getAlliance();
              if (allianceOptional.isPresent()) {
                Alliance alliance = allianceOptional.get();
                if (RebuiltFieldUtils.isInAllianceZone(alliance, drive.getPose())) {
                  shootToHub(flywheel, turret, hood, drive, alliance);
                }
              }
            },
            flywheel,
            turret,
            hood)
        .until(
            () -> {
              return flywheel.noGamePiecesToShoot();
            })
        .finallyDo(
            () -> {
              System.out.println("no game pieces to shoot");
              flywheel.setRelativeVelocity(0);
              Drive.driveSpeedMultiplier = 1;
            });
  }

  public static Command shootCommand(Flywheel flywheel, Turret turret, Hood hood, Drive drive) {
    return Commands.run(
            () -> {
              Optional<Alliance> allianceOptional = DriverStation.getAlliance();
              if (allianceOptional.isPresent()) {
                Alliance alliance = allianceOptional.get();
                if (RebuiltFieldUtils.isInAllianceZone(alliance, drive.getPose())) {
                  shootToHub(flywheel, turret, hood, drive, alliance);
                } else {
                  shuttle(flywheel, turret, hood, drive, alliance);
                }
              }
            },
            flywheel,
            turret,
            hood)
        .finallyDo(
            () -> {
              flywheel.setRelativeVelocity(0);
              Drive.driveSpeedMultiplier = 1;
            });
  }

  public static Command pointToHub(Turret turret, Drive drive) {
    return Commands.run(
        () -> {
          Optional<Alliance> allianceOptional = DriverStation.getAlliance();
          TrackingTargets target =
              allianceOptional.get() == Alliance.Blue
                  ? TrackingTargets.blueHub
                  : TrackingTargets.redHub;
          Translation3d targetTrans = target.getTranslation3d();
          Translation3d robotTrans = drive.getPose3d().getTranslation();
          double angRad =
              Math.atan2(
                  targetTrans.getY() - robotTrans.getY(), targetTrans.getX() - robotTrans.getX());
          Logger.recordOutput("turret angle deg", Units.radiansToDegrees(angRad));
          turret.setDesiredAngleFieldRelative(
              Rotation2d.fromRadians(angRad), drive.getPose().getRotation());
        },
        turret);
  }
}
