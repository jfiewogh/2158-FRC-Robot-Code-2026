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

package org.austincans.frc2158.commands;

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
import java.util.Set;
import org.austincans.frc2158.subsystems.drive.Drive;
import org.austincans.frc2158.subsystems.flywheel.Flywheel;
import org.austincans.frc2158.subsystems.flywheel.FlywheelConstants;
import org.austincans.frc2158.subsystems.hood.Hood;
import org.austincans.frc2158.subsystems.turret.Turret;
import org.austincans.frc2158.subsystems.turret.TurretConstants;
import org.austincans.frc2158.util.ShooterMath;

public class TurretCommands {
  public enum TrackingTargets {
    redHub(new Translation3d(11.938, 4.034536, 1.5748)),
    blueHub(new Translation3d(4.5974, 4.034536, 1.5748)),
    blueOutTrench(new Translation3d(4.625594, 0.632968, 0)),
    blueDepotTrench(new Translation3d(4.625594, 7.436358, 0)),
    redDepotTrench(new Translation3d(11.915394, 0.632968, 0)),
    redLeftTrench(new Translation3d(11.915394, 7.436358, 0)),

    redShuttleLeft(new Translation3d(14, 3, 0)),
    redShuttleRight(new Translation3d(14, 6, 0)),
    blueShuttleRight(new Translation3d(2, 3, 0)),
    blueShuttleLeft(new Translation3d(2, 6, 0));

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
  private static final double FLYWHEEL_RADIUS = FlywheelConstants.radiusMeters; // meters
  private static final double FLYWHEEL_MAXRPM = FlywheelConstants.maxRPM;

  private static final double TURRET_HEIGHT = TurretConstants.turretHeight; // meters
  public static boolean isShuttling = false;
  private static final double latency = 0.02;

  private static double getActualExitVelocityMetersPerSec(double flywheelVelocityRPM) {
    return Units.rotationsToRadians(flywheelVelocityRPM) / 60 * FLYWHEEL_RADIUS;
  }

  /**
   * Only setter methods used for flywheel, turret, and hood. Only getter methods used for drive.
   */
  public static void shootToTarget(
      Flywheel flywheel, Turret turret, Hood hood, Drive drive, TrackingTargets target) {
    Pose2d robotPose = drive.getPose();
    ChassisSpeeds robotSpeeds = drive.getChassisSpeeds();

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
            .plus(new Rotation2d(robotSpeeds.omegaRadiansPerSecond))
            .times(latency);

    Translation2d turretTranslation =
        futureRobotTranslation.plus(TURRET_OFFSET.rotateBy(futureRobotRotation));

    double flywheelVelocityRPM = FLYWHEEL_MAXRPM * 0.75;

    flywheel.setVelocity(0);

    // TOTAL VALUES

    double totalVelocity = getActualExitVelocityMetersPerSec(flywheelVelocityRPM);

    double totalXYDistance =
        turretTranslation.getDistance(target.getTranslation3d().toTranslation2d());

    double totalPitchRad =
        ShooterMath.calculateProjectileAngle(
            TURRET_HEIGHT, totalVelocity, totalXYDistance, targetHeight);

    if (Double.isNaN(totalPitchRad)) {
      System.out.println("Cannot shoot into hub (1)");
    } else {
      Rotation2d totalYaw =
          Rotation2d.fromRadians(
              Math.atan2(
                  target.getTranslation3d().getY() - turretTranslation.getY(),
                  target.getTranslation3d().getX() - turretTranslation.getX()));

      double totalXYVelocity = totalVelocity * Math.cos(totalPitchRad);
      double totalXVelocity = totalXYVelocity * totalYaw.getCos();
      double totalYVelocity = totalXYVelocity * totalYaw.getSin();

      // CALCULATE EFFECTIVE YAW (TURRET ANGLE)

      double effectiveSpeedX = totalXVelocity - robotSpeeds.vxMetersPerSecond;
      double effectiveSpeedY = totalYVelocity - robotSpeeds.vyMetersPerSecond;
      double effectiveSpeed = Math.hypot(effectiveSpeedX, effectiveSpeedY);
      Rotation2d turretAngle = Rotation2d.fromRadians(Math.atan2(effectiveSpeedY, effectiveSpeedX));

      // CALCULATE EFFECTIVE PITCH (HOOD ANGLE)

      double totalTime = totalXYDistance / totalXYVelocity;

      double hoodAngleRad =
          ShooterMath.calculateProjectileAngle(
              TURRET_HEIGHT, totalVelocity, effectiveSpeed * totalTime, targetHeight);

      if (Double.isNaN(hoodAngleRad)) {
        System.out.println("Cannot shoot into hub (2)");
      } else {
        // if (target.getTranslation3d().getY() != 4.034536) {
        // hoodAngleRad = Math.PI / 2;
        // }

        // Should run only if angle is possible
        flywheel.setVelocity(flywheelVelocityRPM);
        turret.setDesiredAngleFieldRelative(turretAngle);
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

  public static Command shuttle(Flywheel flywheel, Turret turret, Hood hood, Drive drive) {
    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    if (allianceOptional.isPresent()) {
      Alliance alliance = allianceOptional.get();
      return Commands.defer(
          () ->
              Commands.run(
                  () -> {
                    shootToTarget(flywheel, turret, hood, drive, getShuttleTarget(drive, alliance));
                    isShuttling = true;
                  }),
          Set.of());
    }
    return Commands.none();
  }

  public static Command shootToHub(Flywheel flywheel, Turret turret, Hood hood, Drive drive) {
    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    if (allianceOptional.isPresent()) {
      Alliance alliance = allianceOptional.get();
      return Commands.run(
              () -> {
                if (RebuiltFieldCommands.isInAllianceZone(alliance, drive.getPose())) {
                  shootToTarget(
                      flywheel,
                      turret,
                      hood,
                      drive,
                      alliance == Alliance.Blue ? TrackingTargets.blueHub : TrackingTargets.redHub);
                }
              },
              flywheel,
              turret,
              hood)
          .until(
              () -> {
                System.out.println(flywheel.noGamePiecesToShoot());
                return flywheel.noGamePiecesToShoot();
              })
          .handleInterrupt(
              () -> {
                System.out.println("no game pieces to shoot");
                flywheel.setVelocity(0);
              });
    }
    return Commands.none();
  }
}
