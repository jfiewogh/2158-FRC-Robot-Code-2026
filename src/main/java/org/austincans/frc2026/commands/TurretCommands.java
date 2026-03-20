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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.austincans.frc2026.Constants;
import org.austincans.frc2026.subsystems.drive.Drive;
import org.austincans.frc2026.subsystems.flywheel.Flywheel;
import org.austincans.frc2026.subsystems.flywheel.FlywheelConstants;
import org.austincans.frc2026.subsystems.hood.Hood;
import org.austincans.frc2026.subsystems.turret.Turret;
import org.austincans.frc2026.subsystems.turret.TurretConstants;
import org.littletonrobotics.junction.Logger;

public class TurretCommands {
  public static double flywheelExitVelocityConstant = 1;
  public static double robotShootTime = 2;
  public static InterpolatingDoubleTreeMap distancesAnglesFromHub = InterpolatingDoubleTreeMap
      .ofEntries(Map.entry(0.0, 0.0), Map.entry(1.0, 1.0));
  public static InterpolatingDoubleTreeMap distancesAnglesFromShuttlingPoint = InterpolatingDoubleTreeMap
      .ofEntries(Map.entry(5.0, 5.0), Map.entry(10.0, 10.0));

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
   * Only setter methods used for flywheel, turret, and hood. Only getter methods
   * used for drive.
   */
  public static double getHoodAngleFromDistance(
      double distance, InterpolatingDoubleTreeMap points) {
    return points.get(distance);
  }

  public static Translation2d getFutureTranslation(Translation2d currentTranslation, double time,
      Translation2d speeds) {
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

  // public static void shootToTarget(
  // Flywheel flywheel,
  // Turret turret,
  // Hood hood,
  // Drive drive,
  // TrackingTargets target,
  // InterpolatingDoubleTreeMap data,
  // double timePassed
  // ) {
  // Pose2d robotPose = drive.getPose();
  // ChassisSpeeds robotSpeeds = drive.getChassisSpeeds();
  // Drive.driveSpeedMultiplier = 0.5;
  // Translation3d speeds =
  // new Translation3d(robotSpeeds.vxMetersPerSecond,
  // robotSpeeds.vyMetersPerSecond, 0);

  // double targetHeight = target.getTranslation3d().getZ();

  // Translation2d futureRobotTranslation =
  // robotPose
  // .getTranslation()
  // .plus(
  // new Translation2d(robotSpeeds.vxMetersPerSecond,
  // robotSpeeds.vyMetersPerSecond)
  // .times(latency));
  // Rotation2d futureRobotRotation =
  // robotPose
  // .getRotation()
  // .plus(new Rotation2d(robotSpeeds.omegaRadiansPerSecond).times(latency));

  // Translation2d turretTranslation =
  // futureRobotTranslation.plus(TURRET_OFFSET.rotateBy(futureRobotRotation));

  // flywheel.setRelativeVelocity(0);
  // double hoodAngleRad =
  // getHoodAngleFromDistance(getDistance(getFuturePose(timePassed, speeds,
  // drive.getPose3d().getTranslation(), drive.getPose3d().getRotation()),
  // target.getTranslation3d()), data);

  // // TOTAL VALUES
  // double totalVelocityMetersPerSec =
  // getActualExitVelocityMetersPerSec(FLYWHEEL_SHOOT_SPEED) *
  // flywheelExitVelocityConstant;
  // double totalXYDistanceMeters =
  // turretTranslation.getDistance(target.getTranslation3d().toTranslation2d());
  // // double totalPitchRad =
  // // ShooterMath.calculateProjectileAngle(
  // // TURRET_HEIGHT, totalVelocityMetersPerSec, totalXYDistanceMeters,
  // targetHeight);

  // if (Double.isNaN(hoodAngleRad)) {
  // System.out.println("Cannot shoot into hub (1)");
  // } else {
  // Rotation2d totalYaw =
  // Rotation2d.fromRadians(
  // Math.atan2(
  // target.getTranslation3d().getY() - turretTranslation.getY(),
  // target.getTranslation3d().getX() - turretTranslation.getX()));
  // double totalXYVelocity = totalVelocityMetersPerSec * Math.cos(hoodAngleRad);

  // Translation3d velocityVector =
  // new Translation3d(
  // totalXYVelocity * totalYaw.getCos(),
  // totalXYVelocity * totalYaw.getSin(),
  // totalVelocityMetersPerSec * Math.sin(hoodAngleRad));

  // // CALCULATE EFFECTIVE YAW (TURRET ANGLE)
  // Translation3d effectiveSpeedVector = speeds.minus(velocityVector);
  // double effectiveSpeed = Math.hypot(effectiveSpeedVector.getX(),
  // effectiveSpeedVector.getY());
  // Rotation2d turretAngle =
  // Rotation2d.fromRadians(
  // Math.atan2(effectiveSpeedVector.getY(), effectiveSpeedVector.getY()));

  // // CALCULATE EFFECTIVE PITCH (HOOD ANGLE)
  // double totalTime = totalXYDistanceMeters / totalXYVelocity;

  // if (Double.isNaN(hoodAngleRad)) {
  // System.out.println("Cannot shoot into hub (2)");
  // } else if (Constants.isSim
  // && (turretAngle.minus(futureRobotRotation).getRotations() > 2.75
  // || turretAngle.minus(futureRobotRotation).getRotations() < -2.75)) {
  // System.out.println("Cannot Rotate Turret to Position");
  // } else {
  // // Should run only if angle is possible
  // flywheel.setRelativeVelocity(FLYWHEEL_SHOOT_SPEED);
  // turret.setDesiredAngleFieldRelative(turretAngle, futureRobotRotation);
  // Logger.recordOutput("turret field rel angle", turretAngle);
  // hood.setAngle(Rotation2d.fromRadians(hoodAngleRad));
  // }
  // }
  // }

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
      possibleTargets = new TrackingTargets[] { TrackingTargets.blueShuttleLeft, TrackingTargets.blueShuttleRight };
    } else {
      possibleTargets = new TrackingTargets[] { TrackingTargets.redShuttleLeft, TrackingTargets.redShuttleRight };
    }
    return getClosestTarget(robotPose, possibleTargets);
  }

  // private static void shuttle(
  // Flywheel flywheel, Turret turret, Hood hood, Drive drive, Alliance alliance,
  // double timePassed) {
  // TrackingTargets target = getShuttleTarget(drive, alliance);
  // shootToTarget(
  // flywheel, turret, hood, drive, target,
  // distancesAnglesFromShuttlingPoint,timePassed);
  // }

  // // TODO: Maybe make it so that robot does not shoot over hub
  // public static Command shuttleCommand(Flywheel flywheel, Turret turret, Hood
  // hood, Drive drive) {
  // return Commands.run(
  // () -> {
  // Optional<Alliance> allianceOptional = DriverStation.getAlliance();
  // if (allianceOptional.isPresent()) {
  // shuttle(flywheel, turret, hood, drive, allianceOptional.get());
  // }
  // },
  // flywheel,
  // turret,
  // hood)
  // .until(flywheel::noGamePiecesToShoot)
  // .finallyDo(
  // () -> {
  // flywheel.setRelativeVelocity(0);
  // Drive.driveSpeedMultiplier = 1;
  // isShuttling = false;
  // });
  // }

  // private static void shootToHub(
  // Flywheel flywheel,
  // Turret turret,
  // Hood hood,
  // Drive drive,
  // Alliance alliance,
  // double timePassed) {
  // shootToTarget(
  // flywheel,
  // turret,
  // hood,
  // drive,
  // alliance == Alliance.Blue ? TrackingTargets.blueHub : TrackingTargets.redHub,
  // distancesAnglesFromHub,timePassed);
  // }

  // public static Command shootToHubCommand(
  // Flywheel flywheel, Turret turret, Hood hood, Drive drive) {
  // return Commands.run(
  // () -> {
  // Optional<Alliance> allianceOptional = DriverStation.getAlliance();
  // if (allianceOptional.isPresent()) {
  // Alliance alliance = allianceOptional.get();
  // if (RebuiltFieldUtils.isInAllianceZone(alliance, drive.getPose())) {
  // shootToHub(
  // flywheel,
  // turret,
  // hood,
  // drive,
  // alliance,
  // getRobotDistanceFromHub(drive, alliance));
  // }
  // }
  // },
  // flywheel,
  // turret,
  // hood)
  // .until(
  // () -> {
  // return flywheel.noGamePiecesToShoot();
  // })
  // .finallyDo(
  // () -> {
  // System.out.println("no game pieces to shoot");
  // flywheel.setRelativeVelocity(0);
  // Drive.driveSpeedMultiplier = 1;
  // });
  // }

  // public static Command shootCommand(Flywheel flywheel, Turret turret, Hood
  // hood, Drive drive) {
  // return Commands.run(
  // () -> {
  // Optional<Alliance> allianceOptional = DriverStation.getAlliance();
  // if (allianceOptional.isPresent()) {
  // Alliance alliance = allianceOptional.get();
  // if (RebuiltFieldUtils.isInAllianceZone(alliance, drive.getPose())) {
  // shootToHub(
  // flywheel,
  // turret,
  // hood,
  // drive,
  // alliance,
  // );
  // } else {
  // shuttle(flywheel, turret, hood, drive, alliance);
  // }
  // }
  // },
  // flywheel,
  // turret,
  // hood)
  // .finallyDo(
  // () -> {
  // flywheel.setRelativeVelocity(0);
  // Drive.driveSpeedMultiplier = 1;
  // });
  // }

  public static void shootToTarget(Drive drive, Turret turret, Hood hood, Flywheel flywheel, TrackingTargets target,
      InterpolatingDoubleTreeMap data) {
    Translation2d speeds = new Translation2d(drive.getChassisSpeeds().vxMetersPerSecond,
        drive.getChassisSpeeds().vyMetersPerSecond);
    Translation2d futureTranslation = getFutureTranslation(drive.getPose().getTranslation(), robotShootTime, speeds);
    double distancefromTarget = futureTranslation.getDistance(target.getTranslation3d().toTranslation2d());
    Translation2d tranlationFromTarget = futureTranslation.minus(target.getTranslation3d().toTranslation2d());
    Rotation2d turretAngle = Rotation2d
        .fromRadians(Math.atan(tranlationFromTarget.getY() / tranlationFromTarget.getX()));
    flywheel.setRelativeVelocity(
        getDistance(futureTranslation, target.getTranslation3d().toTranslation2d()) / robotShootTime);
    turret.setDesiredAngleRobotRelative(turretAngle);
    hood.setAngle(Rotation2d.fromRadians(getHoodAngleFromDistance(distancefromTarget, data)));
  }

  private static void shuttle(
      Flywheel flywheel, Turret turret, Hood hood, Drive drive, Alliance alliance) {
    TrackingTargets target = getShuttleTarget(drive, alliance);
    shootToTarget(
        drive, turret, hood, flywheel, target, distancesAnglesFromShuttlingPoint);
  }

  private static void shootToHub(
      Flywheel flywheel,
      Turret turret,
      Hood hood,
      Drive drive,
      Alliance alliance) {
    shootToTarget(drive, turret, hood, flywheel,
        alliance == Alliance.Blue ? TrackingTargets.blueHub : TrackingTargets.redHub, distancesAnglesFromHub);
  }

  public static Command shootToHubCommand(
      Flywheel flywheel, Turret turret, Hood hood, Drive drive) {
    return Commands.run(
        () -> {
          Optional<Alliance> allianceOptional = DriverStation.getAlliance();
          if (allianceOptional.isPresent()) {
            Alliance alliance = allianceOptional.get();
            if (RebuiltFieldUtils.isInAllianceZone(alliance, drive.getPose())) {
              shootToHub(
                  flywheel,
                  turret,
                  hood,
                  drive,
                  alliance);
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

  public static Command shootCommand(Flywheel flywheel, Turret turret, Hood hood, Drive drive) {
    return Commands.run(
        () -> {
          Optional<Alliance> allianceOptional = DriverStation.getAlliance();
          if (allianceOptional.isPresent()) {
            Alliance alliance = allianceOptional.get();
            if (RebuiltFieldUtils.isInAllianceZone(alliance, drive.getPose())) {
              shootToHub(
                  flywheel,
                  turret,
                  hood,
                  drive,
                  alliance);
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

  // TODO: fix
  public static Command pointToHub(Turret turret, Drive drive) {
    return Commands.run(
        () -> {
          Optional<Alliance> allianceOptional = DriverStation.getAlliance();
          TrackingTargets target = allianceOptional.get() == Alliance.Blue
              ? TrackingTargets.blueHub
              : TrackingTargets.redHub;
          Translation3d targetTrans = target.getTranslation3d();
          Translation3d robotTrans = drive.getPose3d().getTranslation();
          double angRad = Math.atan2(
              targetTrans.getY() - robotTrans.getY(),
              targetTrans.getX() - robotTrans.getX());
          Logger.recordOutput("turret angle deg", Units.radiansToDegrees(angRad));
          turret.setDesiredAngleFieldRelative(
              Rotation2d.fromRadians(angRad), drive.getPose().getRotation());
        },
        turret)
        .withTimeout(5);
  }

  private static Rotation2d setpointJoystickTurretAngle = Rotation2d.kZero; // should be zero on start

  public static Command joystickTurretAngleCommand(
      Turret turret, Drive drive, DoubleSupplier vertSupplier, DoubleSupplier horizSupplier) {
    return Commands.run(
        () -> {
          if (Math.abs(vertSupplier.getAsDouble()) > 0.1
              && Math.abs(horizSupplier.getAsDouble()) > 0.1) {
            setpointJoystickTurretAngle = Rotation2d.fromRadians(
                Math.atan2(horizSupplier.getAsDouble(), vertSupplier.getAsDouble()));
            turret.setDesiredAngleRobotRelative(
                setpointJoystickTurretAngle); // , drive.getPose().getRotation());
          }
        },
        turret);
  }
}
