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
// Updated on Saturday January 31, 2026 at 16:59:54 CST

package org.austincans.frc2158.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.austincans.frc2158.subsystems.flywheel.Flywheel;
import org.austincans.frc2158.subsystems.flywheel.FlywheelConstants;
import org.austincans.frc2158.subsystems.hood.Hood;
import org.austincans.frc2158.subsystems.turret.Turret;
import org.austincans.frc2158.subsystems.turret.TurretConstants;
import org.austincans.frc2158.util.ShooterMath;

public class TurretTracking extends Command {

  private final Turret turret;
  private final Flywheel flywheel;
  private final Hood hood;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> robotVelSupplier;

  private static final double FLYWHEEL_RADIUS = FlywheelConstants.radiusMeters; // meters
  private static final double GEAR_RATIO = FlywheelConstants.gearing;
  private static final double FLYWHEEL_MAXRPM = FlywheelConstants.maxRPM;

  private TrackingTargets target = TrackingTargets.redHub;

  private static final Translation2d TURRET_OFFSET = new Translation2d(0, 0); // meters

  private static final double TURRET_HEIGHT = TurretConstants.turretHeight; // meters
  private static final double TARGET_HEIGHT = Units.inchesToMeters(72); // meters

  public TurretTracking(
      Turret turret,
      Flywheel flywheel,
      Hood hood,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> robotVelSupplier) {

    this.turret = turret;
    this.flywheel = flywheel;
    this.hood = hood;
    this.robotPoseSupplier = robotPoseSupplier;
    this.robotVelSupplier = robotVelSupplier;

    addRequirements(turret, flywheel);
  }

  public void setTarget(TrackingTargets target) {
    this.target = target;
  }

  public enum TrackingTargets {
    redHub(new Translation3d(11.938, 4.034536, 1.5748)),
    blueHub(new Translation3d(4.5974, 4.034536, 1.5748)),
    blueOutTrench(new Translation3d(4.625594, 0.632968, 0)),
    blueDepotTrench(new Translation3d(4.625594, 7.436358, 0)),
    redDepotTrench(new Translation3d(11.915394, 0.632968, 0)),
    redLeftTrench(new Translation3d(11.915394, 7.436358, 0));

    private final Translation3d targetTrans3d;

    private TrackingTargets(
        Translation3d target) { // Constructor must be private or package-private
      this.targetTrans3d = target;
    }

    public Translation3d getTrans3d() { // Getter method
      return this.targetTrans3d;
    }
  }

  public static double getActualExitVelocityMetersPerSec(double flywheelVelocityRPM) {
    return Units.rotationsToRadians(flywheelVelocityRPM) / 60 * FLYWHEEL_RADIUS;
  }

  private static final double latency = 0.02;

  @Override
  public void execute() {
    Pose2d robotPose = robotPoseSupplier.get();
    ChassisSpeeds robotSpeeds = robotVelSupplier.get();

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

    double totalXYDistance = turretTranslation.getDistance(target.getTrans3d().toTranslation2d());

    double totalPitchRad =
        ShooterMath.calculateProjectileAngle(
            TURRET_HEIGHT, totalVelocity, totalXYDistance, TARGET_HEIGHT);

    if (Double.isNaN(totalPitchRad)) {
      System.out.println("Cannot shoot into hub (1)");
    } else {
      Rotation2d totalYaw =
          Rotation2d.fromRadians(
              Math.atan2(
                  target.getTrans3d().getY() - turretTranslation.getY(),
                  target.getTrans3d().getX() - turretTranslation.getX()));

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
              TURRET_HEIGHT, totalVelocity, effectiveSpeed * totalTime, TARGET_HEIGHT);

      if (Double.isNaN(hoodAngleRad)) {
        System.out.println("Cannot shoot into hub (2)");
      } else {
        if (target.getTrans3d().getY() != 4.034536) {
          hoodAngleRad = Math.PI / 2;
        }

        // Should run only if angle is possible
        flywheel.setVelocity(flywheelVelocityRPM);
        turret.setDesiredAngleFieldRelative(turretAngle, futureRobotRotation);
        hood.setAngle(Rotation2d.fromRadians(hoodAngleRad));
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    flywheel.setVelocity(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
