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
// Updated on Wednesday January 28, 2026 at 19:56:47 CST

package org.austincans.frc2158.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

  private final Translation3d blueHubPose = new Translation3d(4.5974, 4.034536, 1.5748);
  private final Translation3d redHubPose = new Translation3d(11.938, 4.034536, 1.5748);

  private Translation2d target = new Translation2d();

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

  public void setTarget(Translation2d target) {
    this.target = target;
  }

  @Override
  public void execute() {
    Translation2d hubTranslation;
    Pose2d pose = robotPoseSupplier.get();

    Translation2d turretTranslation =
        pose.getTranslation().plus(TURRET_OFFSET.rotateBy(pose.getRotation()));

    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      hubTranslation = blueHubPose.toTranslation2d();
    } else {
      hubTranslation = redHubPose.toTranslation2d();
    }

    double flywheelVelocityRPM = FLYWHEEL_MAXRPM * 0.75;

    // TOTAL VALUES

    double totalVelocity = Units.rotationsToRadians(flywheelVelocityRPM) / 60 * FLYWHEEL_RADIUS;

    double totalXYDistance = turretTranslation.getDistance(hubTranslation);

    double totalPitchRad =
        ShooterMath.calculateProjectileAngle(
            TURRET_HEIGHT, totalVelocity, totalXYDistance, TARGET_HEIGHT);

    if (Double.isNaN(totalPitchRad)) {
      System.out.println("Cannot shoot into hub");
    } else {
      flywheel.setVelocity(flywheelVelocityRPM);

      Rotation2d totalYaw =
          Rotation2d.fromRadians(
              Math.atan2(
                  hubTranslation.getY() - turretTranslation.getY(),
                  hubTranslation.getX() - turretTranslation.getX()));

      double totalXYVelocity = totalVelocity * Math.cos(totalPitchRad);
      double totalXVelocity = totalXYVelocity * totalYaw.getCos();
      double totalYVelocity = totalXYVelocity * totalYaw.getSin();

      // CALCULATE EFFECTIVE YAW (TURRET ANGLE)

      ChassisSpeeds robotSpeeds = robotVelSupplier.get();
      double effectiveSpeedX = totalXVelocity - robotSpeeds.vxMetersPerSecond;
      double effectiveSpeedY = totalYVelocity - robotSpeeds.vyMetersPerSecond;
      double effectiveSpeed = Math.hypot(effectiveSpeedX, effectiveSpeedY);
      Rotation2d turretAngle = Rotation2d.fromRadians(Math.atan2(effectiveSpeedY, effectiveSpeedX));
      turret.setDesiredAngleFieldRelative(turretAngle, pose);

      // CALCULATE EFFECTIVE PITCH (HOOD ANGLE)

      double totalTime = totalXYDistance / totalXYVelocity;

      double hoodAngleRad =
          ShooterMath.calculateProjectileAngle(
              TURRET_HEIGHT, totalVelocity, effectiveSpeed * totalTime, TARGET_HEIGHT);
      if (Double.isNaN(hoodAngleRad)) {
        hoodAngleRad = hood.getAngle().getRadians();
      }

      hood.setAngle(Rotation2d.fromRadians(hoodAngleRad));
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
