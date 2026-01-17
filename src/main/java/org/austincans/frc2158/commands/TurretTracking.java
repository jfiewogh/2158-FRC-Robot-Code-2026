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
// Updated on Saturday January 17, 2026 at 17:14:59 CST

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

    flywheel.setVelocity(FLYWHEEL_MAXRPM * 0.75);

    // Effective values

    double effectiveVelocity =
        Units.rotationsToRadians(FLYWHEEL_MAXRPM * 0.75) / 60 * FLYWHEEL_RADIUS;

    double effectiveXYDistance = turretTranslation.getDistance(hubTranslation);

    double effectivePitchRad =
        ShooterMath.calculateProjectileAngle(
            TURRET_HEIGHT, effectiveVelocity, effectiveXYDistance, TARGET_HEIGHT);
    System.out.println("first pitch: " + effectivePitchRad);

    Rotation2d effectiveYaw =
        Rotation2d.fromRadians(
            Math.atan2(
                hubTranslation.getY() - turretTranslation.getY(),
                hubTranslation.getX() - turretTranslation.getX()));
    System.out.println("first yaw: " + effectiveYaw.getRadians());

    double effectiveXYVelocity = effectiveVelocity * Math.cos(effectivePitchRad);
    double effectiveXVelocity = effectiveXYVelocity * effectiveYaw.getCos();
    double effectiveYVelocity = effectiveXYVelocity * effectiveYaw.getSin();

    // CALCULATE TURRET YAW

    ChassisSpeeds robotSpeeds = robotVelSupplier.get();
    double turretSpeedX = effectiveXVelocity - robotSpeeds.vxMetersPerSecond;
    double turretSpeedY = effectiveYVelocity - robotSpeeds.vyMetersPerSecond;
    double turretSpeed = Math.hypot(turretSpeedX, turretSpeedY);
    Rotation2d turretYaw = Rotation2d.fromRadians(Math.atan2(turretSpeedY, turretSpeedX));
    System.out.println("second yaw: " + turretYaw.getRadians());
    turret.setDesiredAngleFieldRelative(turretYaw, pose);

    // CALCULATE HOOD ANGLE

    double time = effectiveXYDistance / effectiveXYVelocity;

    double turretPitchRad =
        ShooterMath.calculateProjectileAngle(
            TURRET_HEIGHT, effectiveVelocity, turretSpeed * time, TARGET_HEIGHT);
    System.out.println("second pitch: " + turretPitchRad);

    hood.setAngle(Rotation2d.fromRadians(turretPitchRad));
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
