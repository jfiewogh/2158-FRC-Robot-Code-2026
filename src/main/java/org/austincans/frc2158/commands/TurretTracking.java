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
// Updated on Friday January 16, 2026 at 12:36:02 CST

package org.austincans.frc2158.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.Rotation;

import java.util.function.Supplier;

import org.austincans.frc2158.subsystems.Flywheel;
import org.austincans.frc2158.subsystems.hood.Hood;
import org.austincans.frc2158.subsystems.turret.Turret;
import org.austincans.frc2158.util.ShooterMath;

public class TurretTracking extends Command {

  private final Turret turret;
  private final Flywheel flywheel;
  private final Hood hood;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> robotVelSupplier;

  private static final double FLYWHEEL_RADIUS = 0.0508; // meters
  private static final double GEAR_RATIO = 1.0;
  private static final double FLYWHEEL_MAXRPM = 0;

  private final Translation3d blueHubPose = new Translation3d(4.5974, 4.034536, 1.5748);
  private final Translation3d redHubPose = new Translation3d(11.938, 4.034536, 1.5748);

  private Translation2d target = new Translation2d();

  private static final Translation2d TURRET_OFFSET = new Translation2d(0, 0); // meters

  private static final double TURRET_HEIGHT = 0.85; // meters
  private static final double TARGET_HEIGHT = 2.05; // meters

  public TurretTracking(
      Turret turret, Flywheel flywheel, Hood hood, Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> robotVelSupplier) {

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
    Translation2d hubPose;
    Pose2d pose = robotPoseSupplier.get();

    Translation2d turretPos =
        pose.getTranslation().plus(TURRET_OFFSET.rotateBy(pose.getRotation()));
    if (DriverStation.getAlliance().get() == Alliance.Blue){
      hubPose = blueHubPose.toTranslation2d();
    } else {
      hubPose = redHubPose.toTranslation2d();
    }
    flywheel.setVelocity(FLYWHEEL_MAXRPM * 0.75);
    hood.setAngle(Rotation2d.fromRadians(ShooterMath.calculateProjectileAngle(TURRET_HEIGHT, FLYWHEEL_MAXRPM * 0.75, turretPos.getDistance(hubPose), redHubPose.getZ())));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
