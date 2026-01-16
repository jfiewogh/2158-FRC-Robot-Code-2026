package org.austincans.frc2158.commands;

import java.util.function.Supplier;

import org.austincans.frc2158.subsystems.Flywheel;
import org.austincans.frc2158.subsystems.hood.Hood;
import org.austincans.frc2158.subsystems.turret.Turret;
import org.austincans.frc2158.util.ShooterMath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class TurretTracking extends Command {

  private final Turret turret;
  private final Flywheel flywheel;
  private final Hood hood;
  private final Supplier<Pose2d> robotPoseSupplier;
  
  private static final double FLYWHEEL_RADIUS = 0.0508; //meters
  private static final double GEAR_RATIO = 1.0;

  private Translation2d target = new Translation2d();

  private static final Translation2d TURRET_OFFSET =
      new Translation2d(0, 0); // meters

  private static final double SHOOTER_HEIGHT = 0.85; // meters
  private static final double TARGET_HEIGHT = 2.05;  // meters

  public TurretTracking(
      Turret turret,
      Flywheel flywheel,
      Hood hood,
      Supplier<Pose2d> robotPoseSupplier) {

    this.turret = turret;
    this.flywheel = flywheel;
    this.hood = hood;
    this.robotPoseSupplier = robotPoseSupplier;

    addRequirements(turret, flywheel);
  }

  public void setTarget(Translation2d target) {
    this.target = target;
  }

  @Override
  public void execute() {
    Pose2d pose = robotPoseSupplier.get();

    Translation2d turretPos =
        pose.getTranslation()
            .plus(TURRET_OFFSET.rotateBy(pose.getRotation()));

    Rotation2d aimAngle =
        target.minus(turretPos).getAngle();

    turret.setDesiredAngleFieldRelative(aimAngle, pose);

    double distance = turretPos.getDistance(target);

    // Hood angle (initially fixed; later interpolate)
    Rotation2d hoodAngle = Rotation2d.fromDegrees(35.0);
    hood.setAngle(hoodAngle);

    // meters/sec
    double exitVelocity =
        ShooterMath.calculateExitVelocity(
            distance,
            TARGET_HEIGHT - SHOOTER_HEIGHT,
            hoodAngle.getRadians());

    if (!Double.isNaN(exitVelocity)) {
      flywheel.setVelocity(exitVelocity/FLYWHEEL_RADIUS * GEAR_RATIO);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}