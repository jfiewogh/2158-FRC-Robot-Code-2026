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

package org.austincans.frc2026.superstructure;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.austincans.frc2026.subsystems.turret.Turret;
import org.austincans.frc2026.subsystems.turret.TurretConstants;
import org.austincans.frc2026.superstructure.shooter.TargetUtil.TrackingTargets;
import org.littletonrobotics.junction.Logger;

public class TurretWrapper extends SubsystemWrapper<Turret> {
  /** Should probably be moved to some constants class */
  private static final double deadband = 0.1;

  /** Should be zero on start */
  private Rotation2d setpointJoystickTurretAngle = Rotation2d.kZero;

  public TurretWrapper(Optional<Turret> subsystem) {
    super(subsystem);
  }

  public void setAngle(Rotation2d angle) {
    subsystem.ifPresent((s) -> s.setDesiredAngle(angle));
  }

  public void setRobotRelAngle(Rotation2d robotRelAngle) {
    subsystem.ifPresent((s) -> s.setDesiredAngleRobotRelative(robotRelAngle));
  }

  public void setFieldRelAngle(Rotation2d fieldRelAngle, Rotation2d robotFieldRelAngle) {
    subsystem.ifPresent((s) -> s.setDesiredAngleFieldRelative(fieldRelAngle, robotFieldRelAngle));
  }

  private void pointToHub(Turret turret, Pose3d fieldRelRobotPose) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
    TrackingTargets target =
        alliance == Alliance.Blue ? TrackingTargets.blueHub : TrackingTargets.redHub;
    Translation3d targetTranslation = target.getTranslation3d();

    // Transform3d applies Translation3d first, then Rotation3d
    Pose3d turretPose = fieldRelRobotPose.plus(TurretConstants.robotRelTurretTransform);
    Translation3d turretTranslation = turretPose.getTranslation();
    Logger.recordOutput("turret pose", turretPose);

    Rotation2d fieldRelAngle =
        targetTranslation.toTranslation2d().minus(turretTranslation.toTranslation2d()).getAngle();

    // We need this for calculating
    Logger.recordOutput(
        "turret distance from hub",
        turretTranslation.toTranslation2d().getDistance(targetTranslation.toTranslation2d()));

    turret.setDesiredAngleFieldRelative(
        fieldRelAngle, fieldRelRobotPose.getRotation().toRotation2d());
  }

  public Command pointToHubCommand(Supplier<Pose3d> fieldRelRobotPoseSupplier) {
    return subsystem
        .map((s) -> Commands.run(() -> pointToHub(s, fieldRelRobotPoseSupplier.get()), s))
        .orElse(Commands.none());
  }

  private void joystickRobotRelAngle(Turret turret, double rotationX, double rotationY) {
    if (Math.abs(rotationX) > deadband && Math.abs(rotationY) > deadband) {
      setpointJoystickTurretAngle = new Translation2d(rotationX, rotationY).getAngle();
    }
    turret.setDesiredAngleRobotRelative(setpointJoystickTurretAngle);
  }

  public Command joystickRobotRelAngleCommand(
      DoubleSupplier rotationXSupplier, DoubleSupplier rotationYSupplier) {
    return subsystem
        .map(
            (s) ->
                Commands.run(
                    () ->
                        joystickRobotRelAngle(
                            s, rotationXSupplier.getAsDouble(), rotationYSupplier.getAsDouble()),
                    s))
        .orElse(Commands.none());
  }

  public Command resetAngleCommand() {
    return subsystem.map((s) -> s.resetAngle()).orElse(Commands.none());
  }
}
