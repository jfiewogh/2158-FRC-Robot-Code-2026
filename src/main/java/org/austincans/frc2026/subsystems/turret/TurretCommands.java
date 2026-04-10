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

package org.austincans.frc2026.subsystems.turret;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.austincans.frc2026.subsystems.drive.Drive;
import org.austincans.frc2026.subsystems.superstructure.shooter.TargetUtil.TrackingTargets;
import org.littletonrobotics.junction.Logger;

public class TurretCommands {
  private TurretCommands() {}

  public static Command pointToHub(Optional<Turret> turret, Optional<Drive> drive) {
    if (turret.isEmpty() || drive.isEmpty()) {
      return Commands.none();
    }
    Turret t = turret.get();
    Drive d = drive.get();
    return Commands.run(
            () -> {
              Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
              TrackingTargets target =
                  alliance == Alliance.Blue ? TrackingTargets.blueHub : TrackingTargets.redHub;
              Translation3d targetTranslation = target.getTranslation3d();
              Pose3d drivePose = d.getPose3d();

              // Transform3d applies Translation3d first, then Rotation3d
              Pose3d turretPose = drivePose.plus(TurretConstants.robotRelTurretTransform);
              Translation3d turretTranslation = turretPose.getTranslation();
              Logger.recordOutput("turret pose", turretPose);

              Rotation2d fieldRelAngle =
                  targetTranslation
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

  private static Rotation2d setpointJoystickTurretAngle =
      Rotation2d.kZero; // should be zero on start

  public static Command joystickTurretAngleCommand(
      Turret turret, DoubleSupplier vertSupplier, DoubleSupplier horizSupplier) {
    return Commands.run(
        () -> {
          if (Math.abs(vertSupplier.getAsDouble()) > 0.1
              && Math.abs(horizSupplier.getAsDouble()) > 0.1) {
            setpointJoystickTurretAngle =
                Rotation2d.fromRadians(
                    Math.atan2(horizSupplier.getAsDouble(), vertSupplier.getAsDouble()));
            turret.setDesiredAngleRobotRelative(setpointJoystickTurretAngle);
          }
        },
        turret);
  }

  public static Command resetAngleCommand(Optional<Turret> turret) {
    return turret.map((t) -> t.resetAngle()).orElse(Commands.none());
  }

  public static Command setFieldRelTurretAngleCommand(
      Optional<Turret> turret, Rotation2d fieldRelAngle, Rotation2d driveFieldRelRotation) {
    return turret
        .map(
            (t) ->
                Commands.runOnce(
                    () -> t.setDesiredAngleFieldRelative(fieldRelAngle, driveFieldRelRotation), t))
        .orElse(Commands.none());
  }

  public static boolean atSetpoint(Optional<Turret> turret) {
    return turret.map((t) -> t.atSetpoint()).orElse(true);
  }
}
