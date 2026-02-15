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
// Updated on Sunday February 15, 2026 at 11:44:15 CST

package org.austincans.frc2158.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import org.austincans.frc2158.subsystems.drive.Drive;
import org.austincans.frc2158.subsystems.vision.ObjectDetection;

public class LimelightHelpersCommands {

  private static final double kP = 3;
  private static final double kI = 0;
  private static final double kD = 0;
  private static final double maxVelocityRadPerSec = Math.PI;
  private static final double maxAccelerationRadPerSecSq = Math.PI * 2;
  private static ProfiledPIDController pid =
      new ProfiledPIDController(
          kP,
          kI,
          kD,
          new TrapezoidProfile.Constraints(maxVelocityRadPerSec, maxAccelerationRadPerSecSq));

  static {
    pid.enableContinuousInput(-Math.PI, Math.PI);
  }

  private static void alignRobotToPose(Drive drive, Pose2d pose) {
    drive.runVelocity(
        new ChassisSpeeds(
            0,
            0,
            pid.calculate(
                drive.getPose().getRotation().getRadians(),
                Math.atan2(
                    pose.getY() - drive.getPose().getY(), pose.getX() - drive.getPose().getX()))));
  }

  public static Command alignHeadingCommand(Drive drive, ObjectDetection objectDetection) {
    Supplier<Pose3d> nearestGamePiecePoseSupplier = objectDetection::getNearestGamePiecePose;
    return Commands.run(
        () -> {
          if (nearestGamePiecePoseSupplier.get() != null) {
            alignRobotToPose(drive, nearestGamePiecePoseSupplier.get().toPose2d());
          }
        });
  }
}
