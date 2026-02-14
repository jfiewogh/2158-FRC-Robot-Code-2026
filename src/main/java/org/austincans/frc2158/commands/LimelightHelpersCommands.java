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
// Updated on Saturday February 14, 2026 at 11:44:36 CST

package org.austincans.frc2158.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.austincans.frc2158.RobotState;
import org.austincans.frc2158.subsystems.drive.Drive;
import org.austincans.frc2158.subsystems.vision.LimelightHelpers;
import org.austincans.frc2158.subsystems.vision.LimelightHelpers.LimelightTarget_Classifier;
import org.austincans.frc2158.subsystems.vision.ObjectDetection;

public class LimelightHelpersCommands {
  private final ObjectDetection nearestGamePiece;

  final double Kp = 0;
  final double Ki = 0;
  final double Kd = 0;
  final double maxVelocity = 5;
  final double maxAcceleration = 10;
  static LimelightTarget_Classifier[] detections =
      LimelightHelpers.getLatestResults("").targets_Classifier;
  ProfiledPIDController pid =
      new ProfiledPIDController(
          Kp, Ki, Kd, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));

  public LimelightHelpersCommands(ObjectDetection nearestGamePiece) {
    this.nearestGamePiece = nearestGamePiece;
  }

  public void alignRobotToAngle(Drive drive, Rotation2d goalDegrees) {
    drive.runVelocity(
        new ChassisSpeeds(
            0,
            0,
            pid.calculate(
                RobotState.getInstance().getRotation().getDegrees(), goalDegrees.getDegrees())));
  }

  public Command alignHeadingCommand(Drive drive) {
    return Commands.run(
        () -> alignRobotToAngle(drive, nearestGamePiece.getNearestPose().toPose2d().getRotation()));
  }
}
