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
// Updated on Saturday February 14, 2026 at 14:16:40 CST

package org.austincans.frc2158.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

import java.util.Set;
import org.austincans.frc2158.subsystems.drive.Drive;
import org.austincans.frc2158.subsystems.vision.ObjectDetection;

public class LimelightHelpersCommands {

  static final double Kp = 0.2;
  static final double Ki = 0;
  static final double Kd = 0;
  static final double maxVelocity = 5;
  static final double maxAcceleration = 10;
  static ProfiledPIDController pid =
      new ProfiledPIDController(
          Kp, Ki, Kd, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));

  private static void alignRobotToPose(Drive drive, Pose2d pose) {
    drive.runVelocity(
        new ChassisSpeeds(
            0,
            0,
            pid.calculate(
                drive.getPose().getRotation().getDegrees(), pose.getRotation().getDegrees())));
  }

  public static Command alignHeadingCommand(Drive drive, ObjectDetection objectDetection) {
    return Commands.defer(
        () ->
            Commands.run(
                () -> {
                  System.out.println(objectDetection.getNearestGamePiecePose().toPose2d());
                  if (objectDetection.getGamePiecePoses().length != 0){
                    alignRobotToPose(drive, objectDetection.getNearestGamePiecePose().toPose2d());
                  }
                }),
        Set.of(drive, objectDetection));
  }
}
