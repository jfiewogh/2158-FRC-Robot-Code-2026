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
// Updated on Sunday February 22, 2026 at 14:08:53 CST

package org.austincans.frc2158.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.austincans.frc2158.subsystems.drive.Drive;
import org.austincans.frc2158.subsystems.drive.DriveConstants;

public class PathCommands {
  // Create the constraints to use while pathfinding
  public static final PathConstraints pathConstraints =
      new PathConstraints(
          DriveConstants.maxLinearSpeed, DriveConstants.maxLinearAcceleration,
          DriveConstants.maxAngularSpeed, DriveConstants.maxAngularAcceleration);

  public static Command pathfindToPoseCommand(
      Drive drive, Pose2d endPose, double goalEndVelocityMetersPerSec) {
    return AutoBuilder.pathfindToPose(endPose, pathConstraints, goalEndVelocityMetersPerSec);
  }

  /* Trajectory (currently unused) */

  private static PIDController xController = new PIDController(1, 0.0, 0);
  private static PIDController yController = new PIDController(1, 0.0, 0);
  private static ProfiledPIDController omegaController =
      new ProfiledPIDController(
          2,
          0,
          0,
          new TrapezoidProfile.Constraints(
              DriveConstants.maxAngularSpeed, DriveConstants.maxAngularAcceleration));

  static {
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private static final Timer followTrajectoryTimer = new Timer();
  private static final double endPathDistance = 0.1;

  public static Command followTrajectory(Drive drive, PathPlannerTrajectory trajectory) {
    return Commands.run(
            () -> {
              Pose2d currentPose = drive.getPose();
              Pose2d desiredPose = trajectory.sample(followTrajectoryTimer.get()).pose;
              drive.runVelocity(
                  new ChassisSpeeds(
                      xController.calculate(currentPose.getX(), desiredPose.getX()),
                      yController.calculate(currentPose.getY(), desiredPose.getY()),
                      omegaController.calculate(
                          currentPose.getRotation().getRadians(),
                          desiredPose.getRotation().getRadians())));
            })
        .beforeStarting(followTrajectoryTimer::restart)
        .until(
            () ->
                AutonCommands.getDistanceBetweenPoses(
                        drive.getPose(), trajectory.getEndState().pose)
                    < endPathDistance);
  }
}
