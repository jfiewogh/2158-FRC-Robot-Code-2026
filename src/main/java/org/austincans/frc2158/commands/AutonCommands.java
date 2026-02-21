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
// Updated on Thursday February 19, 2026 at 20:08:44 CST

package org.austincans.frc2158.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.austincans.frc2158.subsystems.drive.Drive;
import org.austincans.frc2158.subsystems.intake.Intake;
import org.austincans.frc2158.subsystems.intake.IntakeConstants;
import org.austincans.frc2158.subsystems.vision.ObjectDetection;

public class AutonCommands {
  private static final double maxVelocityMetersPerSec = 5;
  private static final double maxAccelerationMetersPerSecSq = 10;

  private static PIDController drivePIDX = new PIDController(1, 0.0, 0);
  private static PIDController drivePIDY = new PIDController(1, 0.0, 0);

  private static final double maxVelocityRadPerSec = Math.PI * 2;
  private static final double maxAccelerationRadPerSecSq = Math.PI * 4;
  private static ProfiledPIDController rotationPID =
      new ProfiledPIDController(
          8,
          0,
          0,
          new TrapezoidProfile.Constraints(maxVelocityRadPerSec, maxAccelerationRadPerSecSq));

  static {
    rotationPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  private static void driveToPose(Drive drive, Pose2d pose) {
    ChassisSpeeds speed =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            drivePIDX.calculate(drive.getPose().getX(), pose.getX()),
            drivePIDY.calculate(drive.getPose().getY(), pose.getY()),
            rotationPID.calculate(
                drive.getPose().getRotation().getRadians(),
                Math.atan2(
                    pose.getY() - drive.getPose().getY(), pose.getX() - drive.getPose().getX())),
            drive.getRotation());
    drive.runVelocity(speed);
  }

  private static void intakeUntilFull(Intake intake) {
    if (intake.getGamePieceAmount() < IntakeConstants.maxIntakeAmount) {
      intake.setRelativeVelocity(0.5);
    } else {
      intake.setRelativeVelocity(0);
    }
  }

  public static Command driveToPoseCommand(Drive drive, ObjectDetection objectDetection) {
    Supplier<Pose2d> drivePose = drive::getPose;
    Supplier<Pose3d> nearestGamePiecePoseSupplier = objectDetection::getNearestGamePiecePose;

    return Commands.run(
            () -> {
              if (!Drive.joystickHasInput && nearestGamePiecePoseSupplier.get() != null) {
                driveToPose(drive, nearestGamePiecePoseSupplier.get().toPose2d());
              }
            },
            drive)
        .onlyWhile(
            () ->
                nearestGamePiecePoseSupplier.get() != null
                    && nearestGamePiecePoseSupplier
                            .get()
                            .getTranslation()
                            .getDistance(new Translation3d(drivePose.get().getTranslation()))
                        > 0.05
                    && !Drive.joystickHasInput);
  }

  public static Command intakeUntilFullCommand(Intake intake) {
    return Commands.runOnce(() -> intake.setRelativeVelocity(0.5));
  }

  private static final PathConstraints constraints =
      new PathConstraints(
          maxVelocityMetersPerSec, maxAccelerationMetersPerSecSq,
          maxVelocityRadPerSec, maxAccelerationRadPerSecSq);

  public static Command pathfindToPoseCommand(Drive drive, ObjectDetection objectDetection) {
    Supplier<Pose3d> nearestGamePiecePoseSupplier = objectDetection::getNearestGamePiecePose;
    Supplier<Pose2d> drivePos = () -> drive.getPose();

    return Commands.defer(
            () ->
                AutoBuilder.pathfindToPose(
                    new Pose2d(
                        nearestGamePiecePoseSupplier.get().getTranslation().toTranslation2d(),
                        Rotation2d.fromRadians(
                            Math.atan2(
                                nearestGamePiecePoseSupplier.get().getY() - drivePos.get().getY(),
                                nearestGamePiecePoseSupplier.get().getX()
                                    - drivePos.get().getX()))),
                    constraints),
            Set.of(drive))
        .onlyIf(() -> nearestGamePiecePoseSupplier.get() != null)
        .onlyWhile(
            () ->
                !Drive.joystickHasInput
                    && nearestGamePiecePoseSupplier.get() != null
                    && nearestGamePiecePoseSupplier
                            .get()
                            .getTranslation()
                            .getDistance(new Translation3d(drivePos.get().getTranslation()))
                        > 0.05);
  }

  public static Command goToHubAndShoot(Drive drive){
    return Commands.parallel{
        TurretCommands.shootToHub(flywheel,turret,hood,false,() -> intakeSimulation.getGamePiecesInIntake() > 0),
        

    }
  }

  public static Command alignDriveIntakeCommand(
      ObjectDetection objectDetection, Drive drive, Intake intake, DoubleSupplier... suppliers) {
    return Commands.parallel(
            Commands.run(() -> DriveCommands.updateJoystickHasInput(suppliers)),
            pathfindToPoseCommand(drive, objectDetection),
            intakeUntilFullCommand(intake))
        .onlyWhile(
            () ->
                intake.getGamePieceAmount() < IntakeConstants.maxIntakeAmount
                    && !Drive.joystickHasInput);
  }
}
