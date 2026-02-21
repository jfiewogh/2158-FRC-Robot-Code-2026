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
// Updated on Saturday February 21, 2026 at 16:51:46 CST

package org.austincans.frc2158.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.austincans.frc2158.RobotState;
import org.austincans.frc2158.subsystems.drive.Drive;
import org.austincans.frc2158.subsystems.flywheel.Flywheel;
import org.austincans.frc2158.subsystems.hood.Hood;
import org.austincans.frc2158.subsystems.intake.Intake;
import org.austincans.frc2158.subsystems.turret.Turret;
import org.austincans.frc2158.subsystems.vision.ObjectDetection;
import org.littletonrobotics.junction.Logger;

public class AutonCommands {
  private static final double maxVelocityMetersPerSec = 5;
  private static final double maxAccelerationMetersPerSecSq = 10;

  private static PIDController drivePIDX = new PIDController(2, 0.0, 0);
  private static PIDController drivePIDY = new PIDController(2, 0.0, 0);

  private static final double maxVelocityRadPerSec = Math.PI * 2;
  private static final double maxAccelerationRadPerSecSq = Math.PI * 4;
  private static ProfiledPIDController rotationPID =
      new ProfiledPIDController(
          1,
          0,
          0,
          new TrapezoidProfile.Constraints(maxVelocityRadPerSec, maxAccelerationRadPerSecSq));

  private static final PathConstraints constraints =
      new PathConstraints(
          maxVelocityMetersPerSec, maxAccelerationMetersPerSecSq,
          maxVelocityRadPerSec, maxAccelerationRadPerSecSq);

  private static final double pathfindEndDistanceMeters = 0.5;
  private static final double intakeDistanceMeters = 1;

  private static final double intakeDurationSeconds = 3;
  private static double intakeStartTime;
  private static DoubleSupplier time = Timer::getFPGATimestamp;

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

  private static double getDistanceBetweenPoses(Pose2d pose1, Pose2d pose2) {
    return pose1.getTranslation().getDistance(pose2.getTranslation());
  }

  private static Command driveToNearestGamePieceCommand(
      Drive drive, ObjectDetection objectDetection, Intake intake) {
    Supplier<Pose3d> nearestGamePiecePoseSupplier = objectDetection::getNearestGamePiecePose;
    return Commands.run(
            () -> {
              if (!Drive.joystickHasInput && nearestGamePiecePoseSupplier.get() != null) {
                driveToPose(drive, nearestGamePiecePoseSupplier.get().toPose2d());
              }
            },
            drive)
        .onlyWhile(() -> !Drive.joystickHasInput && nearestGamePiecePoseSupplier.get() != null)
        .raceWith(Commands.waitSeconds(5));
  }

  public static Command runIntakeCommand(
      Intake intake, ObjectDetection detection, Drive drive, double distance) {
    Supplier<Pose3d> target = detection.getNearestGamePiecePoseSupplier();
    Supplier<Pose2d> drivePose = drive.getPoseSupplier();
    return Commands.run(
            () -> {
              if (target.get() != null
                  && getDistanceBetweenPoses(target.get().toPose2d(), drivePose.get()) < distance) {
                intake.setRelativeVelocity(0.5);
              } else {
                intake.setRelativeVelocity(0);
              }
              Logger.recordOutput("intake duration", time.getAsDouble() - intakeStartTime);
            },
            intake)
        .beforeStarting(() -> intakeStartTime = time.getAsDouble())
        .onlyWhile(() -> target.get() != null);
  }

  public static Command pathfindToPoseCommand(Pose2d pose) {
    return AutoBuilder.pathfindToPose(pose, constraints);
  }

  public static Command pathfindToNearestGamePieceCommand(
      Drive drive, ObjectDetection objectDetection) {
    Supplier<Pose3d> targetPoseSupplier = objectDetection.getNearestGamePiecePoseSupplier();
    Supplier<Pose2d> currentPoseSupplier = drive::getPose;
    return Commands.defer(
            () ->
                pathfindToPoseCommand(
                    new Pose2d(
                        targetPoseSupplier.get().getTranslation().toTranslation2d(),
                        Rotation2d.fromRadians(
                            Math.atan2(
                                targetPoseSupplier.get().getY() - currentPoseSupplier.get().getY(),
                                targetPoseSupplier.get().getX()
                                    - currentPoseSupplier.get().getX())))),
            Set.of(drive))
        .onlyIf(() -> targetPoseSupplier.get() != null)
        .onlyWhile(
            () ->
                targetPoseSupplier.get() != null
                    && getDistanceBetweenPoses(
                            targetPoseSupplier.get().toPose2d(), currentPoseSupplier.get())
                        > pathfindEndDistanceMeters);
  }

  /* Gal's Code
  public static Command driveToHubAndShoot(Drive drive, Turret turret, Flywheel flywheel, Hood hood, ObjectDetection detection, double allianceZone){
    Alliance alliance = DriverStation.getAlliance().get();
    BooleanSupplier inAllianceZone;
    if (alliance != null){
      if (alliance == Alliance.Blue){
          inAllianceZone = () -> drive.getPoseSupplier().get().getX() < allianceZone;
      }
      else {
          inAllianceZone = () -> drive.getPoseSupplier().get().getX() > allianceZone;
      }
    } else {
      return Commands.none();
    }
    return Commands.parallel(
      TurretCommands.shootToHub(flywheel, turret, hood).onlyIf(inAllianceZone),
      findPathToPose()
    );
  }
  */

  private static double blueAllianceZoneX = Units.inchesToMeters(156.61 - 25);
  private static double redAllianceZoneX = Units.inchesToMeters(651.22 - 156.61 + 25);
  private static double shootingLeftY = Units.inchesToMeters(50.35 + 12 + 73 * 0.5);
  private static double shootingRightY = Units.inchesToMeters(317.688 - (50.59 + 12 + 73 * 0.5));
  private static Rotation2d initialRobotRotation = Rotation2d.kZero;
  private static ArrayList<Translation2d> blueHubShootingTranslations =
      new ArrayList<>(
          Arrays.asList(
              new Translation2d(blueAllianceZoneX, shootingLeftY),
              new Translation2d(blueAllianceZoneX, shootingRightY)));
  private static ArrayList<Translation2d> redHubShootingTranslations =
      new ArrayList<>(
          Arrays.asList(
              new Translation2d(redAllianceZoneX, shootingLeftY),
              new Translation2d(redAllianceZoneX, shootingRightY)));

  // TODO: fix
  // if the robot goes over the hump, the robot continues with a constant velocity
  public static Command pathFindToNearestHubShootingPose(Drive drive, Alliance alliance) {
    Supplier<Translation2d> targetTranslationSupplier =
        () -> {
          ArrayList<Translation2d> hubShootingTranslations =
              alliance == Alliance.Blue ? blueHubShootingTranslations : redHubShootingTranslations;
          return RobotState.getInstance()
              .getEstimatedPose()
              .getTranslation()
              .nearest(hubShootingTranslations);
        };
    return Commands.defer(
            () ->
                pathfindToPoseCommand(
                    new Pose2d(targetTranslationSupplier.get(), initialRobotRotation)),
            Set.of(drive))
        .onlyIf(() -> !TurretCommands.isInAllianceZone(alliance, drive.getPose()))
        .until(() -> TurretCommands.isInAllianceZone(alliance, drive.getPose()))
        .handleInterrupt(() -> drive.runVelocity(new ChassisSpeeds()));
  }

  public static Command driveToHubAndShoot(
      Drive drive,
      Turret turret,
      Flywheel flywheel,
      Hood hood,
      ObjectDetection detection,
      DoubleSupplier... suppliers) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return Commands.parallel(
              updateJoystickHasInputCommand(suppliers),
              pathFindToNearestHubShootingPose(drive, alliance.get()),
              TurretCommands.shootToHub(flywheel, turret, hood))
          .beforeStarting(() -> initialRobotRotation = drive.getPose().getRotation())
          .until(() -> Drive.joystickHasInput);
    }
    return Commands.none();
  }

  public static Command driveToAndIntakeGamePieces(
      ObjectDetection objectDetection, Drive drive, Intake intake, DoubleSupplier... suppliers) {
    return Commands.deadline(
            Commands.waitSeconds(intakeDurationSeconds),
            updateJoystickHasInputCommand(suppliers),
            runIntakeCommand(intake, objectDetection, drive, intakeDistanceMeters),
            Commands.sequence(
                pathfindToNearestGamePieceCommand(drive, objectDetection),
                driveToNearestGamePieceCommand(drive, objectDetection, intake)))
        .until(() -> Drive.joystickHasInput)
        .handleInterrupt(
            () -> {
              intake.setRelativeVelocity(0);
              drive.stop();
            });
  }

  private static Command updateJoystickHasInputCommand(DoubleSupplier... suppliers) {
    return Commands.run(() -> DriveCommands.updateJoystickHasInput(suppliers));
  }
}
