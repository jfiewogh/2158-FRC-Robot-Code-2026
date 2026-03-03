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

package org.austincans.frc2026.commands;

import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.Pair;
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
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import org.austincans.frc2026.subsystems.drive.Drive;
import org.austincans.frc2026.subsystems.drive.DriveConstants;
import org.austincans.frc2026.subsystems.flywheel.Flywheel;
import org.austincans.frc2026.subsystems.hood.Hood;
import org.austincans.frc2026.subsystems.intake.Intake;
import org.austincans.frc2026.subsystems.turret.Turret;
import org.austincans.frc2026.subsystems.vision.ObjectDetection;

public class AutonCommands {
  private static PIDController drivePIDX = new PIDController(3, 0.0, 0);
  private static PIDController drivePIDY = new PIDController(3, 0.0, 0);
  private static ProfiledPIDController rotationPID =
      new ProfiledPIDController(
          1,
          0,
          0,
          new TrapezoidProfile.Constraints(
              DriveConstants.maxAngularSpeed, DriveConstants.maxAngularAcceleration));

  private static final double pathfindEndDistance = 0.5;
  private static final double intakeDistanceMeters = 1;
  private static final double intakeDurationSeconds = 3;
  private static double intakeStartTime;

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

  public static double getDistanceBetweenPoses(Pose2d pose1, Pose2d pose2) {
    return pose1.getTranslation().getDistance(pose2.getTranslation());
  }

  // NOTE: PathCommands.pathfindToPoseCommand does not work well in parallel with
  // other commands
  // If another command runs, there is a high chance that the
  // pathfindToPoseCommand stops working
  // and the robot continues to drive with its last velocity

  // TODO: put this in Constants file, different code should based on what field is chosen
  private static final double fieldLength = Units.inchesToMeters(651.22);
  private static final double fieldWidth = Units.inchesToMeters(317.69);

  private static boolean poseInAllianceHalf(Pose3d pose) {
    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    return allianceOptional.isPresent() && allianceOptional.get() == Alliance.Blue
        ? pose.getX() < fieldLength / 2 - DriveConstants.driveBaseRadius * 1.2
        : pose.getX() > fieldLength / 2 + DriveConstants.driveBaseRadius * 1.2;
  }

  private static boolean robotInAllianceHalf(Drive drive) {
    return poseInAllianceHalf(drive.getPose3d());
  }

  public static Command driveToAndIntakeGamePieces(
      Drive drive, ObjectDetection detection, Intake intake, DoubleSupplier... suppliers) {
    return Commands.defer(
            () ->
                // Pathfind (used to navigate quickly to nearest game piece)
                PathCommands.pathfindToPoseCommand(
                        drive,
                        new Pose2d(
                            detection.getNearestGamePiecePose().getTranslation().toTranslation2d(),
                            Rotation2d.fromRadians(
                                Math.atan2(
                                    detection.getNearestGamePiecePose().getY()
                                        - drive.getPose().getY(),
                                    detection.getNearestGamePiecePose().getX()
                                        - drive.getPose().getX()))),
                        0)
                    // PID Control (used for aligning and driving to nearby game pieces)
                    .andThen(
                        // TODO: this should run only if pathfind worked
                        Commands.run(
                            () -> {
                              Pose3d targetPose =
                                  DriverStation.isAutonomous()
                                      ? detection.getNearestGamePiecePoseSatisfyingCondition(
                                          drive, pose -> poseInAllianceHalf(pose))
                                      : detection.getNearestGamePiecePose();
                              if (!Drive.joystickHasInput && targetPose != null) {
                                if (!DriverStation.isAutonomous() || robotInAllianceHalf(drive)) {
                                  driveToPose(
                                      drive, detection.getNearestGamePiecePose().toPose2d());
                                }
                              } else {
                                drive.stop();
                              }
                            }))
                    // Intake
                    .alongWith(
                        Commands.run(
                            () -> {
                              if (detection.getNearestGamePiecePose() != null
                                  && getDistanceBetweenPoses(
                                          detection.getNearestGamePiecePose().toPose2d(),
                                          drive.getPose())
                                      < intakeDistanceMeters) {
                                if (intake.getVelocityRadPerSec() == 0) {
                                  intakeStartTime = Timer.getFPGATimestamp();
                                }
                                intake.setRelativeVelocity(0.5);
                              }
                            }))
                    .alongWith(Commands.run(() -> updateJoystickHasInputCommand(suppliers))),
            Set.of(drive, detection, intake))
        .beforeStarting(
            () -> {
              System.out.println("start drive intake command");
              //   System.out.println(detection.getNearestGamePiecePose());
              Pathfinding.setDynamicObstacles(
                  List.of(
                      DriverStation.getAlliance().get() == Alliance.Blue
                          ? Pair.of(
                              new Translation2d(
                                  fieldLength / 2 - DriveConstants.driveBaseRadius, 0),
                              new Translation2d(fieldLength, fieldWidth))
                          : Pair.of(
                              new Translation2d(0, 0),
                              new Translation2d(
                                  fieldLength / 2 + DriveConstants.driveBaseRadius, fieldWidth))),
                  drive.getPose().getTranslation());
            })
        .onlyIf(() -> detection.getNearestGamePiecePose() != null)
        .onlyWhile(() -> detection.getNearestGamePiecePose() != null)
        .until(
            () ->
                intake.getVelocityRadPerSec() != 0
                        && Timer.getFPGATimestamp() - intakeStartTime > intakeDurationSeconds
                    || Drive.joystickHasInput)
        .finallyDo(
            () -> {
              System.out.println("end drive intake command");
              drive.stop();
              intake.setRelativeVelocity(0);
              Pathfinding.setDynamicObstacles(List.of(), drive.getPose().getTranslation());
            });
  }

  private static final double blueShootingX = Units.inchesToMeters(156.61 - 25);
  private static final double redShootingX = Units.inchesToMeters(651.22 - 156.61 + 25);
  private static final double shootingLeftY = Units.inchesToMeters(50.35 + 12 + 73 * 0.5);
  private static final double shootingRightY =
      Units.inchesToMeters(317.688 - (50.59 + 12 + 73 * 0.5));
  private static final ArrayList<Translation2d> blueHubShootingTranslations =
      new ArrayList<>(
          Arrays.asList(
              new Translation2d(blueShootingX, shootingLeftY),
              new Translation2d(blueShootingX, shootingRightY)));
  private static final ArrayList<Translation2d> redHubShootingTranslations =
      new ArrayList<>(
          Arrays.asList(
              new Translation2d(redShootingX, shootingLeftY),
              new Translation2d(redShootingX, shootingRightY)));

  public static boolean isFullyInAllianceZone(Alliance alliance, Pose2d robotPose) {
    return alliance == Alliance.Blue
        ? robotPose.getX() < blueShootingX
        : robotPose.getX() > redShootingX;
  }

  private static Translation2d getTargetHubShootingTranslation(Drive drive, Alliance alliance) {
    return drive
        .getPose()
        .getTranslation()
        .nearest(
            alliance == Alliance.Blue ? blueHubShootingTranslations : redHubShootingTranslations);
  }

  private static Rotation2d targetRotation = Rotation2d.kZero;
  private static Rotation2d[] possibleBumpRotations =
      new Rotation2d[] {
        Rotation2d.fromDegrees(45),
        Rotation2d.fromDegrees(135),
        Rotation2d.fromDegrees(225),
        Rotation2d.fromDegrees(315)
      };

  private static double getDifferenceOfTwoRotations(Rotation2d first, Rotation2d second) {
    double difference = (first.getRadians() - second.getRadians()) % (Math.PI * 2);
    if (difference > Math.PI) {
      difference -= Math.PI * 2;
    }
    if (difference < -Math.PI) {
      difference += Math.PI * 2;
    }
    return Math.abs(difference);
  }

  public static Command driveToAndShootIntoHub(
      Drive drive,
      Turret turret,
      Flywheel flywheel,
      Hood hood,
      ObjectDetection detection,
      DoubleSupplier... suppliers) {
    return Commands.defer(
            () -> {
              Alliance alliance =
                  DriverStation.getAlliance().get(); // NOTE: assuming it is not null
              Pose2d targetPose =
                  new Pose2d(getTargetHubShootingTranslation(drive, alliance), targetRotation);
              Command pathfindCommand =
                  PathCommands.pathfindToPoseCommand(drive, targetPose, 0)
                      .onlyIf(
                          () -> !RebuiltFieldCommands.isInAllianceZone(alliance, drive.getPose()))
                      .until(
                          () ->
                              isFullyInAllianceZone(alliance, drive.getPose())
                                  || getDistanceBetweenPoses(drive.getPose(), targetPose)
                                      < pathfindEndDistance)
                      .andThen(Commands.run(drive::stop)); // must use Commands.run to stop robot
              return Commands.deadline(
                  TurretCommands.shootToHub(flywheel, turret, hood, drive),
                  pathfindCommand,
                  updateJoystickHasInputCommand(suppliers));
            },
            Set.of(drive, turret, flywheel, hood, detection))
        .beforeStarting(
            () -> {
              Rotation2d robotRotation = drive.getPose().getRotation();
              Rotation2d minRotation = possibleBumpRotations[0];
              double minDifference = getDifferenceOfTwoRotations(robotRotation, minRotation);
              for (int i = 1; i < possibleBumpRotations.length; i++) {
                Rotation2d possibleRotation = possibleBumpRotations[i];
                double difference = getDifferenceOfTwoRotations(robotRotation, possibleRotation);
                if (difference < minDifference) {
                  minDifference = difference;
                  minRotation = possibleRotation;
                }
              }
              targetRotation = minRotation;
            })
        .until(() -> Drive.joystickHasInput)
        .finallyDo(
            () -> {
              drive.stop();
              flywheel.setVelocity(0);
              System.out.println("end drive score command");
            });
  }

  private static Command updateJoystickHasInputCommand(DoubleSupplier... suppliers) {
    return Commands.run(() -> DriveCommands.updateJoystickHasInput(suppliers));
  }
}
