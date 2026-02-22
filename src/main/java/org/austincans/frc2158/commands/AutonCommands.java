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
// Updated on Saturday February 21, 2026 at 23:31:39 CST

package org.austincans.frc2158.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
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
import org.austincans.frc2158.subsystems.drive.DriveConstants;
import org.austincans.frc2158.subsystems.flywheel.Flywheel;
import org.austincans.frc2158.subsystems.hood.Hood;
import org.austincans.frc2158.subsystems.intake.Intake;
import org.austincans.frc2158.subsystems.turret.Turret;
import org.austincans.frc2158.subsystems.vision.ObjectDetection;

public class AutonCommands {
  private static PIDController drivePIDX = new PIDController(2, 0.0, 0);
  private static PIDController drivePIDY = new PIDController(2, 0.0, 0);
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
                        Commands.run(
                            () -> {
                              if (!Drive.joystickHasInput
                                  && detection.getNearestGamePiecePose() != null) {
                                driveToPose(drive, detection.getNearestGamePiecePose().toPose2d());
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
        .onlyIf(() -> detection.getNearestGamePiecePose() != null)
        .onlyWhile(() -> detection.getNearestGamePiecePose() != null)
        .until(
            () ->
                intake.getVelocityRadPerSec() != 0
                        && Timer.getFPGATimestamp() - intakeStartTime > intakeDurationSeconds
                    || Drive.joystickHasInput)
        .finallyDo(
            () -> {
              System.out.println("STOP");
              drive.stop();
              intake.setRelativeVelocity(0);
            });
  }

  /*
   * Gal's Code
   * public static Command driveToHubAndShoot(Drive drive, Turret turret, Flywheel
   * flywheel, Hood hood, ObjectDetection detection, double allianceZone){
   * Alliance alliance = DriverStation.getAlliance().get();
   * BooleanSupplier inAllianceZone;
   * if (alliance != null){
   * if (alliance == Alliance.Blue){
   * inAllianceZone = () -> drive.getPoseSupplier().get().getX() < allianceZone;
   * }
   * else {
   * inAllianceZone = () -> drive.getPoseSupplier().get().getX() > allianceZone;
   * }
   * } else {
   * return Commands.none();
   * }
   * return Commands.parallel(
   * TurretCommands.shootToHub(flywheel, turret, hood).onlyIf(inAllianceZone),
   * findPathToPose()
   * );
   * }
   */

  private static double blueShootingX = Units.inchesToMeters(156.61 - 25);
  private static double redShootingX = Units.inchesToMeters(651.22 - 156.61 + 25);
  private static double shootingLeftY = Units.inchesToMeters(50.35 + 12 + 73 * 0.5);
  private static double shootingRightY = Units.inchesToMeters(317.688 - (50.59 + 12 + 73 * 0.5));
  private static Rotation2d initialRobotRotation = Rotation2d.kZero;
  private static ArrayList<Translation2d> blueHubShootingTranslations =
      new ArrayList<>(
          Arrays.asList(
              new Translation2d(blueShootingX, shootingLeftY),
              new Translation2d(blueShootingX, shootingRightY)));
  private static ArrayList<Translation2d> redHubShootingTranslations =
      new ArrayList<>(
          Arrays.asList(
              new Translation2d(redShootingX, shootingLeftY),
              new Translation2d(redShootingX, shootingRightY)));

  public static boolean isFullyInAllianceZone(Alliance alliance, Pose2d robotPose) {
    return alliance == Alliance.Blue
        ? robotPose.getX() < blueShootingX
        : robotPose.getX() > redShootingX;
  }

  public static Command driveToAndShootIntoHub(
      Drive drive,
      Turret turret,
      Flywheel flywheel,
      Hood hood,
      ObjectDetection detection,
      DoubleSupplier... suppliers) {
    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    if (allianceOptional.isPresent()) {
      Alliance alliance = allianceOptional.get();
      Supplier<Translation2d> targetTranslationSupplier =
          () -> {
            ArrayList<Translation2d> hubShootingTranslations =
                alliance == Alliance.Blue
                    ? blueHubShootingTranslations
                    : redHubShootingTranslations;
            return RobotState.getInstance()
                .getEstimatedPose()
                .getTranslation()
                .nearest(hubShootingTranslations);
          };
      return Commands.defer(
              () -> {
                Pose2d targetPose =
                    new Pose2d(targetTranslationSupplier.get(), initialRobotRotation);
                Command pathfindCommand =
                    PathCommands.pathfindToPoseCommand(drive, targetPose, 0)
                        .onlyIf(() -> !isFullyInAllianceZone(alliance, drive.getPose()))
                        .until(
                            () ->
                                isFullyInAllianceZone(alliance, drive.getPose())
                                    || getDistanceBetweenPoses(drive.getPose(), targetPose)
                                        < pathfindEndDistance)
                        .andThen(Commands.run(drive::stop)); // must use Commands.run to stop robot
                return Commands.parallel(
                    pathfindCommand,
                    updateJoystickHasInputCommand(suppliers),
                    TurretCommands.shootToHub(flywheel, turret, hood));
              },
              Set.of(drive, turret, flywheel, hood, detection))
          .beforeStarting(() -> initialRobotRotation = drive.getPose().getRotation())
          .until(() -> Drive.joystickHasInput);
    }
    return Commands.none();
  }

  private static Command updateJoystickHasInputCommand(DoubleSupplier... suppliers) {
    return Commands.run(() -> DriveCommands.updateJoystickHasInput(suppliers));
  }
}
