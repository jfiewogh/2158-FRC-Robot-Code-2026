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

package org.austincans.frc2026.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import java.util.Optional;
import org.austincans.frc2026.Constants;
import org.austincans.frc2026.commands.RebuiltFieldUtils;
import org.austincans.frc2026.commands.TurretCommands;
import org.austincans.frc2026.commands.TurretCommands.TrackingTargets;
import org.austincans.frc2026.commands.shooter.ShotCalculator.ShotInputs;
import org.austincans.frc2026.commands.shooter.ShotCalculator.ShotParameters;
import org.austincans.frc2026.subsystems.drive.Drive;
import org.austincans.frc2026.subsystems.flywheel.Flywheel;
import org.austincans.frc2026.subsystems.hood.Hood;
import org.austincans.frc2026.subsystems.turret.Turret;
import org.austincans.frc2026.subsystems.turret.TurretConstants;
import org.austincans.frc2026.util.LoggedTunableNumber;

/** Commands dealing with turret, hood, and flywheel */
// TODO: maybe make a ShooterSuperstructure class to handle all. this is low
// priority
public class ShooterCommands {
  private static ShotCalculator hubShotCalculator;
  private static final ShotCalculator shuttleShotCalculator;

  private final LoggedTunableNumber speedMultiplier =
      new LoggedTunableNumber("Speed Multiplier", 1);

  static {
    hubShotCalculator =
        new ShotCalculator(
            Constants.loopPeriodSecs,
            ShotConstants.scoringHoodAngleMap,
            ShotConstants.scoringFlywheelSpeedMap,
            ShotConstants.scoringTimeOfFlightMap,
            TurretConstants.robotRelTurretTransform);

    shuttleShotCalculator =
        new ShotCalculator(
            Constants.loopPeriodSecs,
            ShotConstants.passingHoodAngleMap,
            ShotConstants.passingFlywheelSpeedMap,
            ShotConstants.passingTimeOfFlightMap,
            TurretConstants.robotRelTurretTransform);
  }

  private static ShotParameters getHubShotParameters(Drive drive) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
    ShotInputs inputs =
        new ShotInputs(
            drive.getPose3d(),
            drive.getChassisSpeeds(),
            drive.getChassisSpeeds(),
            alliance == Alliance.Red
                ? TrackingTargets.redHub.getTranslation3d()
                : TrackingTargets.blueHub.getTranslation3d());
    return hubShotCalculator.calculate(inputs);
  }

  private static ShotParameters getShuttleShotParameters(Drive drive) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
    ShotInputs inputs =
        new ShotInputs(
            drive.getPose3d(),
            drive.getChassisSpeeds(),
            drive.getChassisSpeeds(),
            TurretCommands.getShuttleTarget(drive, alliance).getTranslation3d());
    return shuttleShotCalculator.calculate(inputs);
  }

  public static void trackHub(Drive drive, Hood hood, Turret turret) {
    ShotParameters parameters = getHubShotParameters(drive);
    // flywheel.setVelocity(parameters.flywheelSpeedRPM());
    hood.setAngle(parameters.hoodAngle());
    turret.setDesiredAngleFieldRelative(
        parameters.fieldRelTurretAngle(), parameters.driveFieldRelRotation2d());
  }

  public static boolean staticShooter = false;

  public static void shootToHub(Drive drive, Flywheel flywheel, Hood hood, Turret turret) {
    ShotParameters parameters = getHubShotParameters(drive);
    flywheel.runIndexerAndFlywheel(parameters.flywheelSpeedRPM());
    hood.setAngle(parameters.hoodAngle());

    if (staticShooter) {
      turret.setDesiredAngle(Rotation2d.kZero);
    } else {
      turret.setDesiredAngleFieldRelative(
          parameters.fieldRelTurretAngle(), parameters.driveFieldRelRotation2d());
    }
  }

  public static Command shootToHubCommand(
      Optional<Flywheel> flywheel,
      Optional<Hood> hood,
      Optional<Turret> turret,
      Optional<Drive> drive) {
    return flywheel
        .flatMap(
            (f) ->
                hood.flatMap(
                    (h) ->
                        turret.flatMap(
                            (t) ->
                                drive.map(
                                    (d) ->
                                        Commands.run(() -> shootToHub(d, f, h, t), d, f, h, t)
                                            .finallyDo(f::stopMotors)))))
        .orElse((WrapperCommand) Commands.none());
  }

  public static void trackShuttle(Drive drive, Hood hood, Turret turret) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
    hood.setAngle(Rotation2d.fromDegrees(50));
    turret.setDesiredAngleFieldRelative(
        alliance == Alliance.Blue ? Rotation2d.k180deg : Rotation2d.kZero, drive.getRotation());
  }

  public static Command shuttleTrackingCommand(Drive drive, Hood hood, Turret turret) {
    return Commands.run(() -> trackShuttle(drive, hood, turret));
  }

  public static void shuttle(Drive drive, Flywheel flywheel, Hood hood, Turret turret) {
    // ShotParameters parameters = getShuttleShotParameters(drive);

    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
    flywheel.runIndexerAndFlywheel(6000);
    hood.setAngle(Rotation2d.fromDegrees(60));
    turret.setDesiredAngleFieldRelative(
        alliance == Alliance.Blue ? Rotation2d.k180deg : Rotation2d.kZero, drive.getRotation());
  }

  public static Command shuttleCommand(
      Optional<Flywheel> flywheel,
      Optional<Hood> hood,
      Optional<Turret> turret,
      Optional<Drive> drive) {
    return flywheel
        .flatMap(
            (f) ->
                hood.flatMap(
                    (h) ->
                        turret.flatMap(
                            (t) ->
                                drive.map(
                                    (d) ->
                                        Commands.run(() -> shuttle(d, f, h, t), d, f, h, t)
                                            .finallyDo(f::stopMotors)))))
        .orElse((WrapperCommand) Commands.none());
  }

  public static Command shootCommand(
      Optional<Flywheel> flywheel,
      Optional<Hood> hood,
      Optional<Turret> turret,
      Optional<Drive> drive) {
    return flywheel
        .flatMap(
            (f) ->
                hood.flatMap(
                    (h) ->
                        turret.flatMap(
                            (t) ->
                                drive.map(
                                    (d) ->
                                        Commands.run(
                                                () -> {
                                                  Alliance alliance =
                                                      DriverStation.getAlliance()
                                                          .orElse(Alliance.Red);
                                                  if (RebuiltFieldUtils.isInAllianceZone(
                                                      alliance, d.getPose())) {
                                                    shootToHub(d, f, h, t);
                                                  } else {
                                                    shuttle(d, f, h, t);
                                                  }
                                                },
                                                d,
                                                f,
                                                h,
                                                t)
                                            .finallyDo(f::stopMotors)))))
        .orElse((WrapperCommand) Commands.none());
  }

  public static Command trackCommand(Turret turret, Hood hood, Drive drive) {
    return Commands.run(
            () -> {
              Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
              if (RebuiltFieldUtils.isInAllianceZone(alliance, drive.getPose())) {
                trackHub(drive, hood, turret);
              } else {
                trackShuttle(drive, hood, turret);
              }
            },
            turret,
            hood)
        .finallyDo(
            () -> {
              Drive.driveSpeedMultiplier = 1;
            });
  }

  public static Command staticShooterToggleCommand() {
    return Commands.runOnce(() -> staticShooter = !staticShooter);
  }
}
