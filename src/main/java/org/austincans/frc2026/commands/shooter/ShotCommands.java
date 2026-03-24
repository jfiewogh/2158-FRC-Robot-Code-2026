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

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

public class ShotCommands {
  private static final ShotCalculator hubShotCalculator =
      new ShotCalculator(
          Constants.loopPeriodSecs,
          ShotConstants.scoringHoodAngleMap,
          ShotConstants.scoringFlywheelSpeedMap,
          ShotConstants.scoringTimeOfFlightMap,
          Transform3d.kZero);

  public static void shootToHub(Drive drive, Flywheel flywheel, Hood hood, Turret turret) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);

    ShotInputs inputs =
        new ShotInputs(
            drive.getPose3d(),
            drive.getRobotRelVelocities(),
            drive.getFieldRelVelocities(),
            alliance == Alliance.Red
                ? TrackingTargets.redHub.getTranslation3d()
                : TrackingTargets.blueHub.getTranslation3d());
    ShotParameters parameters = hubShotCalculator.calculate(inputs);

    flywheel.setVelocity(parameters.flywheelSpeedRPM());
    hood.setAngle(parameters.hoodAngle());
    turret.setDesiredAngleFieldRelative(
        parameters.fieldRelTurretAngle(), parameters.driveFieldRelRotation2d());
  }

  public static Command shootToHubCommand(
      Drive drive, Flywheel flywheel, Hood hood, Turret turret) {
    return Commands.run(() -> shootToHub(drive, flywheel, hood, turret))
        .finallyDo(
            () -> {
              flywheel.setVelocity(0);
            });
  }

  private static final ShotCalculator shuttleShotCalculator =
      new ShotCalculator(
          Constants.loopPeriodSecs,
          ShotConstants.passingHoodAngleMap,
          ShotConstants.passingFlywheelSpeedMap,
          ShotConstants.passingTimeOfFlightMap,
          Transform3d.kZero);

  public static void shuttle(Drive drive, Flywheel flywheel, Hood hood, Turret turret) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);

    ShotInputs inputs =
        new ShotInputs(
            drive.getPose3d(),
            drive.getRobotRelVelocities(),
            drive.getFieldRelVelocities(),
            TurretCommands.getShuttleTarget(drive, alliance).getTranslation3d());
    ShotParameters parameters = shuttleShotCalculator.calculate(inputs);

    flywheel.setVelocity(parameters.flywheelSpeedRPM());
    hood.setAngle(parameters.hoodAngle());
    turret.setDesiredAngleFieldRelative(
        parameters.fieldRelTurretAngle(), parameters.driveFieldRelRotation2d());
  }

  public static Command shuttleCommand(Drive drive, Flywheel flywheel, Hood hood, Turret turret) {
    return Commands.run(() -> shuttle(drive, flywheel, hood, turret))
        .finallyDo(
            () -> {
              flywheel.setVelocity(0);
            });
  }

  public static Command shootCommand(Flywheel flywheel, Turret turret, Hood hood, Drive drive) {
    return Commands.run(
            () -> {
              Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
              if (RebuiltFieldUtils.isInAllianceZone(alliance, drive.getPose())) {
                shootToHub(drive, flywheel, hood, turret);
              } else {
                shuttle(drive, flywheel, hood, turret);
              }
            },
            flywheel, turret, hood)
        .finallyDo(
            () -> {
              flywheel.setVelocity(0);
              Drive.driveSpeedMultiplier = 1;
            });
  }
}
