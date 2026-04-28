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

package org.austincans.frc2026.superstructure;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.stream.Collectors;
import org.austincans.frc2026.RobotState;
import org.austincans.frc2026.commands.RebuiltFieldUtils;
import org.austincans.frc2026.subsystems.drive.Drive;
import org.austincans.frc2026.superstructure.shooter.ShotCalculator.ShotParameters;
import org.austincans.frc2026.superstructure.shooter.ShotUtil;
import org.littletonrobotics.junction.AutoLogOutput;

/** This is season specific */
public class Superstructure {
  @SuppressWarnings("unused")
  private final DriveWrapper drive;

  private final TurretWrapper turret;
  private final HoodWrapper hood;

  @SuppressWarnings("unused")
  private final RollerWrapper roller;

  @SuppressWarnings("unused")
  private final DeployWrapper deploy;

  private final FlywheelWrapper flywheel;
  private final IndexerWrapper indexer;
  private final SpindexerWrapper spindexer;

  @AutoLogOutput private boolean staticShooter = false;

  public Superstructure(
      DriveWrapper drive,
      TurretWrapper turret,
      HoodWrapper hood,
      RollerWrapper roller,
      DeployWrapper deploy,
      FlywheelWrapper flywheel,
      IndexerWrapper indexer,
      SpindexerWrapper spindexer) {
    this.drive = drive;
    this.turret = turret;
    this.hood = hood;
    this.roller = roller;
    this.deploy = deploy;
    this.flywheel = flywheel;
    this.indexer = indexer;
    this.spindexer = spindexer;
  }

  private static List<SubsystemBase> getRequirements(SubsystemWrapper<?>... wrappers) {
    return Arrays.stream(wrappers)
        .flatMap(wrapper -> wrapper.getSubsystem().stream())
        .collect(Collectors.toList());
  }

  private static SubsystemBase[] getRequirementsArray(SubsystemWrapper<?>... wrappers) {
    return getRequirements(wrappers).toArray(new SubsystemBase[0]);
  }

  public Command turretPointToHub() {
    return turret.pointToHubCommand(() -> new Pose3d(RobotState.getInstance().getEstimatedPose()));
  }

  public Command turretTrackShuttle() {
    return turret.trackShuttleCommand(
        () -> new Pose3d(RobotState.getInstance().getEstimatedPose()));
  }

  public void runFeederAndFlywheel(double flywheelVelocityRPM) {
    flywheel.setVelocity(flywheelVelocityRPM);
    indexer.run();
    if (flywheel.atSetpoint()) {
      spindexer.run();
    }
  }

  public void stopFeederAndFlywheel() {
    flywheel.stop();
    indexer.stop();
    spindexer.stop();
  }

  public Command runFeederAndFlywheelCommand(DoubleSupplier flywheelVelocityRPM) {
    return Commands.run(
            () -> runFeederAndFlywheel(flywheelVelocityRPM.getAsDouble()),
            getRequirementsArray(flywheel, indexer, spindexer))
        .finallyDo(this::stopFeederAndFlywheel);
  }

  /* Shooter */

  public void trackHub() {
    ShotParameters parameters = ShotUtil.getHubShotParameters();
    hood.setAngle(parameters.hoodAngle());
    turret.setFieldRelAngle(parameters.fieldRelTurretAngle(), parameters.driveFieldRelRotation2d());
  }

  public void shootToHub() {
    ShotParameters parameters = ShotUtil.getHubShotParameters();
    runFeederAndFlywheel(parameters.flywheelSpeedRPM());
    hood.setAngle(parameters.hoodAngle());

    if (staticShooter) {
      turret.setAngle(Rotation2d.kZero);
    } else {
      turret.setFieldRelAngle(
          parameters.fieldRelTurretAngle(), parameters.driveFieldRelRotation2d());
    }
  }

  public Command shootToHubCommand() {
    return Commands.run(
            this::shootToHub, getRequirementsArray(flywheel, hood, turret, indexer, spindexer))
        .finallyDo(this::stopFeederAndFlywheel);
  }

  public void trackShuttle() {
    ShotParameters parameters = ShotUtil.getShuttleShotParameters();
    hood.setAngle(parameters.hoodAngle());
    turret.setFieldRelAngle(parameters.fieldRelTurretAngle(), parameters.driveFieldRelRotation2d());
  }

  public Command shuttleTrackingCommand() {
    return Commands.run(this::trackShuttle, getRequirementsArray(hood, turret));
  }

  public void shuttle() {
    ShotParameters parameters = ShotUtil.getShuttleShotParameters();
    runFeederAndFlywheel(parameters.flywheelSpeedRPM());
    hood.setAngle(parameters.hoodAngle());
    turret.setFieldRelAngle(parameters.fieldRelTurretAngle(), parameters.driveFieldRelRotation2d());
  }

  public Command shuttleCommand() {
    return Commands.run(
            this::shuttle, getRequirementsArray(flywheel, hood, indexer, spindexer, turret))
        .finallyDo(this::stopFeederAndFlywheel);
  }

  public Command shootCommand() {
    return Commands.run(
            () -> {
              Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
              if (RebuiltFieldUtils.isInAllianceZone(
                  alliance, RobotState.getInstance().getEstimatedPose())) {
                shootToHub();
              } else {
                shuttle();
              }
            },
            getRequirementsArray(flywheel, hood, indexer, spindexer, turret))
        .finallyDo(this::stopFeederAndFlywheel);
  }

  public Command trackCommand() {
    return Commands.run(
            () -> {
              Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
              if (RebuiltFieldUtils.isInAllianceZone(
                  alliance, RobotState.getInstance().getEstimatedPose())) {
                trackHub();
              } else {
                trackShuttle();
              }
            },
            getRequirementsArray(turret, hood))
        .finallyDo(() -> Drive.driveSpeedMultiplier = 1);
  }

  public Command staticShooterToggleCommand() {
    return Commands.runOnce(() -> staticShooter = !staticShooter);
  }

  public Command deployCommand() {
    return deploy.deployCommand().alongWith(roller.stopCommand());
  }

  public Command retractCommand() {
    return deploy.retractCommand().alongWith(roller.stopCommand());
  }

  /* Auton Only Commands */

  public Command stillShootCommand() {
    return stillShootCommand(7);
  }

  public Command stillShootCommand(double seconds) {
    return Commands.run(
            () -> {
              Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
              if (RebuiltFieldUtils.isInAllianceZone(
                  alliance, RobotState.getInstance().getEstimatedPose())) {
                shootToHub();
              } else {
                shuttle();
              }
            },
            getRequirementsArray(flywheel, hood, indexer, spindexer, turret, drive))
        .withTimeout(seconds)
        .finallyDo(this::stopFeederAndFlywheel);
  }

  public Command deployAndRunIntake() {
    return Commands.parallel(
        Commands.sequence(Commands.waitSeconds(0.2), roller.runCommand()),
        deploy.deploySequenceCommand());
  }

  public Command retractAndStopIntake() {
    return Commands.sequence(roller.stopCommand(), deploy.retractSequenceCommand());
  }
}
