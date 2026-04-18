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

package org.austincans.frc2026.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;
import org.austincans.frc2026.subsystems.turret.TurretIO.TurretIOOutputMode;
import org.austincans.frc2026.subsystems.turret.TurretIO.TurretIOOutputs;
import org.austincans.frc2026.subsystems.vision.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

public class Turret extends AdvancedSubsystem {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final TurretIOOutputs outputs = new TurretIOOutputs();

  public Turret(TurretIO io) {
    this.io = io;
    io.registerSelfCheckHardware(this);
    registerCheckCommand(systemCheckCommand());
    setDesiredAngle(Rotation2d.kZero);
  }

  public void updateLimelightRobotSpace() {
    Rotation2d ccwPositiveRobotRelYaw =
        Rotation2d.fromRadians(inputs.positionRad)
            .unaryMinus()
            .plus(TurretConstants.robotRelTurretRotationOffset);

    double gearToCameraMeters = 0.2665;
    double forwardMeters = -0.123 + gearToCameraMeters * ccwPositiveRobotRelYaw.getCos();
    // right positive, negative because sin of ccw positive is left positive
    double sideMeters =
        -gearToCameraMeters * ccwPositiveRobotRelYaw.getSin(); // left of robot center

    double upMeters = 0.331; // up from robot center
    double rollDeg = 0.0;
    double pitchDeg = 15.0; // tilted down (e.g. camera tilted down as arm moves)
    double yawDeg = ccwPositiveRobotRelYaw.getDegrees();

    LimelightHelpers.setCameraPose_RobotSpace(
        "limelight-turret", forwardMeters, sideMeters, upMeters, rollDeg, pitchDeg, yawDeg);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    updateLimelightRobotSpace();
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  public void setVoltage(double voltage) {
    outputs.mode = TurretIOOutputMode.VOLTAGE;
    outputs.voltage = voltage;
  }

  public void setVelocity(double velocityRadPerSec) {
    outputs.mode = TurretIOOutputMode.VELOCITY;
    outputs.velocityRadPerSec = velocityRadPerSec;
  }

  public void stop() {
    setVoltage(0);
  }

  /** Relative to turret's zero */
  public void setDesiredAngle(Rotation2d turretRelAngle) {
    outputs.mode = TurretIOOutputMode.ANGLE;
    outputs.angle = turretRelAngle.unaryMinus();
  }

  /** Relative to robot's zero */
  public void setDesiredAngleRobotRelative(Rotation2d robotRelativeAngle) {
    setDesiredAngle(robotRelativeAngle.plus(TurretConstants.robotRelTurretRotationOffset));
  }

  /**
   * Set the desired angle of the turret to a field relative angle
   *
   * @param fieldRelativeAngle The field relative angle to set turret to
   * @param currentPose Current pose of the robot
   */
  public void setDesiredAngleFieldRelative(
      Rotation2d fieldRelativeAngle, Rotation2d driveFieldRelRotation) {
    // unary minus because turret angle is clockwise positive,
    // while robot rotation is counterclockwise positive
    setDesiredAngleRobotRelative(fieldRelativeAngle.minus(driveFieldRelRotation));
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(inputs.positionRad);
  }

  public boolean atSetpoint() {
    return Math.abs(inputs.positionRad - outputs.angle.getRadians()) < TurretConstants.atSetpointThresholdRad;
  }

  public Command resetAngle() {
    return Commands.runOnce(
        () -> {
          io.resetAngle(Rotation2d.kZero);
        },
        this);
  }

  @Override
  public Command systemCheckCommand() {
    return Commands.sequence(
            Commands.waitSeconds(1.0),
            Commands.runOnce(
                () -> {
                  if (inputs.encoderVelocityRadPerSec < Units.degreesToRadians(5)) {
                    addSubsystemFault(
                        "TRRT",
                        "[System Check] Turret absolute encoder velocity measured too slow",
                        false);
                  }
                  setVelocity(0);
                },
                this),
            Commands.runOnce(() -> setDesiredAngle(Rotation2d.fromDegrees(-20.0)), this),
            Commands.waitSeconds(2.0),
            Commands.runOnce(
                () -> {
                  if (inputs.positionRad < Units.degreesToRadians(-25)
                      || inputs.positionRad > Units.degreesToRadians(-15)) {
                    addSubsystemFault(
                        "TRRT", "[System Check] Turret did not reach desired angle", false);
                  }
                },
                this),
            Commands.runOnce(() -> setDesiredAngle(Rotation2d.kZero), this),
            Commands.waitSeconds(2.0),
            Commands.runOnce(
                () -> {
                  if (Math.abs(inputs.positionRad) > Units.degreesToRadians(5)) {
                    addSubsystemFault(
                        "TRRT", "[System Check] Turret did not reach desired angle", false);
                  }
                },
                this),
            Commands.runOnce(
                () -> {
                  setDesiredAngle(Rotation2d.fromDegrees(0));
                },
                this),
            Commands.waitSeconds(2))
        // Commands.runOnce(
        // () -> {
        // if (magLimitSwitch.get() != true) {
        // addSubsystemFault(
        // "TRRT", "[System Check] Turret did not reach it's zero position");
        // }
        // }))
        .andThen(Commands.runOnce(() -> setVelocity(0), this));
  }
}
