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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import org.austincans.frc2026.subsystems.selfcheck.AdvancedSubsystem;
import org.littletonrobotics.junction.Logger;

public class Turret extends AdvancedSubsystem {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  public Turret(TurretIO io) {
    this.io = io;
    io.getHardware().forEach(e -> registerHardware(e));
    registerCheckCommand(systemCheckCommand());
    io.setDesiredAngle(Rotation2d.fromDegrees(0));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
  }

  @Override
  public void simulationPeriodic() {}

  public void setTurretVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void setTurretVelocity(double velocityRadPerSec) {
    io.setVelocity(velocityRadPerSec);
  }

  /**
   * Get the value of the absolute encoder
   *
   * @return Absolute turret angle in radians
   */
  public double getTurretAngle() {
    return inputs.positionRad;
  }

  /**
   * Create a command to set the turret angle
   *
   * @param turretAngle Desired turret angle
   * @return Command to rotate turret to given angle
   */
  public Command setTurretAngleCommand(Rotation2d turretAngle) {
    return Commands.run(() -> io.setDesiredAngle(turretAngle), this);
  }

  public void pointTurretToPosition(Translation3d targetPos, double aimPointLeft) {
    Pose2d robotPose = new Pose2d(); // TODO insert robot pose
    Rotation2d jointToTargetAngle =
        targetPos.toTranslation2d().minus(robotPose.getTranslation()).getAngle();

    double distance = targetPos.toTranslation2d().getDistance(robotPose.getTranslation());

    double cosC =
        (Math.pow(distance, 2) + Math.pow(distance, 2) - Math.pow(aimPointLeft, 2))
            / (2 * distance * distance);
    double angleOffset = Math.copySign(Math.acos(cosC), aimPointLeft);

    setDesiredAngleFieldRelative(
        jointToTargetAngle.minus(new Rotation2d(angleOffset)), robotPose.getRotation());
  }

  public double getTurretVelocity() {
    return inputs.velocityRadPerSec;
  }

  public Command pointTurretToPositionCommand(Translation3d fieldPos, double aimPointLeft) {
    return Commands.run(() -> pointTurretToPosition(fieldPos, aimPointLeft), this);
  }

  /**
   * Create a command to move the turret to the resting position
   *
   * @return Command to move turret to resting position
   */
  public Command turretRestingCommand() {
    return Commands.run(() -> io.setDesiredAngle(Rotation2d.fromDegrees(0)), this);
    // .until(() -> magLimitSwitch.get());
  }

  public void setDesiredAngleRobotRelative(Rotation2d robotRelativeAngle) {
    io.setDesiredAngle(robotRelativeAngle.unaryMinus());
  }

  /**
   * Set the desired angle of the turret to a field relative angle
   *
   * @param fieldRelativeAngle The field relative angle to set turret to
   * @param currentPose Current pose of the robot
   */
  public void setDesiredAngleFieldRelative(
      Rotation2d fieldRelativeAngle, Rotation2d driveFieldRelRotation) {
    Rotation2d desiredAngle = fieldRelativeAngle.minus(driveFieldRelRotation);

    Logger.recordOutput("Turret angle", desiredAngle);

    io.setDesiredAngle(desiredAngle.plus(Rotation2d.k180deg).unaryMinus());
  }

  /**
   * Create a command to hold the turret at a field relative angle
   *
   * @param fieldRelativeAngle The field relative angle to hold, in degrees
   * @return Command to hold turret at field relative angle
   */
  public Command fieldRelativeAngleCommand(
      Rotation2d fieldRelativeAngle, Rotation2d robotRotation) {
    return Commands.run(
        () -> setDesiredAngleFieldRelative(fieldRelativeAngle, robotRotation), this);
  }

  /**
   * Create a command to move the turret angle manually
   *
   * @param angleControl gets the button configured to command
   * @return A command to move the turret angle manually
   */
  public Command angleManualControl(DoubleSupplier angleControl) {
    return Commands.run(
        () ->
            io.setDesiredAngle(
                Rotation2d.fromRadians(getTurretAngle() + angleControl.getAsDouble() * 1)),
        this);
  }

  public Command runTurret(double velocityRadPerSec) {
    return Commands.run(() -> setTurretVelocity(velocityRadPerSec));
  }

  @Override
  public Command systemCheckCommand() {
    return Commands.sequence(
            Commands.waitSeconds(1.0),
            Commands.runOnce(
                () -> {
                  if (inputs.velocityRadPerSec < Units.degreesToRadians(5)) {
                    addSubsystemFault(
                        "TRRT",
                        "[System Check] Turret absolute encoder velocity measured too slow",
                        false);
                  }
                  io.setVelocity(0);
                },
                this),
            Commands.runOnce(() -> io.setDesiredAngle(Rotation2d.fromDegrees(-20.0)), this),
            Commands.waitSeconds(2.0),
            Commands.runOnce(
                () -> {
                  if (getTurretAngle() < Units.degreesToRadians(-25)
                      || getTurretAngle() > Units.degreesToRadians(-15)) {
                    addSubsystemFault(
                        "TRRT", "[System Check] Turret did not reach desired angle", false);
                  }
                },
                this),
            Commands.runOnce(() -> io.setDesiredAngle(Rotation2d.fromDegrees(0.0)), this),
            Commands.waitSeconds(2.0),
            Commands.runOnce(
                () -> {
                  if (Math.abs(getTurretAngle()) > Units.degreesToRadians(5)) {
                    addSubsystemFault(
                        "TRRT", "[System Check] Turret did not reach desired angle", false);
                  }
                },
                this),
            Commands.runOnce(
                () -> {
                  io.setDesiredAngle(Rotation2d.fromDegrees(0));
                },
                this),
            Commands.waitSeconds(2))
        // Commands.runOnce(
        //     () -> {
        //       if (magLimitSwitch.get() != true) {
        //         addSubsystemFault(
        //             "TRRT", "[System Check] Turret did not reach it's zero position");
        //       }
        //     }))
        .andThen(Commands.runOnce(() -> io.setVelocity(0), this));
  }
}
