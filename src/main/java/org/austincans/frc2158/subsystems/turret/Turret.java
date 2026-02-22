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
// Updated on Saturday February 21, 2026 at 20:04:14 CST

package org.austincans.frc2158.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import org.austincans.frc2158.subsystems.selfcheck.AdvancedSubsystem;
import org.littletonrobotics.junction.Logger;

public class Turret extends AdvancedSubsystem {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  public Turret(TurretIO io) {
    this.io = io;
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

  public void setTurretVelocity(double degPerSec) {
    io.setVelocity(degPerSec);
  }

  /**
   * Get the value of the absolute encoder
   *
   * @return Absolute turret angle in degrees
   */
  public double getTurretAngle() {
    return inputs.positionDeg;
  }

  /**
   * Create a command to set the turret angle
   *
   * @param turretAngle Desired turret angle in degrees
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
    return inputs.velocityDegPerSec * 360.0;
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
  }

  /**
   * Set the desired angle of the turret to a field relative angle
   *
   * @param fieldRelativeAngle The field relative angle to set turret to
   * @param currentPose Current pose of the robot
   */
  public void setDesiredAngleFieldRelative(
      Rotation2d fieldRelativeAngle, Rotation2d currentRotation) {
    Rotation2d desiredAngle = fieldRelativeAngle.minus(currentRotation);

    // if (desiredAngle.getDegrees() < ConfigUtil.getConfigDouble("turretForwardLimit", 360)
    //     && desiredAngle.getDegrees() > ConfigUtil.getConfigDouble("turretReverseLimit", 0)) {
    io.setDesiredAngle(fieldRelativeAngle);
    // } else {
    //   io.setDesiredAngle(Rotation2d.fromDegrees(0));
    // }
  }

  /**
   * Create a command to hold the turret at a field relative angle
   *
   * @param fieldRelativeAngle The field relative angle to hold, in degrees
   * @return Command to hold turret at field relative angle
   */
  public Command fieldRelativeAngleCommand(double fieldRelativeAngle) {
    return Commands.run(
        () ->
            setDesiredAngleFieldRelative(
                Rotation2d.fromDegrees(fieldRelativeAngle),
                new Rotation2d() // TODO: insert Swerve Pose
                ),
        this);
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
                Rotation2d.fromDegrees(getTurretAngle() + angleControl.getAsDouble() * 1)),
        this);
  }

  @Override
  protected Command systemCheckCommand() {
    return Commands.sequence(
            Commands.waitSeconds(1.0),
            Commands.runOnce(
                () -> {
                  if (inputs.velocityDegPerSec * 360.0 < 5) {
                    addFault(
                        "[System Check] Turret absolute encoder velocity measured too slow",
                        false,
                        true);
                  }
                  io.setVelocity(0);
                },
                this),
            Commands.runOnce(() -> io.setDesiredAngle(Rotation2d.fromDegrees(-20.0)), this),
            Commands.waitSeconds(2.0),
            Commands.runOnce(
                () -> {
                  if (getTurretAngle() < -25 || getTurretAngle() > -15) {
                    addFault("[System Check] Turret did not reach desired angle", false, true);
                  }
                },
                this),
            Commands.runOnce(() -> io.setDesiredAngle(Rotation2d.fromDegrees(0.0)), this),
            Commands.waitSeconds(2.0),
            Commands.runOnce(
                () -> {
                  if (Math.abs(getTurretAngle()) > 5) {
                    addFault("[System Check] Turret did not reach desired angle", false, true);
                  }
                },
                this))
        .until(() -> getFaults().size() > 0)
        .andThen(Commands.runOnce(() -> io.setVelocity(0), this));
  }
}
