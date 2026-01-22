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
// Updated on Thursday January 22, 2026 at 17:29:37 CST

package org.austincans.frc2158.subsystems.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.austincans.frc2158.subsystems.selfcheck.AdvancedSubsystem;

public class Arm extends AdvancedSubsystem {
  private ArmIO io;
  private ArmIOInputsAutoLogged inputs;
  private double gearRatio = ArmConstants.armGearRatio;

  public Arm(ArmIO io) {
    this.io = io;
  }

  /**
   * @return Returns the position value divided by the gear ratio.
   */
  public double getPosition() {
    return inputs.position / gearRatio;
  }

  public Command moveToAngleCommand(double angleDegrees) {
    return new ConditionalCommand(
        new InstantCommand(
            () -> {
              io.setPosition(angleDegrees / gearRatio);
            }),
        null,
        () -> {
          double currentAngle = Units.radiansToDegrees(getPositionRadians());
          return Math.abs(angleDegrees - currentAngle) < 2.0;
        });
  }

  public Command stopCommand() {
    return new InstantCommand(() -> io.setVoltage(0));
  }

  public Command moveAtVelocityCommand(double velocityDegPerSec) {
    return new InstantCommand(() -> io.setVoltage(velocityDegPerSec));
  }

  public double getVelocity() {
    return inputs.velocity;
  }

  public double getPositionRadians() {
    double rotations = getPosition();
    return rotations * 2.0 * Math.PI;
  }

  public double getAmps() {
    return inputs.currentAmps;
  }

  public double getCurrentVoltage() {
    return inputs.currentVoltage;
  }

  public double getTemperature() {
    return inputs.temperature;
  }

  public void setAngle(double angleDegrees) {
    io.setPosition(angleDegrees);
  }

  public Command setAngleCommand(double angleDegrees) {
    return new InstantCommand(() -> setAngle(angleDegrees));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.checkForFaults().forEach(this::addFault);
  }

  @Override
  protected Command systemCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(() -> io.setVelocity(2)),
            Commands.waitSeconds(0.5),
            Commands.runOnce(
                () -> {
                  if (getVelocity() < 0.1) {
                    addFault("[System Check] Extension velocity measured too slow", false, true);
                  }
                  io.setVelocity(0);
                },
                this),
            Commands.runOnce(() -> setAngle(0), this),
            Commands.waitSeconds(2.0),
            Commands.runOnce(
                () -> {
                  if (Units.rotationsToDegrees(getPosition()) < -15
                      || Units.rotationsToDegrees(getPosition()) > 15) {
                    addFault("[System Check] Joint did not reach target position", false, true);
                  }
                },
                this),
            Commands.runOnce(() -> setAngle(90), this),
            Commands.waitSeconds(2.0),
            Commands.runOnce(
                () -> {
                  if (Units.rotationsToDegrees(getPosition()) < 75
                      || Units.rotationsToDegrees(getPosition()) > 115) {
                    addFault("[System Check] Joint did not reach target position", false, true);
                  }
                },
                this))
        .until(() -> getFaults().size() > 0)
        .andThen(
            Commands.runOnce(
                () -> {
                  io.setVoltage(0);
                },
                this));
  }
}
