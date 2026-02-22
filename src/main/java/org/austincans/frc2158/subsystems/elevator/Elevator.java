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

package org.austincans.frc2158.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.austincans.frc2158.Constants;
import org.austincans.frc2158.subsystems.elevator.ElevatorConstants.ElevatorState;
import org.austincans.frc2158.subsystems.selfcheck.AdvancedSubsystem;
import org.littletonrobotics.junction.Logger;

public class Elevator extends AdvancedSubsystem {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final ProfiledPIDController elevatorPID =
      new ProfiledPIDController(
          2,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ElevatorConstants.maxVelocityMetersPerSec,
              ElevatorConstants.maxAccelerationMetersPerSecSq));

  private final ElevatorFeedforward feedforward =
      new ElevatorFeedforward(
          0.1, // kS
          0.1, // kG
          0.1, // kV
          0.1 // kA
          );

  public Elevator(ElevatorIO io) {
    this.io = io;
    io.getHardware().forEach(e -> registerHardware(e));
    elevatorPID.setTolerance(ElevatorConstants.pidTolerance);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator/Inputs", inputs);

    double feedbackOutput = elevatorPID.calculate(getHeight()) * Constants.maxVoltage;
    double feedforwardOutput = feedforward.calculate(elevatorPID.getSetpoint().velocity);
    double output = Math.max(feedbackOutput + feedforwardOutput, ElevatorConstants.minVoltage);
    io.runVoltage(output);
  }

  /**
   * Create command to move elevator to position (height)
   *
   * @param position in meters
   * @return Command
   */
  public void setPosition(double position) {
    elevatorPID.setGoal(position);
  }

  /**
   * Create command to move elevator to goal state
   *
   * @param newGoal ElevatorState
   * @return Command
   */
  public void setGoalState(ElevatorState newGoal) {
    setPosition(newGoal.getSetpointMeters());
  }

  /** Elevator height in meters */
  public double getHeight() {
    return inputs.heightMeters;
  }

  /** Return whether elevator at setpoint */
  public boolean atSetpoint() {
    return elevatorPID.atGoal();
  }

  public Command moveAtVelocityCommand(double velocity) {
    return Commands.runOnce(
        () ->
            io.runVoltage(
                velocity / ElevatorConstants.maxVelocityMetersPerSec * Constants.maxVoltage));
  }

  public Command stopCommand() {
    return Commands.runOnce(() -> io.runVoltage(0));
  }

  @Override
  public Command systemCheckCommand() {
    // TODO Auto-generated method stub
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  clearFaults();
                  io.runVoltage(0);
                  setPosition(0.5);
                },
                this),
            Commands.waitSeconds(0.3),
            Commands.runOnce(
                () -> {
                  if (inputs.velocityMetersPerSec < 1) {
                    addFault(
                        "[System Check] Rotation motor encoder velocity measured too slow",
                        false,
                        true);
                  }
                },
                this),
            Commands.run(
                    () -> {
                      setPosition(1);
                    },
                    this)
                .withTimeout(1.0),
            Commands.runOnce(
                () -> {
                  if (inputs.heightMeters < 0.95 || inputs.heightMeters > 1.05) {
                    addFault(
                        "[System Check] Rotation Motor did not reach target position", false, true);
                  }
                },
                this),
            Commands.runOnce(
                () -> {
                  moveAtVelocityCommand(1);
                },
                this),
            Commands.waitSeconds(0.5),
            Commands.runOnce(
                () -> {
                  if (inputs.velocityMetersPerSec < 0.95) {
                    addFault("[System Check] Drive motor encoder velocity too slow", false, true);
                  }
                  moveAtVelocityCommand(0);
                },
                this),
            Commands.waitSeconds(0.25))
        .until(() -> getFaults().size() > 0)
        .andThen(Commands.runOnce(this::stopCommand, this));
  }
}
