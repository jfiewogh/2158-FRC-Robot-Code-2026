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

package org.austincans.lib.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.Volts;

import org.austincans.lib.subsystems.elevator.ElevatorConfig.ElevatorState;
import org.austincans.lib.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import org.austincans.lib.subsystems.elevator.ElevatorIO.ElevatorIOOutputs;
import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;
import org.littletonrobotics.junction.Logger;

public class Elevator extends AdvancedSubsystem {
  private final ElevatorIO io;
  private final ElevatorIOInputs inputs = new ElevatorIOInputs();
  private final ElevatorIOOutputs outputs = new ElevatorIOOutputs();

  private final ProfiledPIDController elevatorPID =
      new ProfiledPIDController(
          2,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ElevatorConfig.maxVelocityMetersPerSec,
              ElevatorConfig.maxAccelerationMetersPerSecSq));

  private final ElevatorFeedforward feedforward =
      new ElevatorFeedforward(
          0.1, // kS
          0.1, // kG
          0.1, // kV
          0.1 // kA
          );

  public Elevator(ElevatorIO io) {
    this.io = io;
    registerCheckCommand(systemCheckCommand());
    elevatorPID.setTolerance(ElevatorConfig.pidTolerance);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // Logger.processInputs("Elevator/Inputs", inputs);

    double feedbackOutput = elevatorPID.calculate(getHeight()) * ElevatorConfig.maxVoltage;
    double feedforwardOutput = feedforward.calculate(elevatorPID.getSetpoint().velocity);
    double output = Math.max(feedbackOutput + feedforwardOutput, ElevatorConfig.minVoltage);
    outputs.motorOutputs.setVoltage(Volts.of(output));
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
        () -> outputs.motorOutputs.setVoltage(Volts.of(velocity / ElevatorConfig.maxVelocityMetersPerSec * ElevatorConfig.maxVoltage)));
  }

  public Command stopCommand() {
    return Commands.runOnce(() -> outputs.motorOutputs.setVoltage(Volts.of(0)));
  }

  @Override
  public Command systemCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  clearFaults();
                  outputs.motorOutputs.setVoltage(Volts.of(0));
                  setPosition(0.5);
                },
                this),
            Commands.waitSeconds(0.3),
            Commands.runOnce(
                () -> {
                  if (inputs.velocityMetersPerSec < 1) {
                    addSubsystemFault(
                        "ELEV",
                        "[System Check] Rotation motor encoder velocity measured too slow",
                        false);
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
                    addSubsystemFault(
                        "ELEV",
                        "[System Check] Rotation Motor did not reach target position",
                        false);
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
                    addSubsystemFault(
                        "ELEV", "[System Check] Drive motor encoder velocity too slow", false);
                  }
                  moveAtVelocityCommand(0);
                },
                this),
            Commands.waitSeconds(0.25))
        .andThen(Commands.runOnce(this::stopCommand, this));
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }
}
